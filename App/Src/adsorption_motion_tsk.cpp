
#include "adsorption_motion_tsk.h"
#include "ethernet_tsk.h"
#include "cmd_tsk.h"
#include <math.h>
#include <string.h>

namespace TskMotion
{
    const int tskStkSize = 512;
    AdsorptionCmd *adsorptionCmd = nullptr;
    AdsorptionVal *adsorptionVal = nullptr;
    moto_measure_t *motor = nullptr;

    float delta_x0, delta_x1, delta_y0, delta_y1, x0, y0, z0;
    float nx, ny, nz, L, alpha, beta;
    double mag;
    inline float saturate(float v, float max, float min)
    {
        return v > max ? max : v < min ? min
                                       : v;
    }

    void displacementToPose(const float z[3], float pose[3], uint8_t type)
    {
        /*
         * pose[0] x angle
         * pose[1] y angle
         * pose[2] z displacement
         */
        float delta_z0 = z[0] - z[1];
        float delta_z1 = z[1] - z[2];
        if (type == displacementSrc::SENSOR)
        {
            delta_x0 = sensor_delta_x[0];
            delta_x1 = sensor_delta_x[1];
            delta_y0 = sensor_delta_y[0];
            delta_y1 = sensor_delta_y[1];
            x0 = sensor_x[1];
            y0 = sensor_y[1];
        }
        else if (type == displacementSrc::MOTOR)
        {
            delta_x0 = motor_delta_x[0];
            delta_x1 = motor_delta_x[1];
            delta_y0 = motor_delta_y[0];
            delta_y1 = motor_delta_y[1];
            x0 = motor_x[1];
            y0 = motor_y[1];
        }
        /*
         * | i  j  k |
         * | x0 y0 z0|
         * | x1 y1 z1|
         */
        nx = delta_y0 * delta_z1 - delta_z0 * delta_y1;
        ny = delta_z0 * delta_x1 - delta_x0 * delta_z1;
        nz = -(delta_x0 * delta_y1 - delta_y0 * delta_x1);
        L = sqrtf(nx * nx + ny * ny + nz * nz);
        alpha = acosf(nx / L);
        beta = acosf(ny / L);
        pose[0] = beta - PI_2;
        pose[1] = PI_2 - alpha;
        pose[2] = z[1] - (nx * x0 + ny * y0) / nz;
    }

    void poseToDisplacement(const float cmd[3], float z[3], uint8_t type)
    {
        /*
         * cmd[0] x angle
         * cmd[1] y angle
         * cmd[2] z displacement
         *
         */
        float real_cmd[3];
        real_cmd[0] = saturate(cmd[0], xMaxAngle, -xMaxAngle);
        real_cmd[1] = saturate(cmd[1], yMaxAngle, -yMaxAngle);
        real_cmd[2] = saturate(cmd[2], maxPos, minPos);

        if (type == displacementSrc::SENSOR)
        {
            for (int index = 0; index < 3; index++)
            {
                z[index] = real_cmd[2] + (sensor_x[index] * sinf(real_cmd[1]) - sensor_y[index] * sinf(real_cmd[0])) /
                                             sqrtf(1 - sinf(real_cmd[0]) * sinf(real_cmd[0]) - sinf(real_cmd[1]) * sinf(real_cmd[1]));
                z[index] = saturate(z[index], maxPos, minPos);
            }
        }
        else if (type == displacementSrc::MOTOR)
        {
            for (int index = 0; index < 3; index++)
            {
                z[index] = real_cmd[2] + (motor_x[index] * sinf(real_cmd[1]) - motor_y[index] * sinf(real_cmd[0])) /
                                             sqrtf(1 - sinf(real_cmd[0]) * sinf(real_cmd[0]) - sinf(real_cmd[1]) * sinf(real_cmd[1]));
                z[index] = saturate(z[index], maxPos, minPos);
            }
        }
    }

    void motionTask(void *pvParameters)
    {
        BaseType_t rtn;
        uint8_t curCmd[8] = {0};
        uint8_t motor_state = 0;
        can1RxQueueHandle = xQueueCreate(6, sizeof(moto_measure_t));

        uint16_t rawADC[3] = {0};
        uint32_t motionCnt = 0;
        float motor_tar_p[3] = {0.f}, motor_p[3] = {0.f}, delta_p[3] = {0.f};
        float tar_v[3] = {0.f}, real_v[3] = {0.f};
        int16_t tar_i[3] = {0}, real_i[3] = {0};
        char dbgStr[64];
        StagePid *adPidMotion[3];
        PidParam pidparam;

        uint32_t stalled_cnt[3] = {0};
        CAN_Start_Trans();
        // obtain 3 pos val period
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)rawADC, 3);
        CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);

        for (int index = 0; index < 3; index++)
        {
            adPidMotion[index] = new StagePid(pidparam.adPidP1[index], pidparam.adPidI1[index], pidparam.adPidD1[index], 
                                              pidparam.adPidP2[index], pidparam.adPidI2[index], pidparam.adPidD2[index], 
                                              50, 2.f * motorTs, pidparam.adPidDiffband[index], pidparam.adPidValband[index],
                                              -adsorptionMaxCur, adsorptionMaxCur, -adsorptionMaxCur, adsorptionMaxCur);
        }

        while (true)
        {
            rtn = xSemaphoreTake(motionTickSem, 2);
            configASSERT(rtn);
            motionCnt++;

            if (pdPASS == xQueueReceive(can1RxQueueHandle, motor, 0))
            {
                // 正转归零，因此电机位置转换到位移需负向（转速同）
                adsorptionVal->motor_real_displacement[motor->id] = -motor->total_angle * adsorptionRatio;
                real_v[motor->id] = -motor->speed_rpm * adsorptionVelRatio;
                real_i[motor->id] = motor->given_current;
            }

            if (motionCnt % motorTick == 0)
            {
            #if 1
                if (motionCnt % (2 * motorTick) == 0)
                {
                    adsorptionVal->sensor_real_displacement[0] = 0.4f*(float)rawADC[0] * posCoeff + 0.6f*adsorptionVal->sensor_real_displacement[0];
                    adsorptionVal->sensor_real_displacement[1] = 0.4f*(float)rawADC[1] * posCoeff + 0.6f*adsorptionVal->sensor_real_displacement[1];
                    adsorptionVal->sensor_real_displacement[2] = 0.4f*(float)rawADC[2] * posCoeff + 0.6f*adsorptionVal->sensor_real_displacement[2];
                    displacementToPose(adsorptionVal->sensor_real_displacement, adsorptionVal->plate_real_pose, displacementSrc::SENSOR);
                    poseToDisplacement(adsorptionVal->plate_real_pose, motor_p, displacementSrc::MOTOR);
                }
                
                xQueueReceive(TskEth::adsorptionCmdQueue, adsorptionCmd, 0);
                // initial condition, must reset
                if ((adsorptionVal->state == motionState::INIT) && (adsorptionCmd->state != motionState::RESET))
                {
                    if (motionCnt % 1000 == 0)
                    {
                        eprint((char *)"please RESET before control type2\r\n");
                        print((char *)"please RESET before control\r\n");
                    }
                }
                else if ((adsorptionCmd->state == motionState::RESET) && (adsorptionVal->state != motionState::RESETOVER))
                {
                    for (int idx = 0; idx < 3; idx++)
                    {
                        stalled_cnt[idx] += 1;
                        if (adsorptionVal->state != motionState::FINDZERO)
                        {
                            if (fabs(real_v[idx]) < 0.1f && stalled_cnt[idx] >= 1000)
                            {
                                motor_state |= (1 << idx);
                            }
                            else
                            {
                                motor_state &= ~(1 << idx);
                            }
                            // 避免自带数据产生干扰，清除目标值
                            adsorptionCmd->sensor_tar_displacement[idx] = 0.f;
                            adsorptionCmd->plate_tar_pose[idx] = 0.f;
                        }
                        else
                        {
                            if (fabs(adsorptionVal->sensor_real_displacement[idx] - posHigh) < 0.5f)
                            {
                                motor_state |= (1 << idx);
                            }
                            else
                            {
                                motor_state &= ~(1 << idx);
                            }
                            // 传感器下探
                            adsorptionCmd->sensor_tar_displacement[idx] = posHigh;
                            adsorptionCmd->plate_tar_pose[idx] = 0.f;
                        }
                    }
                    if (adsorptionVal->state != motionState::FINDZERO)
                    {
                        // if (motor_state == 0x7)
                        // {
                        //     motor_state = 0;
                        //     stalled_cnt[0] = 0;
                        //     stalled_cnt[1] = 0;
                        //     stalled_cnt[2] = 0;
                        //     adsorptionVal->state = motionState::RESETOVER;
                        //     // adsorptionVal->state = motionState::FINDZERO;
                        // }
                        // else
                        // {
                        //     adsorptionVal->state = motionState::RESETTING;
                        // }
                        adsorptionVal->state = motionState::FINDZERO;
                    }
                    else
                    {
                        if (motor_state == 0x7)
                        {
                            motor_state = 0;
                            stalled_cnt[0] = 0;
                            stalled_cnt[1] = 0;
                            stalled_cnt[2] = 0;
                            adsorptionVal->state = motionState::RESETOVER;
                        }
                    }
                }
                else if (adsorptionCmd->state == motionState::START)
                {
                    adsorptionVal->state = adsorptionCmd->state;
                    if (motionCnt % (10 * motorTick) == 0)
                    {
                        // 将目标位移数据转换成目标姿态
                        displacementToPose(adsorptionCmd->sensor_tar_displacement, adsorptionCmd->plate_tar_pose, displacementSrc::SENSOR);
                        // 将目标姿态转换成电机位置
                        poseToDisplacement(adsorptionCmd->plate_tar_pose, motor_tar_p, displacementSrc::MOTOR);
                    }
                }
                else if (adsorptionCmd->state == motionState::MANUAL)
                {
                    adsorptionVal->state = adsorptionCmd->state;
                    if (motionCnt % (10 * motorTick) == 0)
                    {
                        // 将姿态解算成传感器位移数据
                        poseToDisplacement(adsorptionCmd->plate_tar_pose, adsorptionCmd->sensor_tar_displacement, displacementSrc::SENSOR);
                        // 将姿态解算成电机位移数据
                        poseToDisplacement(adsorptionCmd->plate_tar_pose, motor_tar_p, displacementSrc::MOTOR);
                    }
                }
                else if (adsorptionCmd->state == motionState::STOP)
                {
                    adsorptionVal->state = adsorptionCmd->state;
                }

                if (adsorptionCmd->state > motionState::STOP && (adsorptionVal->state < RESETOVER || adsorptionVal->state == motionState::INIT))
                {
                    for (int idx = 0; idx < 3; idx++)
                    {
                        if (adsorptionVal->state == motionState::RESETTING)
                        {
                            tar_i[idx] = (int)(resetCurrent[idx] / C610ICoeff);
                            curCmd[2 * idx] = tar_i[idx] >> 8;
                            curCmd[2 * idx + 1] = tar_i[idx] & 0xFF;
                        }
                        else
                        {
                            if (motionCnt % (10 * motorTick) == 0)
                            {
                                if (adsorptionVal->state != motionState::INIT)
                                    delta_p[idx] = adsorptionCmd->sensor_tar_displacement[idx] - adsorptionVal->sensor_real_displacement[idx];
                                else
                                    delta_p[idx] = adsorptionCmd->plate_tar_pose[idx] - adsorptionVal->motor_real_displacement[idx];
                                // delta_p[idx] = saturate(motor_tar_p[idx], maxMotPos, 0) - adsorptionVal->motor_real_displacement[idx];
                                tar_v[idx] = 0.5f * (delta_p[idx]);
                            }
                            
                            if (motionCnt % (2 * motorTick) == 0)
                            {                              
                                // 正电流归零，因此需加负号
                                tar_i[idx] = -adPidMotion[idx]->Tick(tar_v[idx] - real_v[idx], real_v[idx]);
                            }

                            if (adsorptionVal->state != motionState::FINDZERO)
                            {
                                // 限制电机运动位移，超过最大值且继续下降，则停机；超过最小值且继续上升，则停机
                                if ((adsorptionVal->motor_real_displacement[idx] > maxMotPos && tar_i[idx] < 0.f) ||
                                    (adsorptionVal->motor_real_displacement[idx] < minMotPos && tar_i[idx] > 0.f))
                                {
                                    tar_i[idx] = 0.f;
                                    if (motionCnt % (100 * motorTick) == 0)
                                    {                              
                                        eprint((char *)"motor displace over limit\r\n");
                                    }
                                }
                            }
                            curCmd[2 * idx] = (int)(tar_i[idx]) >> 8;
                            curCmd[2 * idx + 1] = (int)(tar_i[idx]) & 0xFF;
                        }
                    }
                }
                else
                {
                    for (int idx = 0; idx < 3; idx++)
                    {
                        adPidMotion[idx]->Reset();
                        if (adsorptionVal->state == motionState::RESETOVER)
                        {
                            // 触发复位消息
                            xQueueSend(reset_flag, (void *)&idx, 0);
                            adsorptionVal->motor_real_displacement[idx] = 0.f;
                        }
                        tar_i[idx] = 0.f;
                    }
                    memset(curCmd, 0, 8);
                }
                #endif
                CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);
            }

#ifdef _DEBUG
            if (motionCnt % (10 * motorTick) == 0)
            {
                sprintf(dbgStr, "p:%.2f v:%.2f i:%d rp:%.2f rv:%.2f ri:%d\r\n", adsorptionCmd->sensor_tar_displacement[0], tar_v[0], tar_i[0],
                        adsorptionVal->sensor_real_displacement[0], real_v[0], real_i[0]);
                print(dbgStr);
            }
#endif
            if (motionCnt % ethPeriod == 0)
            {
                for (int idx = 0; idx < 3; idx++)
                {
                   adsorptionVal->sensor_tar_displacement[idx] = adsorptionCmd->sensor_tar_displacement[idx];
                   adsorptionVal->plate_tar_pose[idx] = adsorptionCmd->plate_tar_pose[idx];						
                }
                xQueueOverwrite(TskEth::adsorptionValQueue, adsorptionVal);
            }
            
osDelay(2);
//            print((char *)"adsorption_motion_tsk\r\n");
             
        }
    }

    void Init()
    {
        BaseType_t rtn;

        adsorptionCmd = (AdsorptionCmd *)pvPortMalloc(sizeof(AdsorptionCmd));
        if (adsorptionCmd == nullptr)
            return;

        adsorptionVal = (AdsorptionVal *)pvPortMalloc(sizeof(AdsorptionVal));
        if (adsorptionVal == nullptr)
            return;
        adsorptionVal->state = motionState::INIT;

        motor = (moto_measure_t *)pvPortMalloc(sizeof(moto_measure_t));
        if (motor == nullptr)
            return;

        // Create tasks
        rtn = xTaskCreate(motionTask, (const portCHAR *)"motionTask",
                          tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }
}