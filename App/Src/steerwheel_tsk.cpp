#include "steerwheel_tsk.h"
#include "ethernet_tsk.h"
#include "math.h"

#include "cmd_tsk.h"
namespace TskSteer
{
    const int tskStkSize = 512;
    SteerCmd *steerCmd = nullptr;
    SteerVal *steerVal = nullptr;
    moto_measure_t *motor = nullptr;
    uint32_t steerCnt = 0;

    // linear velocity (m/s) to motor rotate speed (rpm)
    inline float dr1_vel2rpm(const float vel)
    {
        return vel * ratio * toRPM / wheelR;
    }
    //  motor rotate speed (rpm) to linear velocity (m/s)
    inline float dr1_rpm2vel(int16_t rpm)
    {
        return ((float)rpm) * wheelR / ratio / toRPM;
    }
    // motor angle to distance (m)
    inline float dr1_ang2dis(int32_t total_angle)
    {
        return ((float)total_angle) / 8192.0f / ratio * 2.f * PI * wheelR;
    }

    // motor angle to wheel angle (rad)
    inline float dr2_angConvert(int32_t total_angle)
    {
        return ((float)total_angle) / 8192.0f / ratio2 * 2.f * PI;
    }
    // motor rotate speed (rpm) to wheel rotate speed (rad/s)
    inline float dr2_rpmConvert(int16_t rpm)
    {
        return ((float)rpm) / ratio2 / toRPM;
    }
    inline float saturate(float v, float max, float min)
    {
        return v > max ? max : v < min ? min
                                       : v;
    }
    inline float cacul_ppi_angle(const float tar, const float cur)
    {
        return (tar - cur < -PI) ? (tar - cur + 2 * PI) : (tar - cur > PI) ? (tar - cur - 2 * PI)
                                                                           : (tar - cur);
    }

    void steerTask(void *pvParameters)
    {
        BaseType_t rtn;
        can1RxQueueHandle = xQueueCreate(4, sizeof(moto_measure_t));

        CAN_Start_Trans();
        // dr1 vel loop
        Pid vVelPidMotion(vVelPidP, vVelPidI, vVelPidD, 1,
                          motorTs, -0.95f * C610Current, 0.95f * C610Current, vVelPidIband, -0.95f * C610Current, 0.95f * C610Current);

        // dr2 pos loop
        ffPid thPosPidMotion(thPosPidP, thPosPidI, 50,
                             motorTs, -maxSteerOmg / 5, maxSteerOmg / 5, thPosPidIband, -maxSteerOmg, maxSteerOmg);

        // dr2 vel loop
        Pid thVelPidMotion(thVelPidP, thVelPidI, thVelPidD, 1,
                           motorTs, -0.95f * C610Current, 0.95f * C610Current, thVelPidIband, -0.95f * C610Current, 0.95f * C610Current);

        float dr1_delta_v = 0.f, dr2_delta_p = 0.f;
        float dr1_tar_i = 0.f, dr2_tar_i = 0.f;
        uint8_t zero = 1;
        uint8_t curCmd[8] = {0};
        // 上电时电流均给0
        CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);

        while (true)
        {
            rtn = xSemaphoreTake(motionTickSem, 2);
            configASSERT(rtn);
            steerCnt++;
            // update motor info
            if (pdPASS == xQueueReceive(can1RxQueueHandle, motor, 0))
            {
                if (motor->id == 0)
                {
                    steerVal->dr1_real_cur = motor->given_current * C610ICoeff;
                    steerVal->dr1_real_vel = dr1_rpm2vel(motor->speed_rpm);
                }
                else if (motor->id == 1)
                {
                    steerVal->dr2_real_cur = motor->given_current * C610ICoeff;
                    steerVal->dr2_real_pos = dr2_angConvert(motor->total_angle);
                    steerVal->dr2_real_vel = dr2_rpmConvert(motor->speed_rpm);
                }
            }

            // update motor control command
            xQueueReceive(TskEth::steerCmdQueue, steerCmd, 0);

            if (steerCnt % motorTick == 0)
            {
                if (steerCmd->state == steerState::RESET && steerVal->state != steerState::RESETOVER)
                {
                    if (steerVal->state != steerState::RESETTING && fabs(steerVal->dr1_real_vel) < 0.05f && fabs(steerVal->dr2_real_vel) < 0.05f)
                    {
                        steerCmd->dr1_tar_vel = 0.f;
                        // pi/2 rad/s旋转
                        steerVal->dr2_tar_vel = 0.f;
                        steerCmd->dr2_tar_pos = 0.f;
                        steerVal->state = steerState::RESETTING;
                    }
                    else if (steerVal->state == steerState::RESETTING)
                    {
                        steerCmd->dr1_tar_vel = 0.f;
                        // pi/2 rad/s旋转
                        steerVal->dr2_tar_vel = PI / 4.f;
                        steerCmd->dr2_tar_pos = 0.f;
                    }
                    else
                    {
                        // 非停机情况下，不允许复位
                        eprint((char *)"MUST stop before resetting");
                        steerVal->state = steerState::STOP;
                    }
                    /*
                     * update steerwheel M2006 zero info, if blocked light reflects, it gets 0, else 1
                     * LIGHT_SW0 is output in other boardtype, only in steerwheel as input
                     */
                    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(LaserSensor0_GPIO_Port, LaserSensor0_Pin))
                    {
                        // 触发复位消息
                        xQueueSend(reset_flag, (void *)&zero, 0);
                        steerVal->state = steerState::RESETOVER;
                    }
                }
                else if (steerCmd->state == steerState::NORMAL)
                {
                    steerVal->state = steerCmd->state;
                    // dr1 限幅，单位换算 m/s to rad/s，计算增量
                    dr1_delta_v = dr1_vel2rpm(saturate(steerCmd->dr1_tar_vel, maxVel, -maxVel) - steerVal->dr1_real_vel);
                    // dr2 限幅，优弧处理，计算增量
                    dr2_delta_p = cacul_ppi_angle(saturate(steerCmd->dr2_tar_pos, PI, -PI), steerVal->dr2_real_pos);
                }
                else if (steerCmd->state == steerState::STOP)
                {
                    vVelPidMotion.Reset();
                    thPosPidMotion.Reset();
                    thVelPidMotion.Reset();
                    steerVal->state = steerState::STOP;
                }
            }
            
            if (steerCmd->state == steerState::TORQUE)
            {
                steerVal->state = steerCmd->state;
                // 限幅，电流值换算
                int16_t cur = (int16_t)(saturate(steerCmd->dr1_tar_cur, maxCurrent, -maxCurrent) / C610ICoeff);
                // 扭矩控制时，目标转速默认为实际转速
                steerCmd->dr1_tar_vel = steerVal->dr1_real_vel;
                // 电流模式目前仅控制dr1 M3508 C620，舵轮相关参数将被忽略
                // 停机位置值
                // steerCmd->dr2_tar_pos = steerVal->dr2_real_pos;
                // steerVal->dr2_tar_vel = 0.f;
                // steerVal->dr2_tar_cur = 0.f;
                curCmd[0] = cur >> 8;
                curCmd[1] = cur & 0xFF;                
                if (steerCnt % motorTick == 0)
                {
                    // dr2 计算轮速，电流，更新指令
                    // 复位过程中，轮速固定，其他过程中通过PID计算得到
                    // dr2 限幅，优弧处理，计算增量
                    dr2_delta_p = cacul_ppi_angle(saturate(steerCmd->dr2_tar_pos, PI, -PI), steerVal->dr2_real_pos);
                    if (steerVal->state != steerState::RESETTING)
                        steerVal->dr2_tar_vel = thPosPidMotion.Tick(dr2_delta_p, 0.f);
                    dr2_tar_i = thVelPidMotion.Tick(steerVal->dr2_tar_vel - steerVal->dr2_real_vel);
                    steerVal->dr2_tar_cur = dr2_tar_i * C610ICoeff;
                    curCmd[2] = (int)dr2_tar_i >> 8;
                    curCmd[3] = (int)dr2_tar_i & 0xFF;
                }
                memset(curCmd + 4, 0, 4);
            }
            else if (steerCmd->state > steerState::STOP && steerVal->state < steerState::RESETOVER)
            {                    
                if (steerCnt % motorTick == 0)
                {
                    // dr1 计算电流，更新指令
                    dr1_tar_i = vVelPidMotion.Tick(dr1_delta_v);
                    // 转速控制时，目标电流通过计算值更新
                    steerCmd->dr1_tar_cur = dr1_tar_i * C610ICoeff;
                    curCmd[0] = (int)dr1_tar_i >> 8;
                    curCmd[1] = (int)dr1_tar_i & 0xFF;
                    // dr2 计算轮速，电流，更新指令
                    // 复位过程中，轮速固定，其他过程中通过PID计算得到
                    if (steerVal->state != steerState::RESETTING)
                        steerVal->dr2_tar_vel = thPosPidMotion.Tick(dr2_delta_p, 0.f);
                    dr2_tar_i = thVelPidMotion.Tick(steerVal->dr2_tar_vel - steerVal->dr2_real_vel);
                    steerVal->dr2_tar_cur = dr2_tar_i * C610ICoeff;
                    curCmd[2] = (int)dr2_tar_i >> 8;
                    curCmd[3] = (int)dr2_tar_i & 0xFF;
                    memset(curCmd + 4, 0, 4);
                }
            }
            else
            {
                // 停机值
                steerCmd->dr1_tar_vel = 0.f;
                steerCmd->dr1_tar_cur = 0.f;
                steerCmd->dr2_tar_pos = steerVal->dr2_real_pos;
                steerVal->dr2_tar_vel = 0.f;
                steerVal->dr2_tar_cur = 0.f;
                memset(curCmd, 0, 8);
            }
            CAN_SendMsg(CAN_Moto_ALL_ID, curCmd);

            // update feedback info
            if (steerCnt % ethPeriod == 0)
            {
                steerVal->dr1_tar_vel = steerCmd->dr1_tar_vel;
                steerVal->dr1_tar_cur = steerCmd->dr1_tar_cur;
                steerVal->dr2_tar_pos = steerCmd->dr2_tar_pos;
                xQueueOverwrite(TskEth::steerValQueue, steerVal);
            }
            
            
//            print((char *)"steerwheel_tsk\r\n");
//             osDelay(2);
        }
    }

    void Init()
    {
        BaseType_t rtn;

        steerCmd = (SteerCmd *)pvPortMalloc(sizeof(SteerCmd));
        if (steerCmd == nullptr)
            return;

        steerVal = (SteerVal *)pvPortMalloc(sizeof(SteerVal));
        if (steerVal == nullptr)
            return;

        motor = (moto_measure_t *)pvPortMalloc(sizeof(moto_measure_t));
        if (motor == nullptr)
            return;

        // Create tasks
        rtn = xTaskCreate(steerTask, (const portCHAR *)"steerTask",
                          tskStkSize, NULL, osPriorityAboveNormal, NULL);
        configASSERT(rtn == pdPASS);
    }
}