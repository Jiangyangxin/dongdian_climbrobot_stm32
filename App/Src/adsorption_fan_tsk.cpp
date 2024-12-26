#include "adsorption_fan_tsk.h"
#include "ethernet_tsk.h"
#include "adsorption_motion_tsk.h"

#include "cmd_tsk.h"

TaskHandle_t fanTask_Handler;
volatile uint16_t cnt;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            BaseType_t pxHigherPriorityTaskWoken;
            cnt = __HAL_TIM_GET_COUNTER(&htim3);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            xTaskNotifyFromISR(fanTask_Handler, cnt, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
        }
    }
}
float adc_true=0;
namespace TskFan
{
    const int tskStkSize = 512;
    FanCmd *fanCmd = nullptr;
    FanVal *fanVal = nullptr;

    inline float saturate(float v, float max, float min)
    {
        return v > max ? max : v < min ? min
                                       : v;
    }

    void fanTask(void *pvParameters)
    {
        uint32_t rawAdcVal = 0;
        float zeroOffset = 0.f;
        float zeroAcc = 0.f;
        float vacuum = 0.f;
        uint16_t zeroCnt = 0, fanCnt = 0, fanbia = 200;
        BaseType_t rtn;
        uint32_t notification_value = 0;

        Pid fanPidMotion(fanP, fanI, 0.f, 0.f, fanTs, -500, 500, fanIBand, 60 - fanbia, 950 - fanbia);
        // obtain barometer val period
        HAL_ADC_Start(&hadc2);
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

        while (true)
        {
            rtn = xSemaphoreTake(fanTickSem, 2);
            configASSERT(rtn);
            fanCnt++;

            if (fanCnt % fanTick == 0)
            {
                if (HAL_OK == HAL_ADC_PollForConversion(&hadc2, 0))
                {
									rawAdcVal = HAL_ADC_GetValue(&hadc2);
									HAL_ADC_Start(&hadc2);
								}
								//adc_true=rawAdcVal;
                vacuum = -((float)rawAdcVal * adcResolution - barometerOffset) * barometerCoeff;
                xQueueReceive(TskEth::fanCmdQueue, fanCmd, 0);
                if ((fanCmd->state == fanState::VACZERO) && (fanVal->state != VACZEROCOMPLETE))
                {
                    zeroCnt++;
                    if (zeroCnt < 12)
                    {
                        fanVal->state = fanCmd->state;
                        // stop fan
                        htim1.Instance->CCR2 = 0;
                        fanVal->fan_pwm = 0;
                        HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_RESET);
                        zeroOffset = 0.f;
                        zeroAcc = 0.f;
                    }
                    else if (zeroCnt < 1012)
                    {
                        zeroAcc += vacuum;
                    }
                    else
                    {
                        zeroOffset = zeroAcc * 0.001f;
                        zeroCnt = 0;
                        fanVal->state = VACZEROCOMPLETE;
                    }
                }
                else if (fanCmd->state == fanState::STOP)
                {
                    fanVal->state = fanCmd->state;
                    // stop fan
                    htim1.Instance->CCR2 = 0;
                    HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_RESET);
                }
                else if (fanCmd->state == fanState::START)
                {
                    fanVal->state = fanCmd->state;
                    HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_SET);
                    fanVal->fan_tar_pre = 0.8f * fanVal->fan_tar_pre + 0.2f * fanCmd->fan_tar_pre;
                    // 反馈量 负压显示为正值，目标值也为正
                    fanVal->fan_tar_pre = saturate(fanVal->fan_tar_pre, 25.f, 0.f);
                    htim1.Instance->CCR2 = fanbia + (int)fanPidMotion.Tick(fanVal->fan_tar_pre - fanVal->fan_real_pre);
                    fanVal->fan_pwm = (float)htim1.Instance->CCR2 / 10;
                }
                fanVal->fan_real_pre = vacuum - zeroOffset;
                if (pdPASS == xTaskNotifyWait(0x00000000, 0x00000000, &notification_value, 0))
                {
                    fanVal->fan_speed = 1000000.f / (float)notification_value;
                }
            }
            if (fanCnt % ethPeriod == 0)
            {
                xQueueOverwrite(TskEth::fanValQueue, fanVal);
            }
            
            
//            print((char *)"adsorption_fan_tsk\r\n");
             
        }
    }

    void Init()
    {
        BaseType_t rtn;

        fanCmd = (FanCmd *)pvPortMalloc(sizeof(FanCmd));
        if (fanCmd == nullptr)
            return;

        fanVal = (FanVal *)pvPortMalloc(sizeof(FanVal));
        if (fanVal == nullptr)
            return;

        // Create tasks
        rtn = xTaskCreate(fanTask, (const portCHAR *)"fanTask",
                          tskStkSize, NULL, osPriorityAboveNormal, (TaskHandle_t *)&fanTask_Handler);
        configASSERT(rtn == pdPASS);
    }
}