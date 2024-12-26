#include "cmd_tsk.h"

uint8_t rx_data[UART_RX_MAX_LEN];
uint8_t printf1_data[PRINTF1_LEN];
char printStr[PRINTF1_LEN];
void print(char *dbgStr)
{
    bool rtn;
    rtn = xQueueSend(TskPrint::MbCmd, dbgStr, portMAX_DELAY);
    configASSERT(rtn == pdPASS);
}

void Usart1Receive_IDLE(UART_HandleTypeDef *huart)
{
    BaseType_t pxHigherPriorityTaskWoken;
    if (huart->Instance == USART1)
    {
        if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            if (pdFALSE == xQueueSendFromISR(TskPrint::uartDMAQueue, rx_data, &pxHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
            }
            HAL_UART_AbortReceive(huart);
            portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
            memset(rx_data, 0x00, UART_RX_MAX_LEN);
            HAL_UART_Receive_DMA(huart, rx_data, UART_RX_MAX_LEN);
        }
    }
}

namespace TskPrint
{
    const int tskPrio = osPriorityLow3;
    const int tskStkSize = 512;

    QueueHandle_t MbCmd;
    QueueHandle_t uartDMAQueue;
    char dbgStr[PRINTF1_LEN];

    void printf1(char *fmt, ...)
    {
        uint16_t i;
        va_list ap;
        va_start(ap, fmt);
        i = vsnprintf((char *)printf1_data, PRINTF1_LEN, fmt, ap);
        if (i == 0)
        {
            return;
        }
        HAL_UART_Transmit(&huart1, printf1_data, i, 1000);
        va_end(ap);
    }

    void uartTask(void *pvParameters)
    {
        BaseType_t rtn;

        while (true)
        {
            rtn = xQueueReceive(MbCmd, printStr, portMAX_DELAY);
            configASSERT(rtn == pdPASS);
            printf1(printStr);
                
        }
    }

    void Init()
    {
        BaseType_t rtn;

        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);             // 使能idle中断
        HAL_UART_Receive_DMA(&huart1, rx_data, UART_RX_MAX_LEN); // 打开DMA接收

        MbCmd = xQueueCreate(10, PRINTF1_LEN);
        configASSERT(MbCmd);
        uartDMAQueue = xQueueCreate(2, UART_RX_MAX_LEN);
        configASSERT(uartDMAQueue);

        // Create tasks
        rtn = xTaskCreate(uartTask, (const portCHAR *)"PrintTask",
                          tskStkSize, NULL, tskPrio, NULL);
        configASSERT(rtn == pdPASS);
    }
}
