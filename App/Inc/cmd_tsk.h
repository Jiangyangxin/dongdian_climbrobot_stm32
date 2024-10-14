/*
 * DbgUart.h
 *
 *  Created on: Aug 14, 2016
 *      Author: loywong
 */

#ifndef _CMDTSK_H_
#define _CMDTSK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    void Usart1Receive_IDLE(UART_HandleTypeDef *huart);
    void print(char *dbgStr);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define UART_RX_MAX_LEN 64
#define PRINTF1_LEN 64

namespace TskPrint
{
    extern QueueHandle_t MbCmd;
    extern QueueHandle_t uartDMAQueue;
    extern char dbgStr[PRINTF1_LEN];

    void Init();
}
#endif

#endif /* _CMDTSK_H_ */
