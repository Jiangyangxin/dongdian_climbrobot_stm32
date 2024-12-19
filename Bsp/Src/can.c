/**
 ******************************************************************************
 * File Name          : CAN.c
 * Description        : This file provides code for the configuration
 *                      of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"

// 交互复位信息
QueueHandle_t reset_flag;
moto_measure_t moto_chassis[3] = {0};

void get_moto_measure(moto_measure_t *ptr, uint8_t *data)
{
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(data[0] << 8 | data[1]);
    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temp = data[6];

    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;
    ptr->total_angle = (ptr->round_cnt << 13) + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, uint8_t *data)
{
    ptr->angle = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_angle = ptr->angle;
}

/*
 * when working as Steering wheel, use it to clear 2006 angle when reset.
 */
void reset_total_angle(moto_measure_t *ptr)
{
    ptr->offset_angle = ptr->angle;
    ptr->total_angle = 0;
    ptr->round_cnt = 0;
}

void CAN_Start_Trans(void)
{
    reset_flag = xQueueCreate(3, 1);
    // 初始化CAN
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;
    /*********************can1*********************/
    uint16_t StdIdArray1[3] = {CAN_Moto1_ID, CAN_Moto2_ID, CAN_Moto3_ID};
    uint16_t mask, num, tmp, i;

    // 下面开始计算屏蔽码
    mask = 0x7ff; // 下面开始计算屏蔽码
    num = sizeof(StdIdArray1) / sizeof(StdIdArray1[0]);
    for (i = 0; i < num; i++) // 屏蔽码位StdIdArray[]数组中所有成员的同或结果
    {
        tmp = StdIdArray1[i] ^ (~StdIdArray1[0]); // 所有数组成员与第0个成员进行同或操作
        mask &= tmp;
    }

    sFilterConfig.FilterIdHigh = ((StdIdArray1[0]) << 5); // 验证码可以设置为StdIdArray[]数组中任意一个，这里使用StdIdArray[0]作为验证码
    //    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    //    sFilterConfig.FilterMaskIdHigh = 0;
    //    sFilterConfig.FilterMaskIdLow = 0; 		//只接收数据帧
    sFilterConfig.FilterMaskIdHigh = (mask << 5);
    sFilterConfig.FilterMaskIdLow = 0 | 0x02;              // 只接收数据帧
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1; // 设置通过的数据帧进入到FIFO0中
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 配置为掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 设置为32位宽
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_Status = HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

    HAL_Status = HAL_CAN_Start(&hcan1);
    if (HAL_Status != HAL_OK)
    {
        Error_Handler();
    }
    // 使能中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING); // 使能中断
}

void CAN_SendMsg(uint16_t id, uint8_t TxData[])
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 8;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t aRxData[8], i, flag;
    CAN_RxHeaderTypeDef hCAN1_RxHeader;
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &hCAN1_RxHeader, aRxData);
        switch (hCAN1_RxHeader.StdId)
        {
        case CAN_Moto1_ID:
        case CAN_Moto2_ID:
        case CAN_Moto3_ID:
        {
            i = hCAN1_RxHeader.StdId - CAN_Moto1_ID;
            moto_chassis[i].id = i;
            moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], aRxData) : get_moto_measure(&moto_chassis[i], aRxData);
            // 接收到触发信息，M2006角度复位
            if (pdTRUE == xQueueReceiveFromISR(reset_flag, (void *)&flag, 0))
            {
                reset_total_angle(&moto_chassis[flag]);
            }
            BaseType_t pxHigherPriorityTaskWoken;
            xQueueSendFromISR(can1RxQueueHandle, &moto_chassis[i], &pxHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
        }
        break;
        }
    }
}
