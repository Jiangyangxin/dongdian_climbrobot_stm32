/**
 ******************************************************************************
 * File Name          : CAN.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
  /*
   * Each control board has three motors at most.
   * Adsorption:
   *        M2006 1
   *        M2006 2
   *        M2006 3
   * Steering wheel:
   *        M3508 1
   *        M2006 2
   */
  typedef enum
  {
    CAN_Moto_ALL_ID = 0x200,
    CAN_Moto1_ID = 0x201,
    CAN_Moto2_ID = 0x202,
    CAN_Moto3_ID = 0x203,
  } CAN_Message_ID;

#define FILTER_BUF_LEN 5

  typedef struct
  {
    // feedback data
    uint16_t angle;    // ° [0,8191] -> [0, 360°]
    int16_t speed_rpm; // rpm
    int16_t given_current;
    uint8_t temp;
    uint8_t id;
    // record data
    uint16_t last_angle; // abs angle range:[0,8191]
    uint16_t offset_angle;
    int32_t round_cnt;
    int32_t total_angle;
    uint32_t msg_cnt;
  } moto_measure_t;

  extern QueueHandle_t reset_flag;
  void CAN_Start_Trans(void);
  void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
  void CAN_SendMsg(uint16_t id, uint8_t TxData[]);
  void reset_total_angle(moto_measure_t *ptr);

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */
