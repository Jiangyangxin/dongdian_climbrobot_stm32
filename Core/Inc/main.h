/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stdarg.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  typedef enum
  {
    idle,
    steeringWheel,
    activeAdsorption,
    normalAdsorption,
    ioState,
    steeringCurrent,
  } BoardType;
  typedef struct
  {
    uint32_t state;     // 0:stop 1:start 2:reset
    float dr1_tar_vel;  // -0.3~0.3, m/s
    float dr2_tar_pos;  // -pi~pi , rad
    float dr1_tar_cur;  // -19~19 , A
    uint64_t timestamp; // us
  } SteerCmd;

  typedef struct
  {
    uint32_t state;     // 0:stop 1:start
    float dr1_tar_vel;  // -0.3~0.3, m/s
    float dr1_real_vel; //
    float dr1_tar_cur;  // -19~19 , A
    float dr1_real_cur; //
    float dr2_tar_pos;  // -pi~pi , rad
    float dr2_real_pos; // -pi~pi , rad
    float dr2_tar_vel;  // -pi~pi , rad/s
    float dr2_real_vel; //
    float dr2_tar_cur;  // -9.5~9.5 , A
    float dr2_real_cur; //
    uint64_t timestamp; // us
  } SteerVal;
  typedef struct
  {
    // dr1_cur  -19~19      -> 0~38000   31-16 16bit
    // dr2_pos  -pi~pi      -> 0~62800   15- 0 16bit
    // 
    uint32_t dr2_pos:16;
    uint32_t dr1_cur:16;
    // state                             31-28 4bit
    // dr1_vel -0.3~0.3     -> 0~60000   15- 0 16bit
    uint32_t dr1_vel:16;
    uint32_t resv   :12;
    uint32_t state  :4;
    uint64_t timestamp; // us
  } SteerCurInfo;
  typedef struct
  {
    uint32_t state;     // 0:stop 1:start
    float fan_tar_pre;  // 0-10, kpa
    uint64_t timestamp; // us
  } FanCmd;
  typedef struct
  {
    uint32_t state;     // 0:stop 1:start
    float fan_tar_pre;  // 0-10, kpa
    float fan_real_pre; // 0-10, kpa
    float fan_pwm;      // 0-100, %
    float fan_speed;    // rps
    uint64_t timestamp; // us
  } FanVal;
  typedef struct
  {
    uint32_t state;                   // 0:stop 1:start 2:reset 3:manual
    float sensor_tar_displacement[3]; // 0~20, mm; target displacement  of the sensor
    float plate_tar_pose[3];          // target x angle，y angle，z pos
    uint64_t timestamp;               // us
  } AdsorptionCmd;
  typedef struct
  {
    uint32_t state;                    // 0:stop 1:start 2:reset 3:manual
    float sensor_tar_displacement[3];  // 0~20, mm; target displacement  of sensor
    float sensor_real_displacement[3]; // 0~20, mm; real displacement  of sensor
    float motor_real_displacement[3];  // real pos of motor (angle convert to pos)
    float plate_tar_pose[3];           // target x angle，y angle，z displacement
    float plate_real_pose[3];          // real x angle，y angle，z displacement
    uint64_t timestamp;                // us
  } AdsorptionVal;
  typedef struct
  {
    uint32_t state;     //
    uint64_t timestamp; // us
  } IOCmd;
  typedef struct
  {
    uint32_t state;     //
    uint64_t timestamp; // us
  } IOVal;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_13
#define KEY0_GPIO_Port GPIOC
#define MOVE0_Pin GPIO_PIN_0
#define MOVE0_GPIO_Port GPIOC
#define MOVE1_Pin GPIO_PIN_1
#define MOVE1_GPIO_Port GPIOC
#define MOVE2_Pin GPIO_PIN_2
#define MOVE2_GPIO_Port GPIOC
#define SensePressure_Pin GPIO_PIN_1
#define SensePressure_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOA
#define LaserSensor0_Pin GPIO_PIN_3
#define LaserSensor0_GPIO_Port GPIOA
#define W5500_CS_Pin GPIO_PIN_4
#define W5500_CS_GPIO_Port GPIOA
#define W5500_INT_Pin GPIO_PIN_4
#define W5500_INT_GPIO_Port GPIOC
#define W5500_RST_Pin GPIO_PIN_5
#define W5500_RST_GPIO_Port GPIOC
#define FAN_Mea_Pin GPIO_PIN_1
#define FAN_Mea_GPIO_Port GPIOB
#define FAN_EN_Pin GPIO_PIN_2
#define FAN_EN_GPIO_Port GPIOB
#define ZERO1_Pin GPIO_PIN_6
#define ZERO1_GPIO_Port GPIOC
#define ZERO2_Pin GPIO_PIN_7
#define ZERO2_GPIO_Port GPIOC
#define ZERO3_Pin GPIO_PIN_8
#define ZERO3_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
  extern ADC_HandleTypeDef hadc1, hadc2;
  extern DMA_HandleTypeDef hdma_adc1;
  extern TIM_HandleTypeDef htim1;
  extern TIM_HandleTypeDef htim3;
  extern I2C_HandleTypeDef hi2c2;
  extern SPI_HandleTypeDef hspi1;
  extern DMA_HandleTypeDef hdma_spi1_rx;
  extern DMA_HandleTypeDef hdma_spi1_tx;
  extern UART_HandleTypeDef huart1;
  extern CAN_HandleTypeDef hcan1;
  extern SemaphoreHandle_t ethDealTickSem, ethTxTickSem;
  extern SemaphoreHandle_t motionTickSem, fanTickSem;
  extern QueueHandle_t can1RxQueueHandle;
  extern QueueHandle_t dbgQueue;
  extern uint8_t ethPeriod; // ms



  extern void eprint(char *str);
//#define _DEBUG
#define CMD_TIMEOUT 5 * 1000 // 5s
#define TASK_LOG_NUM 400

#define KEY_0_Pin GPIO_PIN_0
#define KEY_1_Pin GPIO_PIN_1
#define KEY_GPIO_Port GPIOC
#define LED_0_Pin GPIO_PIN_2
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_3
#define LED_1_GPIO_Port GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
