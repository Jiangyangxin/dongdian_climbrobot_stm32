/*
  ******************************************************************************
  * File Name          : at24cxx.h
  * Description        : this code is used for at24cxx application
  ******************************************************************************
*/
 
#ifndef __AT24CXX_H
#define __AT24CXX_H

/********************************* Configuration Start *********************************/
#define		_EEPROM_SIZE_KBIT		    2       /* 2K (256 x 8) */
#define		_EEPROM_USE_FREERTOS		1
#define		_EEPROM_ADDRESS			    0xA0
#define		_EEPROM_USE_WP_PIN		  0
#define		_EEPROM_USE_IWDG		    0

/* The AT24CXX has a hardware data protection scheme that allows the
user to write protect the whole memory when the WP pin is at VCC. */

#if (_EEPROM_USE_WP_PIN == 1)
#define		_EEPROM_WP_GPIO			WP_GPIO_Port
#define		_EEPROM_WP_PIN			WP_Pin
#endif

/********************************* Configuration End *********************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
  * @brief  Checks if memory device is ready for communication.
  * @param  none
  * @retval bool
  */
bool at24_isConnected(void);

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration
  * @retval bool status
  */
bool at24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);

/**
  * @brief  Read an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration
  * @retval bool status
  */
bool at24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);

/**
  * @brief  Erase memory.
  * @note   This requires time in seconds
  * @param  none
  * @retval bool status
  */
bool at24_eraseChip(void);

#ifdef __cplusplus
}
#endif

#endif

/* EOF */
