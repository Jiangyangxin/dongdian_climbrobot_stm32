#include "at24cxx.h"
#include "main.h"

/* For watchdog timer refresh, required during erase operation */
#if (_EEPROM_USE_IWDG == 1)
#define		_EEPROM_IWDG			hiwdg
#endif

#if (_EEPROM_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define at24_delay(x)   osDelay(x)
#else
#define at24_delay(x)   HAL_Delay(x)
#endif

#if (_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2)
#define _EEPROM_PSIZE     8
#elif (_EEPROM_SIZE_KBIT == 4) || (_EEPROM_SIZE_KBIT == 8) || (_EEPROM_SIZE_KBIT == 16)
#define _EEPROM_PSIZE     16
#else
#define _EEPROM_PSIZE     32
#endif

static uint8_t at24_lock = 0;

I2C_HandleTypeDef _EEPROM_I2C;
/**
  * @brief  Checks if memory device is ready for communication.
  * @param  none
  * @retval bool status
  */
bool at24_isConnected(void)
{
	_EEPROM_I2C = hi2c2;
  #if (_EEPROM_USE_WP_PIN==1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_SET);
  #endif
  if (HAL_I2C_IsDeviceReady(&_EEPROM_I2C, _EEPROM_ADDRESS, 2, 100) == HAL_OK)
    return true;
  else
    return false;	
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration
  * @retval bool status
  */
bool at24_write(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  if (at24_lock == 1)
    return false;
	
  at24_lock = 1; 
  uint16_t w;
  uint32_t startTime = HAL_GetTick();
	
  #if	(_EEPROM_USE_WP_PIN==1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN,GPIO_PIN_RESET);
  #endif
	
  while (1)
  {
    #if (_EEPROM_USE_IWDG == 1)
		HAL_IWDG_Refresh(&_EEPROM_IWDG);
		#endif
    w = _EEPROM_PSIZE - (address  % _EEPROM_PSIZE);
    if (w > len)
      w = len;        
    #if ((_EEPROM_SIZE_KBIT==1) || (_EEPROM_SIZE_KBIT==2))
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)//HAL库中用于执行I2C内存写操作的一个函数
    //MCU先发送一个开始信号(START)启动总线
    //接着跟上首字节，发送器件写操作地址(DEVICE ADDRESS)+写数据(0xA0)
    //等待应答信号(ACK)
    //发送数据的存储地址。24C02一共有256个字节的存储空间，地址从0x00~0xFF，想把数据存储在哪个位置，此刻写的就是哪个地址。
    //发送要存储的数据第一字节、第二字节、…注意在写数据的过程中，E2PROM每个字节都会回应一个“应答位0”，老告诉我们写E2PROM数据成功，如果没有回应答位，说明写入不成功。
    //发送结束信号（STOP）停止总线

    #elif (_EEPROM_SIZE_KBIT==4)
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==8)
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==16)
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)		
    #else
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, w, 100) == HAL_OK)
    #endif
    {
      at24_delay(10);//AT24C02的IIC每次写之后要延时一段时间才能继续写 每次写之后要delay 5ms左右 不管硬件IIC采用何种形式（DMA，IT），都要确保两次写入的间隔大于5ms;
      len -= w;
      data += w;
      address += w;
      if (len == 0)
      {
        #if (_EEPROM_USE_WP_PIN==1)
        HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
        #endif
        at24_lock = 0;
        return true;
      }
      if (HAL_GetTick() - startTime >= timeout) 
      {
        at24_lock = 0;
        return false;
      }
    }
    else
    {
      #if (_EEPROM_USE_WP_PIN==1)
      HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
      #endif
      at24_lock = 0;
      return false;
    }
  }
}

/**
  * @brief  Read an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration
  * @retval bool status
  */
bool at24_read(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  #if (_EEPROM_USE_IWDG == 1)
		HAL_IWDG_Refresh(&_EEPROM_IWDG);
	#endif
  if (at24_lock == 1)
    return false;
  at24_lock = 1;
  #if (_EEPROM_USE_WP_PIN==1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
  #endif
  #if ((_EEPROM_SIZE_KBIT==1) || (_EEPROM_SIZE_KBIT==2))
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 4)
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 8)
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT==16)
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #else
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, len, timeout) == HAL_OK)
  #endif
  {
    at24_lock = 0;
    return true;
  }
  else
  {
    at24_lock = 0;
    return false;	
  }    
}

/**
  * @brief  Erase memory.
  * @param  none
  * @retval bool status
  */
bool at24_eraseChip(void)
{
  const uint8_t eraseData[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF\
    , 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint32_t bytes = 0;
  while ( bytes < (_EEPROM_SIZE_KBIT * 128))
  {
    if (at24_write(bytes, (uint8_t*)eraseData, sizeof(eraseData), 100) == false)
      return false;
    bytes += sizeof(eraseData);           
  }
  return true;  
}

/* EOF */
