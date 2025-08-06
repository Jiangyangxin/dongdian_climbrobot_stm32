#include "w5500_dev.h"
#include "socket.h"
#include "at24cxx.h"
#include "main.h"
#include "cmd_tsk.h"

#if (_EEPROM_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define w5500_delay(x) osDelay(x)
#else
#define w5500_delay(x) HAL_Delay(x)
#endif

wiz_NetInfo defaultInfo = {
    .mac = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00},
    .ip = {192, 168, 0, 199},
    .sn = {255, 255, 255, 0},
    .gw = {192, 168, 0, 1},
    .dns = {144, 144, 144, 144},
    .dhcp = NETINFO_STATIC,
    .type = 4,
    .period = 10};

void W5500_Select(void)
{
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void)
{
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_Restart(void)
{
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  w5500_delay(5); // delay 1ms
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  w5500_delay(1600); // delay 1600ms
}

/* SPI */
uint8_t spiReadByte(void)
{
  uint8_t readByte = 0;
  uint8_t writeByte = 0xFF;

  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    w5500_delay(1);
  HAL_SPI_TransmitReceive(&hspi1, &writeByte, &readByte, 1, 10);

  return readByte;
}

void spiWriteByte(uint8_t writeByte)
{
  uint8_t readByte = 0;

  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    w5500_delay(1);
  HAL_SPI_TransmitReceive(&hspi1, &writeByte, &readByte, 1, 10);
}

#ifdef _SPI_DMA_
int counter_i=0;
/* SPI DMA */
uint8_t spiDmaReadByte(void)
{
     uint8_t rxBuf[20];
  uint8_t readByte = 0;
  uint8_t writeByte = 0xFF;

  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY &&
         HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY)
    w5500_delay(1);

  HAL_SPI_Receive_DMA(&hspi1, &readByte, 1);

  while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_RESET)
    ;
  while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_RESET)
    ;
  return readByte;
}


void spiDmaWriteByte(uint8_t writeByte)
{
    uint8_t rxBuf[20];
  uint8_t readByte = 0;

  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY &&
         HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY)
    w5500_delay(1);

  HAL_SPI_Transmit_DMA(&hspi1, &writeByte, 1);

  while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_RESET)
    ;
  while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_RESET)
    ;
  
  return;
}

void spiReadBurst(uint8_t *pBuf, uint16_t len)
{
     uint8_t rxBuf[20];
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY &&
         HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY)
    w5500_delay(1);

  HAL_SPI_Receive_DMA(&hspi1, pBuf, len);

  while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_RESET)
    ;
	
  while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_RESET)
    ;
  return;
}

void spiWriteBurst(uint8_t *pBuf, uint16_t len)
{
//    if(len >=512)len = 0;
  uint8_t rxBuf[20];
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY &&
         HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY)
    w5500_delay(1);
  
  HAL_SPI_Transmit_DMA(&hspi1, pBuf, len);

  while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_RESET)
    ;
  while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY || HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_RESET)
   ;
  

//  
//  for(counter_i=0;counter_i<500;counter_i++)
//  {
//    if(HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_BUSY && HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_RESET)
//        break;
//    else if(counter_i>=499)
//    {
//        hspi1.State = HAL_SPI_STATE_READY;
//        hdma_spi1_tx.State = HAL_DMA_STATE_READY;
//        __HAL_UNLOCK(&hdma_spi1_tx);
//        	sprintf((char *)rxBuf, "i = %d\r\n",counter_i);
//                    print((char *)rxBuf);
//    }
//    w5500_delay(1);
//  }
//	

	
  return;
}
#endif /* _SPI_DMA_ */

void W5500_write_config(wiz_NetInfo *_set)
{
  if (at24_isConnected())
  {
    at24_write(0, (uint8_t *)_set, sizeof(wiz_NetInfo), 100);
  }
}

void W5500_get_config(wiz_NetInfo *_get)
{
  uint32_t cpuId[3];
  uint32_t macCode;
//  cpuId[0] = *(uint32_t *)(0x1ffff7e8);
//  cpuId[1] = *(uint32_t *)(0x1ffff7ec);
//  cpuId[2] = *(uint32_t *)(0x1ffff7f0);
	cpuId[0] = *(uint32_t *)(0x1fff7a10);
  cpuId[1] = *(uint32_t *)(0x1fff7a14);
  cpuId[2] = *(uint32_t *)(0x1fff7a18);

  defaultInfo.mac[2] = (macCode & 0x000000FF);
  defaultInfo.mac[3] = (macCode & 0x0000FF00) >> 8;
  defaultInfo.mac[4] = (macCode & 0x00FF0000) >> 16;
  defaultInfo.mac[5] = (macCode & 0xFF000000) >> 24;
	
  if (at24_isConnected())
  {
    at24_read(0, (uint8_t *)_get, sizeof(wiz_NetInfo), 100);
  }

  if ((_get->mac[0] == 0xff) && (_get->mac[1] == 0xff) && (_get->mac[2] == 0xff) && (_get->mac[3] == 0xff) && (_get->mac[4] == 0xff) && (_get->mac[5] == 0xff))
  {
    memcpy(_get, &defaultInfo, sizeof(wiz_NetInfo));
  }
}

/**
 * @brief valid the result of set net info
 * @return 1: Success
 *         0: Failed
 */
uint8_t validSetNetInfoResult(wiz_NetInfo *_set, wiz_NetInfo *_get)
{
#if 0
  char buf[64];
  sprintf(buf, "get IP: %d %d %d %d\r\n", _get->ip[0], _get->ip[1], _get->ip[2], _get->ip[3]);
  print(buf);
  sprintf(buf, "valid set memcmp %d\r\n", memcmp(_set, _get, sizeof(wiz_NetInfo) - 2));
  print(buf);
#endif
  return (!memcmp(_set, _get, sizeof(wiz_NetInfo) - 2)); // if same, memcmp return 0
}

uint8_t W5500_init(wiz_NetInfo *gSetNetInfo)
{
  wiz_NetInfo gGetNetInfo;
  wiz_NetTimeout wiz_nettimeout;

  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
  reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte);
#ifdef _SPI_DMA_
  reg_wizchip_spiburst_cbfunc(spiReadBurst, spiWriteBurst);
#endif

  W5500_Restart(); // hardware restart through RESET pin

  ctlnetwork(CN_SET_NETINFO, (void *)gSetNetInfo); // set net info
  // maybe need delay
  ctlnetwork(CN_GET_NETINFO, (void *)&gGetNetInfo); // get net info

  wiz_nettimeout.retry_cnt = 5;
  wiz_nettimeout.time_100us = 1000; // 100ms
  ctlnetwork(CN_SET_TIMEOUT, (void *)&wiz_nettimeout);

  if (Success == validSetNetInfoResult(gSetNetInfo, &gGetNetInfo)) // compare
  {
#if 0
    print("Net info set success!\n");
#endif
    return Success;
  }
  else
  {
#ifdef _DEBUG
    print("Net info set failed!\n");
#endif
    // do something
    return Failed;
  }

  // W5500 has 8 channel, 32k buffer, 2 means 2KBytes
  uint8_t buffer_size_8channel_tx_rx[16] = {2, 2, 2, 2, 2, 2, 2, 2,  // 8 channel tx
                                            2, 2, 2, 2, 2, 2, 2, 2}; // 8 channel rx
  if (ctlwizchip(CW_INIT_WIZCHIP, (void *)buffer_size_8channel_tx_rx))
  {
    // failed
    // UART_Printf("buffer size set failed!\n");
    return Failed;
  }
}