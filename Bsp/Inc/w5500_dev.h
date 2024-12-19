#ifndef _W5500_DEV_H_
#define _W5500_DEV_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "socket.h"
#include "stm32f4xx_hal.h"
#define _SPI_DMA_
  enum Status
  {
    Failed = 0,
    Success = 1
  };
  void W5500_write_config(wiz_NetInfo *_set);
  void W5500_get_config(wiz_NetInfo *_get);
  uint8_t W5500_init(wiz_NetInfo *gSetNetInfo);
  extern wiz_NetInfo defaultInfo;
#ifdef __cplusplus
}
#endif

#endif
