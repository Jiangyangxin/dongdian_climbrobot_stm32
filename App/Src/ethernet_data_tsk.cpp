#include "ethernet_tsk.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500_dev.h"
#include "physparams.h"
#include "cmd_tsk.h"

uint8_t rxBuf[RECV_BUF_SIZE];
uint8_t txBuf[SEND_BUF_SIZE];
uint8_t rxDealBuf[RECV_BUF_SIZE];
uint8_t txDealBuf[SEND_BUF_SIZE];

volatile TSRecord sysTs;
SemaphoreHandle_t xMutex;




uint16_t pack_struct_short(void *p, const char name[][13], const char *type, uint8_t len, uint16_t prefix)
{
    memset(txDealBuf + prefix, 0, SEND_BUF_SIZE - prefix); 
    for (int i = 0; i < len; i++)
    {
        volatile char type_idx = type[i];
        if (type_idx == 1)
        {
//            sprintf((char *)txDealBuf + prefix, "%s:", name[i]);
//            prefix += strlen(name[i])+1;
            memcpy(txDealBuf+prefix, (void *)p, 4);
            txDealBuf[prefix+4] = ' ';
            prefix += 5;
        }
        else if (type_idx == 2)
        {
//            sprintf((char *)txDealBuf + prefix, "%s:", name[i]);
//            prefix += strlen(name[i])+1;
            memcpy(txDealBuf+prefix, (void *)p, 4);
            txDealBuf[prefix+4] = ' ';
            prefix += 5;
        }        
        else if (type_idx == 3)
        {
            uint64_t ts = sysTs.ethTs + (xTaskGetTickCount() - sysTs.localTs) * 1000;
//            sprintf((char *)txDealBuf + prefix, "%s:", name[i]);
//            prefix += strlen(name[i])+1;
            memcpy(txDealBuf+prefix, (void *)&ts, 8);
            txDealBuf[prefix+8] = ' ';
            prefix += 9;
        }
        p = ((char *)p + (type[i] == 3 ? 8 : 4));
    }
    txDealBuf[prefix] = '\r';
    txDealBuf[prefix + 1] = '\n';
    prefix += 2;
    return prefix;
}

void pack_struct(void *p, const char name[][13], const char *type, uint8_t len, uint16_t prefix)
{
	  char var_name[13];
    memset(txDealBuf + prefix, 0, SEND_BUF_SIZE - prefix);
	  memset(var_name, 0, 13);
    for (int i = 0; i < len; i++)
    {
        prefix = strlen((char *)txDealBuf);
			  if (name == NULL)
				{
					var_name[0] = (char)((i%10)+'0');
					var_name[1] = i / 10 ? (char)((i/10)+'0') : 0;
				}
				else
				{
					memcpy(var_name, name[i], 13);
				}
        switch (type[i])
        {
        case 1:
            sprintf((char *)txDealBuf + prefix, "%s:%u ", var_name, *((uint32_t *)p));
            break;
        case 2:
            sprintf((char *)txDealBuf + prefix, "%s:%.2f ", var_name, *((float *)p));
            break;
        case 3:
            uint64_t ts = sysTs.ethTs + (xTaskGetTickCount() - sysTs.localTs) * 1000;
            sprintf((char *)txDealBuf + prefix, "%s:%llu ", var_name, ts);
            break;
        }
        p = ((char *)p + (type[i] == 3 ? 8 : 4));
    }
    prefix = strlen((char *)txDealBuf);
    txDealBuf[prefix] = '\r';
    txDealBuf[prefix + 1] = '\n';
    return;
}

void unpack_to_struct(char *frame, void **p, const char name[][13], const char *type, const uint8_t len)
{
    const char *d = " ";
    bool exit = false;
    char *val, *cmd;
    uint16_t prefix = 0;
    char *substr = strstr((char *)frame, name[0]);
    while (!exit)
    {
        cmd = strtok((char *)substr, d);
        // ts is uint64_t, its length is 16, so MUST give >= 16;
        char cmd_s[2][17];
        while (cmd != NULL)
        {
            sscanf(cmd, "%[^:]:%s", cmd_s[0], cmd_s[1]);
            prefix = 0;
            for (int i = 0; i < len; i++)
            {
                val = ((char *)*p + prefix);
                prefix += (type[i] == 3 ? 8 : 4);
                if (!strcmp(cmd_s[0], name[i]))
                {
                    switch (type[i])
                    {
                    case 1:
                        sscanf(cmd_s[1], "%u", (uint32_t *)val);
                        break;
                    case 2:
                        sscanf(cmd_s[1], "%f", (float *)val);
                        break;
                    case 3:
                        sscanf(cmd_s[1], "%llu", (uint64_t *)val);
                        sysTs.ethTs = *(uint64_t *)val;
                        sysTs.localTs = xTaskGetTickCount();
                        break;
                    }
                    break;
                }
            }
            cmd = strtok(NULL, d);
        }
        exit = true;
    }
}
extern  wiz_NetInfo  cmdEthInfo;
namespace TskEth
{
    const int tskStkSize = 512;//512
    uint8_t type = BoardType::idle;
    SteerCmd *steerCmd = nullptr;
    SteerVal *steerVal = nullptr;
    SteerCurInfo *steerCurCmd = nullptr;
    SteerCurInfo *steerCurVal = nullptr;
    FanCmd *fanCmd = nullptr;
    FanVal *fanVal = nullptr;
    AdsorptionCmd *adsorptionCmd = nullptr;
    AdsorptionVal *adsorptionVal = nullptr;
    IOCmd *ioCmd = nullptr;
    IOVal *ioVal = nullptr;
    uint8_t io_pre_state = 0;
    uint8_t io_cur_state = 0;

    
    
    QueueHandle_t steerCmdQueue;
    QueueHandle_t steerValQueue;
    QueueHandle_t fanCmdQueue;
    QueueHandle_t fanValQueue;
    QueueHandle_t adsorptionCmdQueue;
    QueueHandle_t adsorptionValQueue;
    QueueHandle_t rawDataQueue;
    QueueHandle_t sendDataQueue;
		

		

    void ethTransRecvTask(void *pvParameters)
    {
        BaseType_t rtn;

        int32_t ret;
			
			int32_t flag_ret =0;
        uint8_t sn = DATA_SN;
        uint16_t size = 0, sentsize = 0;
#ifdef _DEBUG
        uint8_t destip[4];
        uint16_t destport;
#endif
        while (true)
        {
            rtn = xSemaphoreTake(ethTxTickSem, ethPeriod + 1);
            configASSERT(rtn);
            
            
			flag_ret = getSn_SR(sn);
			sprintf((char *)rxBuf, "flag_ret = %d\r\n",flag_ret);
            print((char *)rxBuf);
            
            
            switch (getSn_SR(sn))
            { 
            case SOCK_ESTABLISHED:
                if (getSn_IR(sn) & Sn_IR_CON)
                {
#ifdef _DEBUG
                    getSn_DIPR(sn, destip);
                    destport = getSn_DPORT(sn);
                    sprintf((char *)rxBuf, "Connected - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
                    print((char *)rxBuf);
#endif
                    setSn_IR(sn, Sn_IR_CON);
                    
                    
                }
                memset(txBuf, 0, LEN_IDX+1);
                // 收、发任一操作进行时都需占用spi双向通信，因此不能进行全双工通信，因此将收发放在一起，另开线程进行解包
                if (pdPASS == xQueueReceive(sendDataQueue, txBuf, 0))
                {
                    size = strlen((char *)txBuf);
                    // size = txBuf[LEN_IDX];
                    ret = send(sn, txBuf, size);                   
                }
                if ((size = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
                {
                    memset(rxBuf, 0, RECV_BUF_SIZE);
                    if (size > RECV_BUF_SIZE)
                        size = RECV_BUF_SIZE;
                    ret = recv(sn, rxBuf, size);
                    xQueueSend(rawDataQueue, rxBuf, 0);
                }
                break;
            case SOCK_CLOSE_WAIT:
                ret = disconnect(sn);
                if (ret == SOCK_OK)
                {
#ifdef _DEBUG
                    print((char *)("Socket Closed\r\n"));
#endif
                }
                break;
            case SOCK_INIT:
                ret = listen(sn);
                break;
            case SOCK_CLOSED:
                ret = socket(sn, Sn_MR_TCP, ETH_DATA_PORT, 0x00);
                setsockopt(sn, SO_KEEPALIVEAUTO, (void *)0);/////0
                break;

            default:

                break;
            }
        }
    }

    void ethDealTask(void *pvParameters)
    {
        BaseType_t rtn;
        int32_t ret;
        uint16_t size = 0;
        uint8_t sn = 0;

        while (true)
        {
            rtn = xSemaphoreTake(ethDealTickSem, ethPeriod + 1);
            configASSERT(rtn);

            // unpack control data
            if (pdPASS == xQueueReceive(rawDataQueue, rxDealBuf, 0))
            {
                if (type == BoardType::ioState)
                {
                    void *p = ioCmd;
                    unpack_to_struct((char *)rxDealBuf, &p, IOCmdName, (const char *)IOCmdTypeRecord, IOCmdMemberNum);
                    // IO write 0 亮 1 灭
                    HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, (GPIO_PinState)(ioCmd->state & 0x1));
                    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (GPIO_PinState)((ioCmd->state & 0x2) >> 1));
                }
                else if (type == BoardType::steeringCurrent)
                {
                    void *p = steerCurCmd;
                    unpack_to_struct((char *)rxDealBuf, &p, SteerCurName, (const char *)SteerCurTypeRecord, SteerCurMemberNum);
                    steerCmd->state = steerCurCmd->state;
                    steerCmd->dr1_tar_cur = (steerCurCmd->dr1_cur * current_resolution) - maxCurrent;
                    steerCmd->dr1_tar_vel = (steerCurCmd->dr1_vel * motion_resolution) - maxVel;
                    steerCmd->dr2_tar_pos = (steerCurCmd->dr2_pos * motion_resolution) - PI;
                    if (pdFAIL == xQueueOverwrite(steerCmdQueue, steerCmd))
                    {
                        print((char *)("steerCmd send error\r\n"));
                    }                    
                }
                else if (type > BoardType::steeringWheel)
                {
                    if (adsorptionCmd != nullptr)
                    {
                        memset(txDealBuf, 0, RECV_BUF_SIZE);
                        memcpy(txDealBuf, rxDealBuf, strlen((char *)rxDealBuf));
                        void *p = adsorptionCmd;
                        unpack_to_struct((char *)txDealBuf, &p, AdsorptionCmdName, (const char *)AdsorptionCmdTypeRecord, AdsorptionCmdMemberNum);
                        if (pdFAIL == xQueueOverwrite(adsorptionCmdQueue, adsorptionCmd))
                        {
                            print((char *)("adsorptionCmd send error\r\n"));
                        }
                    }
                    void *p = fanCmd;
                    unpack_to_struct((char *)rxDealBuf, &p, FanCmdName, (const char *)FanCmdTypeRecord, FanCmdMemberNum);
                    if (pdFAIL == xQueueOverwrite(fanCmdQueue, fanCmd))
                    {
                        print((char *)("fanCmd send error\r\n"));
                    }
                }
                else if (type == BoardType::steeringWheel)
                {
                    void *p = steerCmd;
                    unpack_to_struct((char *)rxDealBuf, &p, SteerCmdName, (const char *)SteerCmdTypeRecord, SteerCmdMemberNum);
                    if (pdFAIL == xQueueOverwrite(steerCmdQueue, steerCmd))
                    {
                        print((char *)("steerCmd send error\r\n"));
                    }
                }
            }

            uint16_t cur_prefix = 0;
            // pack data to translation
            memset(txDealBuf, 0, SEND_BUF_SIZE);
            {
                if (type == BoardType::ioState)
                {
                    // IO read 0 按下 1 松开
                    io_cur_state = (io_cur_state & 0xFE) | HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_0_Pin);
                    io_cur_state = (io_cur_state & 0xFD) | (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_1_Pin) << 1);

                    // 利用10ms周期 状态比较 去抖
                    if (io_cur_state == io_pre_state)
                    {
                        ioVal->state = io_cur_state;
                    }
                    else
                    {
                        io_pre_state = io_cur_state;
                    }

                    void *p = ioVal;
										pack_struct(p, IOValName, (const char *)IOValTypeRecord, IOValMemberNum, 0);
                }
                else if (type == BoardType::steeringCurrent)
                {
                    xQueueReceive(steerValQueue, steerVal, 0);
                    steerCurVal->state = steerVal->state;
                    steerCurVal->dr1_cur = (steerVal->dr1_real_cur + maxCurrent) * current_params_coeff;
                    steerCurVal->dr1_vel = (steerVal->dr1_real_vel + maxVel) * motion_params_coeff;
                    steerCurVal->dr2_pos = (steerVal->dr2_real_pos + PI) * motion_params_coeff;
                    void *p = steerCurVal;
                    pack_struct(p, SteerCurName, (const char *)SteerCurTypeRecord, SteerCurMemberNum, 0);
                }
                else if (type > BoardType::steeringWheel)
                {
                    if (adsorptionVal != nullptr)
                    {
                        xQueueReceive(adsorptionValQueue, adsorptionVal, 0);
                        void *p = adsorptionVal;
                        // cur_prefix = pack_struct(p, AdsorptionValName, (const char *)AdsorptionValTypeRecord, AdsorptionValMemberNum, 0);
                        // cur_prefix -= 2;
                        // pack_struct(p, AdsorptionValName, (const char *)AdsorptionValTypeRecord, AdsorptionValMemberNum, 0);
                        pack_struct(p, NULL, (const char *)AdsorptionValTypeRecord, AdsorptionValMemberNum, 0);
                        cur_prefix = strlen((char *)txDealBuf);
                    }
                    xQueueReceive(fanValQueue, fanVal, 0);
                    void *p = fanVal;
                    // prefix = pack_struct(p, FanValName, (const char *)FanValTypeRecord, FanValMemberNum, cur_prefix);
                    pack_struct(p, FanValName, (const char *)FanValTypeRecord, FanValMemberNum, cur_prefix);
                    cur_prefix = strlen((char *)txDealBuf);
                }
                else if (type == BoardType::steeringWheel)
                {
                    xQueueReceive(steerValQueue, steerVal, 0);
                    void *p = steerVal;
                    pack_struct(p, SteerValName, (const char *)SteerValTypeRecord, SteerValMemberNum, 0);
                }
                // txDealBuf[LEN_IDX] = cur_prefix;
                xQueueSend(sendDataQueue, txDealBuf, 1);
            }
        }
    }

    void Init()
    {
        BaseType_t rtn;
        if (type != BoardType::idle)
        {
            if (type == BoardType::activeAdsorption || type == BoardType::normalAdsorption)
            {
                fanCmd = (FanCmd *)pvPortMalloc(sizeof(FanCmd));
                if (fanCmd == nullptr)
                    return;

                fanVal = (FanVal *)pvPortMalloc(sizeof(FanVal));
                if (fanVal == nullptr)
                    return;

                fanCmdQueue = xQueueCreate(1, sizeof(FanCmd));
                configASSERT(fanCmdQueue);

                fanValQueue = xQueueCreate(1, sizeof(FanVal));
                configASSERT(fanValQueue);

                if (type == BoardType::activeAdsorption)
                {
                    adsorptionCmd = (AdsorptionCmd *)pvPortMalloc(sizeof(AdsorptionCmd));
                    if (adsorptionCmd == nullptr)
                        return;
                    adsorptionVal = (AdsorptionVal *)pvPortMalloc(sizeof(AdsorptionVal));
                    if (adsorptionVal == nullptr)
                        return;

                    adsorptionCmdQueue = xQueueCreate(1, sizeof(AdsorptionCmd));
                    configASSERT(adsorptionCmdQueue);

                    adsorptionValQueue = xQueueCreate(1, sizeof(AdsorptionVal));
                    configASSERT(adsorptionValQueue);
                }
            }
            else if (type == BoardType::steeringWheel || type == BoardType::steeringCurrent)
            {
                steerCmd = (SteerCmd *)pvPortMalloc(sizeof(SteerCmd));
                if (steerCmd == nullptr)
                    return;

                steerVal = (SteerVal *)pvPortMalloc(sizeof(SteerVal));
                if (steerVal == nullptr)
                    return;

                steerCurCmd = (SteerCurInfo *)pvPortMalloc(sizeof(SteerCurInfo));
                if (steerCurCmd == nullptr)
                    return;

                steerCurVal = (SteerCurInfo *)pvPortMalloc(sizeof(SteerCurInfo));
                if (steerCurVal == nullptr)
                    return;

                steerCmdQueue = xQueueCreate(1, sizeof(SteerCmd));
                configASSERT(steerCmdQueue);

                steerValQueue = xQueueCreate(1, sizeof(SteerVal));
                configASSERT(steerValQueue);
            }
            else
            {
                ioCmd = (IOCmd *)pvPortMalloc(sizeof(IOCmd));
                if (ioCmd == nullptr)
                    return;

                ioVal = (IOVal *)pvPortMalloc(sizeof(IOVal));
                if (ioVal == nullptr)
                    return;
            }
        }

        rawDataQueue = xQueueCreate(2, RECV_BUF_SIZE);
        configASSERT(rawDataQueue);

        sendDataQueue = xQueueCreate(1, SEND_BUF_SIZE);
        configASSERT(sendDataQueue);

        xMutex = xSemaphoreCreateMutex();
        configASSERT(xMutex != NULL);
        // Create tasks
//        rtn = xTaskCreate(ethTransRecvTask, (const portCHAR *)"ethTransRecvTask",
//                          tskStkSize, NULL, osPriorityBelowNormal1, NULL);
//        configASSERT(rtn == pdPASS);
        rtn = xTaskCreate(ethDealTask, (const portCHAR *)"ethDealTask",
                          tskStkSize, NULL, osPriorityBelowNormal, NULL);
        configASSERT(rtn == pdPASS);
    }
}
