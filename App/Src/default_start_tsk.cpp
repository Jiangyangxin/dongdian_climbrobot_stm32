#include "main.h"
#include "w5500_dev.h"
#include "ethernet_tsk.h"
#include "adsorption_fan_tsk.h"
#include "adsorption_motion_tsk.h"
#include "steerwheel_tsk.h"
#include "cmd_tsk.h"
#include <string.h>
#include "at24cxx.h"

char dbgStr[CMD_BUF_SIZE];
uint8_t cmdRxBuf[CMD_BUF_SIZE];
uint8_t cmdTxBuf[CMD_BUF_SIZE];


extern uint8_t rxBuf[RECV_BUF_SIZE];
extern uint8_t txBuf[SEND_BUF_SIZE];
int8_t is_ipv4_addr(char *ip, uint8_t *real_ip)
{
    int8_t rtn = -1;
    if (ip == NULL || ip[0] == '0' || ip[0] == '\0')
    {
        return rtn;
    }

    for (int i = 0, count = 0; i < strlen(ip); i++)
    {
        if ((ip[i] != '.') && (ip[i] < '0' || ip[i] > '9'))
        {
            return rtn;
        }
        if (ip[i] == '.')
        {
            count++;
            if (count > 3)
            {
                return rtn;
            }
        }
    }

    int ip_num[4] = {-1, -1, -1, -1};
    char ip_s[4][4];
    memset(ip_s, 0, sizeof(char[4]) * 4);

    sscanf(ip, "%[^.].%[^.].%[^.].%[^ ]", ip_s[0], ip_s[1], ip_s[2], ip_s[3]);
    sscanf(ip_s[0], "%d", &ip_num[0]);
    sscanf(ip_s[1], "%d", &ip_num[1]);
    sscanf(ip_s[2], "%d", &ip_num[2]);
    sscanf(ip_s[3], "%d", &ip_num[3]);

    for (int i = 0; i < 4; i++)
    {
        if (strlen(ip_s[i]) == 0 || (ip_s[i][0] == '0' && ip_s[i][1] != '\0') || ip_num[i] < 0 || ip_num[i] > 255)
        {
            return rtn;
        }
        real_ip[i] = ip_num[i];
    }

    rtn = 0;
    return rtn;
}

uint8_t *ipv4_addr(char *ip)
{
    int ip_num[4] = {-1, -1, -1, -1};
    uint8_t *real_ip = (uint8_t *)pvPortMalloc(4);
    char ip_s[4][4];
    memset(real_ip, 255, 4);
    sscanf(ip, "%[^.].%[^.].%[^.].%[^ ]", ip_s[0], ip_s[1], ip_s[2], ip_s[3]);
    sscanf(ip_s[0], "%d", &ip_num[0]);
    sscanf(ip_s[1], "%d", &ip_num[1]);
    sscanf(ip_s[2], "%d", &ip_num[2]);
    sscanf(ip_s[3], "%d", &ip_num[3]);
    for (int i = 0; i < 4; i++)
    {
        real_ip[i] = ip_num[i];
    }
    return real_ip;
}

// io扩展板，与4路ADC引脚复用
static void MX_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PC0 PC1 input
    GPIO_InitStruct.Pin = KEY_0_Pin | KEY_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    // 引脚悬空，避免默认状态下板切换类型时电平冲突
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

    // PC2 output
    GPIO_InitStruct.Pin = LED_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    // 引脚悬空，避免板切换类型时电平冲突
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_0_GPIO_Port, &GPIO_InitStruct);

    // PA3 output
    GPIO_InitStruct.Pin = LED_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    // 引脚悬空，避免板切换类型时电平冲突
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);
}

uint8_t ip_set_unpack(uint8_t *buf, wiz_NetInfo *ethInfo, uint8_t cmdType)
{
    uint8_t res = 0;
    uint16_t freq = 100;
    const char *d = " ";
    char *cmd;
    bool exit = false;

    while (!exit)
    {
        cmd = strtok((char *)buf, d);
        char cmd_s[2][20];
        while (cmd != NULL)
        {
            sscanf(cmd, "%[^:]:%s", cmd_s[0], cmd_s[1]);
            if (!strcmp(cmd_s[0], "ip"))
            {
                if (is_ipv4_addr(cmd_s[1], ethInfo->ip) == -1)
                {
                    if (cmdType == cmdType::UARTCMD)
                    {
                        print(cmd);
                    }
                    else
                    {
                        eprint(cmd);
                    }
                    sprintf(dbgStr, " invalid ip, please re-enter \r\n");
                    if (cmdType == cmdType::UARTCMD)
                    {
                        print(dbgStr);
                    }
                    else
                    {
                        eprint(dbgStr);
                    }
                }
                else
                {
                    memcpy(ethInfo->gw, ethInfo->ip, 4);
                    ethInfo->gw[3] = 1;
                    res = res | (1 << 0);
                }
            }
            else if (!strcmp(cmd_s[0], "type"))
            {
                ethInfo->type = cmd_s[1][0] - '0';
                if (ethInfo->type < BoardType::steeringWheel || ethInfo->type > BoardType::steeringCurrent)
                {
                    if (cmdType == cmdType::UARTCMD)
                    {
                        print(cmd);
                    }
                    else
                    {
                        eprint(cmd);
                    }
                    sprintf(dbgStr, " invalid type, please re-enter\r\n");
                    if (cmdType == cmdType::UARTCMD)
                    {
                        print(dbgStr);
                    }
                    else
                    {
                        eprint(dbgStr);
                    }
                }
                else
                {
                    res = res | (1 << 1);
                }
            }
            else if (!strcmp(cmd_s[0], "freq"))
            {
                sscanf(cmd_s[1], "%u", (uint16_t *)&(freq));
                if (freq < 2 || freq > 1000)
                {
                    if (cmdType == cmdType::UARTCMD)
                    {
                        print(cmd);
                    }
                    else
                    {
                        eprint(cmd);
                    }
                    sprintf(dbgStr, " invalid feedback freq, please re-enter\r\n");
                    if (cmdType == cmdType::UARTCMD)
                    {
                        print(dbgStr);
                    }
                    else
                    {
                        eprint(dbgStr);
                    }
                }
                else
                {
                    ethInfo->period = 1000 / freq;
                    res = res | (1 << 2);
                }
            }
            if ((res & 0x07) == 0x7)
            {
                // ethernet setting
                if (cmdType == cmdType::UARTCMD)
                {
                    print(dbgStr);
                }
                else
                {
                    eprint((char *)("update board info, please poweroff and reboot\r\n"));
                }
                W5500_write_config(ethInfo);
            }
            cmd = strtok(NULL, d);
        }
        exit = true;
    }
    return res;
}

void sendTask(void *pvParameters)
{
    BaseType_t rtn;
    uint8_t sn = CMD_SN;
    uint16_t size = 0;

    while (true)
    {
        rtn = xQueueReceive(dbgQueue, cmdTxBuf, portMAX_DELAY);
        configASSERT(rtn == pdPASS);
        size = strlen((char *)cmdTxBuf);
        send(CMD_SN, cmdTxBuf, size);
    }
}
//wiz_NetInfo defaultEthInfo, cmdEthInfo;
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    BaseType_t rtn=pdPASS;
    uint8_t  res = 0;
    uint16_t size = 0;


//    uint8_t *cpu_run_info;
    TickType_t curTick = xTaskGetTickCount();

    TskPrint::Init();
    osDelay(10);

    ethDealTickSem = xSemaphoreCreateBinary();
    if (ethDealTickSem == NULL)
        Error_Handler();
    xSemaphoreGive(ethDealTickSem);

    ethTxTickSem = xSemaphoreCreateBinary();
    if (ethTxTickSem == NULL)
        Error_Handler();
    xSemaphoreGive(ethTxTickSem);

    fanTickSem = xSemaphoreCreateBinary();
    if (fanTickSem == NULL)
        Error_Handler();
    xSemaphoreGive(fanTickSem);

    motionTickSem = xSemaphoreCreateBinary();
    if (motionTickSem == NULL)
        Error_Handler();
    xSemaphoreGive(motionTickSem);

    dbgQueue = xQueueCreate(5, CMD_BUF_SIZE);

    wiz_NetInfo defaultEthInfo, cmdEthInfo;

    sprintf(dbgStr, "\f===== Welcome to the Console MUST add space in the end =====\r\n");
    print(dbgStr);

    sprintf(dbgStr, "Please input ip:val type:val freq:val \r\n");
    print(dbgStr);

    sprintf(dbgStr, "For type:\r\n");
    print(dbgStr);

    sprintf(dbgStr, "\t 1 steering wheel \r\n");
    print(dbgStr);

    sprintf(dbgStr, "\t 2 active adsorption \r\n");
    print(dbgStr);

    sprintf(dbgStr, "\t 3 normal adsorption \r\n");
    print(dbgStr);

    sprintf(dbgStr, "\t 4 io state \r\n");
    print(dbgStr);

    sprintf(dbgStr, "freq (MUST NOT larger than 200)\r\n\r\n");
    print(dbgStr);

    sprintf(dbgStr, "if TIMEOUT, using the last setting\r\n");
    print(dbgStr);
		  
    int32_t ret=0;	
    int32_t flag_ret =0;
    uint16_t sentsize = 0;
//    uint8_t rxBuf[RECV_BUF_SIZE]={0};
//    uint8_t txBuf[SEND_BUF_SIZE]={0};
		
//		
//		
//		int erase = at24_eraseChip();
//		HAL_Delay(2000);
//		if(erase)
//		    sprintf(dbgStr, "at24_eraseChip\r\n");
//		else
//			sprintf(dbgStr, "at24_erase Error!\r\n");
//    print(dbgStr);

    // 获取默认参数
    W5500_get_config(&defaultEthInfo);
    memcpy(&cmdEthInfo, &defaultEthInfo, sizeof(wiz_NetInfo));
    /* Infinite loop */
    for (;;)
    {
        // 启动后等待2s，确认是否配置ip type freq
        if (xQueueReceive(TskPrint::uartDMAQueue, cmdRxBuf, 0) && (xTaskGetTickCount() - curTick) < CMD_TIMEOUT)
        {
            res = ip_set_unpack(cmdRxBuf, &cmdEthInfo, cmdType::UARTCMD);
        }
        else if ((xTaskGetTickCount() - curTick) > CMD_TIMEOUT && res != 0xF)
        {
            HAL_UART_AbortReceive(&huart1);
            res = 0x7;
        }
        // timeout or recv valid params, start system task
        if ((res & 0xF) == 0x7)
        {
            W5500_init(&cmdEthInfo);
            ethPeriod = cmdEthInfo.period;
            sprintf(dbgStr, "ip:%d.%d.%d.%d, type:%d, freq:%d\r\n", cmdEthInfo.ip[0], cmdEthInfo.ip[1], cmdEthInfo.ip[2],
                    cmdEthInfo.ip[3], cmdEthInfo.type, 1000 / ethPeriod);
            print(dbgStr);

            TskEth::type = cmdEthInfo.type;
            HAL_TIM_Base_Start_IT(&htim1);
            TskEth::Init();

            // Create ethernet 5001 port send tasks
            rtn = xTaskCreate(sendTask, (const portCHAR *)"sendTask",
                              512, NULL, osPriorityLow, NULL);
            configASSERT(rtn == pdPASS);

            if (cmdEthInfo.type == BoardType::activeAdsorption || cmdEthInfo.type == BoardType::normalAdsorption)
            {
                if (cmdEthInfo.type == BoardType::activeAdsorption)
                {
                    TskMotion::Init();
                }
                TskFan::Init();
                HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            }
            else if (cmdEthInfo.type == BoardType::steeringWheel || cmdEthInfo.type == BoardType::steeringCurrent)
            {
                TskSteer::Init();
            }
            // pinmux, init task
            else if (cmdEthInfo.type == BoardType::ioState)
            {
                MX_IO_Init();
            }
            // check task to ethernet 5001 recv task, DO NOT deal uart cmd!
            res = 0xF;
        }
        else if(res == 0xf)
        {
//            flag_ret = getSn_SR(CMD_SN);
//			sprintf((char *)rxBuf, "cmd_ret = %d\r\n",flag_ret);
//            print((char *)rxBuf);
            
            switch (getSn_SR(CMD_SN))
            {
                case SOCK_ESTABLISHED:
                 osDelay(1);
                if (getSn_IR(CMD_SN) & Sn_IR_CON)
                {
                    setSn_IR(CMD_SN, Sn_IR_CON);
                    sprintf(dbgStr, "ethInfo:ip%d.%d.%d.%d type%d freq%d\r\n",
                            cmdEthInfo.ip[0],
                            cmdEthInfo.ip[1],
                            cmdEthInfo.ip[2],
                            cmdEthInfo.ip[3],
                            cmdEthInfo.type,
                            1000 / cmdEthInfo.period);
                    eprint(dbgStr);
                }
                if ((size = getSn_RX_RSR(CMD_SN)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
                {
                    memset(cmdRxBuf, '\0', CMD_BUF_SIZE);
                    if (size > CMD_BUF_SIZE)
                        size = CMD_BUF_SIZE;
                    recv(CMD_SN, cmdRxBuf, size);
                    W5500_get_config(&cmdEthInfo);
                    ip_set_unpack(cmdRxBuf, &cmdEthInfo, cmdType::ETHCMD);
                }
                break;
            case SOCK_CLOSE_WAIT:
                disconnect(CMD_SN);
                break;
            case SOCK_INIT:
                listen(CMD_SN);
                break;
            case SOCK_CLOSED:
                
                socket(CMD_SN, Sn_MR_TCP, ETH_CMD_PORT, 0x00);
                setsockopt(CMD_SN, SO_KEEPALIVEAUTO, (void *)0);
            
                break;
					
									
            default:
                break;
            }
            
           
            rtn = xSemaphoreTake(ethTxTickSem, ethPeriod + 1);
            configASSERT(rtn);
            
            
//			flag_ret = getSn_SR(DATA_SN);
//			sprintf((char *)rxBuf, "flag_ret = %d\r\n",flag_ret);
//            print((char *)rxBuf);
            
            
            switch (getSn_SR(DATA_SN))
            { 
            case SOCK_ESTABLISHED:
                if (getSn_IR(DATA_SN) & Sn_IR_CON)
                {

                    setSn_IR(DATA_SN, Sn_IR_CON);
                       
                }
                memset(txBuf, 0, LEN_IDX+1);
                // 收、发任一操作进行时都需占用spi双向通信，因此不能进行全双工通信，因此将收发放在一起，另开线程进行解包
                if (pdPASS == xQueueReceive(TskEth::sendDataQueue, txBuf, 0))
                {
                    size = strlen((char *)txBuf);
                    // size = txBuf[LEN_IDX];
                    ret = send(DATA_SN, txBuf, size);                   
                }
                if ((size = getSn_RX_RSR(DATA_SN)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
                {
                    memset(rxBuf, 0, RECV_BUF_SIZE);
                    if (size > RECV_BUF_SIZE)
                        size = RECV_BUF_SIZE;
                    ret = recv(DATA_SN, rxBuf, size);
                    xQueueSend(TskEth::rawDataQueue, rxBuf, 0);
                }
                break;
            case SOCK_CLOSE_WAIT:
                ret = disconnect(DATA_SN);
                if (ret == SOCK_OK)
                {
#ifdef _DEBUG
                    print((char *)("Socket Closed\r\n"));
#endif
                }
                break;
            case SOCK_INIT:
                ret = listen(DATA_SN);
                break;
            case SOCK_CLOSED:
                ret = socket(DATA_SN, Sn_MR_TCP, ETH_DATA_PORT, 0x00);
                setsockopt(DATA_SN, SO_KEEPALIVEAUTO, (void *)0);/////0
            
            
                socket(CMD_SN, Sn_MR_TCP, ETH_CMD_PORT, 0x00);
                setsockopt(CMD_SN, SO_KEEPALIVEAUTO, (void *)0);
            
                break;

            default:

                break;
            }
         
        }
        if ((xTaskGetTickCount() / 100) % 20 == 0)
        {

        }
        osDelay(10);
    }
}
