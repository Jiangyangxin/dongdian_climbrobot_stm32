/*
 * ethernet_tsk.h
 *
 *  Created on: Aug 1, 2014
 *      Author: loywong
 */

#ifndef ETHERNET_TSK_H_
#define ETHERNET_TSK_H_

#define ETH_NOTIFICATION_DEAL_VALUE 0x1000
#define ETH_NOTIFICATION_SEND_VALUE 0x2000

#define CMD_BUF_SIZE 256
#define SEND_BUF_SIZE 1024
#define RECV_BUF_SIZE 1024
#define ETH_DATA_PORT 5000
#define ETH_CMD_PORT 5001
#define DATA_SN 0
#define CMD_SN 1
#define LEN_IDX 255

#define VNAME(name) (#name)
#define STRVAL(str, name) (str##name)

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
typedef enum
{
    UARTCMD,
    ETHCMD,
} cmdType;

/*
 * Fan cmd val
 */
const char FanCmdName[3][13] = {"fan_state", "fan_tar_pre", "fan_ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char FanCmdTypeRecord[3] = {1, 2, 3};
const char FanCmdMemberNum = 3;

const char FanValName[6][13] = {"fan_state", "fan_tar_pre", "fan_real_pre", "fan_pwm", "fan_speed", "fan_ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char FanValTypeRecord[6] = {1, 2, 2, 2, 2, 3};
const char FanValMemberNum = 6;

/*
 * Steer cmd val
 */
const char SteerCmdName[5][13] = {"steer_state", "dr1_tar_v", "dr2_tar_p", "dr1_tar_i", "ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char SteerCmdTypeRecord[5] = {1, 2, 2, 2, 3};
const char SteerCmdMemberNum = 5;

const char SteerValName[12][13] = {"steer_state", "dr1_tar_v", "dr1_real_v", "dr1_tar_i", "dr1_real_i", "dr2_tar_p",
                                   "dr2_real_p", "dr2_tar_v", "dr2_real_v", "dr2_tar_i", "dr2_real_i", "ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char SteerValTypeRecord[12] = {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3};
const char SteerValMemberNum = 12;

/*
 * Adsorption cmd val
 */
const char AdsorptionCmdName[8][13] = {
    "active_state", "s1_tar_p", "s2_tar_p", "s3_tar_p",
    "plate_tar_x", "plate_tar_y", "plate_tar_z", "active_ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char AdsorptionCmdTypeRecord[8] = {1, 2, 2, 2, 2, 2, 2, 3};
const char AdsorptionCmdMemberNum = 8;

const char AdsorptionValName[17][13] = {
    "active_state", "s1_tar_p", "s2_tar_p", "s3_tar_p",
    "s1_real_p", "s2_real_p", "s3_real_p", "dr1_real_p",
    "dr2_real_p", "dr3_real_p", "plate_tar_x", "plate_tar_y",
    "plate_tar_z", "plate_real_x", "plate_real_y", "plate_real_z", "active_ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char AdsorptionValTypeRecord[17] = {1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3};
const char AdsorptionValMemberNum = 16;

/*
 * IO cmd val
 */
const char IOCmdName[2][13] = {"io_state", "ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char IOCmdTypeRecord[2] = {1, 3};
const char IOCmdMemberNum = 2;

const char IOValName[2][13] = {"io_state", "ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char IOValTypeRecord[2] = {1, 3};
const char IOValMemberNum = 2;


/*
 * steer current cmd val
 */
const char SteerCurName[3][13] = {"sc1", "sc2", "ts"};
// 1-uint32_t; 2-float；3-uint64_t
const char SteerCurTypeRecord[3] = {1, 1, 3};
const char SteerCurMemberNum = 3;

typedef struct
{
    uint64_t ethTs;
    uint32_t localTs;
} TSRecord;

namespace TskEth
{
    extern uint8_t type;
    extern QueueHandle_t steerCmdQueue;
    extern QueueHandle_t steerValQueue;
    extern QueueHandle_t fanCmdQueue;
    extern QueueHandle_t fanValQueue;
    extern QueueHandle_t adsorptionCmdQueue;
    extern QueueHandle_t adsorptionValQueue;
 
    extern QueueHandle_t  rawDataQueue;
    extern QueueHandle_t  sendDataQueue;
    void Init();
};

#endif /* __cplusplus */

#endif