#ifndef _VESC_CAN_H
#define _VESC_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif
	
#include "main.h"


/*******************************************************************************
	对29位拓展形式ID分段定义
*******************************************************************************/
typedef struct
{
	uint32_t DesDEVICE_ID   	    :8;             //源设备ID
	uint32_t Property		        :8;		        //指令属性值定义
	uint32_t SrcDEVICE_ID   	    :8;             //目标设备ID	
	uint32_t Priority				:4;			    //优先级
	uint32_t Permit                 :1;
}EXT_ID_Typedef;								    //定义29位拓展ID

/*******************************************************************************
	对ID类型定义
*******************************************************************************/
typedef union
{
	uint32_t	all;
	uint32_t	StdID		:11;		            //标准ID
	EXT_ID_Typedef	  ExtID;		                //拓展ID
}ID;

/*******************************************************************************
	对CAN帧内的数据格式定义
*******************************************************************************/
typedef union
{
	int8_t			chars[8];			            //8个char
	int16_t			shorts[4];			            //4个short
	int32_t			ints[2];			            //2个int
	int64_t			longs[1];			            //1个Long
	uint8_t			uchars[8];			            //8个无符号char
	uint16_t		ushorts[4];			            //4个无符号short
	uint32_t		uints[2];			            //2个无符号int
	uint64_t		ulongs[1];			            //1个无符号Long
	float           floats[2];
}CAN_Data;								            //定义CAN的帧内的数据类型

/*******************************************************************************
	¶ÔCANÖ¡ÄÚµÄÌØÊâÎ»¶¨Òå
*******************************************************************************/
typedef struct
{
	ID 			id;						            //ID
	char		isRemote;			                //是否是远程帧
	char 		length;				                //数据长度
	CAN_Data	data;				                //数据
}Frame;

#define VESC_WHEEL_1_ID             1	//轮电机VESC_ID
#define VESC_WHEEL_2_ID             2
#define VESC_WHEEL_3_ID             3
#define VESC_WHEEL_4_ID             4
#define VESC_SANDING_ID             54	//打磨头电机VESC_ID

#define VESC_HELM_1_ID              2	//舵电机VESC_ID
#define VESC_HELM_2_ID              12
#define VESC_HELM_3_ID              13
#define VESC_HELM_4_ID              14

#define HELM1_START_POS             135.0f
#define HELM2_START_POS             -135.0f
#define HELM3_START_POS             -45.0f
#define HELM4_START_POS             45.0f

#define VESC_WHEEL_1_DIR           	1	//轮电机方向
#define VESC_WHEEL_2_DIR            -1
#define VESC_WHEEL_3_DIR            -1
#define VESC_WHEEL_4_DIR            1
#define VESC_SANDING_DIR           	1	//打磨头电机方向

#define VESC_HELM_1_DIR             1	//舵电机方向
#define VESC_HELM_2_DIR             1
#define VESC_HELM_3_DIR             1
#define VESC_HELM_4_DIR             1

#define WHEEL_PARA									17.576148267524f

typedef struct 
{
	float PositionExpected;
	float PositionMeasure;
	float SpeedExpected;
	float SpeedMeasure;
    float StartPosition;
    float RobotStartPosition;
    uint32_t ID;
    int16_t DIR;
}MotorTypeDef;

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,   // 1
    CAN_PACKET_SET_CURRENT_BRAKE, // 2
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_GET_STATUS
} CAN_PACKET_ID;

void buffer_append_int32(uint8_t* , int32_t , int32_t *);
void comm_can_set_duty          (uint8_t, float);
void comm_can_set_current       (uint8_t, float);
void comm_can_set_current_brake (uint8_t, float);
void comm_can_set_rpm           (uint8_t, float);
void comm_can_set_pos           (uint8_t, float);

void comm_can_get_status    (uint8_t);
uint16_t buffer_get_uint16  (const uint8_t *, int32_t *);
int32_t  buffer_get_int32   (const uint8_t *, int32_t *);

float NormalizeAnglePositive(float);
float NormalizeAngle(float);
float RobotAngle2MotorAngle(float);
float MotorAngle2RobotAngle(float);

void ProcessVESCFrame(Frame *Frame_Process);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


namespace TskVESC
{
	
	
    void VESC_SANDING_INIT();
};
//void VESC_SANDING_INIT(void);

#endif /* __cplusplus */

#endif

