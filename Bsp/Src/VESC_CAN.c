#include "VESC_CAN.h"
#include "can.h"

extern uint8_t io_state_cmd;
/***********************************数学处理函数***********************************/

float NormalizeAnglePositive(float angle_rad) 
{
    float angle_tmp = fmod(angle_rad, 360.0f);
    if (angle_tmp < 0) 
    {
        angle_tmp += 360.0f;
    }
    return angle_tmp;
}

float NormalizeAngle(float angle_rad) 
{
    float angle_tmp = fmod(angle_rad + 180.0f, 360.0f);
    if (angle_tmp < 0) 
    {
        angle_tmp += 360.0f;
    }
    return angle_tmp - 180.0f;
}

//float RobotAngle2MotorAngle(float Robot_Angle)
//{
//    float Motor_Angle = 0.0f;

//	//
//    Motor_Angle = - MotorHelm.RobotStartPosition + Robot_Angle + 
//        MotorHelm.StartPosition;

//    Motor_Angle = NormalizeAnglePositive(Motor_Angle);

//    return Motor_Angle;
//}

//float MotorAngle2RobotAngle(float Motor_Angle)
//{
//    float Robot_Angle;

//    Robot_Angle = - MotorHelm.StartPosition + Motor_Angle + MotorHelm.RobotStartPosition;
//	//舵  -初始测量+31.94+现在测量值+45
//	//FINISH_ANGLE               = MotorHelm.PositionMeasure + finish_angle;

//    Robot_Angle = NormalizeAngle(Robot_Angle);

//    return Robot_Angle;
//}

/***********************************VESC消息处理函数***********************************/

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}
// comm_can_transmit_eid函数的实现原理，
// 首先是CAN_TxHeaderTypeDef结构体的定义，这个结构体是CAN的发送结构体，
// TxMessage.IDE = CAN_ID_EXT;表示使用扩展ID，TxMessage.ExtId = id;表示扩展ID的值为id，
// TxMessage.RTR = CAN_RTR_DATA;表示发送的是数据帧，TxMessage.DLC = len;表示数据长度为len，
// TransmitMailbox = HAL_CAN_AddTxMessage(&hcan1,&TxMessage,data,&Tx_MailBox);表示将数据发送出去，
void comm_can_transmit_eid(uint32_t id,  uint8_t *data, uint8_t len) 
{
	uint8_t TransmitMailbox = HAL_ERROR;	
	uint16_t  time_out_count = 0;
	uint32_t Tx_MailBox;

    if (len > 8) 
    {
		len = 8;
	}
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.ExtId = id;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len;
    while((TransmitMailbox != HAL_OK) && (time_out_count++ != 0xFF))
	{
		TransmitMailbox = HAL_CAN_AddTxMessage(&hcan,&TxMessage,data,&Tx_MailBox);
	}
}

void comm_can_get_status(uint8_t controller_id)
{
    int32_t send_index = 0;
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_GET_STATUS << 8), 0, send_index);
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
//    rpm *= MotorWheel.DIR;
	    rpm *= VESC_SANDING_DIR;
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}
// 解释一下下面的函数实现原理，
// 首先是buffer_append_int32函数，将float类型的pos乘以1000000，
// 然后转换成int32_t类型，再将其转换成uint8_t类型，存入buffer中，
// 最后将send_index加4，即buffer的下一个位置。
void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
    // NormalizeAnglePositive函数的作用：将pos限制在0~360度之间
    pos = NormalizeAnglePositive(VESC_SANDING_DIR * pos);
    // buffer_append_int32的作用：将float类型的pos乘以1000000，
	buffer_append_int32(buffer, (int32_t)(pos * 1000000.0f), &send_index);
    // int32_t 的上限是2147483647，
	comm_can_transmit_eid(controller_id | 
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

float SpeedMeasure=0;
float PositionMeasure=0;
void ProcessVESCFrame(Frame *Frame_Process)
{
    int32_t get_index            = 0;
    EXT_ID_Typedef ExtId;

    ExtId = Frame_Process->id.ExtID;

    if(ExtId.Property == CAN_PACKET_STATUS)
    {
        if(ExtId.DesDEVICE_ID == VESC_SANDING_ID)
        {
            SpeedMeasure    = 
                (float)buffer_get_int32(Frame_Process->data.uchars,&get_index);
            PositionMeasure = NormalizeAnglePositive(VESC_SANDING_DIR *(float)buffer_get_int32(Frame_Process->data.uchars,&get_index) / 
                10000.0f);
        }

    }
}

void VESC_Task(void *pvParameters)
{

    for(;;)
	{
		if(io_state_cmd==1)
		comm_can_set_rpm(VESC_SANDING_ID,5000);
		else
		comm_can_set_rpm(VESC_SANDING_ID,0);
	}

	
    /* 安全起见，如果程序由于异常执行到这，则删除当前任务 */
    vTaskDelete( NULL );

}


void VESC_SANDING_INIT()
{
	const int tskStkSize = 512;
	BaseType_t rtn;
	CAN_Start_Trans();
	comm_can_set_current(VESC_SANDING_ID,0);
	// Create tasks
	rtn = xTaskCreate(VESC_Task, (const portCHAR *)"VESC_Task",
						tskStkSize, NULL, osPriorityAboveNormal, NULL); //Priority值越小，任务的执行优先级就越低
	configASSERT(rtn == pdPASS);

}

