#ifndef __PID_H
#define __PID_H
#include "main.h"
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
typedef struct
{
	CAN_TxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_TxPacketTypeDef;
typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_RxPacketTypeDef;
typedef struct
{
  int p_int;
  int v_int;
  int t_int;
  float position;
  float velocity;
  float torque;
}Motor_t;
//接收包							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	
void set_motor_voltage(uint8_t id_range, int32_t v1, int16_t v2, int16_t v3, int16_t v4);	
void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void Enable_CtrlMotor(uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7);
void Enable_Ctrl(uint16_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7);
void down_inherit();//速度继承			
uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);	
void Speed_CtrlMotor(uint16_t ID, float _vel);		
void PosSpeed_CtrlMotor( uint16_t ID, float _pos, float _vel);	
int float_to_uint(float x, float x_min, float x_max, int bits);
void MIT_CtrlMotor(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
void Enable_Ctrl2(uint16_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7);
void MIT_CtrlMotor2(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
void MY_MIT_CtrlMotor2(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
void MY_MIT_CtrlMotor(uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
float Abs(float num);
#endif
							