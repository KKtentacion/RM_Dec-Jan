#include "cmsis_os.h"
#include "PID.h"
#include "cmsis_os.h"
#include "judge.h"
#include "rc_potocal.h"
#include "pid.h"
#include "drv_can.h"
extern Motor_t MOTOR1_can1;
extern Motor_t MOTOR2_can1;
extern Motor_t MOTOR3_can1;
extern Motor_t MOTOR1_can2;
extern Motor_t MOTOR2_can2;
extern Motor_t MOTOR3_can2;

extern float Target_pos1_Can1;
extern float Target_pos1_Can2;
extern float Target_pos2_Can2;
extern float Target_pos3_Can2;

static void DM_Enable(); //达妙电机使能
static void DM_Init();   //达妙电机初始化到初始位置
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  HAL_Delay(1000);
  DM_Enable();
  HAL_Delay(1000);
//  DM_Init();
  for(;;)
  {
//		MIT_CtrlMotor(0x01,Target_pos1_Can1,0.2,10,0.5,1);
//		MIT_CtrlMotor2(0x01,Target_pos1_Can2,0.2,10,0.5,1);
//		MIT_CtrlMotor2(0x02,Target_pos2_Can2,0.2,10,0.5,1);
//		MIT_CtrlMotor2(0x03,Target_pos3_Can2,0.2,10,0.5,1);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

static void DM_Enable()
{
  Enable_Ctrl2(0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
  HAL_Delay(1000);
  Enable_Ctrl2(0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
  HAL_Delay(1000);
  Enable_Ctrl2(0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
  HAL_Delay(1000); 
  Enable_Ctrl(0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC);
}

static void DM_Init()
{        
//Expend Progress	
  MIT_CtrlMotor2(0x01,3.14,0.2,1,0.5,1);
  HAL_Delay(2000);
  MIT_CtrlMotor(0x01,-2.2,0.2,1,0.5,1);
  HAL_Delay(2000);
  MIT_CtrlMotor2(0x03,0.5,0.2,1,0.5,1);
  HAL_Delay(2000);
  MIT_CtrlMotor2(0x02,-5.3,0.2,1,0.5,1);
	HAL_Delay(2000);
		
//Recycle Progress
	MIT_CtrlMotor2(0x02,-1.7,0.01,1,0.5,1);
	HAL_Delay(2000);
	MIT_CtrlMotor2(0x03,-3.5,0.2,1,0.5,1);
	HAL_Delay(2000);
  MIT_CtrlMotor(0x01,-2.2,0.2,1,0.5,1);
}