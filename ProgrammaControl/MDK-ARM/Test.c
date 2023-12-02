#include "Test.h"

extern RC_Ctl_t RC_CtrlData;

void Test()
{
	LED_GREEN_ON();
	float speed=(float)(RC_CtrlData.rc.ch0-RC_CH_VALUE_OFFSET)/660*2;
	Motor_enable();
	Motor_control(speed);
}