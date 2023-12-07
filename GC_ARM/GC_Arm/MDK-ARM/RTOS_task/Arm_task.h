#ifndef ARM_TASK_H
#define ARM_TASK_H

//MOTOR1_CAN1 MOTOR3_CAN2 MOTOR2_CAN2 MOTOR1_CAN2

float ExpandPos[4]={-1.51,0.8,-5.3,3.14};

float RecyclePos[4]={-1.06,-2.05,4.6,3.05};

void StartTask02(void const * argument);

#endif