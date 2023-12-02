#include "DMPower.h"

CANx_t CAN_1,CAN_2;
char Selection=0;

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//���ʹ������
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//���ʧ������
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//��������������


/**
 * @brief  �����IDΪ0x02��0x03��0x04��3�������������ʹ�ܣ��ڶԵ�����п���ǰ����ʹ��
 * @param  void     	
 * @param  vodi      
 * @param  void    	
 * @param  void      	
 */
void Motor_enable(void)
{
	#if Motar_mode==0
	CANx_SendStdData(&hcan1,0x02,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x03,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x04,Data_Enable,8);
	HAL_Delay(10);
	#elif Motar_mode==1
	CANx_SendStdData(&hcan1,0x102,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x103,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x104,Data_Enable,8);
	HAL_Delay(10);
	#elif Motar_mode==2
	CANx_SendStdData(&hcan1,0x202,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x203,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x204,Data_Enable,8);
	HAL_Delay(10);
	#endif

}

/**
* @brief  �����IDΪ0x02��0x03��0x04��3������������ο��ƣ���freertos.c��CAN_Send_Task������1msִ��һ�Σ�
����ʹ����3�����Ҳ����ÿ�����3ms����һ��,ע�����������ͬʱһ�𷢣�����Ƶ�ʹ��죬���ʹ��ЩIDû���͵�
 * @param  void     	
 * @param  vodi      
 * @param  void    	
 * @param  void      	
 */

void Motor_control(int SetSpeed)
{
	#if Motar_mode==0
		switch(Selection)
		{
			case 0:
			{
				MIT_CtrlMotor(&hcan1,0X02, 0, SetSpeed,0, 1, 0);	  //IMT����ģʽ��IDΪ0X02��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 1:
			{
				MIT_CtrlMotor(&hcan1,0X03, 0, SetSpeed,0, 1, 0);			//IMT����ģʽ��IDΪ0X03��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 2:
			{
				MIT_CtrlMotor(&hcan1,0X04, 0, SetSpeed,0, 1, 0);			//IMT����ģʽ��IDΪ0X04��������ٶȿ��Ʒ���
				Selection=0;
				break;
			}
		}
	#elif Motar_mode==1
		switch(Selection)
		{
			case 0:
			{
				PosSpeed_CtrlMotor(&hcan1,0X102, 10, SetSpeed);	  //λ���ٶȿ���ģʽ��IDΪ0X02��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 1:
			{
				PosSpeed_CtrlMotor(&hcan1,0X103,  10, SetSpeed);			//λ���ٶȿ���ģʽ��IDΪ0X03��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 2:
			{
				PosSpeed_CtrlMotor(&hcan1,0X104,  10, SetSpeed);			//λ���ٶȿ���ģʽ��IDΪ0X04��������ٶȿ��Ʒ���
				Selection=0;
				break;
			}
		}
	#elif Motar_mode==2
		switch(Selection)
		{
			case 0:
			{
				Speed_CtrlMotor(&hcan1,0X202, SetSpeed);	  //�ٶȿ���ģʽ��IDΪ0X02��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 1:
			{
				Speed_CtrlMotor(&hcan1,0X203, SetSpeed);			//�ٶȿ���ģʽ��IDΪ0X03��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 2:
			{
				Speed_CtrlMotor(&hcan1,0X204, SetSpeed);			//�ٶȿ���ģʽ��IDΪ0X04��������ٶȿ��Ʒ���
				Selection=0;
				break;
			}
		}
	#endif


}

/**
 * @brief  ���ø������ݵȱ���ת��������
 * @param  x_int     	Ҫת�����޷�������
 * @param  x_min      Ŀ�긡��������Сֵ
 * @param  x_max    	Ŀ�긡���������ֵ
 * @param  bits      	�޷���������λ��
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  ��������ת��Ϊ�޷�������
 * @param  x     			Ҫת���ĸ�����
 * @param  x_min      ����������Сֵ
 * @param  x_max    	�����������ֵ
 * @param  bits      	�޷���������λ��
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  MITģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 * @param  _KP    λ�ñ���ϵ��
 * @param  _KD    λ��΢��ϵ��
 * @param  _torq  ת�ظ���ֵ
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq)
 { 
	static CAN_TxHeaderTypeDef   Tx_Header;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;
	
	CAN_1.Tx_Data[0] = (pos_tmp >> 8);
	CAN_1.Tx_Data[1] = pos_tmp;
	CAN_1.Tx_Data[2] = (vel_tmp >> 4);
	CAN_1.Tx_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	CAN_1.Tx_Data[4] = kp_tmp;
	CAN_1.Tx_Data[5] = (kd_tmp >> 4);
	CAN_1.Tx_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	CAN_1.Tx_Data[7] = tor_tmp;
	 
	 //Ѱ�����䷢������
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
        }
   }
 }

/**
 * @brief  λ���ٶ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _pos   λ�ø���
 * @param  _vel   �ٶȸ���
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
    static CAN_TxHeaderTypeDef Tx_Header;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

		Tx_Header.StdId=id;
		Tx_Header.IDE=CAN_ID_STD;
		Tx_Header.RTR=CAN_RTR_DATA;
		Tx_Header.DLC=0x08;

    CAN_1.Tx_Data[0] = *pbuf;;
    CAN_1.Tx_Data[1] = *(pbuf+1);
    CAN_1.Tx_Data[2] = *(pbuf+2);
    CAN_1.Tx_Data[3] = *(pbuf+3);
    CAN_1.Tx_Data[4] = *vbuf;
    CAN_1.Tx_Data[5] = *(vbuf+1);
    CAN_1.Tx_Data[6] = *(vbuf+2);
    CAN_1.Tx_Data[7] = *(vbuf+3);

    //�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
 
/**
 * @brief  �ٶ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  _vel   �ٶȸ���
 */
void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel)
{
		static CAN_TxHeaderTypeDef   Tx_Header;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    Tx_Header.StdId = ID;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = 0x04;

    CAN_1.Tx_Data[0] = *vbuf;
    CAN_1.Tx_Data[1] = *(vbuf+1);
    CAN_1.Tx_Data[2] = *(vbuf+2);
    CAN_1.Tx_Data[3] = *(vbuf+3);

    //�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
 



/*
 * @brief: CAN Receive Message.
 * @param: "RxData[]" will store the message which has been received, which length must between 0 and 8.
 * @retval: receive status.
 */
 /*CAN�жϽ���*/
 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	if(hcan->Instance==CAN1)
	{
		
	if(HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&CAN_1.Rx_pHeader,CAN_1.RxData)==HAL_OK)//��ȡ����
	{
	if(CAN_1.Rx_pHeader.StdId == 0X02)
 {

	CAN_1.p_int[0]=(CAN_1.RxData[1]<<8)|CAN_1.RxData[2];
	CAN_1.v_int[0]=(CAN_1.RxData[3]<<4)|(CAN_1.RxData[4]>>4);
	CAN_1.t_int[0]=((CAN_1.RxData[4]&0xF)<<8)|CAN_1.RxData[5];
	CAN_1.position[0] = uint_to_float(CAN_1.p_int[0], P_MIN, P_MAX, 16); // (-12.5,12.5)
	CAN_1.velocity[0] = uint_to_float(CAN_1.v_int[0], V_MIN, V_MAX, 12); // (-45.0,45.0)
	CAN_1.torque[0] = uint_to_float(CAN_1.t_int[0], T_MIN, T_MAX, 12); // (-18.0,18.0)
 }
 if(CAN_1.Rx_pHeader.StdId == 0X03)
 {
	CAN_1.p_int[1]=(CAN_1.RxData[1]<<8)|CAN_1.RxData[2];
	CAN_1.v_int[1]=(CAN_1.RxData[3]<<4)|(CAN_1.RxData[4]>>4);
	CAN_1.t_int[1]=((CAN_1.RxData[4]&0xF)<<8)|CAN_1.RxData[5];
	CAN_1.position[1] = uint_to_float(CAN_1.p_int[1], P_MIN, P_MAX, 16); // (-12.5,12.5)
	CAN_1.velocity[1] = uint_to_float(CAN_1.v_int[1], V_MIN, V_MAX, 12); // (-45.0,45.0)
	CAN_1.torque[1] = uint_to_float(CAN_1.t_int[1], T_MIN, T_MAX, 12); // (-18.0,18.0)
 }
  if(CAN_1.Rx_pHeader.StdId == 0X04)
 {
	CAN_1.p_int[2]=(CAN_1.RxData[1]<<8)|CAN_1.RxData[2];
	CAN_1.v_int[2]=(CAN_1.RxData[3]<<4)|(CAN_1.RxData[4]>>4);
	CAN_1.t_int[2]=((CAN_1.RxData[4]&0xF)<<8)|CAN_1.RxData[5];
	CAN_1.position[2] = uint_to_float(CAN_1.p_int[2], P_MIN, P_MAX, 16); // (-12.5,12.5)
	CAN_1.velocity[2] = uint_to_float(CAN_1.v_int[2], V_MIN, V_MAX, 12); // (-45.0,45.0)
	CAN_1.torque[2] = uint_to_float(CAN_1.t_int[2], T_MIN, T_MAX, 12); // (-18.0,18.0)
 }
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	//����CAN�ж�֪ͨ
	}
}
 
}

