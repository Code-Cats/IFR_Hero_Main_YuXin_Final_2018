#include "can2_analysis.h"

LIFT_POSITION_ENCODER chassis_position_encoder[4]={0};

extern CHASSIS_DATA chassis_Data;
extern SHOOT_DATA shoot_Data_Down;
extern SHOOT_MOTOR_DATA shoot_Motor_Data_Down;

extern SHOOT_DATA shoot_Data_Up;
extern SHOOT_MOTOR_DATA shoot_Motor_Data_Up;

extern u16 t_up_sm_count_1s;
/******************************************
��������CAN2_Feedback_Analysis
�������ܣ��Ե��̵���Լ���̨����������ݽ���
          �õ���������
������������
��������ֵ����
������������
*******************************************/
u32 t_yun_count=0;
void CAN2_Feedback_Analysis(CanRxMsg *rx_message)
{		
		CAN_Receive(CAN2, CAN_FIFO0, rx_message);//��ȡ����	
		switch(rx_message->StdId)
		{
			 case 0x201:
			{
				Speed_Data_deal(&chassis_Data.lf_wheel_fdbV,rx_message);
				Position_Data_deal_DIV8(&chassis_Data.lf_wheel_fdbP,&chassis_position_encoder[LF],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM1]);
				break;
			}
			case 0x202:
			{
				Speed_Data_deal(&chassis_Data.rf_wheel_fdbV,rx_message);
				Position_Data_deal_DIV8(&chassis_Data.rf_wheel_fdbP,&chassis_position_encoder[RF],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM2]);
				break;
			}
			case 0x203:
			{
				Speed_Data_deal(&chassis_Data.lb_wheel_fdbV,rx_message);
				Position_Data_deal_DIV8(&chassis_Data.lb_wheel_fdbP,&chassis_position_encoder[LB],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM3]);
				break;
			}
			case 0x204:
			{
				Speed_Data_deal(&chassis_Data.rb_wheel_fdbV,rx_message);
				Position_Data_deal_DIV8(&chassis_Data.rb_wheel_fdbP,&chassis_position_encoder[RB],rx_message);
				LostCountFeed(&Error_Check.count[LOST_CM4]);
				break;
			}
			case 0x205:	//shoot ��
			{
			  Shoot_Feedback_Deal(&shoot_Data_Down,&shoot_Motor_Data_Down,rx_message);	//��ʱ��
				LostCountFeed(&Error_Check.count[LOST_SM_DOWN]);
			  break;
			}
			case 0x206:	//shoot ��
			{
			  Shoot_Feedback_Deal(&shoot_Data_Up,&shoot_Motor_Data_Up,rx_message);	//��ʱ��
				LostCountFeed(&Error_Check.count[LOST_SM_UP]);
			  break;
			}
			 default:
			 break;
		}
}


/****************************************************
�������ƣ�CAN_Chassis_SendMsg
�������ܣ����������ݽ����󷢳�
����������motor_201*******������ǰ���ת��
          motor_202*******������ǰ���ת��
          motor_203*******���������ת��
          motor_204*******�����Һ���ת��
��������ֵ�� ��
�����������ݴ���TxMessage�ṹ������CAN_Transmit����
****************************************************/
void CAN2_Chassis_SendMsg(int motor_201,int motor_202,int motor_203,int motor_204)
{	
		CanTxMsg TxMessage;
	  TxMessage.StdId = 0x200;      //֡IDΪ���������CAN_ID
    TxMessage.IDE = CAN_ID_STD;    //��׼֡
    TxMessage.RTR = CAN_RTR_DATA;  //����֡
    TxMessage.DLC = 0x08;          //֡����Ϊ8
    
    TxMessage.Data[0] =(unsigned char)((motor_201>>8)&0xff);
    TxMessage.Data[1] = (unsigned char)(motor_201&0xff);
    TxMessage.Data[2] =(unsigned char)((motor_202>>8)&0xff);
    TxMessage.Data[3] = (unsigned char)(motor_202&0xff);
    TxMessage.Data[4] =(unsigned char)((motor_203>>8)&0xff);
    TxMessage.Data[5] = (unsigned char)(motor_203&0xff);
    TxMessage.Data[6] =(unsigned char)((motor_204>>8)&0xff);
    TxMessage.Data[7] = (unsigned char)(motor_204&0xff);
		 
    CAN_Transmit(CAN2,&TxMessage);
}


/****************************************************
�������ƣ�CAN2_Shoot_SendMsg
�������ܣ�������������ݽ����󷢳�
����������motor_205*******�²������ת��
          motor_206*******�ϲ������ת��

��������ֵ�� ��
�����������ݴ���tx_message�ṹ������CAN_Transmit����
****************************************************/
void CAN2_Shoot_SendMsg(int16_t motor_205,int16_t motor_206)
{	  
    CanTxMsg tx_message;
    tx_message.StdId = 0x1ff;
    tx_message.IDE = CAN_Id_Standard;//��׼֡
    tx_message.RTR = CAN_RTR_Data;   //����֡
    tx_message.DLC = 0x08;           //֡����Ϊ8
    
    tx_message.Data[0] = (unsigned char)(motor_205>>8);
    tx_message.Data[1] = (unsigned char)motor_205;
    tx_message.Data[2] = (unsigned char)(motor_206>>8);
    tx_message.Data[3] = (unsigned char)motor_206;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN2,&tx_message);
}


