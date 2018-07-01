#include "can1_analysis.h"


extern YUN_MOTOR_DATA	yunMotorData;	//云台挂载在CAN1上，因为CAN2预留了6pin接口，云台不需要该接口，为不浪费，故接CAN1

extern SHOOT_DATA shoot_Data_Down;
extern SHOOT_MOTOR_DATA shoot_Motor_Data_Down;
extern SHOOT_DATA shoot_Data_Up;
extern SHOOT_MOTOR_DATA shoot_Motor_Data_Up;

LIFT_POSITION_ENCODER lift_position_encoder[4]={0};
/**************** **************************
函数名：CAN1_Feedback_Analysis
函数功能：对底盘电机数据进行解析
          得到反馈数据
函数参数：无
函数返回值：无
函数描述：无
*******************************************/
void CAN1_Feedback_Analysis(CanRxMsg *rx_message)
{		
		CAN_Receive(CAN1, CAN_FIFO0, rx_message);//读取数据	
		switch(rx_message->StdId)
		{
			case 0x201:
			 {
				 LostCountFeed(&Error_Check.count[LOST_BULLETROTATE1]);
				 break;
			 }
			 case 0x202:
			 {
				 break;
			 }
			 case 0x203:	//shoot 下
			 {
				 Shoot_Feedback_Deal(&shoot_Data_Down,&shoot_Motor_Data_Down,rx_message);	//临时用
					LostCountFeed(&Error_Check.count[LOST_SM_DOWN]);
				 break;
			 }
			 case 0x204:	//shoot 上
			 {
				 Shoot_Feedback_Deal(&shoot_Data_Up,&shoot_Motor_Data_Up,rx_message);	//临时用
					LostCountFeed(&Error_Check.count[LOST_SM_UP]);
				 break;
			 }
			 case 0x205:	//yaw
			 {
				 yunMotorData.yaw_fdbP=((rx_message->Data[0]<<8)|rx_message->Data[1])&0xffff;  //机械角度
				 yunMotorData.yaw_fdbV=(s16)((rx_message->Data[2]<<8)|rx_message->Data[3]);  //转速
				 LostCountFeed(&Error_Check.count[LOST_YAW]);
				 break;
			 }case 0x206:	//pitch
			 {
				 yunMotorData.pitch_fdbP=((rx_message->Data[0]<<8)|rx_message->Data[1])&0xffff;  //机械角度
				 yunMotorData.pitch_fdbV=(s16)((rx_message->Data[2]<<8)|rx_message->Data[3]);  //转速
				 LostCountFeed(&Error_Check.count[LOST_PITCH]);
				 break;
			 }
			default:
			break;
		}
}


/****************************************************
函数名称：CAN2_Shoot_Bullet_SendMsg
函数功能：将拨弹电机及取弹电机数据解析后发出
函数参数： motor_201				取弹电机
					motor_203*******下拨弹电机转速
          motor_204*******上拨弹电机转速

2017.7.1
函数返回值： 无
描述：将数据存入tx_message结构体再由CAN_Transmit发送
****************************************************/
void CAN2_Shoot_Bullet_SendMsg(int16_t motor_201,int16_t motor_202,int16_t motor_203,int16_t motor_204)
{	  
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;//标准帧
    tx_message.RTR = CAN_RTR_Data;   //数据帧
    tx_message.DLC = 0x08;           //帧长度为8
    
    tx_message.Data[0] = (char)(motor_201>>8);
    tx_message.Data[1] = (char)motor_201;
    tx_message.Data[2] = (char)(motor_202>>8);
    tx_message.Data[3] = (char)motor_202;
    tx_message.Data[4] = (char)(motor_203>>8);
    tx_message.Data[5] = (char)motor_203;
    tx_message.Data[6] = (char)(motor_204>>8);
    tx_message.Data[7] = (char)motor_204;
    
    CAN_Transmit(CAN2,&tx_message);
}


/****************************************************
函数名称：CAN_Lift_SendMsg
函数功能：将升降数据解析后发出
函数参数：motor_201*******升降左前电机转速
          motor_202*******升降右前电机转速
          motor_203*******升降左后电机转速
          motor_204*******升降右后电机转速
函数返回值： 无
描述：将数据存入TxMessage结构体再由CAN_Transmit发送
****************************************************/
void CAN1_Lift_SendMsg(int motor_201,int motor_202,int motor_203,int motor_204)
{	
		CanTxMsg TxMessage;
	  TxMessage.StdId = 0x200;      //帧ID为传入参数的CAN_ID
    TxMessage.IDE = CAN_ID_STD;    //标准帧
    TxMessage.RTR = CAN_RTR_DATA;  //数据帧
    TxMessage.DLC = 0x08;          //帧长度为8
    
    TxMessage.Data[0] =(unsigned char)((motor_201>>8)&0xff);
    TxMessage.Data[1] = (unsigned char)(motor_201&0xff);
    TxMessage.Data[2] =(unsigned char)((motor_202>>8)&0xff);
    TxMessage.Data[3] = (unsigned char)(motor_202&0xff);
    TxMessage.Data[4] =(unsigned char)((motor_203>>8)&0xff);
    TxMessage.Data[5] = (unsigned char)(motor_203&0xff);
    TxMessage.Data[6] =(unsigned char)((motor_204>>8)&0xff);
    TxMessage.Data[7] = (unsigned char)(motor_204&0xff);
		 
    CAN_Transmit(CAN1,&TxMessage);
}

/****************************************************
函数名称：CAN1_Yun_SendMsg
函数功能：将云台电机数据解析后发出
函数参数：motor_205*******Yaw轴电机转速
          motor_206*******Pitch轴电机转速

函数返回值： 无
描述：将数据存入tx_message结构体再由CAN_Transmit发送
****************************************************/
void CAN1_Yun_SendMsg(int16_t motor_205,int16_t motor_206)	//yaw  pitch
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1ff;
    tx_message.IDE = CAN_Id_Standard;//标准帧
    tx_message.RTR = CAN_RTR_Data;   //数据帧
    tx_message.DLC = 0x08;           //帧长度为8
    
    tx_message.Data[0] = (unsigned char)(motor_205 >> 8);
    tx_message.Data[1] = (unsigned char)motor_205;
    tx_message.Data[2] = (unsigned char)(motor_206 >> 8);
    tx_message.Data[3] = (unsigned char)motor_206;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}

void CAN_Motor6623_calibration(void)	//6623标定
{
	  CanTxMsg tx_message;
    tx_message.StdId = 0x3f0;
    tx_message.IDE = CAN_Id_Standard;//标准帧
    tx_message.RTR = CAN_RTR_Data;   //数据帧
    tx_message.DLC = 0x08;           //帧长度为8
    
    tx_message.Data[0] = 'c';
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}	
	



void Speed_Data_deal(s32 * fdbV,CanRxMsg * msg)
{
	s16 v_tem=(msg->Data[2]<<8)|msg->Data[3];
	*fdbV=v_tem;//接收到的真实数据值  处理频率1KHz
}


void Position_Data_deal_DIV8(s32 * value,LIFT_POSITION_ENCODER *Receive,CanRxMsg * msg)	//分辨率转子1/8圈
{
	Receive->calc=(msg->Data[0]<<8)|msg->Data[1];//接收到的真实数据值  处理频率1KHz
	Position_To_Turns(Receive);
	*value=Receive->turns*8+(s32)(0.192f*Receive->turns+Receive->calc/1000.0);	//0.192为了消除累积误差
}

void Position_Data_deal_DIV81(s32 * value,LIFT_POSITION_ENCODER *Receive,CanRxMsg * msg)	//分辨率转子1/81圈
{
	Receive->calc=(msg->Data[0]<<8)|msg->Data[1];//接收到的真实数据值  处理频率1KHz
	Position_To_Turns(Receive);
	*value=Receive->turns*81+(s32)(0.92f*Receive->turns+Receive->calc/100.0);	//0.192为了消除累积误差
}

void Position_To_Turns(LIFT_POSITION_ENCODER *Receive)	//按照6倍采样来计算，机械角度共8192个挡位，则过界表现差值为6826
{																								//注：此函数未对第一次运行时的可能的圈数直接为1的偏差做处理（处理方法在初始化中标定初始角度值）	//可经逻辑推算，将过界点设置为8192/2是最极限分辨率，而且可向下兼容，最为安全
	Receive->calc_diff=Receive->calc_last-Receive->calc;
	if(Receive->calc_diff>4096)
	{
		Receive->turns=Receive->turns+1;
	}
	else if(Receive->calc_diff<-4096)
	{
		Receive->turns=Receive->turns-1;
	}
	else
	{
	}
	Receive->calc_last=Receive->calc;
}


