#include "shoot.h"
#include "math.h"

SHOOT_DATA shoot_Data_Down=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data_Down ={0};

SHOOT_DATA shoot_Data_Up=SHOOT_DATA_INIT;
SHOOT_MOTOR_DATA shoot_Motor_Data_Up ={0};

PID_GENERAL   PID_Shoot_Down_Position=PID_SHOOT_POSITION_DEFAULT;
PID_GENERAL   PID_Shoot_Down_Speed=PID_SHOOT_SPEED_DEFAULT;

PID_GENERAL   PID_Shoot_Up_Position=PID_SHOOT_POSITION_DEFAULT;
PID_GENERAL   PID_Shoot_Up_Speed=PID_SHOOT_SPEED_DEFAULT;

extern u32 time_1ms_count;

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];

extern tGameRobotState         testGameRobotState;      //����������״̬
extern u8 Robot_Level;

u8 Friction_State=0;	//��ʼ��������
//const u16 FRICTION_INIT=800;
u16 FRICTION_SHOOT=1965;	//������PWM	�ڼ�¼���������13��ÿ��
u16 Friction_Send=FRICTION_INIT;
void Shoot_Task(void)	//��ʱƵ�ʣ�1ms
{ 
	if(time_1ms_count%100==0)
	{
		FRICTION_SHOOT=Friction_Adjust_DependOn_Vol(testPowerHeatData.chassisVolt);	//�Զ��������
	}
	
	Shoot_Instruction();
	shoot_Motor_Data_Down.tarP=(s32)shoot_Data_Down.motor_tarP;
//	shoot_Motor_Data_Up.tarP=(s32)shoot_Data_Up.motor_tarP;
	
	shoot_Motor_Data_Down.tarV=PID_General(shoot_Motor_Data_Down.tarP,shoot_Motor_Data_Down.fdbP,&PID_Shoot_Down_Position);
//	shoot_Motor_Data_Up.tarV=PID_General(shoot_Motor_Data_Up.tarP,shoot_Motor_Data_Up.fdbP,&PID_Shoot_Up_Position);

	Friction_Send=FRICTION_INIT-(FRICTION_INIT-FRICTION_SHOOT)*Friction_State;	//1888��Ӧ����20,1800-14	1830-14.7	1840-15.1��5.14��	1850���16�����15		//�����۲죬���ܺ͵�ѹ�й�ϵ������ʱ1860Ϊ17.7���յ�Ϊ15.7

	shoot_Motor_Data_Down.output=PID_General(shoot_Motor_Data_Down.tarV,shoot_Motor_Data_Down.fdbV,&PID_Shoot_Down_Speed);//down
	shoot_Motor_Data_Up.output=PID_General(shoot_Motor_Data_Up.tarV,shoot_Motor_Data_Up.fdbV,&PID_Shoot_Up_Speed);//Up
	SetFrictionWheelSpeed(Friction_Send);	//Ħ������ֵ����

}


u16 shoot_time_record=0;
u16 shoot_time_measure(const s16 tarP,const s16 fbdP,const u8 last_mouse_press_l)
{
	static u8 once_statu=0;
	static u16 time_count_start=0;
	static u16 time_count_end=0;
	static u16 time_count_tem=0;
	if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)
	{
		time_count_tem=time_1ms_count;
		once_statu=0;
	}
	if(abs(tarP-fbdP)<2&&once_statu!=1)
	{
		once_statu=1;
		time_count_end=time_1ms_count;
		time_count_start=time_count_tem;
	}
	
	return  time_count_end-time_count_start;
}




#define SINGLE_INCREMENT_OLD_2006 196.608f	//8192*96/4/1000	һȦ���ۼ�ֵ8192*96����һȦ7���ӵ����Ա�����ת������=����һ���ӵ���λ������
#define SINGLE_INCREMENT_NEW_2006 73//65.536f		//8192*32/4/1000
#define SINGLE_INCREMENT SINGLE_INCREMENT_NEW_2006	//5.11��
//���Ϊ����������λ��
//ע��Ӧ���ڱ�����������һָ����������������߼����л�״̬�����÷���ָ�����ͻ�����ʹ��������ͺ��ԣ�
//���߽������߼���Ϊ��������ʽ������Ƶ�ʿ���
void Shoot_Instruction(void)	//����ָ��ģ��
{
	static u8 auto_takebullet_statu_last=0;
	static WorkState_e State_Record=CHECK_STATE;
	PC_Control_Shoot(&Friction_State);
	RC_Control_Shoot(&Friction_State);
	
//	if((auto_takebullet_statu_last==0&&auto_takebullet_statu==1)||KeyBoardData[KEY_R].value==1)	//��Ħ����
//	{
//		Friction_State=1;	//�Զ���Ħ����
//	}
//	shoot_time_record=shoot_time_measure(shoot_Data_Down.count,shoot_Data_Down.count_fdb,last_mouse_press_l);////////////////////////////////
	
	shoot_Data_Down.motor_tarP=((float)shoot_Data_Down.count*SINGLE_INCREMENT);	//��2006
//	shoot_Data_Up.motor_tarP=((float)shoot_Data_Up.count*SINGLE_INCREMENT);	//��2006
	
	shoot_Motor_Data_Up.tarV=-3500;
	
	Prevent_Jam_Down(&shoot_Data_Down,&shoot_Motor_Data_Down);
	Prevent_Jam_Up(&shoot_Data_Up,&shoot_Motor_Data_Up);
	
	State_Record=GetWorkState();
}

u8 Shoot_RC_Control_State=1;	//�����а��������󣬽���RC����
void RC_Control_Shoot(u8* fri_state)
{
	static u8 swicth_Last_state=0;	//�Ҳ���
	if(Shoot_RC_Control_State==1)
	{
		if(Shoot_Heat_Limit(testPowerHeatData.shooterHeat1,Robot_Level)==1&&Shoot_Heat_Lost_Fre_Limit()==1&&*fri_state==1)	//��������
		{
			if(RC_Ctl.rc.switch_left==RC_SWITCH_UP&&swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_DOWN)
			{
				shoot_Data_Down.count-=1;
				shoot_Data_Down.last_time=time_1ms_count;
			}

		}
		
		if(RC_Ctl.rc.switch_left==RC_SWITCH_UP&&swicth_Last_state==RC_SWITCH_MIDDLE&&RC_Ctl.rc.switch_right==RC_SWITCH_UP)
		{
			*fri_state=!*fri_state;
		}
	}
	swicth_Last_state=RC_Ctl.rc.switch_right;
}


void PC_Control_Shoot(u8* fri_state)
{
	static u8 last_mouse_press_l=0;
	static u8 last_keyX_state=0;

	if(RC_Ctl.mouse.press_l==1||RC_Ctl.mouse.press_r==1||RC_Ctl.mouse.x>1||RC_Ctl.mouse.y>1)
	{
		Shoot_RC_Control_State=0;	//����RC�����ڼ�����PCʧЧ��ʱ����RC�Ĳ���
	}
	
	
	if(Shoot_Heat_Limit(testPowerHeatData.shooterHeat1,Robot_Level)==1&&Shoot_Heat_Lost_Fre_Limit()==1&&*fri_state==1)	//��������
	{
		if(RC_Ctl.mouse.press_l==1&&last_mouse_press_l==0)	//shoot_Motor_Data.tarP-shoot_Motor_Data.fdbP	//������
		{
			shoot_Data_Down.count--;
			shoot_Data_Down.last_time=time_1ms_count;
		}
	}
	
	if(last_keyX_state==0&&KeyBoardData[KEY_X].value==1)
	{
		*fri_state=1;
	}
	
	else if(KeyBoardData[KEY_X].statu==2)
	{
		*fri_state=0;
	}
	
	if(*fri_state==0)
	{
		LASER_OFF();
	}
	else
	{
		LASER_ON();
	}
		
	last_mouse_press_l=RC_Ctl.mouse.press_l;
	last_keyX_state=KeyBoardData[KEY_X].value;
}


const float Friction_Set_Record[2][2]={	\
{21.6f,1866},\
{24.4f,1838}\
};	//��¼������ѹֵ�˵�����Ħ���ֲ�������һ���ǵͣ��ڶ����Ǹ�
u16 Friction_Adjust_DependOn_Vol(float voltage)	//����Ƶ��10HZ
{
	u16 pwm_set=800;
	static float voltage_deal=24.2f;
	float ratio_vol=0.0;	//��¼��ѹ���������̱���
	voltage_deal=0.96f*voltage_deal+0.04f*voltage;
	
	ratio_vol=(float)(voltage_deal-Friction_Set_Record[0][0])/(float)(Friction_Set_Record[1][0]-Friction_Set_Record[0][0]);	//������
	pwm_set=(u16)((Friction_Set_Record[1][1]-Friction_Set_Record[0][1])*ratio_vol+Friction_Set_Record[0][1]);
	
	if(pwm_set<1820)	pwm_set=1820;
	if(pwm_set>1865)	pwm_set=1865;
//	pwm_set=pwm_set<1840?1840:pwm_set;
//	pwm_set=pwm_set>1865?1865:pwm_set;
	
//	if(Error_Check.statu[LOST_REFEREE]==1)	//����lost
//	{
		pwm_set=1800;//1808;
//	}
	
	return pwm_set;
	//���ݵ�ѹ�ó�����PWM
}


#define G 9.80151f
/**********************************
Visual_Pretreatment
deal the Visual data
output:disitance: dm
			 priority:0-10
**********************************/
void Visual_Pretreatment()
{
	shoot_Data_Down.Visual.distance=20;
	shoot_Data_Down.Visual.priority=10;
}

/*********************************
Shoot_Rate_Set
caculate the rate data based on the visual.distance data
�����Զ����
*********************************/
void Shoot_Rate_Set()
{
	
}

/*********************************
Shoot_Frequency_Set
*********************************/
void Shoot_Frequency_Set()
{
	
}



/*********************************
Shoot_Frequency_Limit	//�ϰ��V^2������
�����Զ����
*********************************/
#define UPPER_LIMIT_OF_HEAT 4500	//��������
#define COOLING_PER_SECOND 1500	//ÿ����ȴ
void Shoot_Frequency_Limit(int* ferquency,u16 rate,u16 heat)	//m/sΪ��λ
{
	u16 heating=rate*rate;
	s16 ferquency_safe=(s16)(COOLING_PER_SECOND/heating);
	if(*ferquency!=0)
	{
		if(heat<5*heating&&heat>=2*heating)	//4������ʱ��ʼ���壬�Է�����
		{
			*ferquency=(u16)ferquency_safe+1;
		}
		else if(heat<=heating)	//����������������
		{
			*ferquency=0;
		}
		else if(heat>=heating&&heat<2*heating)
		{
			*ferquency=(u16)((ferquency_safe-1)>0?(ferquency_safe-1):0);
		}
	}

}



u8 Shoot_Heat_Limit(u16 heating,u8 level)	//��Ӧ��������Ƶ
{
	if((80*pow(2,level-1)-heating>46)&&time_1ms_count-shoot_Data_Down.last_time>250)	//testPowerHeatData.shooterHeat1
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

u8 Shoot_Heat_Lost_Fre_Limit(void)	//����lost�������Ƶ�����ƣ�������1��OK
{
	u8 limit_state=0;
	if(Error_Check.statu[LOST_REFEREE]==1)	//����lost
	{
		if(time_1ms_count-shoot_Data_Down.last_time>250)	//����1s	//��ʱ���ԣ�1��4��	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		{
			limit_state=1;
		}
	}
	else
	{
		limit_state=1;
	}
	return limit_state;
}

void Shoot_Speed_Adjust(u16 * pwm , u16 speed_fdb)	//���ٱջ�
{
	
}

/*********************************
Shoot_Rate_Adjust
*********************************/
void Shoot_Rate_Adjust()
{
	
}
s32 jam_DownfdbP_record;	//���������s32��Ȼ�ڿ�ʼʱ��������ѭ��
#define JAM_FALLBACK 34	//100	//�����ߵľ���
//��tarP�Ĳ���
void Prevent_Jam_Down(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
{
	static s32 deviation=0;	//ƫ��
	static u8 jam_deal_state=0;
//	static u16 ferquency_last=0;
	
	deviation=shoot_motor_Data->tarP-shoot_motor_Data->fdbP;
	
//	if(shoot_data->frequency!=ferquency_last)
//	{
//		shoot_data->Jam.count=0;	//����count
//	}
//	ferquency_last=shoot_data->frequency;	//����
	
	
	if(abs(deviation)>6&&abs(shoot_motor_Data->fdbV)<10)	//�����ٶȲ�Ϊ0ʱλ��δ�����仯	//bug:Ƶ��ˢ��ʱ��Ҫˢ��count	//�ֶ������Ƶ�ʼ��ɾ��
	{
		shoot_data->Jam.count++;
	}
	else
	{
		shoot_data->Jam.count=0;
	}
	
//	if(shoot_data->cycle!=0)
//	{
		if(shoot_data->Jam.count>100&&shoot_data->Jam.sign==0)	//����������ʱ��	//�ҽ�ִ��һ��
		{
			 shoot_data->Jam.sign=1;	//��ǿ���
			 jam_deal_state=1;	//��ǿ����������״̬
		}
//	}
	
	if(shoot_data->Jam.sign==1)	//������ģ��
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					jam_DownfdbP_record=shoot_motor_Data->fdbP-JAM_FALLBACK;	//���ܻ��ڿ�ʼʱ�򿨵���Σ�գ�
					shoot_data->motor_tarP=jam_DownfdbP_record;
					jam_deal_state=2;
					break;
				}
				case 2:
				{
					shoot_data->motor_tarP=jam_DownfdbP_record;
					if(abs(shoot_motor_Data->fdbP-jam_DownfdbP_record)<40)	//��Ϊ�Ѿ�ִ���˶���	//50
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_data->Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_data->count=shoot_data->count_fdb;	//�����ӵ����ݣ���ֹ����	//���Ƿ���Ҫ+-1��
					shoot_data->Jam.count=0;	//���ÿ���������ݣ���ֹ����
					break;
				}
		}
	}
	
}   


void Prevent_Jam_Up(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//�������STOP�л��ı���//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
{
	static u8 jam_deal_state=0;
	static u32 time_record_jam_start=0;
	static u32 time_record_jam_end=0;
	if(abs(shoot_motor_Data->fdbV)<100&&shoot_data->Jam.sign==0)	//������
	{
		shoot_data->Jam.count++;
	}
	else	//���ٶȲ������������������㣬���ڴ�������Ҳ���������ֹ����������״���
	{
		shoot_data->Jam.count=0;
	}
	
	if(shoot_data->Jam.count>100)	//50ms
	{
		shoot_data->Jam.sign=1;
		jam_deal_state=1;
	}
	
	if(shoot_data->Jam.sign==1)	//������ģ��
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					shoot_motor_Data->tarV=2500;
					if(time_record_jam_start==0)
					{
						time_record_jam_start=time_1ms_count;	//��¼����ʱ��
					}
					if(time_1ms_count-time_record_jam_start>300&&time_record_jam_start!=0)	//����
					{
						jam_deal_state=2;
						time_record_jam_start=0;
					}
					break;
				}
				case 2:
				{
					if(time_record_jam_end==0)
					{
						time_record_jam_end=time_1ms_count;
					}
					
					shoot_motor_Data->tarV=-3500;
					if(time_1ms_count-time_record_jam_end>200&&time_record_jam_end!=0)
					{
						jam_deal_state=1;
						shoot_data->Jam.sign=0;
						time_record_jam_end=0;
					}
					break;
				}
		}
	}
	
}

/*******************************��������*******************************
����Ϊ�ϲ���������
s32 jam_UpfdbP_record;
//��tarP�Ĳ���
void Prevent_Jam_Up(SHOOT_DATA * shoot_data,SHOOT_MOTOR_DATA * shoot_motor_Data)	//����������	//ͬʱ�����������Ĺ���	//����tarP�����֮��
{
	static s32 deviation=0;	//ƫ��
	static u8 jam_deal_state=0;
//	static u16 ferquency_last=0;
	
	deviation=shoot_motor_Data->tarP-shoot_motor_Data->fdbP;
	
//	if(shoot_data->frequency!=ferquency_last)
//	{
//		shoot_data->Jam.count=0;	//����count
//	}
//	ferquency_last=shoot_data->frequency;	//����
	
	
	if(abs(deviation)>6&&abs(shoot_motor_Data->fdbV)<10)	//�����ٶȲ�Ϊ0ʱλ��δ�����仯	//bug:Ƶ��ˢ��ʱ��Ҫˢ��count	//�ֶ������Ƶ�ʼ��ɾ��
	{
		shoot_data->Jam.count++;
	}
	else
	{
		shoot_data->Jam.count=0;
	}
	
//	if(shoot_data->cycle!=0)
//	{
		if(shoot_data->Jam.count>100&&shoot_data->Jam.sign==0)	//����������ʱ��	//�ҽ�ִ��һ��
		{
			 shoot_data->Jam.sign=1;	//��ǿ���
			 jam_deal_state=1;	//��ǿ����������״̬
		}
//	}
	
	if(shoot_data->Jam.sign==1)	//������ģ��
	{
		switch (jam_deal_state)
		{
				case 1:
				{
					jam_UpfdbP_record=shoot_motor_Data->fdbP-JAM_FALLBACK;
					shoot_data->motor_tarP=jam_UpfdbP_record;
					jam_deal_state=2;
					break;
				}
				case 2:
				{
					shoot_data->motor_tarP=jam_UpfdbP_record;
					if(abs(shoot_motor_Data->fdbP-jam_UpfdbP_record)<40)	//��Ϊ�Ѿ�ִ���˶���	//50
					{
						jam_deal_state=3;
					}
					break;
				}
				case 3:
				{
					shoot_data->Jam.sign=0;	//Reset
					jam_deal_state=0;	//
					shoot_data->count=shoot_data->count_fdb;	//�����ӵ����ݣ���ֹ����	//���Ƿ���Ҫ+-1��
					shoot_data->Jam.count =0;	//���ÿ���������ݣ���ֹ����
					break;
				}
		}
	}
	
} 
*************************************************/



/*****************************************
�������ƣ�Shoot_Feedback_Deal
�������ܣ���������������ݽ���+����

*****************************************/
void Shoot_Feedback_Deal(SHOOT_DATA *shoot_data,SHOOT_MOTOR_DATA *shoot_motor_data,CanRxMsg *msg)
{
	shoot_motor_data->fdbP_raw=(msg->Data[0]<<8)|msg->Data[1];//���յ�����ʵ����ֵ  ����Ƶ��1KHz
	shoot_motor_data->fdbV=(msg->Data[2]<<8)|msg->Data[3];
	
	shoot_motor_data->fdbP_diff=shoot_motor_data->fdbP_raw_last-shoot_motor_data->fdbP_raw;
	if(shoot_motor_data->fdbP_diff>4096)	//����6�����������㣬��е�Ƕȹ�8192����λ���������ֲ�ֵΪ6826
	{																			//ע���˺���δ�Ե�һ������ʱ�Ŀ��ܵ�Ȧ��ֱ��Ϊ1��ƫ���������������ڳ�ʼ���б궨��ʼ�Ƕ�ֵ��
		shoot_motor_data->fdbP_raw_sum+=8192;
	}
	else if(shoot_motor_data->fdbP_diff<-4096)
	{
		shoot_motor_data->fdbP_raw_sum-=8192;
	}
	
	shoot_motor_data->fdbP=(s32)((shoot_motor_data->fdbP_raw_sum+shoot_motor_data->fdbP_raw)/1000.0f);	//��Ϊ2006���ٱȹ��� ���㾫ȷ
	
	shoot_motor_data->fdbP_raw_last=shoot_motor_data->fdbP_raw;	//���ݵ���
	
	shoot_data->count_fdb=(u16)(shoot_motor_data->fdbP/SINGLE_INCREMENT);	///////��ֹ������ˢ�·�����
}




//һȦ7��
//����15С

