#include "lift.h"
#include "control.h"

extern LIFT_DATA lift_Data;
extern u32 time_1ms_count;

extern KeyBoardTypeDef KeyBoardData[KEY_NUMS];

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
extern u8 IMU_Check_Useless_State;	//������ʧЧ���λ

#define AUTOCHASSIS_LIFT 12
void AutoChassisAttitude_Lift(float chassis_pitch_raw)	//�Զ�������̬	//pitch������Ϊǰ��	//ע�����lift_taskǰ��
{
	static float chassis_pitch=0;
	static float ka=0.1f;
	static u16 steady_flat_count=0;

	chassis_pitch=chassis_pitch*(1-ka)+chassis_pitch_raw*ka;
	
	if(GetWorkState()==NORMAL_STATE)	//����״̬
	{
		
		if(abs(chassis_pitch)>8)	//8����ֵ
		{
			steady_flat_count=0;
			
			lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
			lift_Data.rf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.rf_lift_fdbP;
			lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
			lift_Data.rb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.rb_lift_fdbP;
			
			if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
			{
				s32 l_r_error=(lift_Data.lf_lift_fdbP-lift_Data.rf_lift_fdbP)/2;
				lift_Data.lf_lift_tarP-=l_r_error;
				lift_Data.rf_lift_tarP+=l_r_error;
			}
			
			if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
			{
				s32 l_r_error=(lift_Data.lb_lift_fdbP-lift_Data.rb_lift_fdbP)/2;
				lift_Data.lb_lift_tarP-=l_r_error;
				lift_Data.rb_lift_tarP+=l_r_error;
			}
		}
		else
		{
			if(abs(lift_Data.lf_lift_tarP-lift_Data.lb_lift_tarP)<300)
			{
				if(steady_flat_count<0xFFFE)
				{
					steady_flat_count++;
				}    
			}
				
			if(abs(lift_Data.lf_lift_fdbP+lift_Data.rf_lift_fdbP-2*LIFT_DISTANCE_FALL)>18&&abs(lift_Data.lb_lift_fdbP+lift_Data.rb_lift_fdbP-2*LIFT_DISTANCE_FALL)>18)
			{
				if(time_1ms_count%40==0)	//1ms�ӵ�̫��10MS
				{
					if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
					{
						lift_Data.lf_lift_tarP=lift_Data.lf_lift_fdbP<lift_Data.rf_lift_fdbP?lift_Data.rf_lift_fdbP:lift_Data.rf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;
					}
					
					if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
					{
						lift_Data.lb_lift_tarP=lift_Data.lb_lift_fdbP<lift_Data.rb_lift_fdbP?lift_Data.lb_lift_fdbP:lift_Data.rb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					lift_Data.lf_lift_tarP-=5;
					lift_Data.rf_lift_tarP-=5;
					lift_Data.lb_lift_tarP-=5;
					lift_Data.rb_lift_tarP-=5;
				}
			}
			
		}
		
		if(steady_flat_count>800)
		{
			lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
			lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
		}
//		if(lift_Data.lf_lift_fdbP!=lift_Data.rf_lift_fdbP)
//		{
//			lift_Data.lf_lift_tarP=lift_Data.lf_lift_fdbP<lift_Data.rf_lift_fdbP?lift_Data.rf_lift_fdbP:lift_Data.rf_lift_fdbP;
//			lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;
//		}
//		
//		if(lift_Data.lb_lift_fdbP!=lift_Data.rb_lift_fdbP)
//		{
//			lift_Data.lb_lift_tarP=lift_Data.lb_lift_fdbP<lift_Data.rb_lift_fdbP?lift_Data.lb_lift_fdbP:lift_Data.rb_lift_fdbP;
//			lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
//		}
		
		
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//�����г�
		lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lf_lift_tarP;
		
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//�����г�
		lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rf_lift_tarP;
		
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//�����г�
		lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lb_lift_tarP;
		
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//�����г�
		lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rb_lift_tarP;
	}
}



extern GYRO_DATA Gyro_Data;
	//��ʵ���ö��ڳ������ϵ��˵����ǰ����ʱ���ȵ���˵����ǰ����ʱ��̬���Ϊǰ���󸽣����������
	//														��ǰ����ʱǰ�ȵ���˵����ǰ���٣����ٶȸ���ʱ���Ϊ��
	//��ʵ����IMU_Uranus��оƬ��ǰ����ʱ��ǰ���ٶ�������Ϊǰ��Ϊacc��0λ	//����200���ٶ�4����������㣬����ϵ��Ϊ4/200
#define ATTITUDE_ACC_COMPENSATE (5.0f/200)	//���ٶȶ�����̬����ϵ��

#define TILT 1	//��б״̬
#define STAEDY_REAL 0	//ƽ��״̬
#define STAEDY_ADJUST 2	//������ƽ��״̬
u8 Adjust_Statu=STAEDY_REAL;
void AutoChassisAttitude_Lift_V2(float chassis_pitch_raw)	//�Զ�������̬	//pitch������Ϊǰ��	//ע�����lift_taskǰ��
{
	static float chassis_pitch=0;
	static float ka=0.05f;
	
//	chassis_pitch_raw-=Gyro_Data.acc[0]*ATTITUDE_ACC_COMPENSATE;	//��IMU_Uranus�����㷨ȱ�ݽ��в���
	
	chassis_pitch=chassis_pitch*(1-ka)+chassis_pitch_raw*ka;
	
	if(GetWorkState()==NORMAL_STATE&&KeyBoardData[KEY_CTRL].value==1&&IMU_Check_Useless_State==0)	//����������ʧЧ����//����CTRL����   &&KeyBoardData[KEY_CTRL].value==1		//�Ȳ������Ա���ԣ����ڱ������
	{
		switch(Adjust_Statu)
		{
			case STAEDY_REAL:
			{
				static u16 tilt_change_count=0;	//����ֵ���Ч�����ã���ʹ���������
				lift_Data.lf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rf_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.lb_lift_tarP=LIFT_DISTANCE_FALL;
				lift_Data.rb_lift_tarP=LIFT_DISTANCE_FALL;
				if(chassis_pitch>(8)&&tilt_change_count<0xFFFE)	//������ֵ	Ϊ7ʱ�����ⴥ������			+Gyro_Data.acc[0]*4.0f/200��̨��ˮƽӰ��ϴ�
				{
					tilt_change_count++;
				}
				else if((chassis_pitch<(-8)&&tilt_change_count<0xFFFE))		//+Gyro_Data.acc[0]*4.0f/200
				{
					tilt_change_count++;
				}
				else
				{
					tilt_change_count=0;
				}
				
				if(tilt_change_count>100)
				{
					Adjust_Statu=TILT;
					tilt_change_count=0;
				}
				break;
			}
			case TILT:
			{
				static u16 staedy_adjust_count=0;
				if(chassis_pitch>0)	//ǰ��������ͨ����ǰ���������ǰ�޷�����������Ϻ�
				{
					if(lift_Data.lf_lift_tarP<=LIFT_DISTANCE_FALL)	//����ǰ������
					{
						lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					else
					{
						lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//����ǰ���Ϊ��׼
					}
					
				
				}
				else	//ǰ��������ͨ���º��������º��޷������������ǰ
				{
					if(lift_Data.lb_lift_tarP<=LIFT_DISTANCE_FALL)	//�º�
					{
						lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
						lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//����ǰ���Ϊ��׼
					}
					else
					{
						lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
						lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
					}
					
					
				}
				
//				lift_Data.lf_lift_tarP=-chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lf_lift_fdbP;
//				lift_Data.rf_lift_tarP=lift_Data.lf_lift_tarP;	//����ǰ���Ϊ��׼
//				lift_Data.lb_lift_tarP=chassis_pitch*AUTOCHASSIS_LIFT+lift_Data.lb_lift_fdbP;
//				lift_Data.rb_lift_tarP=lift_Data.lb_lift_tarP;
				
////////				if(chassis_pitch>0)	//ǰ��������ͨ����ǰ���������ǰ�޷�����������Ϻ�	���ڴ˴�дǿ���½���ǰ�º�
////////				{
////////					if(abs(lift_Data.lf_lift_tarP-FALL)>)
////////				}
////////				else	//ǰ��������ͨ���º��������º��޷������������ǰ
////////				{
////////					
////////				}
				
//				if(abs(lift_Data.lf_lift_tarP-FALL)>10&&abs(lift_Data.lb_lift_tarP-FALL)>10)	//δ��������ߵ�	��Ч
//				{
//					if(time_1ms_count%20==0)
//					{
//						lift_Data.lf_lift_tarP-=5;
//						lift_Data.rf_lift_tarP-=5;
//						lift_Data.lb_lift_tarP-=5;
//						lift_Data.rb_lift_tarP-=5;
//					}
//				}
				
				if(abs(chassis_pitch)<2.2f&&staedy_adjust_count<0xFFFE)
				{
					staedy_adjust_count++;
				}
				else
				{
					staedy_adjust_count=0;
				}
				
				if(staedy_adjust_count>300)	//�ȶ���	//������
				{
					Adjust_Statu=STAEDY_ADJUST;
					staedy_adjust_count=0;
				}
				break;
			}
			case STAEDY_ADJUST:
			{
				static u16 staedy_real_count=0;
				if(abs(chassis_pitch)>4)
				{
					Adjust_Statu=TILT;
				}
				else
				{
					if(abs(lift_Data.lf_lift_tarP-lift_Data.lb_lift_tarP)<300&&staedy_real_count<0xFFFE)
					{
						staedy_real_count++;
					}
				}
				
				if(staedy_real_count>150)	//0.3s
				{
					Adjust_Statu=STAEDY_REAL;
					staedy_real_count=0;
				}
				break;
			}
		}	
	}
	
	
	lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lf_lift_tarP;	//�����г�
	lift_Data.lf_lift_tarP=lift_Data.lf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lf_lift_tarP;
	
	lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rf_lift_tarP;	//�����г�
	lift_Data.rf_lift_tarP=lift_Data.rf_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rf_lift_tarP;
	
	lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.lb_lift_tarP;	//�����г�
	lift_Data.lb_lift_tarP=lift_Data.lb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.lb_lift_tarP;
	
	lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP<LIFT_DISTANCE_FALL?LIFT_DISTANCE_FALL:lift_Data.rb_lift_tarP;	//�����г�
	lift_Data.rb_lift_tarP=lift_Data.rb_lift_tarP>LIFT_DISTANCE_ISLAND?LIFT_DISTANCE_ISLAND:lift_Data.rb_lift_tarP;
}


/***********************************************************
����V2�汾�����ֻ���ʱ��������������˼��
1.������ʱ�����ڼ�⵽��ƽ�����һ��ֵʱ��ʱ�������ƽ�棨������ֵ���³�����ӳ����⣩
2.�ڷ����׶α��⣬��pitch>0,����ǰ������TITL�����׶�
	���������Ĳ���������ͨ���½������ȫͨ���½����
	
***********************************************************/

