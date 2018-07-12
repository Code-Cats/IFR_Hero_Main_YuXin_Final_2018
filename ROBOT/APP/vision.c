#include "vision.h"
//#include "arm_math.h"
#include "math.h"
//PITCH_GYRO_INIT 2720

extern YUN_MOTOR_DATA 			yunMotorData;
extern GYRO_DATA Gyro_Data;
extern VisionDataTypeDef	VisionData;
extern GYRO_DATA Gyro_Data;	//融合用

float t_yaw_error=0;	//临时测试

float t_gravity_ballistic_set_angel=0;
	
float Pixel_to_angle(s16 pix_error)	//将像素误差转换成角度
{
	float angel_error=0;
//	angel_error=0.036f*pix_error;
//	t_yaw_angel_error=atan(pix_error/938.2f)*57.3f;
angel_error=atan(pix_error/1855.2f)*57.3f;	//arm_atan_f32为DSP
	t_yaw_error=angel_error;
	return angel_error;
}

#define CAMERA_D	1855	//相机镜片到感光片等效像素量
float Pixel_V_to_angle_V(s16 pix_v,s16 pix_error)	//从最原始的数据进行计算可以减少单片机浮点运算的精度丢失（误差增加）
{
	int camera_d_2=CAMERA_D*CAMERA_D;	//距离平方
	int r_2=camera_d_2+pix_error*pix_error;	//等效半径平方
	float cos_angel_2=(float)camera_d_2/(float)r_2;
	float angel_v=0;
	angel_v=pix_v*cos_angel_2/(float)CAMERA_D;
	angel_v=angel_v*57.3f;//*2*PI;	//进行还原处理
	
//	angel_v=0.036081f*pix_v+0.1f;//+0.3026f;	//MATLAB拟合
	
	return angel_v;
}


//#define PITCH_INIT         3098	//2018.7.10	//限位用
#define YUN_DOWN_VALUELIMIT 2765	//向下限位
#define YUN_UP_VALUELIMIT 3650	//向上限位
#define YUN_UP_DISLIMIT 552	//正常的活动范围，UP为正
#define YUN_DOWN_DISLIMIT 333	//正常的活动范围，DOWN为负


#define VISION_TARX 1053//1035是修正安装偏差1020//580	//左上原点	640
#define VISION_TARY	510//490//480//490//500//520//540//560//360//410//440	//左上原点	480	//打5米内目标：向上补偿518-360个像素点	//因为有阻力恒定静态误差，故补偿
void Vision_Task(float* yaw_tarP,float* pitch_tarP)	//处理目标角度
{
	if(Error_Check.statu[LOST_VISION]==1)	VisionData.armor_sign=0;	//若无反馈=，该Task放在中断中主运行，及放在yun.c中以较慢频率保护运行
	//t_yaw_angel_v=Pixel_V_to_angle_V(VisionData.pix_x_v,(s16)(VisionData.error_x-VISION_TARX));
//	t_target_v=t_yaw_angel_v+Gyro_Data
	if(RC_Ctl.mouse.press_r==1&&VisionData.armor_sign==1)
	{
		VisionData.vision_control_state=1;	//最终控制位
	}
	else
	{
		VisionData.vision_control_state=0;	//最终控制位
	}
	
	if(VisionData.vision_control_state==1)
	{
//		t_yaw_error=Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		
//		t_yaw_error=(float)Gyro_Data.angle[2]*10-Pixel_to_angle((s16)(VisionData.error_x-VISION_TARX))*10;
//		t_pitch_error=(float)yunMotorData.pitch_fdbP+Pixel_to_angle((s16)(VisionData.error_y-VISION_TARY))*8192/360;
		*yaw_tarP=(float)Gyro_Data.angle[2]*10+Pixel_to_angle((s16)(VisionData.tar_x-VISION_TARX))*10;
		*pitch_tarP=(float)yunMotorData.pitch_fdbP-Pixel_to_angle((s16)(VisionData.tar_y-VISION_TARY))*8192/360;
//		t_gravity_ballistic_set_angel=Gravity_Ballistic_Set(pitch_tarP,(float)(VisionData.armor_dis/10.0f+0.2));	//重力补偿
		Tar_Move_Set(yaw_tarP,(float)(VisionData.armor_dis/10.0f+0.2f),VisionData.angle_x_v_filter);	//预测 待调节
		
		*pitch_tarP=*pitch_tarP>(PITCH_INIT+YUN_UP_DISLIMIT)?(PITCH_INIT+YUN_UP_DISLIMIT):*pitch_tarP;	//限制行程
		*pitch_tarP=*pitch_tarP<(PITCH_INIT-YUN_DOWN_DISLIMIT)?(PITCH_INIT-YUN_DOWN_DISLIMIT):*pitch_tarP;	//限制行程
	}
	
}

#define SHOOT_V	15	//14M/s
#define SHOOT_V_2 (SHOOT_V*SHOOT_V)
#define G	9.8f	//重力加速度
float Gravity_Ballistic_Set(float* pitch_tarP,float dis_m)	//重力补偿坐标系中，向下为正
{
	static float tar_angle_rad_fliter=0;
	float tar_angle_rad=(*pitch_tarP-PITCH_GYRO_INIT)*0.000767f;	//弧度制简化计算2pi/8192
	tar_angle_rad_fliter=0.9f*tar_angle_rad_fliter+0.1f*tar_angle_rad;
	float sin_tar_angle=sin(tar_angle_rad);
	float gravity_ballistic_angle_rad=0;	//补偿角 弧度制
	float gravity_ballistic_angle=0;	//补偿角 角度制
	gravity_ballistic_angle_rad=0.5f*(-asin((G*dis_m*(1-sin_tar_angle*sin_tar_angle)-sin_tar_angle*SHOOT_V_2)/SHOOT_V_2)+tar_angle_rad);
	gravity_ballistic_angle=gravity_ballistic_angle_rad*57.3f;

	*pitch_tarP=PITCH_GYRO_INIT-gravity_ballistic_angle*8192.0f/360;

	return gravity_ballistic_angle;
}

s16 pix_x_v_filter_10=0;
s16 tar_v_ronghe=0;
s16 tar_v_ronghe_filter=0;
#define FILTER_FACTOR	3	//5阶滤波
void Tar_Relative_V_Mix(float yaw_angvel,s16 pix_x_v)
{
	static float yaw_angvel_last[2];
//	static s16 pix_x_v_last[FILTER_FACTOR-1]={0};	//4阶中值滤波
//	static u8 last_count=0;
//	s16 pix_x_v_now_sort[FILTER_FACTOR];
//	
//	
//	for(u8 i=0;i<FILTER_FACTOR-1;i++)	//copy last data
//	{
//		  pix_x_v_now_sort[i]=pix_x_v_last[i];
//	}
//	
//	pix_x_v_now_sort[FILTER_FACTOR-1]=pix_x_v;	//copy current data
//	
//	for(u8 i=0;i<FILTER_FACTOR-1;i++)	//冒泡排序	只用循环FILTER_FACTOR-1  FILTER_FACTOR-2  FILTER_FACTOR-3 FILTER_FACTOR-4 次
//	{
//		for(u8 j=0;j<FILTER_FACTOR-1-i;j++)
//		{
//			if(pix_x_v_now_sort[j]<pix_x_v_now_sort[j+1])
//			{
//				float tem_small=pix_x_v_now_sort[j];
//				pix_x_v_now_sort[j]=pix_x_v_now_sort[j+1];
//				pix_x_v_now_sort[j+1]=tem_small;
//			}
//		}
//	}	//排序完成
//	
//	if(FILTER_FACTOR%2==1)	//中位数
//	{
//		pix_x_v_filter=pix_x_v_now_sort[(FILTER_FACTOR-1)/2];
//	}
//	else
//	{
//		pix_x_v_filter=(pix_x_v_now_sort[FILTER_FACTOR/2]+pix_x_v_now_sort[FILTER_FACTOR/2-1])/2;
//	}
	static float f_pix_x_v_filter=0;
	
	f_pix_x_v_filter=f_pix_x_v_filter*0.6f+pix_x_v*0.4f;
	
	pix_x_v_filter_10=(s16)(f_pix_x_v_filter*10);
	
	if(yaw_angvel>45)	yaw_angvel=45;	//限幅
	if(yaw_angvel<-45)	yaw_angvel=-45;
	if(pix_x_v>45)	pix_x_v=45;
	if(pix_x_v<-45)	pix_x_v=-45;
	
	VisionData.angel_x_v=10*yaw_angvel_last[1]+1.15f*pix_x_v_filter_10;	//解算得目标值
tar_v_ronghe=(s16)VisionData.angel_x_v;////////////////////////////////////////////

	VisionData.angle_x_v_filter=VisionData.angle_x_v_filter*0.6f+VisionData.angel_x_v*0.4f;
tar_v_ronghe_filter=(s16)VisionData.angle_x_v_filter;
//	if(abs(VisionData.tar_x-VISION_TARX)<50)
//	{
//		VisionData.angle_x_v_filter=VisionData.angel_x_v*(VisionData.tar_x-VISION_TARX)/50.0f;
//	}
//	else
//	{
//		VisionData.angle_x_v_filter=VisionData.angel_x_v;
//	}
	
//	pix_x_v_last[last_count]=pix_x_v;	//中位数滤波迭代
//	last_count++;
//	last_count=last_count>FILTER_FACTOR-2?0:last_count;


	//发现视觉信号延迟20ms左右，故融合用上一次
	yaw_angvel_last[1]=yaw_angvel_last[0];	//上上
	yaw_angvel_last[0]=yaw_angvel;	//上
}

void Tar_Move_Set(float* yaw_tarP,float dis_m,float tar_v)	//经过计算，只打35度/s内的物体
{
//	static float tar_v_fliter=0;
//	if(abs(tar_v)<2)	tar_v=0;
	if(tar_v>350)	tar_v=350;	//350
	if(tar_v<-350)	tar_v=-350;
//	tar_v_fliter=0.6f*tar_v_fliter+0.4f*tar_v;
	if (dis_m>2.5f)	//1.9
	{
		dis_m=2.5f;
	}
	float shoot_delay=dis_m/SHOOT_V+0.08f;	//以秒为单位
	float pre_angle=tar_v*shoot_delay;
	if(pre_angle>80)	pre_angle=80;
	if(pre_angle<-80)	pre_angle=-80;
	*yaw_tarP+=pre_angle;
}


u8 Auto_Shoot_Aimfdb(void)	//瞄准状态总	//
{
//	if(abs(VisionData.tar_y-VISION_TARY)<30&&abs(VisionData.tar_x-VISION_TARX)<5&&Error_Check.statu[LOST_VISION]==0&&abs(VisionData.angel_x_v)<40)
//	{
//		return 1;
//	}
//	else
//	{
//		return 0;
//	}
	if(VisionData.vision_control_state==1)
	{
		if(Auto_Shoot_AimAppraisal_Dynamic(VisionData.angel_x_v,VisionData.armor_dis,VisionData.tar_x-VISION_TARX)==1)
		{
			return 0;
		}
		else
		{
			return 0;
		}
	}
		
}

u8 Auto_Shoot_AimAppraisal_Static(void)	//静态瞄准评估函数
{
	u8 state=0;
	if(Error_Check.statu[LOST_VISION]==0)	//有反馈
	{
		if(abs(VisionData.tar_y-VISION_TARY)<30&&abs(VisionData.tar_x-VISION_TARX)<5)	//瞄准了
		{
			if(abs(VisionData.angel_x_v)<40)	//目标速度较小
			{
				state=1;
			}
		}
	}
	else
	{
		state=0;
	}
	
	return state;
}


/*******************************
动态瞄准评估-获得瞬时射击时间窗口
1.输入参数相对速度，因为是相对的，所以以自己为静态，目标为动态。以当前角度为实际落点
2.根据相对速度及距离（等效出时间）得到基于当前位置射击目标在子弹到达时即将到达的位置，将其与当前角度比较，若小于角度/位置某个值，则认为可以射击
等效于预测函数的反解验证
注：存在播弹延时
*******************************/

#define SHOOT_DELAY_MS 80	//以毫秒为单位
u8 Auto_Shoot_AimAppraisal_Dynamic(float relative_v,s16 dis_dm,s16 pix_error)	//动态瞄准评估函数
{
	u8 state=0;
	static u8 count=0;
	
	int delay_to_tar=dis_dm*100/SHOOT_V+SHOOT_DELAY_MS;	//延时ms
	float	angel_error=atan(pix_error/1855.2f)*57.3f;	//以度为单位
	float pre_angel_raw=delay_to_tar*relative_v/100;	//乘以以0.1度/s为单位的速度乘以ms处以100等于以度为单位
	
	if(dis_dm>40)
	{
		dis_dm=40;
	}
	
	if(abs(pre_angel_raw-angel_error)<2.2f-dis_dm*0.28f/10)	//以度为单位
	{
		count++;
		if(count>50)	count=50;
		if(count>1)	//连续两帧有效
		{
			if(abs(VisionData.tar_y-VISION_TARY)<30&&Error_Check.statu[LOST_VISION]==0)	//未丢帧、Y方向正常
			{
				state=1;
			}
		}

	}
	else
	{
		count=0;
	}
	
	return state;
}
