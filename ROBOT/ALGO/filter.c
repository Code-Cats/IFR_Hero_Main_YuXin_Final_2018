#include "filter.h"




/*********************************************
函数名称：FirstOrder_General
函数功能：通用一阶滞后滤波算法
函数参数：当前值：cur_val
函数返回值：输出值：output
**********************************************/
float FirstOrder_General(const s32 cur_val,FIRST_ORDER_FILTER*filter)
{
	if(abs(cur_val)>filter->tiny)	//计算死区
	{
		filter->Data_Save[1]=cur_val;
	}
	else
	{
		filter->Data_Save[1]=0;
	}
	
	if(abs(filter->Data_Save[1])>=abs(filter->Data_Save[0]))	//这一次的大
	{
		if(filter->up_flag==1)
		{
			filter->output=(1-filter->ratio)*filter->Data_Save[0]+filter->ratio*filter->Data_Save[1];
		}
		else
		{
			filter->output=filter->Data_Save[1];
		}
	}
	else
	{
		if(filter->down_flag==1)
		{
			filter->output=(1-filter->ratio)*filter->Data_Save[0]+filter->ratio*filter->Data_Save[1];
		}
		else
		{
			filter->output=filter->Data_Save[1];
		}
	}
	
	filter->Data_Save[0]=filter->output;

	return filter->output;
}



///**************************************************************
//函数名称：Filter_firstOrder
//函数功能：改良一阶滞后滤波
//          将阶跃信号变为斜坡型信号
//函数参数：当前参数输入值：input
//          滤波结构体参数：FIRST_ORDER_FILTER
//函数返回值：处理后的输出值：output
//描述：记录当前值和上一次值，根据权重得到输出值
//      可选上升或下降是否需要滤波
//***************************************************************/
//float Filter_firstOrder(int16_t input,FIRST_ORDER_FILTER *ray)
//{
//	  //输入数据大小限制
//		if(input>ray->input_max){input=ray->input_max;}
//		if(input<ray->input_min){input=ray->input_min;}
//    ray->Data_Save[1]=input;
//	  //上升是否需要滤波 Flag==1 需要 反之则反
//		if(abs(ray->Data_Save[1])>=abs(ray->Data_Save[0]))
//		{
//			 if(ray->up_flag==1)
//			 {
//			     ray->Data_Save[1]=ray->Data_Save[0] * ray->former + ray->Data_Save[1] * ray->later;
//			 }
//		}
//	  else
//	  //下降是否需要滤波 Flag==1 需要 反之则反
//		{
//			 if(ray->down_flag==1)
//			 {
//				  ray->Data_Save[1]=ray->Data_Save[0] * ray->former + ray->Data_Save[1] * ray->later;
//				//下降时较小值归零
//				  if((ray->Data_Save[1] >=- ray->back)&&(ray->Data_Save[1] <= ray->back))
//				  ray->Data_Save[1]=0;	
//			 }
//	  }
//		//输出数据大小限制
//		if(ray->Data_Save[1]>ray->output_max)  {ray->Data_Save[1]=ray->output_max;}
//		if(ray->Data_Save[1]<ray->output_min)  {ray->Data_Save[1]=ray->output_min;}
//		
//		ray->output=ray->Data_Save[1];
//    ray->Data_Save[0]=ray->Data_Save[1];
//		return ray->output;
//}

/*************************************
函数名称:Filter_MPU6050_Accel
函数功能：对MPU6050数据滤波
函数参数：MPU6050当前输入值：Accel
         权重参数：param
函数返回值：无
描述：无
**************************************/
float	Filter_MPU6050_Accel(float Accel,float param)
{			
		static float accel[2]= {0.0,0.0};
		static float a_rpm=0;
		float output_accel;
		accel[0]=accel[1];
		accel[1]=Accel;
		a_rpm=accel[0]*(1-param)+accel[1]*param;
		output_accel=((60*abs(a_rpm)*9.8f*19)/(13.24f*PI*0.1510f));
    return output_accel;
}


