#include "super_capacitor.h"
#include "pwm.h"


extern Error_check_t Error_Check;
#define POWERLIMIT 120 	//120w功率限制
#define POWERBUFFER 60	//60J功率缓冲

u8 stateeeeeee=0;

void Super_Capacitor_Task(float power,float powerbuffer)
{
	static u32 time_record_cut=0;
	static u32 time_record_off_judge=0;
	
	if(powerbuffer<POWERBUFFER)	//20
	{
//		time_record_off_judge=time_1ms_count;
		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
		SUPERCAPACITOR_INPUT=0;
		stateeeeeee=1;
	}
	else if(powerbuffer==POWERBUFFER&&power>40)
	{
		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
		SUPERCAPACITOR_OUTPUT=0;
		SUPERCAPACITOR_INPUT=0;
		stateeeeeee=2;
	}
	else if(power<40)
	{
		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
		SUPERCAPACITOR_OUTPUT=0;
		SUPERCAPACITOR_INPUT=PWM_IO_ON;
		stateeeeeee=3;
	}
	
	if(Error_Check.statu[LOST_REFEREE]==1)	//裁判lost
	{
		SUPERCAPACITOR_JUDGE=PWM_IO_ON;
		SUPERCAPACITOR_OUTPUT=PWM_IO_ON;
		SUPERCAPACITOR_INPUT=0;
		stateeeeeee=4;
	}
}



