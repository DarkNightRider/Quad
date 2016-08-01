
#include "Quad_RC.h"

using namespace Quad;

namespace Quad
{

	RC rc;
}

RC::RC()
{
	Init();
}

void RC::Init(void)
{
	
}

void RC::Cal_Command(void)
{
	for (uint8_t i = 0; i < 3; i++) 
	{	//ROLL,PITCH,YAW process datas from these three directions data
		Command[i] = (rawData[i] - RC_MID) * 0.35;
	}	
	Command[YAW] = -Command[YAW];
	Command[THROTTLE] = rawData[THROTTLE];
		
		//-------------------lock of yaw------------------
		if (abs(Command[YAW]) < 70 && rawData[THROTTLE] > RC_MINCHECK) 
		{
			int16_t dif = imu.angle.z - imu.magHold;
			if (dif <= -180)
				dif += 360;
			if (dif >= +180)
				dif -= 360;
			dif = -dif;
			
			Command[YAW] -= dif * fc.pid[PIDMAG].kP * 0.1;  	
		} 	
		else
			imu.magHold = imu.angle.z;	
		
}


const uint8_t stick_min_flag[4] = {1<<0,1<<2,1<<4,1<<6}; 
const uint8_t stick_max_flag[4] = {1<<1,1<<3,1<<5,1<<7};
#define ROL_L 0x01
#define ROL_H 0x02
#define PIT_L	0x04
#define PIT_H 0x08
#define YAW_L 0x10
#define YAW_H 0x20
#define THR_L 0x40
#define THR_H 0x80

void RC::check_sticks(void)
{
	int i;
	static uint8_t rcDelayCommand;
	
	static uint8_t stick_flag = 0;

	for (i = 0; i < 4; i++) 
	{
		//if the stick is not on the normal position,
		//exit the checking.
		if(rawData[i]<900||rawData[i]>2000)	
			break;
		
		if (rawData[i] < RC_MINCHECK)
				stick_flag |= stick_min_flag[i];  // check for MIN
		else
				stick_flag &= ~stick_min_flag[i];
		
		if (rawData[i] > RC_MAXCHECK)
				stick_flag |= stick_max_flag[i];  // check for MAX
		else
				stick_flag &= ~stick_max_flag[i];  // check for MAX
	}	
	//if one of the stricks is on the MAX or MIN position.
	if(stick_flag&0xff)	
	{
		if(rcDelayCommand < 250)
			rcDelayCommand++;
	}
	else
	{
		rcDelayCommand = 0;
		stick_flag &= 0;
	}	
		
	if (rcDelayCommand == 100) //2s: 20ms * 100
	{
		if (quadrotor.f.ARMED) //if the quad has been unlocked
		{ 
			if((stick_flag & YAW_L)&&(stick_flag & THR_L))
			{
				quadrotor.f.ARMED = 0;	//lock on
			}
		}
		else
		{
			if((stick_flag & YAW_H)&&(stick_flag & THR_L))
			{
				quadrotor.f.ARMED = 1;	//unlock
			}
		}
		stick_flag &= 0;
		rcDelayCommand = 0;
	}
}


