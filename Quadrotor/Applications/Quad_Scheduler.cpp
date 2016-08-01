
#include "Quad_Scheduler.h"

using namespace Quad;

char * str = "Hello";

namespace Quad
{
	Scheduler scheduler;
}


static void ANO_Loop_1000Hz(void)	//1ms执行一次
{
	//检查是否有接收到无线数据
	dt.Check_Event();
	//发送飞行器数据
	dt.Data_Exchange();
	
}

static void ANO_Loop_500Hz(void)	//2ms执行一次
{	
	//更新传感器数据	
	imu.updateSensor();		
	
	//计算飞行器姿态
	imu.getAttitude();
	
	//飞行器姿态内环控制
	fc.Attitude_Inner_Loop();	

}

static void ANO_Loop_200Hz(void)	//5ms执行一次
{
	//飞行器姿态外环控制
	fc.Attitude_Outter_Loop();	
}

static void ANO_Loop_100Hz(void)	//10ms执行一次
{
	
}

static void ANO_Loop_50Hz(void)	//20ms执行一次
{
	//遥控通道数据处理
	rc.Cal_Command();
	
	//摇杆位置检查
	rc.check_sticks();
	
	//失控保护检查
	dt.Failsafe_Check();
	
	//LED指示灯控制
	quadrotor.Pilot_Light();
}

void Scheduler::Loop(void)
{
	
	if(scheduler.cnt_1ms >= 1){	
		scheduler.cnt_1ms = 0;
		ANO_Loop_1000Hz();
	}
	if(scheduler.cnt_2ms >= 2){
		scheduler.cnt_2ms = 0;
		ANO_Loop_500Hz();
		
	}		
	if(scheduler.cnt_5ms >= 5){	
		scheduler.cnt_5ms = 0;
		ANO_Loop_200Hz();
	}
	if(scheduler.cnt_10ms >= 10){
		scheduler.cnt_10ms = 0;
		ANO_Loop_100Hz();
	}
	if(scheduler.cnt_20ms >= 20){		
		scheduler.cnt_20ms = 0;
		ANO_Loop_50Hz();	
	}
	
}

Scheduler::Scheduler()
{
	cnt_1ms = cnt_2ms = cnt_5ms = cnt_10ms = cnt_20ms = 0;
}



