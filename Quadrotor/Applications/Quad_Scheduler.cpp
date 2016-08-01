
#include "Quad_Scheduler.h"

using namespace Quad;

char * str = "Hello";

namespace Quad
{
	Scheduler scheduler;
}


static void ANO_Loop_1000Hz(void)	//1msִ��һ��
{
	//����Ƿ��н��յ���������
	dt.Check_Event();
	//���ͷ���������
	dt.Data_Exchange();
	
}

static void ANO_Loop_500Hz(void)	//2msִ��һ��
{	
	//���´���������	
	imu.updateSensor();		
	
	//�����������̬
	imu.getAttitude();
	
	//��������̬�ڻ�����
	fc.Attitude_Inner_Loop();	

}

static void ANO_Loop_200Hz(void)	//5msִ��һ��
{
	//��������̬�⻷����
	fc.Attitude_Outter_Loop();	
}

static void ANO_Loop_100Hz(void)	//10msִ��һ��
{
	
}

static void ANO_Loop_50Hz(void)	//20msִ��һ��
{
	//ң��ͨ�����ݴ���
	rc.Cal_Command();
	
	//ҡ��λ�ü��
	rc.check_sticks();
	
	//ʧ�ر������
	dt.Failsafe_Check();
	
	//LEDָʾ�ƿ���
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



