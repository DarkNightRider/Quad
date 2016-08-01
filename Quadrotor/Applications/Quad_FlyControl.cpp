
#include "Quad_FlyControl.h"

using namespace Quad;

namespace Quad
{
	FlyControl fc;
}

FlyControl::FlyControl()
{
	yawRate = 100;
	//����PID����
	PID_Reset();
}

//����PID����
void FlyControl::PID_Reset(void)
{
	pid[PIDROLL].set_pid(0.26, 0.15, 0.01, 200);
	pid[PIDPITCH].set_pid(0.26, 0.15, 0.01, 200);
	pid[PIDYAW].set_pid(0.8 , 0.35, 0, 200);
	pid[PIDLEVEL].set_pid(3.5, 0, 0, 0);
	pid[PIDMAG].set_pid(0.35, 0, 0, 0);
}

//��������̬�⻷����
void FlyControl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC;
	
	//����Ƕ����ֵ
	errorAngle[ROLL] = (int32_t)(constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10); 
	errorAngle[PITCH] = (int32_t)(constrain_int32(-(rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10); 
	
	//��ȡ���ٶ�
	Gyro_ADC = imu.Gyro / 5.0f;
	
	//�õ��⻷PID���,��Ϊ�ڻ�������ֵ����ȥ���ٶȲ���ֵ���õ��ڻ�PID������
	RateError[ROLL] = (int32_t)(pid[PIDLEVEL].get_p(errorAngle[ROLL]) - Gyro_ADC.x);
	RateError[PITCH] = (int32_t)(pid[PIDLEVEL].get_p(errorAngle[PITCH]) - Gyro_ADC.y);
	RateError[YAW] = (int32_t)(((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z);		
}

//��������̬�ڻ�����
void FlyControl::Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	
	for(uint8_t i=0; i<3;i++)
	{
		//�����ŵ��ڼ��ֵʱ��������
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			pid[i].reset_I();
		
		//�õ��ڻ�PID���
		PIDTerm[i] = pid[i].get_pid(RateError[i], PID_INNER_LOOP_TIME*1e-6);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -500 - abs(rc.Command[YAW]), +500 + abs(rc.Command[YAW]));	
		
	//PID���תΪ���������
	motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}	
	


