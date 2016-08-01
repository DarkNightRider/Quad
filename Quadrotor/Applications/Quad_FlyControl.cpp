
#include "Quad_FlyControl.h"

using namespace Quad;

namespace Quad
{
	FlyControl fc;
}

FlyControl::FlyControl()
{
	yawRate = 100;
	//重置PID参数
	PID_Reset();
}

//重置PID参数
void FlyControl::PID_Reset(void)
{
	pid[PIDROLL].set_pid(0.26, 0.15, 0.01, 200);
	pid[PIDPITCH].set_pid(0.26, 0.15, 0.01, 200);
	pid[PIDYAW].set_pid(0.8 , 0.35, 0, 200);
	pid[PIDLEVEL].set_pid(3.5, 0, 0, 0);
	pid[PIDMAG].set_pid(0.35, 0, 0, 0);
}

//飞行器姿态外环控制
void FlyControl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC;
	
	//计算角度误差值
	errorAngle[ROLL] = (int32_t)(constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10); 
	errorAngle[PITCH] = (int32_t)(constrain_int32(-(rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10); 
	
	//获取角速度
	Gyro_ADC = imu.Gyro / 5.0f;
	
	//得到外环PID输出,作为内环的期望值，减去角速度测量值，得到内环PID的输入
	RateError[ROLL] = (int32_t)(pid[PIDLEVEL].get_p(errorAngle[ROLL]) - Gyro_ADC.x);
	RateError[PITCH] = (int32_t)(pid[PIDLEVEL].get_p(errorAngle[PITCH]) - Gyro_ADC.y);
	RateError[YAW] = (int32_t)(((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z);		
}

//飞行器姿态内环控制
void FlyControl::Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	
	for(uint8_t i=0; i<3;i++)
	{
		//当油门低于检查值时积分清零
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			pid[i].reset_I();
		
		//得到内环PID输出
		PIDTerm[i] = pid[i].get_pid(RateError[i], PID_INNER_LOOP_TIME*1e-6);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -500 - abs(rc.Command[YAW]), +500 + abs(rc.Command[YAW]));	
		
	//PID输出转为电机控制量
	motor.writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}	
	


