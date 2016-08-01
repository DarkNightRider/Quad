
#include "Quad_Motor.h"

using namespace Quad;

namespace Quad
{
	Motor motor;
}


Motor::Motor()
{
	this->pPwm = &(Quad::pwm);
}

void Motor::Init()
{
	pPwm->Init();
}

void Motor::writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	//四轴X型
	motorPWM[0] = throttle - pidTermRoll - pidTermPitch + pidTermYaw; //后右
	motorPWM[1] = throttle - pidTermRoll + pidTermPitch - pidTermYaw; //前右
	motorPWM[2] = throttle + pidTermRoll - pidTermPitch - pidTermYaw; //后左
	motorPWM[3] = throttle + pidTermRoll + pidTermPitch + pidTermYaw; //前左
	
	int16_t maxMotor = motorPWM[0];
	for (uint8_t i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (uint8_t i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//限制电机PWM的最小和最大值
		motorPWM[i] = constrain_int16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}	

	//如果未解锁，则将电机输出设置为最低
	if(!quadrotor.f.ARMED || rc.rawData[THROTTLE] < 1100)	
		for(uint8_t i=0; i< 4 ; i++)
			motorPWM[i] = 1000;

	//写入电机PWM
	pPwm->SetPwm(motorPWM);
	
}

void Motor::getPWM(uint16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
}



