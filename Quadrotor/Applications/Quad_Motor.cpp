
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
	//����X��
	motorPWM[0] = throttle - pidTermRoll - pidTermPitch + pidTermYaw; //����
	motorPWM[1] = throttle - pidTermRoll + pidTermPitch - pidTermYaw; //ǰ��
	motorPWM[2] = throttle + pidTermRoll - pidTermPitch - pidTermYaw; //����
	motorPWM[3] = throttle + pidTermRoll + pidTermPitch + pidTermYaw; //ǰ��
	
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
		//���Ƶ��PWM����С�����ֵ
		motorPWM[i] = constrain_int16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}	

	//���δ�������򽫵���������Ϊ���
	if(!quadrotor.f.ARMED || rc.rawData[THROTTLE] < 1100)	
		for(uint8_t i=0; i< 4 ; i++)
			motorPWM[i] = 1000;

	//д����PWM
	pPwm->SetPwm(motorPWM);
	
}

void Motor::getPWM(uint16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
}



