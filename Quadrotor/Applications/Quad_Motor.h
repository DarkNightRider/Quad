#pragma once

#include "Quad_Config.h"


#define MINTHROTTLE 1100
#define MAXTHROTTLE 2000

namespace Quad
{
  	class PWM;
  
	class Motor
	{

	public:
		Motor();
		
		void Init();
	  
		void writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw);
		
		void getPWM(uint16_t* pwm);
		
		PWM * pPwm;

	private:
		
		uint16_t motorPWM[4];	
		
	};

	extern Motor motor;
}







