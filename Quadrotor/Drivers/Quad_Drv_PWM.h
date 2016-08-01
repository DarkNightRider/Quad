#pragma once

#include "board.h"


#define MAXMOTORS 4

namespace Quad
{
	class PWM
	{

	public:
		void Init(void);
		void SetPwm(uint16_t pwm[MAXMOTORS]);
		
	};
	
	extern Quad::PWM pwm;
}


