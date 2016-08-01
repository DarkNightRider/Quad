
#include "Quad_Drv_PWM.h"

using namespace Quad;


namespace Quad
{
	PWM pwm;
}

extern TIM_HandleTypeDef htim2;

void PWM::Init()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}



void PWM::SetPwm(uint16_t pwm[MAXMOTORS])
{
	for(uint8_t i=0;i<MAXMOTORS;i++)
	{
		if(pwm[i] < 1000)
			pwm[i] = 1000;
		else if(pwm[i] > 2000)
			pwm[i] = 2000;
	}
	
	TIM2->CCR1 = pwm[0] - 1000; 
	TIM2->CCR2 = pwm[1] - 1000;	
	TIM2->CCR3 = pwm[2] - 1000; 
	TIM2->CCR4 = pwm[3] - 1000;
	
}

