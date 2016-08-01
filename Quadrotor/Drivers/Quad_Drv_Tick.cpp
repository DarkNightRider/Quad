#include "Quad_Drv_Tick.h"
#include <stdint.h>

using namespace Quad;

namespace Quad
{
	Tick tick_timer;
}



Tick::Tick()
{
	timer_bits = 0x0000;
	timer_cnt = 0;
	
}

float Tick::get_dT(uint8_t Timer)
{
	uint32_t ms=HAL_GetTick();
	uint32_t val=SysTick->VAL;
	float dt = ((ms-last_mss[Timer])*72e3 - val + last_vals[Timer])/72e6;
	last_mss[Timer] = ms;
	last_vals[Timer] = val;
	return dt;
}

int8_t Tick::getTickTimer(void)
{
	if(timer_cnt<10)
	{
		for(uint8_t timer = 0; timer<10; timer++)
		{
			if(!(timer_bits & 0x0001<<timer))
			{
				timer_bits |= 0x0001<<timer;
				timer_cnt++;
				last_mss[timer]=0;
				last_vals[timer]=71999;
				return timer;
			}
		}
	}
	return -1;
}


int8_t Tick::reclaimTickTimer(uint8_t Timer)
{
	if(timer_cnt>0 && timer_bits & 0x0001<<Timer)
	{
		timer_bits &= ~(0x0001 << Timer);
		timer_cnt--;
		return Timer;
	}
	else
		return -1;
}





