#pragma once

#include "board.h"
namespace Quad
{
	class Tick
	{
		public:
			inline void calculate_elapsed_time_start(void)
			{
				_ms=HAL_GetTick();
				_val=SysTick->VAL;
			}

			inline float calculate_elapsed_time_end(void)
			{
				return (((HAL_GetTick()-_ms)*72e3 - SysTick->VAL + _val)/72e6);
			}
			Tick();
			float get_dT(uint8_t Timer);
			int8_t getTickTimer(void);
			int8_t reclaimTickTimer(uint8_t Timer);
			
		private:
			uint8_t timer_cnt;
			uint16_t timer_bits;
			uint32_t _ms;
			uint32_t _val;	
			uint32_t last_mss[10];
			uint32_t last_vals[10];
			
	};
	extern Tick tick_timer;
}




