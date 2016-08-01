#pragma once
#include "Quad_Config.h"

namespace Quad
{
	class Scheduler
	{
	public:
		//���캯��
		Scheduler();
	
		void Loop(void);
		//����ʱ����������
		uint16_t cnt_1ms,cnt_2ms,cnt_5ms,cnt_10ms,cnt_20ms;

		uint32_t idleTime;
	};

	

	extern Scheduler scheduler;
}












