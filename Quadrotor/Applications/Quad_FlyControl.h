#pragma once

#include "Quad_Config.h"

#define FLYANGLE_MAX 350  //最大飞行倾角25度

namespace Quad
{
	enum {
	    PIDROLL,
	    PIDPITCH,
	    PIDYAW,
			PIDVELX,
			PIDVELY,
	    PIDVELZ,
	    PIDALT,
	    PIDLEVEL,
	    PIDMAG,
			PIDITEMS
	};

	class FlyControl
	{

	public:
		
		PID pid[PIDITEMS];

		FlyControl();

		//姿态外环控制
		void Attitude_Outter_Loop(void);

		//姿态内环控制
		void Attitude_Inner_Loop(void);

		void PID_Reset(void);
			
	private:
		
		uint8_t yawRate;
		int32_t RateError[3];

	};

	extern FlyControl fc;
}

























