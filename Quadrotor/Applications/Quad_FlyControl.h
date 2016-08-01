#pragma once

#include "Quad_Config.h"

#define FLYANGLE_MAX 350  //���������25��

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

		//��̬�⻷����
		void Attitude_Outter_Loop(void);

		//��̬�ڻ�����
		void Attitude_Inner_Loop(void);

		void PID_Reset(void);
			
	private:
		
		uint8_t yawRate;
		int32_t RateError[3];

	};

	extern FlyControl fc;
}

























