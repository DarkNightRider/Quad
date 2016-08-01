#pragma once

#include "Quad_Config.h"

#define RC_MID  			1500                     
#define RC_MINCHECK		1150                      
#define RC_MAXCHECK  	1850 
#define RC_MINTHROTTLE	1150                      
#define RC_MAXTHROTTLE 	1850 

namespace Quad
{
	enum {
	    ROLL = 0,
	    PITCH,
	    YAW,
	    THROTTLE,
	    AUX1,
	    AUX2,
	    AUX3,
	    AUX4,
			AUX5,
			AUX6
	};

	class RC
	{
		
	public:
		
		RC();

		uint16_t rawData[10];
		int16_t Command[4];

		//������ʼ��
		void Init(void);
		//ң��ͨ�����ݴ���
		void Cal_Command(void);
		//ҡ��λ�ü��
		void check_sticks(void);

	private:

	};

	extern RC rc;
}




