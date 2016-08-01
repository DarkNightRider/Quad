/*******************************************************************************
 *Copyright (C), 2016.
 *@file 	Quad_Config.c UTF-8
 *@brief 	the Implementation of Quad class
 *@author	Xinyu Han
 *@version	1.0.0
 *@date		2016.03.15
 *@warning	none
 *@history	none
 ******************************************************************************/

#include "Quad_Config.h"

using namespace Quad;



namespace Quad
{
	Quadrotor quadrotor;
}


Quadrotor::Quadrotor(void)
{
	//make the connection of the modules and the quadrotor with pointers
	pIMU 		= &(Quad::imu);
	pMotor 		= &(Quad::motor);
	pDT 		= &(Quad::dt);
	pFlyControl 	= &(Quad::fc);
	pParam 		= &(Quad::param);
	pScheduler 	= &(Quad::scheduler);
	pRC 		= &(Quad::rc);
}

void Quadrotor::Init()
{
	//init each module of the quadrotor
	//the order of the callings of these function
	//better not be changed
	pDT->Init();
	pMotor->Init();
	pParam->Init();
	pIMU->Init();
	
}

void Quadrotor::Fly()
{
	//Multiple tasks are executed in the Scheduler's Loop.
	while(1)
		pScheduler->Loop();	
}


void Quadrotor::Pilot_Light(void)
{
	static uint8_t cnt = 0;
	//signal light turnning accroding to the ARMED status.
	if(f.ARMED)
	{
		cnt++;
		switch(cnt)
		{
			case 1:
				led.ON();
				break;
			case 10:
				led.OFF();
				break;
			case 20:
				cnt = 0;
				break;
		}
	}
	else
	{
		led.ON(); 
	}
	
}


