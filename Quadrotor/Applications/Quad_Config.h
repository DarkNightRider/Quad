/*******************************************************************************
 *Copyright (C), 2016.
 *@file 	Quad_Config.h UTF-8
 *@brief 	Quadrotor configuration and Quad class defination
 *@author	Xinyu Han
 *@version	1.0.0
 *@date		2016.03.15
 *@warning	none
 *@history	none
 ******************************************************************************/

#pragma once

#include "board.h"
#include "Quad_PID.h"
#include "Quad_Filter.h"
#include "Quad_IMU.h"
#include "Quad_Scheduler.h"
#include "Quad_DT.h"
#include "Quad_Motor.h"
#include "Quad_RC.h"
#include "Quad_FlyControl.h"
#include "Quad_Param.h"

/*----------------------IMU--------------------*/
#define IMU_USE_DCM_CF				//use direction cosine matrix
//#define IMU_USE_Quaternions_CF		//use Quadernions 

//#define IMU_USE_LPF_1st			//use 1st order LPF
#define IMU_USE_LPF_2nd				//use 2nd order LPF

#define IMU_LOOP_TIME			2000	//unit uS
#define PID_INNER_LOOP_TIME		2000	//unit us
#define PID_OUTER_LOOP_TIME		5000	//uint us

#define ACC_1G 				4096	//the range of acc
#define ACC_LPF_CUT 			10.0f	//acc LPF cutoff frequency

#define GYRO_CF_TAU 1.2f
/*---------------------------------------------*/

/*-------choose way to transform data----------*/

#define DT_USE_NRF24l01
/*---------------------------------------------*/

namespace Quad
{
	class IMU;
	class Motor;
	class DT;
	class FlyControl;
	class Param;
	class Scheduler;
	class RC;
	
	class Quadrotor
	{
	public:
		//pointer of each module of the quadrotor
	  	IMU * 		pIMU;
		Motor * 	pMotor;
		DT * 		pDT;
		FlyControl *	pFlyControl;
		Param *		pParam;
		Scheduler * 	pScheduler;
		RC * 		pRC;
		
	  
		Quadrotor();
		
		//init quadrotor
		void Init(void);
		//loop of regular fly
		void Fly(void);
		
		class Factor{
			public:		
				float acc_lpf;		
				float gyro_cf;		
		}factor;
		
		//flag inner class
		class Flag{
			public:
				//if ARMED is set, the output of motor will work
				uint8_t ARMED;	
				//if failsafe is set, indicate that the attitude of the quadrotor is unnormal
				//the output of the motor will be shutdown softly
				uint8_t failsafe;
		}f;
		
		//signal light
		void Pilot_Light(void);
		
		
	};

	extern Quadrotor quadrotor;
}



