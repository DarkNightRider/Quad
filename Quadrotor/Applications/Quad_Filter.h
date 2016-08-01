#pragma once

#include "Quad_Math.h"


namespace Quad
{
	class Filter
	{
		
	public:

		Filter(){
		}
		
		struct LPF2ndData_t
		{
			float b0;
			float a1;
			float a2;
			Vector3f preout;
			Vector3f lastout;
		};
		
		float Acc_lpf_1st;
		
		LPF2ndData_t Acc_lpf_2nd;
		
		float Gyro_cf;
		
		void init();
		
		//1st-order low-pass filter coefficient calculation
		float LPF_1st_Factor_Cal(float deltaT, float Fcut);
		
		//2nd-order low-pass filter coefficient calculation
		void LPF_2nd_Factor_Cal(float deltaT, float Fcut, LPF2ndData_t* lpf_data);
		
		//Complementary filter coefficient calculation
		float CF_Factor_Cal(float deltaT, float tau);
		
		//1st low-pass filter
		Vector3f LPF_1st(Vector3f oldData, Vector3f newData, float lpf_factor);
		
		//2nd low-pass filter
		Vector3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData);

		
		//1st-order complementary filter
		Vector3f CF_1st(Vector3f gyroData, Vector3f accData, float cf_factor);
		


	};
	
	extern Filter filter;
}


