#pragma once

#include "Quad_Config.h"

#include "Quad_AtSensor.h"

namespace Quad
{
	class AtSensor;
}


namespace Quad
{
	class IMU
	{
		
	public:

		IMU();
		
		//欧拉角表示的飞行器姿态
		Vector3f angle;
		
		Vector3f Gyro, Acc, Acc_lpf; 
		
		AtSensor * pSensor;
	
		Filter * pFilter;
	
		float magHold, headFreeHold;
		
	
	
		void Init();
		
		//更新传感器数据
		void updateSensor();	
		
		//计算飞行器姿态
		void getAttitude();
			
	private:

		Quaternion Q;

		int32_t accRatio;

		int8_t TickTimer;

		//基于余弦矩阵和互补滤波的姿态解算
		void DCM_CF(Vector3f gyro,Vector3f acc, float deltaT);
		//基于四元数和互补滤波的姿态解算
		void Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT);

		//滤波器参数初始化
		void filter_Init();
		//传感器初始化
		void sensor_Init();

	};

	extern IMU imu;
}


