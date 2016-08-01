#pragma once

#include "Quad_Math.h"


namespace Quad
{
	class AtSensor
	{
	public:
		virtual void init() = 0;
		virtual int8_t updateAcc() = 0;
		virtual int8_t updateGyro() = 0;
		virtual Vector3f getAcc() = 0;
		virtual Vector3f getGyro() = 0;
		virtual Vector3f getGyroInDeg() = 0;
		virtual Vector3f getGyroInRad() = 0;
		virtual int8_t updateMagn();
		virtual Vector3f getMagn();
	protected:
		Vector3f Acc_ADC,Gyro_ADC;
		Vector3f Gyro_deg;
		Vector3f Gyro_rad;
	};
};