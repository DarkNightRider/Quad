#pragma once

#include "board.h"

#include "Quad_AtSensor.h"

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400

namespace Quad
{
	class MPU6050:public AtSensor
	{
	public:
		
		MPU6050(I2C_HandleTypeDef * pI2C);

		uint8_t Acc_CALIBRATED;
		uint8_t Gyro_CALIBRATED;
		Vector3i Acc_Offset,Gyro_Offset;

		//initialize 6050
		void Init(uint16_t sample_rate, uint16_t lpf);
		//start continuous data auto updata
		void DataUpdateStart(void);
		//data transmit complete callback function
		void TransCplCall(I2C_HandleTypeDef *hi2c);
		
		
		void init();
		int8_t updateAcc();
		int8_t updateGyro();
		Vector3f getAcc();
		Vector3f getGyro();
		Vector3f getGyroInDeg();
		Vector3f getGyroInRad();
		
		I2C_HandleTypeDef * pI2C;
	private:
		
		
		
		//data buffer
		uint8_t mpu6050_buffer[14];
		//update begin start flag
		uint8_t IsDataUpdateStart; 
		//calibrate acc zero offside
		void CalOffset_Acc(void);
		//calibrate gyro zero offside
		void CalOffset_Gyro(void);

		void delayms(uint16_t ms);
	};

	
	extern MPU6050 mpu6050;

}








