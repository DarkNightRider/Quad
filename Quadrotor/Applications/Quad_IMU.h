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
		
		//ŷ���Ǳ�ʾ�ķ�������̬
		Vector3f angle;
		
		Vector3f Gyro, Acc, Acc_lpf; 
		
		AtSensor * pSensor;
	
		Filter * pFilter;
	
		float magHold, headFreeHold;
		
	
	
		void Init();
		
		//���´���������
		void updateSensor();	
		
		//�����������̬
		void getAttitude();
			
	private:

		Quaternion Q;

		int32_t accRatio;

		int8_t TickTimer;

		//�������Ҿ���ͻ����˲�����̬����
		void DCM_CF(Vector3f gyro,Vector3f acc, float deltaT);
		//������Ԫ���ͻ����˲�����̬����
		void Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT);

		//�˲���������ʼ��
		void filter_Init();
		//��������ʼ��
		void sensor_Init();

	};

	extern IMU imu;
}


