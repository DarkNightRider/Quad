
#include "Quad_IMU.h"

using namespace Quad; 
namespace Quad
{
	IMU imu;
}

//extern Tick tick_timer;

IMU::IMU()
{
	this->pFilter = &(Quad::filter);
	this->pSensor = &(Quad::mpu6050);
	
}

//IMU��ʼ��
void IMU::Init()
{
	//�˲���������ʼ��
	filter_Init();
	//��������ʼ��
	sensor_Init();	
	TickTimer = tick_timer.getTickTimer();
}

//���´���������
void IMU::updateSensor()
{
	//��ȡ���ٶ�
	pSensor->updateAcc();
	//��ȡ���ٶ�
	pSensor->updateGyro();	
	//��ȡ���ٶȣ���λΪ��ÿ��
	Gyro = pSensor->getGyro();
	//��ȡ���ٶȲ���ֵ
	Acc = pSensor->getGyro();
}


//�����������̬
void IMU::getAttitude()
{
	float deltaT;
	Vector3d accTemp, gyroTemp;
	
#ifdef IMU_USE_LPF_1st	
	//���ٶ�����һ�׵�ͨ�˲�
	Acc_lpf = LPF_1st(Acc_lpf, Acc,  pFilter->Acc_lpf_1st);
#endif	
	
#ifdef IMU_USE_LPF_2nd	
	//���ٶ����ݶ��׵�ͨ�˲�
	Acc_lpf = pFilter->LPF_2nd(&(pFilter->Acc_lpf_2nd), Acc);
#endif

	//����ʵ�ʲ����ļ��ٶȺ��������ٶȵı�ֵ
	accRatio = (int32_t)(Acc_lpf.length_squared() * 100 / (ACC_1G * ACC_1G));		
	
	deltaT = tick_timer.get_dT(TickTimer);
	
#ifdef IMU_USE_DCM_CF
	DCM_CF(pSensor->getGyroInRad(),Acc_lpf,deltaT);
#endif
#ifdef IMU_USE_Quaternions_CF
	Quaternion_CF(pSensor->getGyroInRad(),Acc_lpf,deltaT);
#endif

}


//���Ҿ��������̬
void IMU::DCM_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	static Vector3f deltaGyroAngle, LastGyro;
	static Vector3f Vector_G(0, 0, ACC_1G), Vector_M(1000, 0, 0);
	Matrix3f dcm;
	
	//���������ǽǶȱ仯�����������������	
	deltaGyroAngle = (gyro + LastGyro) * 0.5 * deltaT;
	LastGyro = gyro;
	
	//�����ʾ������ת�����Ҿ���
	dcm.from_euler(deltaGyroAngle);
	
	//�������Ҿ���������������ڻ�������ϵ��ͶӰ
	Vector_G = dcm * Vector_G;
	
	//�������Ҿ�����µش������ڻ�������ϵ��ͶӰ
	Vector_M = dcm * Vector_M;
	
	//�����˲���ʹ�ü��ٶȲ���ֵ�������ٶȻ���Ư��
	if (50 < (uint16_t)accRatio && (uint16_t)accRatio < 150) 
	{
		Vector_G = pFilter->CF_1st(Vector_G, acc, pFilter->Gyro_cf);
	}

	//�����������ROLL��PITCH
	Vector_G.get_rollpitch(angle);	
	
	//�����������YAW
	Vector_M.get_yaw(angle);
}


#define Kp 0.64f        //���ٶ�Ȩ�أ�Խ��������ٶȲ���ֵ����Խ��
#define Ki 0.00032f      //����������
//��Ԫ��������̬
void IMU::Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	Vector3f V_gravity, V_error; 
	static Vector3f V_error_I(0, 0, 0);
	
	if(accRatio > 5)
	{
		//�������ٶȹ�һ��
		acc.normalize();
		
		//��ȡ��Ԫ���ĵ�Ч���Ҿ����е���������
		Q.vector_gravity(V_gravity);
		
		//��������ó���̬���
		V_error = acc % V_gravity;
		V_error = V_error / deltaT;
		
		//�������л���	
		V_error_I += V_error * Ki;
		
		//�����˲�����̬���������ٶ��ϣ��������ٶȻ���Ư��
		gyro = gyro  + V_error * Kp + V_error_I;		
	}
	//һ�����������������Ԫ��
	Q.Runge_Kutta_1st(gyro, deltaT);
	
	//��Ԫ����һ��
	Q.normalize();
	
	//��Ԫ��תŷ����
	Q.to_euler(&angle.x, &angle.y, &angle.z);
}


void IMU::filter_Init() 
{
	//���ٶ�һ�׵�ͨ�˲���ϵ������
	//quadrotor.factor.acc_lpf = pFilter->LPF_1st_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT);
	pFilter->Acc_lpf_1st = pFilter->LPF_1st_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT);
	//���ٶȶ��׵�ͨ�˲���ϵ������
	pFilter->LPF_2nd_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT, &(pFilter->Acc_lpf_2nd));
	
	
	//�����˲���ϵ������
	//quadrotor.factor.gyro_cf = pFilter->CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);
	pFilter->Gyro_cf = pFilter->CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);
}

void IMU::sensor_Init()
{
	//��ʼ��MPU6050��1Khz������42Hz��ͨ�˲�
	pSensor->init();
}



