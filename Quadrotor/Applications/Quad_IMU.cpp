
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

//IMU初始化
void IMU::Init()
{
	//滤波器参数初始化
	filter_Init();
	//传感器初始化
	sensor_Init();	
	TickTimer = tick_timer.getTickTimer();
}

//更新传感器数据
void IMU::updateSensor()
{
	//读取加速度
	pSensor->updateAcc();
	//读取角速度
	pSensor->updateGyro();	
	//获取角速度，单位为度每秒
	Gyro = pSensor->getGyro();
	//获取加速度采样值
	Acc = pSensor->getGyro();
}


//计算飞行器姿态
void IMU::getAttitude()
{
	float deltaT;
	Vector3d accTemp, gyroTemp;
	
#ifdef IMU_USE_LPF_1st	
	//加速度数据一阶低通滤波
	Acc_lpf = LPF_1st(Acc_lpf, Acc,  pFilter->Acc_lpf_1st);
#endif	
	
#ifdef IMU_USE_LPF_2nd	
	//加速度数据二阶低通滤波
	Acc_lpf = pFilter->LPF_2nd(&(pFilter->Acc_lpf_2nd), Acc);
#endif

	//计算实际测量的加速度和重力加速度的比值
	accRatio = (int32_t)(Acc_lpf.length_squared() * 100 / (ACC_1G * ACC_1G));		
	
	deltaT = tick_timer.get_dT(TickTimer);
	
#ifdef IMU_USE_DCM_CF
	DCM_CF(pSensor->getGyroInRad(),Acc_lpf,deltaT);
#endif
#ifdef IMU_USE_Quaternions_CF
	Quaternion_CF(pSensor->getGyroInRad(),Acc_lpf,deltaT);
#endif

}


//余弦矩阵更新姿态
void IMU::DCM_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	static Vector3f deltaGyroAngle, LastGyro;
	static Vector3f Vector_G(0, 0, ACC_1G), Vector_M(1000, 0, 0);
	Matrix3f dcm;
	
	//计算陀螺仪角度变化，二阶龙格库塔积分	
	deltaGyroAngle = (gyro + LastGyro) * 0.5 * deltaT;
	LastGyro = gyro;
	
	//计算表示单次旋转的余弦矩阵
	dcm.from_euler(deltaGyroAngle);
	
	//利用余弦矩阵更新重力向量在机体坐标系的投影
	Vector_G = dcm * Vector_G;
	
	//利用余弦矩阵更新地磁向量在机体坐标系的投影
	Vector_M = dcm * Vector_M;
	
	//互补滤波，使用加速度测量值矫正角速度积分漂移
	if (50 < (uint16_t)accRatio && (uint16_t)accRatio < 150) 
	{
		Vector_G = pFilter->CF_1st(Vector_G, acc, pFilter->Gyro_cf);
	}

	//计算飞行器的ROLL和PITCH
	Vector_G.get_rollpitch(angle);	
	
	//计算飞行器的YAW
	Vector_M.get_yaw(angle);
}


#define Kp 0.64f        //加速度权重，越大则向加速度测量值收敛越快
#define Ki 0.00032f      //误差积分增益
//四元数更新姿态
void IMU::Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	Vector3f V_gravity, V_error; 
	static Vector3f V_error_I(0, 0, 0);
	
	if(accRatio > 5)
	{
		//重力加速度归一化
		acc.normalize();
		
		//提取四元数的等效余弦矩阵中的重力分量
		Q.vector_gravity(V_gravity);
		
		//向量叉积得出姿态误差
		V_error = acc % V_gravity;
		V_error = V_error / deltaT;
		
		//对误差进行积分	
		V_error_I += V_error * Ki;
		
		//互补滤波，姿态误差补偿到角速度上，修正角速度积分漂移
		gyro = gyro  + V_error * Kp + V_error_I;		
	}
	//一阶龙格库塔法更新四元数
	Q.Runge_Kutta_1st(gyro, deltaT);
	
	//四元数归一化
	Q.normalize();
	
	//四元数转欧拉角
	Q.to_euler(&angle.x, &angle.y, &angle.z);
}


void IMU::filter_Init() 
{
	//加速度一阶低通滤波器系数计算
	//quadrotor.factor.acc_lpf = pFilter->LPF_1st_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT);
	pFilter->Acc_lpf_1st = pFilter->LPF_1st_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT);
	//加速度二阶低通滤波器系数计算
	pFilter->LPF_2nd_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT, &(pFilter->Acc_lpf_2nd));
	
	
	//互补滤波器系数计算
	//quadrotor.factor.gyro_cf = pFilter->CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);
	pFilter->Gyro_cf = pFilter->CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);
}

void IMU::sensor_Init()
{
	//初始化MPU6050，1Khz采样率42Hz低通滤波
	pSensor->init();
}



