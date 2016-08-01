
#include "Quad_Param.h"

using namespace Quad;

namespace Quad
{
	Param param;
}

#define EE_FirstInitFlag 0
#define EE_6050_ACC_X_OFFSET_ADDR	1
#define EE_6050_ACC_Y_OFFSET_ADDR	2
#define EE_6050_ACC_Z_OFFSET_ADDR	3
#define EE_6050_GYRO_X_OFFSET_ADDR	4
#define EE_6050_GYRO_Y_OFFSET_ADDR	5
#define EE_6050_GYRO_Z_OFFSET_ADDR	6
#define EE_PID_ROL_P	7
#define EE_PID_ROL_I	8
#define EE_PID_ROL_D	9
#define EE_PID_PIT_P	10
#define EE_PID_PIT_I	11
#define EE_PID_PIT_D	12
#define EE_PID_YAW_P	13
#define EE_PID_YAW_I	14
#define EE_PID_YAW_D	15
#define EE_PID_ALT_P	16
#define EE_PID_ALT_I	17
#define EE_PID_ALT_D	18
#define EE_PID_LEVEL_P	19
#define EE_PID_LEVEL_I	20
#define EE_PID_LEVEL_D	21
#define EE_PID_MAG_P	22
#define EE_PID_MAG_I	23
#define EE_PID_MAG_D	24

uint16_t FirstInitFlag = 0x44;


void Param::Init(void)
{

	HAL_FLASH_Unlock();
	EE_Init();
	
	if(READ_FirstInitFlag()!= FirstInitFlag)	
	{
		SAVE_PID();
	}
	
	READ_CONF();
	SAVE_FirstInitFlag();
	
}

void Param::SAVE_FirstInitFlag(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_FirstInitFlag], FirstInitFlag);
}

uint16_t Param::READ_FirstInitFlag(void)
{
	uint16_t temp;
	EE_ReadVariable(VirtAddVarTab[EE_FirstInitFlag], &temp);
	return temp;
}

void Param::SAVE_ACC_OFFSET(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], mpu6050.Acc_Offset.x);
	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], mpu6050.Acc_Offset.y);
	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], mpu6050.Acc_Offset.z);
}
void Param::READ_ACC_OFFSET(void)
{
	uint16_t temp[3];
	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], &temp[0]);
	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], &temp[1]);
	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], &temp[2]);
	mpu6050.Acc_Offset.x = temp[0];
	mpu6050.Acc_Offset.y = temp[1];
	mpu6050.Acc_Offset.z = temp[2];
}
void Param::SAVE_GYRO_OFFSET(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], mpu6050.Gyro_Offset.x);
	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], mpu6050.Gyro_Offset.y);
	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], mpu6050.Gyro_Offset.z);
}
void Param::READ_GYRO_OFFSET(void)
{
	uint16_t temp[3];
	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], &temp[0]);
	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], &temp[1]);
	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], &temp[2]);
	mpu6050.Gyro_Offset.x = temp[0];
	mpu6050.Gyro_Offset.y = temp[1];
	mpu6050.Gyro_Offset.z = temp[2];
}
void Param::SAVE_PID(void)
{
	uint16_t _temp;
	_temp = fc.pid[PIDROLL].kP * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_P],_temp);
	_temp = fc.pid[PIDROLL].kI * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_I],_temp);
	_temp = fc.pid[PIDROLL].kD * 1000;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_D],_temp);
	_temp = fc.pid[PIDPITCH].kP * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_P],_temp);
	_temp = fc.pid[PIDPITCH].kI * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_I],_temp);
	_temp = fc.pid[PIDPITCH].kD * 1000;
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_D],_temp);
	_temp = fc.pid[PIDYAW].kP * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_P],_temp);
	_temp = fc.pid[PIDYAW].kI * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_I],_temp);
	_temp = fc.pid[PIDYAW].kD * 1000;
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_D],_temp);
	_temp = fc.pid[PIDALT].kP * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_P],_temp);
	_temp = fc.pid[PIDALT].kI * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_I],_temp);
	_temp = fc.pid[PIDALT].kD * 1000;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_D],_temp);
	_temp = fc.pid[PIDLEVEL].kP * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_LEVEL_P],_temp);
	_temp = fc.pid[PIDLEVEL].kI * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_LEVEL_I],_temp);
	_temp = fc.pid[PIDLEVEL].kD * 1000;
	EE_WriteVariable(VirtAddVarTab[EE_PID_LEVEL_D],_temp);
	_temp = fc.pid[PIDMAG].kP * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_MAG_P],_temp);
	_temp = fc.pid[PIDMAG].kI * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_MAG_I],_temp);
	_temp = fc.pid[PIDMAG].kD * 1000;
	EE_WriteVariable(VirtAddVarTab[EE_PID_MAG_D],_temp);
}

void Param::READ_PID(void)
{
	uint16_t _temp;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_P],&_temp);
	fc.pid[PIDROLL].kP = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_I],&_temp);
	fc.pid[PIDROLL].kI = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_D],&_temp);
	fc.pid[PIDROLL].kD = (float)_temp / 1000;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_P],&_temp);
	fc.pid[PIDPITCH].kP = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_I],&_temp);
	fc.pid[PIDPITCH].kI = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_D],&_temp);
	fc.pid[PIDPITCH].kD = (float)_temp / 1000;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_P],&_temp);
	fc.pid[PIDYAW].kP = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_I],&_temp);
	fc.pid[PIDYAW].kI = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_D],&_temp);
	fc.pid[PIDYAW].kD = (float)_temp / 1000;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_P],&_temp);
	fc.pid[PIDALT].kP = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_I],&_temp);
	fc.pid[PIDALT].kI = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_D],&_temp);
	fc.pid[PIDALT].kD = (float)_temp / 1000;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_LEVEL_P],&_temp);
	fc.pid[PIDLEVEL].kP = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_LEVEL_I],&_temp);
	fc.pid[PIDLEVEL].kI = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_LEVEL_D],&_temp);
	fc.pid[PIDLEVEL].kD = (float)_temp / 1000;
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_MAG_P],&_temp);
	fc.pid[PIDMAG].kP = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_MAG_I],&_temp);
	fc.pid[PIDMAG].kI = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_MAG_D],&_temp);
	fc.pid[PIDMAG].kD = (float)_temp / 1000;
}

void Param::READ_CONF(void)
{
	READ_PID();
	READ_ACC_OFFSET();
	READ_GYRO_OFFSET();
}










