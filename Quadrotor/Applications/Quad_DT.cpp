
#include "Quad_DT.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

using namespace Quad;

extern UART_HandleTypeDef huart1;



namespace Quad
{
	DT dt;
}

void DT::Init()
{
	pNrf->Init();
	if(nrf.Check())
		scheduler.cnt_20ms++;
	pUart->Init();
}

DT::DT()
{
  pNrf = &(Quad::nrf);
  pUart = &(Quad::uart1);
}

void DT::Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	uint16_t index = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;				//check sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//check frame header
	
	quadrotor.f.failsafe = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			mpu6050.Acc_CALIBRATED = 1;
		if(*(data_buf+4)==0X02)
			mpu6050.Gyro_CALIBRATED = 1;
		if(*(data_buf+4)==0X03)
		{
			mpu6050.Acc_CALIBRATED = 1;		
			mpu6050.Gyro_CALIBRATED = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
 			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//read version info
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//reset defualt PID params
		{
			fc.PID_Reset();
			param.SAVE_PID();
		}
	}

	if(*(data_buf+2)==0X03)
	{
		rc.rawData[THROTTLE] 	= (int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		rc.rawData[YAW] 		= (int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		rc.rawData[ROLL] 		= (int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		rc.rawData[PITCH] 	= (int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		rc.rawData[AUX1] 		= (int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		rc.rawData[AUX2] 		= (int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		rc.rawData[AUX3] 		= (int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		rc.rawData[AUX4] 		= (int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
	}

	if(*(data_buf+2)==0X10)								//PID1
	{
		fc.pid[PIDROLL].kP = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5)) / 1000;
		fc.pid[PIDROLL].kI = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7)) / 1000;
		fc.pid[PIDROLL].kD = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9)) / 1000;
		fc.pid[PIDPITCH].kP = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11)) / 1000;
		fc.pid[PIDPITCH].kI = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13)) / 1000;
		fc.pid[PIDPITCH].kD = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15)) / 1000;
		fc.pid[PIDYAW].kP = (float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17)) / 1000;
		fc.pid[PIDYAW].kI = (float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19)) / 1000;
		fc.pid[PIDYAW].kD = (float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21)) / 1000;
		Send_Check(&index, *(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		fc.pid[PIDALT].kP = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5)) / 1000;
		fc.pid[PIDALT].kI = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7)) / 1000;
		fc.pid[PIDALT].kD = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9)) / 1000;
		fc.pid[PIDLEVEL].kP = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11)) / 1000;
		fc.pid[PIDLEVEL].kI = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13)) / 1000;
		fc.pid[PIDLEVEL].kD = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15)) / 1000;
		fc.pid[PIDMAG].kP = (float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17)) / 1000;
		fc.pid[PIDMAG].kI = (float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19)) / 1000;
		fc.pid[PIDMAG].kD = (float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21)) / 1000;
		Send_Check(&index, *(data_buf+2),sum);
		param.SAVE_PID();
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
		Send_Check(&index, *(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
		Send_Check(&index, *(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		Send_Check(&index, *(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		Send_Check(&index, *(data_buf+2),sum);
	}

/////////////////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0x18)					
	{

	}
}

void DT::Check_Event(void)
{
	
#ifdef DT_USE_NRF24l01	
	pNrf->CheckEvent();
#endif
	
}

void DT::Data_Exchange(void)
{
	static uint8_t cnt = 0;
	static uint8_t senser_cnt 	= 10;
	static uint8_t user_cnt 	  = 10;
	static uint8_t status_cnt 	= 15;
	static uint8_t rcdata_cnt 	= 20;
	static uint8_t motopwm_cnt	= 20;
	static uint8_t power_cnt		=	50;
	static uint8_t senser2_cnt = 50;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;	
	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;	
	
	if((cnt % user_cnt) == (user_cnt-1))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	uint16_t index = 0;
	if(f.send_version)
	{
		f.send_version = 0;
		Send_Version(&index, 1,300,110,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
		Send_Status(&index, imu.angle.x,imu.angle.y,imu.angle.z,0,0,quadrotor.f.ARMED);	
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_user)
	{
		f.send_user = 0;
		//Send_User();
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
		Send_Senser(&index, imu.Acc.x,imu.Acc.y,imu.Acc.z,
								imu.Gyro.x,imu.Gyro.y,imu.Gyro.z,
								0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser2)
	{
		f.send_senser2 = 0;
		Send_Senser2(&index, 0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		Send_RCData(&index, rc.rawData[THROTTLE],rc.rawData[YAW],rc.rawData[ROLL],rc.rawData[PITCH],
								rc.rawData[AUX1],rc.rawData[AUX2],rc.rawData[AUX3],
								rc.rawData[AUX4],rc.rawData[AUX5],rc.rawData[AUX6]);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		uint16_t Moto_PWM[4];
		motor.getPWM(Moto_PWM);
		for(uint8_t i=0;i<4;i++)
			Moto_PWM[i] -= 1000;

		Send_MotoPWM(&index, Moto_PWM[0],Moto_PWM[1],Moto_PWM[2],Moto_PWM[3],0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
//		Send_Power(123,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		Send_PID(&index, 1,fc.pid[PIDROLL].kP,fc.pid[PIDROLL].kI,fc.pid[PIDROLL].kD,
							 fc.pid[PIDPITCH].kP,fc.pid[PIDPITCH].kI,fc.pid[PIDPITCH].kD,
							 fc.pid[PIDYAW].kP,fc.pid[PIDYAW].kI,fc.pid[PIDYAW].kD);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		Send_PID(&index, 2,fc.pid[PIDALT].kP,fc.pid[PIDALT].kI,fc.pid[PIDALT].kD,
							 fc.pid[PIDLEVEL].kP,fc.pid[PIDLEVEL].kI,fc.pid[PIDLEVEL].kD,
							 fc.pid[PIDMAG].kP,fc.pid[PIDMAG].kI,fc.pid[PIDMAG].kD);
	}
	Send_Data(data_to_send, index);
/////////////////////////////////////////////////////////////////////////////////////
}
void DT::Send_Version(uint16_t * index, uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x00;
	buf[_cnt++]=0;
	
	buf[_cnt++]=hardware_type;
	buf[_cnt++]=BYTE1(hardware_ver);
	buf[_cnt++]=BYTE0(hardware_ver);
	buf[_cnt++]=BYTE1(software_ver);
	buf[_cnt++]=BYTE0(software_ver);
	buf[_cnt++]=BYTE1(protocol_ver);
	buf[_cnt++]=BYTE0(protocol_ver);
	buf[_cnt++]=BYTE1(bootloader_ver);
	buf[_cnt++]=BYTE0(bootloader_ver);
	
	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	buf[_cnt++]=sum;
	
	*index = *index + _cnt;
	
}
void DT::Send_Status(uint16_t * index, float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	volatile int32_t _temp2 = alt;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x01;
	buf[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	
	buf[_cnt++]=BYTE3(_temp2);
	buf[_cnt++]=BYTE2(_temp2);
	buf[_cnt++]=BYTE1(_temp2);
	buf[_cnt++]=BYTE0(_temp2);
	
	buf[_cnt++] = fly_model;
	
	buf[_cnt++] = armed;
	
	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	buf[_cnt++]=sum;
	
	*index = *index + _cnt;
	
}


void DT::Send_Senser(uint16_t * index, int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x02;
	buf[_cnt++]=0;
	
	_temp = a_x;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	
	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	buf[_cnt++] = sum;
	
	*index = *index + _cnt;
	
}
void DT::Send_Senser2(uint16_t * index, int32_t alt_bar,uint16_t alt_csb)
{
	uint8_t _cnt=0;
	volatile int32_t _temp;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x07;
	buf[_cnt++]=0;
	
	_temp = alt_bar;
	buf[_cnt++]=BYTE3(_temp);
	buf[_cnt++]=BYTE2(_temp);
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = alt_csb;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);

	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	buf[_cnt++] = sum;
	
	*index = *index + _cnt;
	
}

void DT::Send_RCData(uint16_t * index, uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x03;
	buf[_cnt++]=0;
	buf[_cnt++]=BYTE1(thr);
	buf[_cnt++]=BYTE0(thr);
	buf[_cnt++]=BYTE1(yaw);
	buf[_cnt++]=BYTE0(yaw);
	buf[_cnt++]=BYTE1(rol);
	buf[_cnt++]=BYTE0(rol);
	buf[_cnt++]=BYTE1(pit);
	buf[_cnt++]=BYTE0(pit);
	buf[_cnt++]=BYTE1(aux1);
	buf[_cnt++]=BYTE0(aux1);
	buf[_cnt++]=BYTE1(aux2);
	buf[_cnt++]=BYTE0(aux2);
	buf[_cnt++]=BYTE1(aux3);
	buf[_cnt++]=BYTE0(aux3);
	buf[_cnt++]=BYTE1(aux4);
	buf[_cnt++]=BYTE0(aux4);
	buf[_cnt++]=BYTE1(aux5);
	buf[_cnt++]=BYTE0(aux5);
	buf[_cnt++]=BYTE1(aux6);
	buf[_cnt++]=BYTE0(aux6);

	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	
	buf[_cnt++]=sum;
	
	*index = *index + _cnt;
	

}
void DT::Send_Power(uint16_t * index, uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x05;
	buf[_cnt++]=0;
	
	temp = votage;
	buf[_cnt++]=BYTE1(temp);
	buf[_cnt++]=BYTE0(temp);
	temp = current;
	buf[_cnt++]=BYTE1(temp);
	buf[_cnt++]=BYTE0(temp);
	
	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	
	buf[_cnt++]=sum;
	
	*index = *index + _cnt;

}
void DT::Send_MotoPWM(uint16_t * index, uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	uint8_t _cnt=0;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x06;
	buf[_cnt++]=0;
	
	buf[_cnt++]=BYTE1(m_1);
	buf[_cnt++]=BYTE0(m_1);
	buf[_cnt++]=BYTE1(m_2);
	buf[_cnt++]=BYTE0(m_2);
	buf[_cnt++]=BYTE1(m_3);
	buf[_cnt++]=BYTE0(m_3);
	buf[_cnt++]=BYTE1(m_4);
	buf[_cnt++]=BYTE0(m_4);
	buf[_cnt++]=BYTE1(m_5);
	buf[_cnt++]=BYTE0(m_5);
	buf[_cnt++]=BYTE1(m_6);
	buf[_cnt++]=BYTE0(m_6);
	buf[_cnt++]=BYTE1(m_7);
	buf[_cnt++]=BYTE0(m_7);
	buf[_cnt++]=BYTE1(m_8);
	buf[_cnt++]=BYTE0(m_8);
	
	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	
	buf[_cnt++]=sum;
	
	*index = *index + _cnt;
	
	
}

void DT::Send_PID(uint16_t * index, uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	uint8_t * buf = &data_to_send[*index];
	
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0xAA;
	buf[_cnt++]=0x10+group-1;
	buf[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	buf[_cnt++]=BYTE1(_temp);
	buf[_cnt++]=BYTE0(_temp);
	
	buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += buf[i];
	
	buf[_cnt++]=sum;
	
	*index = *index + _cnt;

}
void DT::Send_Check(uint16_t * index, uint8_t head, uint8_t check_sum)
{
	uint8_t * buf = &data_to_send[*index];
	buf[0]=0xAA;
	buf[1]=0xAA;
	buf[2]=0xEF;
	buf[3]=2;
	buf[4]=head;
	buf[5]=check_sum;
	
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += buf[i];
	buf[6]=sum;
	
	*index = *index + 8;
	
}

void DT::Send_Data(uint8_t *dataToSend , uint8_t length)
{
	
	
#ifdef DT_USE_NRF24l01
	pNrf->TransmitData(data_to_send,length);
#endif
	
	pUart->TransmitData(data_to_send, length);
	
}


void DT::Failsafe_Check(void)
{
		static uint8_t failsafeCnt = 0;
		if(failsafeCnt > 30)
		{
			failsafeCnt = 0;
			if(!quadrotor.f.failsafe)
				quadrotor.f.failsafe = 1;
			else
			{	
				quadrotor.f.ARMED = 0;
			}
		}
		failsafeCnt++;	
}




