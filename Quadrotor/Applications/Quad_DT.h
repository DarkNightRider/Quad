/*******************************************************************************
 *Copyright (C), 2016.
 *@file 	Quad_DT.h UTF-8
 *@brief 	Quadrotor Data Transformation module class defination.
 *@author	Xinyu Han
 *@version	1.0.0
 *@date		2016.03.15
 *@warning	none
 *@history	none
 ******************************************************************************/

#pragma once

#include "Quad_Config.h"
#include "Quad_Communication.h"
namespace Quad
{
	class UART;
	class NRF;
	
	class DT
	{
		
	public:
	  
	  	Communication * pUart;
		
		Communication * pNrf;
		
		DT();
		
		void Init();
		
		void Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
		//check data arrival
		void Check_Event(void);
		//data t/r
		void Data_Exchange(void);
		//lost control protect
		void Failsafe_Check(void);

		class flag{
			public:
			uint8_t send_version;
			uint8_t send_status;
			uint8_t send_senser;
			uint8_t send_senser2;
			uint8_t send_pid1;
			uint8_t send_pid2;
			uint8_t send_pid3;
			uint8_t send_pid4;
			uint8_t send_pid5;
			uint8_t send_pid6;
			uint8_t send_rcdata;
			uint8_t send_offset;
			uint8_t send_motopwm;
			uint8_t send_user;
			uint8_t send_power;
		}f;
		
	private:
			
		uint8_t data_to_send[128];

		void Send_Version(uint16_t * index, uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
		void Send_Status(uint16_t * index, float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
		void Send_Senser(uint16_t * index, int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z);
		void Send_Senser2(uint16_t * index, int32_t alt_bar,uint16_t alt_csb);
		void Send_RCData(uint16_t * index, uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
		void Send_Power(uint16_t * index, uint16_t votage, uint16_t current);
		void Send_MotoPWM(uint16_t * index, uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
		void Send_PID(uint16_t * index, uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
		void Send_Check(uint16_t * index, uint8_t head, uint8_t check_sum);

		void Send_Data(uint8_t *dataToSend , uint8_t length);

	};

	extern DT dt;
}











