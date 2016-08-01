#pragma once

#include "board.h"

#include "Quad_Communication.h"
#define MODEL_RX				1			//��ͨ����
#define MODEL_TX				2			//��ͨ����
#define MODEL_RX2				3			//����ģʽ2,����˫����
#define MODEL_TX2				4			//����ģʽ2,����˫����

#define RX_PLOAD_WIDTH  32  	
#define TX_PLOAD_WIDTH  32  	
#define TX_ADR_WIDTH    5 	 	
#define RX_ADR_WIDTH    5   

#define CSN_L()	HAL_GPIO_WritePin(nRF_CSN_GPIO_Port,nRF_CSN_Pin,GPIO_PIN_RESET)
#define CSN_H()	HAL_GPIO_WritePin(nRF_CSN_GPIO_Port,nRF_CSN_Pin,GPIO_PIN_SET)
#define CE_L()	HAL_GPIO_WritePin(nRF_CE_GPIO_Port,nRF_CE_Pin,GPIO_PIN_RESET)
#define CE_H()	HAL_GPIO_WritePin(nRF_CE_GPIO_Port,nRF_CE_Pin,GPIO_PIN_SET)

namespace Quad
{
	
	
	class NRF:public Communication
	{
		
	public:
		
		NRF();
		
		//��ʼ��,model=1/2/3/4,chΪʵ�õ�ͨ����
		void Init(uint8_t model, uint8_t ch);	
		
		void Init(void);
		//�������ݰ�,����model 2/4
		void TxPacket(uint8_t * tx_buf, uint8_t len);	
		//�������ݰ�,����model 3
		void TxPacket_AP(uint8_t * tx_buf, uint8_t len);	
		//���NRFģ���Ƿ���������
		bool Check(void);
		//����Ƿ���ͨ���¼�
		void CheckEvent(void);
		
		int TransmitData(uint8_t * data, int length);
		int ReceiveData(uint8_t * data, int length);

		SPI_HandleTypeDef * pSPIInstance;

	private:

		uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
		uint8_t NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����

		uint8_t Read_Reg(uint8_t reg);
		uint8_t Write_Reg(uint8_t reg, uint8_t value);
		uint8_t Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
		uint8_t Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);

	};

	extern Quad::NRF nrf;

}


















