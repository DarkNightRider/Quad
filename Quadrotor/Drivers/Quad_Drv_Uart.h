#pragma once

#include "board.h"

#include "Quad_Communication.h"

#define DATA_QUERY_MAX_LEN 64

#define INNER_BUFFER_LEN 64

namespace Quad
{
	//class Communication;
	
	
	
	class UART:public Communication
	{
		typedef struct Data_
		{
			uint8_t * dataAddr;
			int dataLength;
		}Data;
	public:
		UART();

		
		void Init(UART_HandleTypeDef * pUART);
		
		void Init();
		
		//int TransmitData(uint8_t * data, int length);
		//int ReceiveData(uint8_t * data, int length);
		
		inline void setRxBufPos(unsigned char pos){
			RxBufferPos = pos;
		}
		
		
		inline unsigned char getRxBufPos(void){
			return RxBufferPos;
		}
		
		//Transmit Data
		int TransmitData(uint8_t * data, int length);
		//Receive Data
		int ReceiveData(uint8_t * data, int length);
		//check event
		void CheckEvent(void);
		
		
		//check if rx new data
		unsigned char checkNewRxData();
		
		//abolish rx new data unread
		unsigned char abolishNewRxData();
		
		//transmit a char
		void putChar(unsigned char DataToSend);
		
		//transmit a string
		void putString(unsigned char *Str, size_t n);

		
		//begin DMA buffer rx
		void startReceive(void);
		
		//tx complete callback function
		void TxCplCallback(void);
		
		UART_HandleTypeDef * pUARTInstance;


	private:
		//inner Tx Buffer.
		unsigned char TxBuffer[256];
		//inner Rx Buffer.
		unsigned char RxBuffer[256];
		//Rx begin to read position.
		unsigned char RxBufferPos;
		
		int getRxBufData(uint8_t * data, int length);
		
		int ungetRxBufData(uint8_t * data, int length);
		
		Data TxDataQuery[DATA_QUERY_MAX_LEN];
		
		Data * TxCurData;
		
		uint8_t TxDataQueryLength;
		
		
	};
	
	extern Quad::UART uart1;
}


