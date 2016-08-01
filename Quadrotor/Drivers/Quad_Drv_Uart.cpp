
#include "Quad_Drv_Uart.h"
#include "usart.h"




using namespace Quad;

extern UART_HandleTypeDef huart1;

namespace Quad
{
	UART uart1;
}

UART::UART()
{
	TxCurData = &TxDataQuery[0];
	TxDataQueryLength = 0;
}

void UART::Init(UART_HandleTypeDef * pUART)
{


	pUARTInstance = pUART;
	RxBufferPos = 0;
	
}

void UART::Init()
{
	Init(&huart1);
}


void UART::putChar(unsigned char DataToSend)
{
	TxBuffer[0] = DataToSend;  
	while( HAL_BUSY == HAL_UART_Transmit_DMA(pUARTInstance, this->TxBuffer, 1) );
}


void UART::putString(unsigned char *Str, size_t n)
{
	TransmitData(Str, strlen((const char *)Str)>n?n:strlen((const char *)Str));
}

//
//void UART::putBuf(unsigned char *DataToSend , size_t n)
//{
//	while( HAL_BUSY == HAL_UART_Transmit_DMA(pUARTInstance, DataToSend, n) );
//}


unsigned char UART::checkNewRxData()
{
	int r = 256-pUARTInstance->hdmarx->Instance->CNDTR;
	r -= RxBufferPos;
	if(r<0)
		r += 256;
	return (unsigned char)r;
}

unsigned char UART::abolishNewRxData()
{
	
	int temp = 256 - pUARTInstance->hdmarx->Instance->CNDTR;
	int r = temp - RxBufferPos;
	RxBufferPos = temp;
	if(r<0)
		r += 256;
	return (unsigned char)r;
}



void UART::startReceive()
{
	HAL_UART_Receive_DMA(this->pUARTInstance, this->RxBuffer, 256);
}


int UART::getRxBufData(uint8_t * data, int length)
{
	//get new data count(byte)
	int rx_num = checkNewRxData();
	//if the amount of new data is less than user's expecting length, ignore user's setting
	rx_num = (rx_num>length)?(length):(rx_num);
	//copy data from buffer to user's memory.
	for(int i=0; i<rx_num; i++)
	{
		*(data+i) = RxBuffer[(RxBufferPos+i)%256];
	}
	//update RX buffer begin position
	RxBufferPos = (RxBufferPos+rx_num)%256;
	//return actually amount of data that user get.
	return rx_num;
}

int UART::ungetRxBufData(uint8_t * data, int length)
{
	//get new data count(byte)
	int rx_num = checkNewRxData();
	//copy data from buffer to user's memory.
	if(length+rx_num>256)
		return 0;
	
	int copystart = (256+RxBufferPos-length)%256;
	
	for(int i=0; i<length; i++)
	{
		*(data+i) = RxBuffer[(copystart+i)%256];
	}
	//update RX buffer begin position
	RxBufferPos = copystart;
	//return actually amount of data that user get.
	return rx_num;
}


int UART::ReceiveData(uint8_t * data, int length)
{
	return getRxBufData(data, length);
}

int UART::TransmitData(uint8_t * data, int length)
{
	Data * add;
	// if DMA Tx is running, Put the data pointer to a query
	if(HAL_BUSY == HAL_UART_Transmit_DMA(pUARTInstance, data, length))
	{
		//if the Query is full, just wait.
		while(TxDataQueryLength==DATA_QUERY_MAX_LEN);
		
		//find the end of the query, the array have a range.so do limint
		add = &TxDataQuery[0] + ((TxCurData + TxDataQueryLength) - &TxDataQuery[0]) % DATA_QUERY_MAX_LEN;
		//the query contents the fixed number of pointers that point to a struct that descript the data (data addrwss and length).
		//add the data pointer to the struct that descript the data.
		add->dataAddr = data;
		//so as the length.
		add->dataLength = length;
		//Query length increase.
		TxDataQueryLength++;
		//range of 0-DATA_QUERY_MAX_LEN
		TxDataQueryLength %= DATA_QUERY_MAX_LEN;
	}
	return length;
}

void UART::CheckEvent(void)
{
	
}

void UART::TxCplCallback()
{
	//when a tx is completed, check if there are datas in the query.
	if(TxDataQueryLength>0)
	{
		//Query length decrease
		TxDataQueryLength--;
		//send the current data in query
		HAL_UART_Transmit_DMA(pUARTInstance, TxCurData->dataAddr, TxCurData->dataLength);
		//pointer move
		TxCurData++;
		//range limit.
		TxCurData = &TxDataQuery[0] + (TxCurData - &TxDataQuery[0]) % DATA_QUERY_MAX_LEN;
		
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

 	uart1.TxCplCallback();
	
}











