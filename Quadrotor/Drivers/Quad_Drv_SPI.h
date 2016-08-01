#ifndef __ANO_DRV_SPI_H__
#define __ANO_DRV_SPI_H__

//#include "board.h"
#include "stm32f1xx_hal.h"

class ANO_SPI
{
	
public:
	
	static void Init(void);
	uint8_t RW(uint8_t dat);


};

#endif










