#pragma once

#include "stm32f1xx.h"
#include <string.h>
#include <stdint.h>
#include "Quad_Math.h"
#include "stm32f1xx_hal.h"

#define ARMAPI extern "C"

/***************LED GPIO定义******************/
//#define ANO_RCC_LED			RCC_APB2Periph_GPIOC
//#define ANO_GPIO_LED		GPIOC
//#define ANO_Pin_LEDA			GPIO_Pin_13
//#define ANO_Pin_LEDB			GPIO_Pin_14
/*********************************************/
/***************I2C GPIO定义******************/
//#define ANO_GPIO_I2C	GPIOB
//#define I2C_Pin_SCL		GPIO_Pin_6
//#define I2C_Pin_SDA		GPIO_Pin_7
//#define ANO_RCC_I2C		RCC_APB2Periph_GPIOB
/*********************************************/
/***************SPI GPIO定义******************/
//#define ANO_GPIO_SPI		GPIOB
//#define ANO_GPIO_CE			GPIOA
//#define SPI_Pin_CSN			GPIO_Pin_12
//#define SPI_Pin_SCK			GPIO_Pin_13
//#define SPI_Pin_MISO		GPIO_Pin_14
//#define SPI_Pin_MOSI		GPIO_Pin_15
//#define SPI_Pin_CE			GPIO_Pin_8
//#define RCC_GPIO_SPI		RCC_APB2Periph_GPIOB
//#define RCC_GPIO_CE			RCC_APB2Periph_GPIOA
/*********************************************/

/***************硬件中断优先级******************/
//#define NVIC_UART_P	5
//#define NVIC_UART_S	1
/***********************************************/


#include "Quad_Config.h"

#include "Quad_Drv_MPU6050.h"
#include "Quad_Drv_Nrf24l01.h"
#include "Quad_Drv_PWM.h"
#include "Quad_Drv_EEPROM.h"
#include "Quad_Drv_Uart.h"
#include "Quad_Drv_Tick.h"
#include "Quad_Drv_LED.h"

void board_Init(void);







