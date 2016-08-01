
#include "Quad_Drv_LED.h"

using namespace Quad;

namespace Quad
{
	LED led;
}

#define LED1_RST() 	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)
#define LED1_SET()	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define LED0_RST()	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET)
#define LED0_SET()	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET)
#define LED_EN_RST()	HAL_GPIO_WritePin(LED_EN_GPIO_Port,LED_EN_Pin,GPIO_PIN_RESET)
#define LED_EN_SET()	HAL_GPIO_WritePin(LED_EN_GPIO_Port,LED_EN_Pin,GPIO_PIN_SET)

void LED::Init(void)
{	
}

void LED::ON(void)
{
	LED0_RST();
	LED1_RST();
	LED_EN_SET();
}

void LED::OFF(void)
{
	LED0_SET();
	LED1_SET();
	LED_EN_RST();
}






