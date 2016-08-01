/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：board.c
 * 描述    ：硬件初始化
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "board.h"



using namespace Quad;

void board_Init(void)
{
	
	
  	dt.Init();
	motor.Init();
	
	
	HAL_FLASH_Unlock();
	EE_Init();
}



/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
