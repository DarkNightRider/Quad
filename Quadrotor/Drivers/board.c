/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ����		 �������ƴ�
 * �ļ���  ��board.c
 * ����    ��Ӳ����ʼ��
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
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
