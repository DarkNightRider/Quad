#ifndef __ANO_DRV_I2C_SOFT2_H__
#define __ANO_DRV_I2C_SOFT2_H__

#include "stm32f1xx.h"
typedef uint8_t  u8;

#define SCL_H         ANO_GPIO_I2C->BSRR = I2C_Pin_SCL
#define SCL_L         ANO_GPIO_I2C->BRR  = I2C_Pin_SCL
#define SDA_H         ANO_GPIO_I2C->BSRR = I2C_Pin_SDA
#define SDA_L         ANO_GPIO_I2C->BRR  = I2C_Pin_SDA
#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA

class ANO_I2C_Soft
{
public:
	ANO_I2C_Soft();
	static void Init(void);
	int I2C_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);	
	int I2C_Single_Read(u8 SlaveAddress,u8 REG_Address);
	int I2C_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

private:
	void I2C_delay(void);
	int I2C_Start(void);
	void I2C_Stop(void);
	void I2C_Ack(void); 
	void I2C_NoAck(void);
	int I2C_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK
	void I2C_SendByte(u8 SendByte);
	u8 I2C_ReadByte(void);  //数据从高位到低位//
};


#endif

