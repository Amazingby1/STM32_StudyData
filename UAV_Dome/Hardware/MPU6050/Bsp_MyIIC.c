#include "Bsp_MyIIC.h"
#include "gpio.h"

//#define MPU6050_IIC_SDA_W(x)      HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,(GPIO_PinState)(x))
//#define MPU6050_IIC_SCL_W(x)      HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,(GPIO_PinState)(x))

void MPU6050_IIC_SDA_W(uint8_t State)
{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,(GPIO_PinState)(State));
		
}
void MPU6050_IIC_SCL_W(uint8_t State)
{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,(GPIO_PinState)(State));
	  
}
uint8_t MPU6050_IIC_SDA_R(void)
{
		uint8_t Bit=0;
	  Bit=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15);
		return Bit;
}

void MPU6050_IIC_Init(void)
{
		MPU6050_IIC_SCL_W(1);
		MPU6050_IIC_SDA_W(1);
}
void MPU6050_IIC_Start(void)
{
		MPU6050_IIC_SDA_W(1);	
		MPU6050_IIC_SCL_W(1);
		MPU6050_IIC_SDA_W(0);
		MPU6050_IIC_SCL_W(0);
}
void MPU6050_IIC_Stop(void)
{
		MPU6050_IIC_SDA_W(0);
		MPU6050_IIC_SCL_W(1);
		MPU6050_IIC_SDA_W(1);
}
void MPU6050_IIC_SendByte(uint8_t Byte)
{
		uint8_t i;
	//	MPU6050_IIC_SDA_W(1);
	  for(i=0;i<8;i++)
		{
				MPU6050_IIC_SDA_W(Byte &(0X80>>i));
				MPU6050_IIC_SCL_W(1);  
			  MPU6050_IIC_SCL_W(0);
		}
}
uint8_t MPU6050_IIC_ReceiveByte(void)
{
		uint8_t Byte=0x00;
		uint8_t i;
		MPU6050_IIC_SDA_W(1);
	  for(i=0;i<8;i++)
		{
				MPU6050_IIC_SCL_W(1);
			  if(MPU6050_IIC_SDA_R()==1)
				{
						Byte|=(0x80>>i);
				}
				MPU6050_IIC_SCL_W(0);
		}
		return Byte;
}
void MPU6050_SendAckBit(uint8_t AckBit)
{
		MPU6050_IIC_SDA_W(AckBit);
		MPU6050_IIC_SCL_W(1);	 
	  MPU6050_IIC_SCL_W(0);
}
uint8_t MPU6050_ReceiveAckBit(void)
{
		uint8_t AckBit=0;
	  MPU6050_IIC_SDA_W(1);
	  MPU6050_IIC_SCL_W(1);
	  AckBit=MPU6050_IIC_SDA_R();
	  MPU6050_IIC_SCL_W(0);
		return AckBit;
}
