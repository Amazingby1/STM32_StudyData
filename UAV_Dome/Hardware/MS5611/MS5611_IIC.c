#include "MS5611_IIC.h"
#include "gpio.h"

#define MS5611_IIC_W_SDA(x)       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)(x))
#define MS5611_IIC_W_SCL(x)       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(GPIO_PinState)(x))


//void MS5611_IIC_W_SDA(uint8_t State)
//{
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)(State));
//	 // MS5611_delay_us(10);
//}
//void MS5611_IIC_W_SCL(uint8_t State)
//{
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(GPIO_PinState)(State));
//	//MS5611_delay_us(10);
//}
uint8_t MS5611_IIC_R_SDA(void)
{
	  uint8_t Bit=0;
	  Bit=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5);
	  return Bit;
}

void MS5611_IIC_Init(void)
{
   MS5611_IIC_W_SCL(1);
	 MS5611_IIC_W_SDA(1);
}


void MS5611_IIC_Start(void)
{
		MS5611_IIC_W_SDA(1);
	  MS5611_IIC_W_SCL(1);
	  MS5611_IIC_W_SDA(0);
	  MS5611_IIC_W_SCL(0);
}

void MS5611_IIC_Stop(void)
{
	  MS5611_IIC_W_SDA(0);
		MS5611_IIC_W_SCL(1);
	  MS5611_IIC_W_SDA(1);
}

void MS5611_IIC_SendByte(uint8_t Byte)
{
	  uint8_t i;
	//	MS5611_IIC_W_SDA(1);
	  for(i=0;i<8;i++)
	  {
		    MS5611_IIC_W_SDA(Byte &(0x80>>i));
			  MS5611_IIC_W_SCL(1);
			  MS5611_IIC_W_SCL(0);
		}
}

uint8_t MS5611_IIC_ReceiveByte(void)
{
   uint8_t Byte=0x00;
	 uint8_t i;
	 MS5611_IIC_W_SDA(1);
	 for(i=0;i<8;i++)
	 {
		  MS5611_IIC_W_SCL(1);
	    if(MS5611_IIC_R_SDA()==1)
			{
					Byte|=(0x80>>i);
			}
			MS5611_IIC_W_SCL(0);
   }
	return Byte;
}

void MS5611_SendAckBit(uint8_t AckBit)
{
		MS5611_IIC_W_SDA(AckBit);
	  MS5611_IIC_W_SCL(1);
	  MS5611_IIC_W_SCL(0);
}
uint8_t MS5611_ReceiveAckBit(void)
{
	  uint8_t  AckBit=0;
	  MS5611_IIC_W_SDA(1);
		MS5611_IIC_W_SCL(1);
	  AckBit=MS5611_IIC_R_SDA();
	  MS5611_IIC_W_SCL(0);
	  return AckBit;
}

void MS5611_delay_us(uint16_t time)
{
	uint16_t i;
	for (i = 0 ; i < time ; i++)
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
	
}



