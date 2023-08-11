#ifndef  __MS5611_IIC_H
#define  __MS5611_IIC_H
#include "gpio.h"



void MS5611_IIC_Init(void);
void MS5611_IIC_Start(void);
void MS5611_IIC_Stop(void);
void MS5611_IIC_SendByte(uint8_t Byte);
uint8_t MS5611_IIC_ReceiveByte(void);
void MS5611_SendAckBit(uint8_t AckBit);
uint8_t MS5611_ReceiveAckBit(void);
void MS5611_delay_us(uint16_t time);


#endif
