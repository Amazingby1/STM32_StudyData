#ifndef __BSP_MYIIC_H
#define __BSP_MYIIC_H
#include "gpio.h"

uint8_t MPU6050_IIC_SDA_R(void);
void MPU6050_IIC_Init(void);
void MPU6050_IIC_Start(void);
void MPU6050_IIC_Stop(void);
void MPU6050_IIC_SendByte(uint8_t Byte);
uint8_t MPU6050_IIC_ReceiveByte(void);
void MPU6050_SendAckBit(uint8_t AckBit);
uint8_t MPU6050_ReceiveAckBit(void);

#endif
