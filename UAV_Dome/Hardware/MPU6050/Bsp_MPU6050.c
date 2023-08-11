#include "Bsp_MPU6050.h"
#include "Bsp_MyIIC.h"
#include "Delay.h"

void MPU6050_W_Reg(uint8_t Regaddress,uint8_t DATA)
{
	  MPU6050_IIC_Start();
	  MPU6050_IIC_SendByte(MPU6050_ADDRESS);
    MPU6050_ReceiveAckBit();
	  MPU6050_IIC_SendByte(Regaddress);
	  MPU6050_ReceiveAckBit();
		MPU6050_IIC_SendByte(DATA);
	  MPU6050_ReceiveAckBit();
	  MPU6050_IIC_Stop();
}
uint8_t MPU6050_R_Reg(uint8_t Regaddress)
{
	  uint8_t Data;
	  MPU6050_IIC_Start();
	  MPU6050_IIC_SendByte(MPU6050_ADDRESS);
	  MPU6050_ReceiveAckBit();
	  MPU6050_IIC_SendByte(Regaddress);
	  MPU6050_ReceiveAckBit();
	  
	  MPU6050_IIC_Start();
	  MPU6050_IIC_SendByte(MPU6050_ADDRESS | 0x01);
	  MPU6050_ReceiveAckBit();
	  Data=MPU6050_IIC_ReceiveByte();
	  MPU6050_SendAckBit(1);
	  MPU6050_IIC_Stop();
	  return Data;
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_R_Reg(MPU6050_WHO_AM_I);
}

void MPU6050_Init(void)
{	
	  MPU6050_IIC_Init();
		MPU6050_W_Reg(MPU6050_PWR_MGMT_1, 0x01);
		MPU6050_W_Reg(MPU6050_PWR_MGMT_2, 0x00);
		MPU6050_W_Reg(MPU6050_SMPLRT_DIV, 0x09);
		MPU6050_W_Reg(MPU6050_CONFIG, 0x06);
		MPU6050_W_Reg(MPU6050_GYRO_CONFIG, 0x18);
		MPU6050_W_Reg(MPU6050_ACCEL_CONFIG, 0x18);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU6050_R_Reg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_R_Reg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_R_Reg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_R_Reg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_R_Reg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_R_Reg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_R_Reg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_R_Reg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_R_Reg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_R_Reg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_R_Reg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_R_Reg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}
