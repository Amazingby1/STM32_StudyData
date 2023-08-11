#ifndef __BSP_MS5611_H
#define __BSP_MS5611_H
#include "gpio.h"
#include "MS5611_IIC.h"

#define  PA_OFFSET_INIT_NUM 		50  
#define  PROM_NB                 	8
 
#define  MS561101BA_SlaveAddress 	0xEE  //����������IIC�����еĴӵ�ַ
#define  MS561101BA_D1 				0x40 
#define  MS561101BA_D2 				0x50
#define  MS561101BA_RST 			0x1E
  
#define  MS561101BA_D1_OSR_256 		0x40
#define  MS561101BA_D1_OSR_512 		0x42
#define  MS561101BA_D1_OSR_1024 	0x44
#define  MS561101BA_D1_OSR_2048 	0x46
#define  MS561101BA_D1_OSR_4096 	0x48
 
#define  MS561101BA_D2_OSR_256 		0x50
#define  MS561101BA_D2_OSR_512 		0x52 
#define  MS561101BA_D2_OSR_1024 	0x54 
#define  MS561101BA_D2_OSR_2048 	0x56 
#define  MS561101BA_D2_OSR_4096 	0x58 
 
#define  MS561101BA_ADC_RD 			0x00
#define  MS561101BA_PROM_RD 		0xA0	//����У׼ֵ��ʼ��ַ
#define  MS561101BA_PROM_CRC 		0xAE	//����У׼ֵCRCУ��ֵ��ַ

extern float MS5611_Pressure;
void MS5611_Init(void);
void MS561101BA_RESET(void);
uint32_t MS561101BA_DO_CONVERSION(uint8_t command);
void MS561101BA_GetTemperature(uint8_t OSR_Temp);
void MS561101BA_GetPressure(uint8_t OSR_Pres);
float getEstimatedAltitude(int32_t baroPressure);
void MS561101BA_READ_PROM(void);
#endif
