#include "Bsp_MS5611.h"
#include <math.h>




uint16_t Cal_C[8];  //���ڴ��PROM�е�8������
/*
C0 ����0
C1 ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
C7  ����CRCУ��ֵ
*/	
 
 
uint32_t D1_Pres , D2_Temp; 	// �������ѹ�����¶�
float Pressure;					//�¶Ȳ�������ѹ
float dT , Temperature , T2;	//ʵ�ʺͲο��¶�֮��Ĳ���,ʵ���¶�,�м�ֵ
double OFF , SENS;  			//ʵ���¶ȵ���,ʵ���¶�������
float Aux , OFF2 , SENS2;  		//�¶�У��ֵ
 
 
static float Alt_offset_Pa=0;
double paOffsetNum = 0;
uint16_t  paInitCnt=0;
uint8_t paOffsetInited=0;
float MS5611_Pressure;

void MS5611_Init(void)
{
		MS5611_IIC_Init();
}
void MS561101BA_RESET(void)
{	
	MS5611_IIC_Start();
	MS5611_IIC_SendByte(MS561101BA_SlaveAddress);	//CSB�ӵأ�������ַ��0xEE������ 0x77
//	if (!MS5611_ReceiveAckBit())
//	{
//		MS5611_IIC_Stop();
//		return (FALSE);
//	}
	MS5611_ReceiveAckBit();
	MS5611_IIC_Stop();
	
	MS5611_IIC_Start();
	MS5611_IIC_SendByte(MS561101BA_RST);			//���͸�λ����
	MS5611_ReceiveAckBit();
	MS5611_IIC_Stop();
}

//��ȡ�¶�ADת��ֵ���߶�ȡѹ��ADת��ֵ
 
uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
	uint32_t conversion = 0x00;
	uint32_t conv1,conv2,conv3; 
	
	
		MS5611_IIC_Start();
		MS5611_IIC_SendByte(MS561101BA_SlaveAddress);
		MS5611_ReceiveAckBit();
		MS5611_IIC_SendByte(command);	
		MS5611_ReceiveAckBit();
		MS5611_IIC_Stop();
   HAL_Delay(12);
	MS5611_IIC_Start();
	MS5611_IIC_SendByte(MS561101BA_SlaveAddress);
	MS5611_ReceiveAckBit();
	MS5611_IIC_SendByte(0x00);
	MS5611_ReceiveAckBit();
	MS5611_IIC_Stop();
	
	MS5611_IIC_Start();
	MS5611_IIC_SendByte(MS561101BA_SlaveAddress + 1);
	MS5611_ReceiveAckBit();
	conv1 = MS5611_IIC_ReceiveByte();	//��ACK�Ķ�����  bit 23-16
		MS5611_SendAckBit(0);
	conv2 = MS5611_IIC_ReceiveByte();	//��ACK�Ķ�����  bit 8-15
  	MS5611_SendAckBit(0);
	conv3 = MS5611_IIC_ReceiveByte();	//��NoACK�Ķ����� bit 0-7
	MS5611_SendAckBit(1);
	MS5611_IIC_Stop();
 
	conversion = (conv1 << 16) + (conv2 << 8) + conv3;
	return (conversion);
}
 
 
 
//��ȡ�����¶�ADת��ֵ,�����¶�
void MS561101BA_GetTemperature(uint8_t OSR_Temp)
{   
	D2_Temp = MS561101BA_DO_CONVERSION(OSR_Temp);
	//MS5611_delay_ms(10);
	
	//dT = D2_Temp - (((u32)Cal_C[5])<<8);
	//Temperature = (float)(2000 + dT*((u32)Cal_C[6])/(float)8388608.0);//����¶�ֵ��100����2001��ʾ20.01��
	
 
//���棺���¶ȵ���20�ȣ�����Ĵ���ѹ������ȷ�Ľ������	
	//����2�д��룬���¶�ֵ���ڵ���20��ʱ�������ѹ��ֵ��ȷ��
	//��������2�д��룬���¶�ֵ����20��ʱ�������⡣��ΪD2_Temp��Cal_C[5] )�����޷�������
	//�޷�����֮��ʹ�ü�����û���õ���ֵ�����Ա���Ҫ�������������޸�һ�¡�
	
	if (D2_Temp > (((uint32_t)Cal_C[5]) << 8 ))
	{
		dT	= D2_Temp - (((uint32_t)Cal_C[5]) << 8 );
	}
	else
	{
		dT	= ((( uint32_t)Cal_C[5]) << 8) - D2_Temp;
		dT *= -1;
	}
	Temperature = (float)(2000 + dT*((uint32_t)Cal_C[6])/(float)8388608.0);//����¶�ֵ��100����2001��ʾ20.01��
}
 
 
 
//��ȡ������ѹADת��ֵ���������ѹ��
void MS561101BA_GetPressure(uint8_t OSR_Pres)
{	
	D1_Pres = MS561101BA_DO_CONVERSION(OSR_Pres);
	//MS5611_delay_ms(10);
	OFF =  (uint32_t)(Cal_C[2] << 16) + ((uint32_t)Cal_C[4] * dT) / 128;
	SENS = (uint32_t)(Cal_C[1] << 15) + ((uint32_t)Cal_C[3] * dT) / 256;
	
	//�¶Ȳ���
	if (Temperature < 2000)	// second order temperature compensation when under 20 degrees C
	{
		T2 = (dT*dT) / 0x80000000;
		Aux = (Temperature - 2000)*(Temperature - 2000);
		OFF2 = (float)(2.5)*Aux;
		SENS2 = (float)(1.25)*Aux;
		if (Temperature < -1500)
		{
			Aux = (Temperature + 1500)*(Temperature + 1500);
			OFF2 = OFF2 + 7 * Aux;
			SENS2 = SENS + (float)(5.5)*Aux;
		}
	}
	else //(Temperature > 2000)
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	Temperature -= T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
	Pressure = (D1_Pres * SENS / 2097152 - OFF)/32768;
	MS5611_Pressure = Pressure / (float)(1000000.0);
}
 
void MS561101BA_READ_PROM(void)
{
	uint16_t d1,d2;
	uint8_t i;
	
	
	for (i = 0 ; i < PROM_NB ; i++)		//��ȡPROM�е�8������
	{
		MS5611_IIC_Start();
		MS5611_IIC_SendByte(MS561101BA_SlaveAddress);
		MS5611_ReceiveAckBit();
				
		MS5611_IIC_SendByte((MS561101BA_PROM_RD + i*2));
		MS5611_ReceiveAckBit();
		MS5611_IIC_Stop();
		MS5611_delay_us(200);
		
		MS5611_IIC_Start();
		MS5611_IIC_SendByte(MS561101BA_SlaveAddress + 0x01);	//�������ģʽ
		MS5611_ReceiveAckBit();
		d1 = MS5611_IIC_ReceiveByte();
    MS5611_SendAckBit(0);	
		d2 = MS5611_IIC_ReceiveByte();
		 MS5611_SendAckBit(1);
		MS5611_IIC_Stop();		
		Cal_C[i] = (d1 << 0x08) | d2;
	}
	
}


/*
 * ��ѹ����Ϊ�߶�ֵ(cm)
 */
float getEstimatedAltitude(int32_t baroPressure)
{
    static float Altitude;
 
    if(Alt_offset_Pa == 0){ 
        if(paInitCnt > PA_OFFSET_INIT_NUM){
            Alt_offset_Pa = paOffsetNum / paInitCnt;
            paOffsetInited=1;
        }else
        paOffsetNum += baroPressure;  
        paInitCnt++; 
        Altitude = 0; 
return Altitude;
 
    }
 
    Altitude = 4433000.0f * (1 - powf((((float) baroPressure) / Alt_offset_Pa), 0.190295f));
 
return Altitude; 
}

