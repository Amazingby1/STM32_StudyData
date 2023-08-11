#include "FFT_Handle.h"
#include <stdio.h>
#include "arm_math.h"
#include "math.h"
#include "arm_const_structs.h"
#include "gpio.h"
#include "adc.h"
#include "OLED.h"
//extern float32_t testInput_f32_10khz[2048];//only have 1024
extern uint16_t adc_buf[2048];
#define NPT 1024
#define Fs 1024
#define PI2 6.28318530717959
static float32_t INPUT_BufArray[NPT*2];
static float32_t OUTPUT_BufArray[NPT];
static float32_t Mag_BufArray[NPT];
//static float32_t Phase_f32[NPT];


//uint32_t fftSize = 1024;
uint8_t ifftFlag = 0;
uint8_t doBitReverse = 1;

float32_t maxValue;
uint16_t refIndex = 213, testIndex = 50.00;

/***********************************************************************/
/**
  * @brief     测试信号
  * @param
  * @retval
  */
/**********************************************************************/
void Creat_Single(void)
{
		//float  i=0;
    unsigned short i;
   // float fx;
    for(i=0; i<NPT*2; i++)
    {
     INPUT_BufArray[i] = 3 * sin(PI2 * i * 150.0 / Fs)+
								12*sin(PI2 * i * 37.0 / Fs)
			+5*sin(PI2 * i * 420/ Fs);
             //520 * sin(PI2 * i * 350.0 / Fs) +
            // 50 * sin(PI2 * i * 150.0 / Fs);
        //INPUT_BufArray[i] = ((signed int)fx) << 16;
    }
}
void FFT_DATA_Handle(void)
{			
	 
	uint16_t i = 0;

  /* Process the data through the CFFT/CIFFT module */
  // arm_cfft_f32(&arm_cfft_sR_f32_len1024,INPUT_BufArray, ifftFlag, doBitReverse);
  arm_rfft_fast_instance_f32 S;
	arm_rfft_fast_init_f32(&S,NPT);
//	Creat_Single();
  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  // arm_cmplx_mag_f32(INPUT_BufArray,OUTPUT_BufArray, Fs);
  arm_rfft_fast_f32(&S,INPUT_BufArray,OUTPUT_BufArray,ifftFlag);
	arm_cmplx_mag_f32(OUTPUT_BufArray,Mag_BufArray,NPT);
  /* Calculates maxValue and returns corresponding BIN value */
  arm_max_f32(Mag_BufArray,500,&maxValue, &testIndex);   //500指的不是频率，而是i的数值，表示最大值的截止索引
  /* 串口打印求解的幅频和相频 */
  //HAL_Delay(100);
  /***********取最大值************/
  //	Max_Fre=Compare_Max();
	for(i=0;i<NPT/2;i++)
	printf("i:%3d, f:%.2f, Power:%5f\r\n", i, (float)i*Fs/NPT, Mag_BufArray[i]);
	printf("Max=%f  Fre=%f\r\n",maxValue,testIndex);
}


/***********************************************************************/
/**
  * @brief      中断处理函数
  * @param      进行FFT运算后，返回结果
  * @retval
  */
///**********************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   	uint16_t i = 0;
  //  float Max_Fre=0;
    HAL_ADC_Stop_DMA(&hadc1);							//完成一次测量 关闭DMA传输
    //Creat_Single();
    printf("Start  into  FFT  interrupt    \r\n");
   	
    //填充数组
    for(i=0;i<2048;i++)
	{
		 printf("adc_Value=%d",adc_buf[i]);
		//INPUT_BufArray[i] =adc_buf[i];//((signed short)(adc_buf[i])) << 16;
	}		
  	
	HAL_ADC_Start_DMA(&hadc1, adc_buf, 2048);
}
void ESP8266_Link_aliyun_Init(void)
{
	    //连接阿里云的代码
		printf("AT+RST\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		printf("AT+CWMODE=1\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
		printf("AT+CIPSNTPCFG=1,8,\"ntp1.aliyun.com\"\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
	  printf("AT+CWJAP=\"RedmiK40S\",\"qwe12345\"\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		printf("AT+MQTTUSERCFG=0,1,\"NULL\",\"MyESP8266&ib4k0kkSwCQ\",\"d97930f90ce7c2a053c31854b67b370b305bbce6d6c5fed0f237fea050e56d70\",0,0,\"\"\r\n");
    HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		printf("AT+MQTTCLIENTID=0,\"ib4k0kkSwCQ.MyESP8266|securemode=2\\,signmethod=hmacsha256\\,timestamp=1679055124637|\"\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		printf("AT+MQTTCONN=0,\"iot-06z00i8tqpjec6w.mqtt.iothub.aliyuncs.com\",1883,1\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		HAL_Delay(2000);
		HAL_Delay(2000);	
		printf("AT+MQTTSUB=0,\"/ib4k0kkSwCQ/MyESP8266/user/get\",1\r\n");
		HAL_Delay(1000);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		OLED_ShowString(3,1,"Stop");
}
