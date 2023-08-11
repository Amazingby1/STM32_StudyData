#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "string.h"
#include "tftlcd.h"
#include "OLED.h"
#include "Bsp_MPU6050.h"
#include "Bsp_MS5611.h"
#include "FFT_Handle.h"
uint8_t rx_bit;          //单字节接收缓冲
uint8_t rx_data[256];    //数组接收缓冲
uint8_t rx_cnt=0;         //接收计数
int16_t AX, AY, AZ, GX, GY, GZ;
int TimeEncoder = 0;
int Get_TimeEncoder(void)
{

    TimeEncoder = (short)(__HAL_TIM_GET_COUNTER(&htim1));
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    return TimeEncoder;
}

/***********************************************************************/
/**
  * @brief       定时0.05s，即50ms进入中断，进行数据处理
  * @param
  * @retval
  */
/**********************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if(htim->Instance == TIM2)
    {
        //MPU6050获取数据

        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

        //编码器获取数值进行PID计算
        //        Get_TimeEncoder();
        //			//	PD13_Duty = LocPIDCalc(TimeEncoder);
        //			   PD13_Duty=IncPIDCalc(TimeEncoder);
        //		  	if(PD13_Duty>900)PD13_Duty=900;
        //			  if(PD13_Duty<100)PD13_Duty=100;
        //        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, PD13_Duty);
        //        printf("PD13_Duty=%d\r\n", PD13_Duty);
        //			  printf("OIERROE=%f\r\n",iError);
        //        printf("TimeNum=%d", TimeEncoder);
    }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//printf("af");

	rx_data[rx_cnt] = rx_bit;
	rx_cnt=rx_cnt+1;
	
	if (rx_data[rx_cnt-1] == 0x0A  && rx_data[rx_cnt-2] == 0x0D)
	{
	//	printf("接收到的数据为：\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t *)rx_data, sizeof(rx_data),0x00FF); 
		//printf("JKJKKL\r\n");
		if(strstr((const char *)rx_data,"on"))
		{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,RESET);
		} 
		if(strstr((const char *)rx_data,"off"))
		{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,SET);
		}
		rx_cnt =0;
		memset(rx_data,0x00,sizeof(rx_data));
	}
	 HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_bit, 1);

}