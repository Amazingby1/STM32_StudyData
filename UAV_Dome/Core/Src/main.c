/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "tftlcd.h"
#include "OLED.h"
#include "Bsp_MPU6050.h"
#include "Bsp_MS5611.h"
#include "FFT_Handle.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int PD13_Duty = 0;
extern int TimeEncoder;
float iError, dError;
//uint16_t ACCX,ACCY,ACCZ,ACCZ1;
extern int16_t AX, AY, AZ, GX, GY, GZ;
uint16_t adc_buf[2048] = {0};
extern uint8_t rx_bit;          //单字节接收缓冲

//#define RXBUFFERSIZE  256     //最大接收字节数
//char RxBuffer[RXBUFFERSIZE];   //接收数据
//uint8_t aRxBuffer[1];			    //接收中断缓冲
//uint8_t Uart1_Rx_Cnt = 0;

int fputc(int ch, FILE *f)
{
    uint8_t temp[1] = {ch};
    HAL_UART_Transmit(&huart1, temp, 1, 2);
    return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);//HAL库串口接收函数
  return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* 私有类型定义 --------------------------------------------------------------*/
//定义PID结构体
typedef struct
{
    float   SetPoint;                                 //设定目标 Desired Value
    float   SumError;                                 //误差累计
    float   Proportion;                               //比例常数 Proportional Const
    float   Integral;                                 //积分常数 Integral Const
    float   Derivative;                               //微分常数 Derivative Const
    float   LastError;                                //Error[-1]
    float   PrevError;                                //Error[-2]
} PID;

/* 私有宏定义 ----------------------------------------------------------------*/
/*************************************/
//定义PID相关宏
// 这三个参数设定对电机运行影响非常大
/*************************************/
#define  P_DATA      29                                //P参数
#define  I_DATA    1.5                         //I参数
#define  D_DATA     0// 0.2                                //D参数

static PID sPID = {0};
static PID *sptr = &sPID;

/* 扩展变量 ------------------------------------------------------------------*/
//extern uint32 CaptureNumber;

/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**************PID参数初始化********************************/
void IncPIDInit(void)
{
    sptr->LastError = 0;          //Error[-1]
    sptr->PrevError = 0;          //Error[-2]
    sptr->Proportion = P_DATA;    //比例常数 Proportional Const
    sptr->Integral = I_DATA;      //积分常数  Integral Const
    sptr->Derivative = D_DATA;    //微分常数 Derivative Const
    sptr->SetPoint = 150;         //设定目标Desired Value
}
/********************增量式PID控制设计************************************/
int IncPIDCalc(int NextPoint)
{
    int iError, iIncpid;                                //当前误差
    //sptr->SetPoint =80;
    iError = sptr->SetPoint - NextPoint;                //增量计算
    iIncpid = (sptr->Proportion * iError)               //E[k]项
              - (sptr->Integral * sptr->LastError)    //E[k-1]项
              + (sptr->Derivative * sptr->PrevError); //E[k-2]项

    sptr->PrevError = sptr->LastError;                  //存储误差，用于下次计算
    sptr->LastError = iError;
    return(iIncpid);                                    //返回增量值
}

/********************位置式 PID 控制设计************************************/
int LocPIDCalc(float NextPoint)
{

    float PWM = 0;
    iError = sptr->SetPoint - NextPoint; //偏差
    sptr->SumError += iError; //积分
    dError = iError - sptr->LastError; //微分
    sptr->LastError = iError;
    PWM = (sptr->Proportion * iError //比例项
           + sptr->Integral * sptr->SumError //积分项
           + sptr->Derivative * dError); //微分项
    return PWM;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI6_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    //__HAL_SPI_ENABLE(&hspi6);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
		
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_bit, 1);//接收中断开启函数
	// HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);//开串口中断
    /*启动ADC的DMA传输 配合下面定时器来触发ADC转换*/
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf,2048);
//    // HAL_Delay(10);
//    /*开启定时器 用溢出事件来触发ADC转换*/
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start(&hadc1);
    //LCD_Init();
    //HAL_Delay(100);
     OLED_Init();
    //LCD_Clear(BLACK); 		//清屏
    //BACK_COLOR = BLACK;
    //POINT_COLOR = CYAN;
    //LCD_ShowString(0, 0, 160, 12, 12, "Boring_TECH");
    //LCD_ShowString(0, 15, 160, 12, 12, "TFTLCD TEST 160*80");
    //LCD_ShowString(0, 30, 160, 16, 12, "STM32H723VGT6");
    //LCD_ShowString(0, 45, 160, 12, 12, "2021/8/20");
    //HAL_Delay(1000);
    IncPIDInit();
    //	MS5611_Init();
    MPU6050_Init();
/***********************************************************************/
	//	ESP8266_Link_aliyun_Init();
    //			MPU6050_IIC_Stop();
    // OLED_ShowNum(3,1,ack,3);
    //    uint8_t ID=MPU6050_GetID();
    //    OLED_ShowHexNum(2, 1,ID, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
		MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);

        //				MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);
        //	      MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);

        //	OLED_ShowNum(3,6,MS5611_Pressure,5);
        //printf("接收到的数据为：\r\n");
        float Angle_X = 0, Angle_Y = 0;
        Angle_X = atan2(AX, AZ) * 180 / 3.14f;
        Angle_Y = atan2(AY, AZ) * 180 / 3.14f;

        OLED_ShowSignedNum(2, 1, AX, 4);
        //OLED_ShowSignedNum(3, 1, AY, 4);
        OLED_ShowSignedNum(4, 1, AZ, 4);
        //				OLED_ShowSignedNum(2, 8, GX, 4);
        //			  OLED_ShowSignedNum(3, 8, GY, 4);
        //				OLED_ShowSignedNum(4, 8, GZ, 4);
        OLED_ShowSignedNum(3, 8, Angle_X, 4);
        OLED_ShowSignedNum(4, 8, Angle_Y, 4);
        OLED_ShowString(1, 1, "Speed:");
        OLED_ShowSignedNum(1, 7, TimeEncoder, 5);
        //			PD13_Duty =500;
        //			 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, PD13_Duty);
       // while(1);
//		  HAL_Delay(200);
//			printf("AT+MQTTPUB=0,\"/ib4k0kkSwCQ/MyESP8266/user/update\",\"{\\\"Angle_X=%f\\\"}\",1,0\r\n",Angle_X);
//			HAL_Delay(2000);
//		
			 HAL_Delay(200);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
