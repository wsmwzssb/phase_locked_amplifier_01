/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*ADC采集储存数组*/
#define ADC_BUFFER_LENGTH 1000
uint16_t adc_buffer[ADC_BUFFER_LENGTH] ;
float adc_voltage;//ADC转换电压值
/*目标频率,输出模式*/
uint16_t f;
uint8_t output_mode;
/*UART中断接收接收字符*/
#define RX_BUFFER_SIZE 64
uint8_t rx_buffer[RX_BUFFER_SIZE];  // 用于UART中断接收
uint8_t rx_index = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	printf("Phase Locked Amplifier.");
	//开启DMA、PWM
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_buffer,ADC_BUFFER_LENGTH);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)adc_buffer, ADC_BUFFER_LENGTH, DAC_ALIGN_12B_R);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)adc_buffer, ADC_BUFFER_LENGTH, DAC_ALIGN_12B_R);
	//开启命令接收
	HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*ADC*/
//ADC采集一半，进入中断
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        // 处理 adc_buffer[0] ~ adc_buffer[ADC_BUFFER_LENGTH/2]
			
    }
}

//ADC采集完成，进入中断
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	{
		/*
		//调试用
		 printf("DMA 采样完成，数据如下：\r\n");
    for (int i = 0; i < ADC_BUFFER_LENGTH; i++)
    {
			adc_voltage = (adc_buffer[i]*3.3)/4095;
     printf("%d ADC  %d  %f\r\n",i,adc_buffer[i],adc_voltage);
    }
    printf("\r\n");
		*/

	}
}
/*DAC*/
//DAC读取一半，进入中断
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    //
}
//DAC读取完成，进入中断
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    //printf("DAC DMA 传输完成\r\n");
}
/*USART*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     if(huart->Instance == USART1)
    {
			/*设定频率值*/
        // 回车或换行符判断命令结束
        if(rx_buffer[rx_index] == '\n' || rx_buffer[rx_index] == '\r')
        {
            rx_buffer[rx_index] = '\0';  // 结束符
            rx_index = 0;

            // 判断命令格式是否正确
            if (strncmp((char *)rx_buffer, "Target Frequency ", 17) == 0)
            {
                int freq = atoi((char *)&rx_buffer[17]);  // 将字符串转换为整数
                if (freq >= 0 && freq <= 10000)  // 简单范围检查
                {
                  f = freq;
									printf("Frequency set to: %d\r\nOutput Mode set to: %d\r\n", f, output_mode);
                }
                else
                {
                    printf("Invalid frequency value\r\n");
                }
            }
						else if (strncmp((char *)rx_buffer, "Output Mode ", 12) == 0)
						{
							int mode = atoi((char *)&rx_buffer[12]);  // 将字符串转换为整数
                if (mode >= 1 && mode <= 3)  // 简单范围检查
                {
                    output_mode = mode;
                    printf("Frequency set to: %d\r\nOutput Mode set to: %d\r\n", f, output_mode);
                }
                else
                {
                    printf("Invalid Output Mode value\r\n");
                }
						}
            else
            {
                printf("Unknown command\r\n");
            }
        }
        else
        {
            rx_index++;
            if(rx_index >= RX_BUFFER_SIZE)
            {
                rx_index = 0;  // 防止越界
                printf("Command too long\r\n");
            }
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
    
}
/**/
/* USER CODE END 4 */

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
