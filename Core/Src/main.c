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
#include <math.h>    // 需要链接 -lm
#include <time.h>    // 添加time.h用于计时
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- 全局常量定义 ---
#ifndef M_PI
#define M_PI (3.14159265358979323846f) // 定义 PI (float)
#endif

// --- 输出模式控制 ---
typedef enum {
    OUTPUT_MODE_RECOVERED_SIGNAL = 1,
    OUTPUT_MODE_INTERNAL_REF_SIGNALS = 2,
    OUTPUT_MODE_REF_COS_AND_RECOVERED = 3
} OutputMode;

// 全局变量，用于控制输出模式
OutputMode g_output_mode = OUTPUT_MODE_REF_COS_AND_RECOVERED;

// --- 可配置参数 ---
// 信号参数
#define SAMPLING_RATE 1000 // 采样率 (Hz)，例如100kHz
#define NUM_SAMPLES ((int)(SAMPLING_RATE * SIGNAL_DURATION)) // 总采样点数
#define AVERAGE_TIME 0.2f      // 平均时间窗口长度（秒）
#define BUFFER_SIZE ((int)(SAMPLING_RATE * AVERAGE_TIME)) // 缓冲区大小


// 待测信号1参数 (用于生成模拟输入)
#define SIGNAL1_DURATION 25.0f  // 第一段信号持续时间 (秒)
#define INPUT1_FREQ 50.0f       // 待测信号1中的目标频率 (Hz)
#define INPUT1_AMPLITUDE 1.3f   // 待测信号1中目标频率分量的幅值 (V)
#define INPUT1_PHASE_DEG 80.0f  // 待测信号1中目标频率分量的相位 (度)
#define INPUT1_DC_OFFSET 1.7f   // 待测信号1的直流偏置 (V)

// 待测信号2参数 (用于生成模拟输入)
#define SIGNAL2_DURATION 25.0f  // 第二段信号持续时间 (秒)
#define INPUT2_FREQ 50.1f       // 待测信号2中的目标频率 (Hz)
#define INPUT2_AMPLITUDE 1.3f   // 待测信号2中目标频率分量的幅值 (V)
#define INPUT2_PHASE_DEG 15.0f  // 待测信号2中目标频率分量的相位 (度)
#define INPUT2_DC_OFFSET 1.7f   // 待测信号2的直流偏置 (V)

#define SIGNAL_DURATION (SIGNAL1_DURATION + SIGNAL2_DURATION) // 信号总持续时间 (秒)

// 可选：添加一些噪声/干扰信号
#define NOISE_FREQ_1 150.0f    // 第一个噪声频率 (Hz)
#define NOISE_AMPLITUDE_1 0.0f // 第一个噪声幅值 (V)
#define NOISE_PHASE_DEG_1 0.0f // 第一个噪声相位 (度)


// 参考信号参数
#define REF_FREQ_OFFSET 0.01f   // 参考信号相对于待测信号的频率偏移 (Hz)
float g_ref_freq = INPUT1_FREQ + REF_FREQ_OFFSET;  // 参考信号频率 (Hz)
#define LUT_SIZE 8192        // 参考信号查找表的大小 (一个周期的点数)

// 输出控制
#define PRINT_INTERVAL_SEC 0.1f   // 每隔多少秒打印一次结果
#define PRINT_INTERVAL_SAMPLES ((int)(PRINT_INTERVAL_SEC * SAMPLING_RATE)) // 转换为对应的采样点数

// --- 全局数组和变量 ---
// float* input_signal_buffer = NULL; // 修改为指针
float ref_cos_lut[LUT_SIZE];            // 参考余弦查找表
float ref_sin_lut[LUT_SIZE];            // 参考正弦查找表

// 环形缓冲区用于存储过去一秒的数据
float* X_buffer = NULL;
float* Y_buffer = NULL;
int buffer_index = 0;
int buffer_filled = 0;
// 用于增量平均值计算的全局变量
float sum_X = 0.0f;     // X缓冲区的总和
float sum_Y = 0.0f;     // Y缓冲区的总和
int buffer_count = 0;   // 当前缓冲区中有效数据数量

// 参考信号相位累加器
double ref_phase_accumulator = 0.0f;

// 在全局变量区域添加
#define MAX_FREQ_ADJUST 1.0f    // 最大频率调整范围 (Hz)
#define PHASE_DIFF_THRESHOLD 0.002f  // 相位差阈值 (度)
#define FREQ_ADJUST_STEP 0.001f   // 频率调整步长 (Hz)

// 用于频率跟踪的变量
float g_last_phase = 0.0f;      // 上一次的相位值
float g_last_freq_adjust_time = 0.0f;  // 上次频率调整的时间（秒）
int g_sample_count = 0;         // 总采样计数，用于时间计算

#define WINDOW_TIME AVERAGE_TIME  // 窗口时间长度
#define WINDOW_SIZE ((int)(WINDOW_TIME * SAMPLING_RATE)) // 窗口大小 (采样点数)
float g_window1_sum = 0.0f;     // 第一个窗口的相位和
float g_window2_sum = 0.0f;     // 第二个窗口的相位和
float g_window3_sum = 0.0f;     // 第三个窗口的相位和
float g_window4_sum = 0.0f;     // 第四个窗口的相位和
int g_window1_count = 0;        // 第一个窗口的计数
int g_window2_count = 0;        // 第二个窗口的计数
int g_window3_count = 0;        // 第三个窗口的计数
int g_window4_count = 0;        // 第四个窗口的计数
int i = 0;                      // 循环计数器
int loop_count = 0;             // 循环计数器,用于控制循环计算速度

// --- 函数声明 ---
void generate_input_signal_buffer(void);
void generate_reference_lut(void);
void get_reference_samples(float* cos_val, float* sin_val);
void calculate_amplitude_and_phase(float X, float Y, float* amplitude, float* phase_deg);
float get_simulated_adc_sample(int sample_index);
void update_reference_frequency(float new_freq);
int auto_frequency_tracking(float current_phase);


uint8_t flag_state = 1; // 用于标记ADC采集数组是否处于半满状态






/*ADC采集储存数组*/
#define ADC_BUFFER_LENGTH 1000 //(双数)
#define ADC_BUFFER_HALF_LENGTH (ADC_BUFFER_LENGTH / 2) // 半个缓冲区长度
uint16_t input_signal_buffer[ADC_BUFFER_LENGTH] ;
float adc_voltage;//ADC转换电压值

uint16_t output_chanel_1[ADC_BUFFER_LENGTH]; // 输出通道1
uint16_t output_chanel_2[ADC_BUFFER_LENGTH]; // 输出通道2

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
	printf("Phase Locked Amplifier.\r\n");
	//开启DMA、PWM
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&input_signal_buffer,ADC_BUFFER_LENGTH);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)input_signal_buffer, ADC_BUFFER_LENGTH, DAC_ALIGN_12B_R);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)output_chanel_2, ADC_BUFFER_LENGTH, DAC_ALIGN_12B_R);
	//开启命令接收
	HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int ppp=0;
    while (1) { 
				ppp = loop_count;
        // 记数超过一千次，则while循环不执行后续任务 TODO数组半满的时候把loop_count清零
        while (loop_count >= ADC_BUFFER_HALF_LENGTH) {

        }
        if (loop_count == 0) {
            flag_state = 1 - flag_state; // 切换标志位

            // 把ADC数组复制到input_signal_buffer中(暂时不做)
        }

        
        
        // // 调试，在time为30s时，更新参考频率 TODO 控制频率命令
        // float time_30s = 25.0f;
        // int sample_index_30s = (int)(time_30s * SAMPLING_RATE);
        // if (i == sample_index_30s) {
        //     update_reference_frequency(101.0f);
        // }


        // 4.1 模拟从ADC获取当前采样值
        float current_input_sample = get_simulated_adc_sample(i);

        // 4.2 从LUT获取当前参考信号样本
        float current_ref_cos, current_ref_sin; // 定义局部变量
        get_reference_samples(&current_ref_cos, &current_ref_sin); // 修正变量名

        // 4.3 混频 (Phase Sensitive Detection - PSD)
        float mixed_X = current_input_sample * current_ref_cos;
        float mixed_Y = current_input_sample * current_ref_sin;

        // 4.4 更新缓冲区并计算平均值
        float current_filtered_X, current_filtered_Y;
        // X_buffer[buffer_index] = mixed_X;
        // Y_buffer[buffer_index] = mixed_Y;
        // if (buffer_index >= BUFFER_SIZE - 1) {
        //     buffer_filled = 1;
        // }
        // // 计算平均值
        // float sum_X = 0.0f;
        // float sum_Y = 0.0f;
        // // 始终使用完整缓冲区大小进行平均，未填充的部分自动为0
        // for (int j = 0; j < BUFFER_SIZE; j++) {
        //     if (j <= buffer_index || buffer_filled) {
        //         sum_X += X_buffer[j];
        //         sum_Y += Y_buffer[j];
        //     }
        // }
        // // 始终除以完整的缓冲区大小
        // current_filtered_X = sum_X / BUFFER_SIZE;
        // current_filtered_Y = sum_Y / BUFFER_SIZE;
        // // 更新缓冲区索引
        // buffer_index = (buffer_index + 1) % BUFFER_SIZE;
        // 4.4 更新缓冲区并计算平均值 - 优化版本
        // 如果缓冲区已满，减去要被覆盖的值
        if (buffer_count >= BUFFER_SIZE) {
            sum_X -= X_buffer[buffer_index];
            sum_Y -= Y_buffer[buffer_index];
        } else {
            buffer_count++;
        }
        // 添加新值到缓冲区和总和
        X_buffer[buffer_index] = mixed_X;
        Y_buffer[buffer_index] = mixed_Y;
        sum_X += mixed_X;
        sum_Y += mixed_Y;
        // 计算平均值
        current_filtered_X = sum_X / buffer_count;
        current_filtered_Y = sum_Y / buffer_count;
        // 更新缓冲区索引
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;

        // 4.5 计算幅值和相位
        float recovered_amplitude, recovered_phase_deg;
        calculate_amplitude_and_phase(current_filtered_X, current_filtered_Y, &recovered_amplitude, &recovered_phase_deg);

        // 4.5.1 执行自动频率跟踪
        int freq_adjusted = auto_frequency_tracking(recovered_phase_deg);
        if (freq_adjusted) {
            printf("时间: %.2f s, 参考频率已调整至: %.3f Hz\n", 
                   (float)i / SAMPLING_RATE, g_ref_freq);
        }

        // 4.6 还原信号
        float current_time = (float)i / SAMPLING_RATE;
        float recovered_phase_rad = recovered_phase_deg * M_PI / 180.0f;
        float recovered_signal = recovered_amplitude * cosf(2.0f * M_PI * g_ref_freq * current_time - recovered_phase_rad);

        // 4.7 定期输出结果
        // // 调试内容
        // if ((i + 1) % PRINT_INTERVAL_SAMPLES == 0) { 
        //     printf("time:%.4f\t input:%.4f\t X:%.4f\t Y:%.4f\t amp:%.4f\t phase:%.2f\t rec:%.4f\t freq:%.4f\n",
        //            current_time,
        //            current_input_sample,
        //            current_filtered_X,
        //            current_filtered_Y,
        //            recovered_amplitude,
        //            recovered_phase_deg,
        //            recovered_signal,
        //            g_ref_freq);
        //     fflush(stdout);
        // }

        // 第一部分输出：重建信号的频率、幅值和相位  TODO串口输出
        // 每秒输出一次
        if (i % 100 == 0) {
    printf("Frequency: %.2f Hz, Amplitude: %.4f V, Phase: %.2f deg\n", g_ref_freq, recovered_amplitude, recovered_phase_deg);
}

        // 根据输出模式添加新的输出  TODO DAC输出
        // printf("--- 当前输出模式 %d 的额外信息 ---\n", g_output_mode);
        switch (g_output_mode) {
            case OUTPUT_MODE_RECOVERED_SIGNAL:
                // printf("待测信号: %.4f, 重建信号: %.4f V\n", current_input_sample, recovered_signal);
                // fprintf(fp_2, "%.4f,%.4f\n", current_input_sample, recovered_signal);
                if (flag_state == 0) {
                    output_chanel_1[i] = (uint16_t)(current_input_sample * 4095 / 3.3);
                    output_chanel_2[i] = (uint16_t)(recovered_signal * 4095 / 3.3);
                } else {
                    output_chanel_1[i+ADC_BUFFER_HALF_LENGTH] = (uint16_t)(current_input_sample * 4095 / 3.3);
                    output_chanel_2[i+ADC_BUFFER_HALF_LENGTH] = (uint16_t)(recovered_signal * 4095 / 3.3);
                }
                break;
            case OUTPUT_MODE_INTERNAL_REF_SIGNALS:
                // printf("内部参考信号 - 余弦: %.4f, 正弦: %.4f\n", current_ref_cos, current_ref_sin);
                // fprintf(fp_2, "%.4f,%.4f\n", current_ref_cos, current_ref_sin);
                if (flag_state == 0) {
                    output_chanel_1[i] = (uint16_t)(current_ref_cos * 4095 / 3.3);
                    output_chanel_2[i] = (uint16_t)(current_ref_sin * 4095 / 3.3);
                } else {
                    output_chanel_1[i+ADC_BUFFER_HALF_LENGTH] = (uint16_t)(current_ref_cos * 4095 / 3.3);
                    output_chanel_2[i+ADC_BUFFER_HALF_LENGTH] = (uint16_t)(current_ref_sin * 4095 / 3.3);
                }
                break;
            case OUTPUT_MODE_REF_COS_AND_RECOVERED:
                // printf("参考余弦: %.4f, 重建信号: %.4f V\n", current_ref_cos, recovered_signal);
                // fprintf(fp_2, "%.4f,%.4f\n", current_ref_cos, recovered_signal);
                if (flag_state == 0) {
                    output_chanel_1[i] = (uint16_t)(current_ref_cos * 4095 / 3.3);
                    output_chanel_2[i] = (uint16_t)(recovered_signal * 4095 / 3.3);
                } else {
                    output_chanel_1[i+ADC_BUFFER_HALF_LENGTH] = (uint16_t)(current_ref_cos * 4095 / 3.3);
                    output_chanel_2[i+ADC_BUFFER_HALF_LENGTH] = (uint16_t)(recovered_signal * 4095 / 3.3);
                }
                break;
        }


        // // 输出最后100个样本 (调试用)
        // if (i >= NUM_SAMPLES - 100) {
        //     printf("样本 %d: 输入=%.4f V, X_filt=%.4f, Y_filt=%.4f, 幅值=%.4f V, 相位=%.2f 度\n",
        //            i, current_input_sample, current_filtered_X, current_filtered_Y,
        //            recovered_amplitude, recovered_phase_deg);
        // }


        i++; // 在循环末尾递增采样索引
        loop_count++;
				printf("i:%d  loop count:%d\n",i,loop_count);
        // // 调试用的退出条件
        // if (i >= NUM_SAMPLES) {
        //     break;
        // }
    
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
        loop_count = 0; // 重置循环计数器
			//printf("reset over\nloop count: %d",loop_count);
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
/*
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    //
}
//DAC读取完成，进入中断
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
    //printf("DAC DMA 传输完成\r\n");
}
*/
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






/**
 * @brief 从预生成的缓冲区获取一个模拟的ADC样本
 * @param sample_index 当前的样本索引
 * @return 该样本的电压值
 */
float get_simulated_adc_sample(int sample_index) {
    // if (sample_index >= 0 && sample_index < NUM_SAMPLES) {
    //     return input_signal_buffer[sample_index];
    // }
    if (flag_state == 0) {
        // 如果标志位为0，返回第一个缓冲区的值
        return (float)input_signal_buffer[sample_index] * 3.3 / 4095;
    } else {
        // 如果标志位为1，返回第二个缓冲区的值
        return (float)input_signal_buffer[sample_index + ADC_BUFFER_HALF_LENGTH] * 3.3 / 4095;
    }

}

/**
 * @brief 生成输入信号缓冲区
 * 该函数生成两个不同频率的正弦信号，并添加噪声成分。
 * 信号1和信号2的持续时间由 SIGNAL1_DURATION 和 SIGNAL2_DURATION 定义。
 */
void generate_input_signal_buffer(void) {
    float t;
    
    // 计算信号1和信号2的采样点边界
    int signal1_samples = (int)(SIGNAL1_DURATION * SAMPLING_RATE);
    int signal2_samples = (int)(SIGNAL2_DURATION * SAMPLING_RATE);
    
    // 确保不超过总采样点数
    if (signal1_samples + signal2_samples > NUM_SAMPLES) {
        signal1_samples = NUM_SAMPLES / 2;
        signal2_samples = NUM_SAMPLES - signal1_samples;
    }

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        t = (float)i / SAMPLING_RATE;
        
        if (i < signal1_samples) {
            // 生成信号1
            float phase_rad1 = INPUT1_PHASE_DEG * M_PI / 180.0f;
            float noise_phase_rad1 = NOISE_PHASE_DEG_1 * M_PI / 180.0f;
            
            float target_component = INPUT1_AMPLITUDE * cosf(2.0f * M_PI * INPUT1_FREQ * t - phase_rad1);
            float noise_component = NOISE_AMPLITUDE_1 * cosf(2.0f * M_PI * NOISE_FREQ_1 * t - noise_phase_rad1);
            
            input_signal_buffer[i] = INPUT1_DC_OFFSET + target_component + noise_component;
        } else {
            // 生成信号2
            float phase_rad2 = INPUT2_PHASE_DEG * M_PI / 180.0f;
            float noise_phase_rad1 = NOISE_PHASE_DEG_1 * M_PI / 180.0f;
            
            float target_component = INPUT2_AMPLITUDE * cosf(2.0f * M_PI * INPUT2_FREQ * t - phase_rad2);
            float noise_component = NOISE_AMPLITUDE_1 * cosf(2.0f * M_PI * NOISE_FREQ_1 * t - noise_phase_rad1);
            
            input_signal_buffer[i] = INPUT2_DC_OFFSET + target_component + noise_component;
        }
    }
}

/**
 * @brief 生成参考正余弦查找表 (LUT)
 * LUT覆盖一个完整的 2*PI 周期
 */
void generate_reference_lut(void) {
    for (int i = 0; i < LUT_SIZE; ++i) {
        float angle = 2.0f * M_PI * (float)i / (float)LUT_SIZE;
        ref_cos_lut[i] = cosf(angle);
        ref_sin_lut[i] = sinf(angle);
    }
}

/**
 * @brief 更新参考信号频率
 * @param new_freq 新的参考信号频率 (Hz)
 */
void update_reference_frequency(float new_freq) {
    g_ref_freq = new_freq;
    // 重置相位累加器
    ref_phase_accumulator = 0.0f;
    // 记录本次频率调整的时间
    float current_time = (float)g_sample_count / SAMPLING_RATE;
    g_last_freq_adjust_time = current_time;
}

/**
 * @brief 从查找表获取当前周期的参考正余弦值，并更新相位累加器
 *
 * @param cos_val 指针，用于返回当前的余弦参考值
 * @param sin_val 指针，用于返回当前的正弦参考值
 */
void get_reference_samples(float* cos_val, float* sin_val) {
    
    float phase_increment = (g_ref_freq / SAMPLING_RATE) * (float)LUT_SIZE;

    // // 查表方法 (四舍五入)
    // unsigned int lut_index = (unsigned int)(ref_phase_accumulator + 0.5f) % LUT_SIZE;
    // *cos_val = ref_cos_lut[lut_index];
    // *sin_val = ref_sin_lut[lut_index];

    // 使用线性插值
    float current_exact_phase = ref_phase_accumulator; // 当前的精确相位
    // 计算插值所需的整数索引和小数部分
    // floorf确保向下取整, 例如 current_exact_phase = 3.7, index_floor = 3
    // current_exact_phase = 3.0, index_floor = 3
    unsigned int index_floor = (unsigned int)floorf(current_exact_phase);
    float fraction = current_exact_phase - (float)index_floor;
    unsigned int lut_idx0 = index_floor % LUT_SIZE;
    unsigned int lut_idx1 = (index_floor + 1) % LUT_SIZE;
    // 对余弦分量进行线性插值
    float cos_val0 = ref_cos_lut[lut_idx0];
    float cos_val1 = ref_cos_lut[lut_idx1];
    *cos_val = cos_val0 + fraction * (cos_val1 - cos_val0);
    // 对正弦分量进行线性插值
    float sin_val0 = ref_sin_lut[lut_idx0];
    float sin_val1 = ref_sin_lut[lut_idx1];
    *sin_val = sin_val0 + fraction * (sin_val1 - sin_val0);

    // 更新相位累加器
    ref_phase_accumulator += phase_increment;
    if (ref_phase_accumulator >= (float)LUT_SIZE) {
         ref_phase_accumulator = fmodf(ref_phase_accumulator, (float)LUT_SIZE);
    }
}
// void get_reference_samples(float* cos_val, float* sin_val) {
//     // 静态整数计数器，跟踪总的采样点数
//     static unsigned int n_sample_count = 0;
//     // 1. 计算从开始到现在的总相位（未进行周期缠绕）
//     float total_unwrapped_phase = (float)n_sample_count * (REF_FREQ / SAMPLING_RATE) * (float)LUT_SIZE;
//     // 2. 将总相位映射到当前 LUT 的一个周期内 [0, LUT_SIZE)
//     float current_phase_in_lut = fmodf(total_unwrapped_phase, (float)LUT_SIZE);
//     // 3. 计算 LUT 索引，使用 +0.5f 实现四舍五入到最近的整数索引
//     unsigned int lut_index = (unsigned int)(current_phase_in_lut + 0.5f);

//     // 4. 取模确保索引在 [0, LUT_SIZE-1] 范围内
//     lut_index %= LUT_SIZE;
//     // 5. 从查找表获取参考值
//     *cos_val = ref_cos_lut[lut_index];
//     *sin_val = ref_sin_lut[lut_index];

//     // 6. 增加采样计数
//     n_sample_count++;
// }

/**
 * @brief 根据 X 和 Y 计算原始信号的幅值和相位
 * 输入信号模型: A_in * cos(2*pi*f*t - phi_in)
 * X = (A_in/2) * cos(phi_in)
 * Y = (A_in/2) * sin(phi_in)
 *
 * @param X 相关滤波得到的 X 分量 (已滤波)
 * @param Y 相关滤波得到的 Y 分量 (已滤波)
 * @param amplitude 输出参数，存储计算得到的幅值 (A_in)
 * @param phase_deg 输出参数，存储计算得到的相位 (phi_in, 度)
 */
void calculate_amplitude_and_phase(float X, float Y, float* amplitude, float* phase_deg) {
    *amplitude = 2.0f * sqrtf(X * X + Y * Y);
    float phase_rad = atan2f(Y, X);
    *phase_deg = phase_rad * 180.0f / M_PI;
}

/**
 * @brief 自动频率跟踪算法
 * @param current_phase 当前测量的相位值 (度)
 * @return 是否需要调整频率
 */
int auto_frequency_tracking(float current_phase) {
    // 获取当前时间
    float current_time = (float)g_sample_count / SAMPLING_RATE;
    g_sample_count++;
      // 如果距离上次频率调整的时间小于AVERAGE_TIME，直接返回
    if (current_time - g_last_freq_adjust_time < AVERAGE_TIME*4) {
        // 重置所有窗口数据
        g_window1_sum = 0.0f;
        g_window2_sum = 0.0f;
        g_window3_sum = 0.0f;
        g_window4_sum = 0.0f;
        g_window1_count = 0;
        g_window2_count = 0;
        g_window3_count = 0;
        g_window4_count = 0;
        return 0;
    }
      // 根据当前窗口状态更新相应的累加器
    if (g_window1_count < WINDOW_SIZE) {
        g_window1_sum += current_phase;
        g_window1_count++;
    } else if (g_window2_count < WINDOW_SIZE) {
        g_window2_sum += current_phase;
        g_window2_count++;
    } else if (g_window3_count < WINDOW_SIZE) {
        g_window3_sum += current_phase;
        g_window3_count++;
    } else if (g_window4_count < WINDOW_SIZE) {
        g_window4_sum += current_phase;
        g_window4_count++;
    }
      // 当四个窗口都填满时，进行比较和判断
    if (g_window1_count == WINDOW_SIZE && g_window2_count == WINDOW_SIZE && 
        g_window3_count == WINDOW_SIZE && g_window4_count == WINDOW_SIZE) {
        // 计算四个窗口的平均相位
        float avg_phase1 = g_window1_sum / WINDOW_SIZE;
        float avg_phase2 = g_window2_sum / WINDOW_SIZE;
        float avg_phase3 = g_window3_sum / WINDOW_SIZE;
        float avg_phase4 = g_window4_sum / WINDOW_SIZE;
        
        // 计算窗口之间的相位差
        float phase_diff_21 = avg_phase2 - avg_phase1; // 窗口2-窗口1
        float phase_diff_32 = avg_phase3 - avg_phase2; // 窗口3-窗口2
        float phase_diff_43 = avg_phase4 - avg_phase3; // 窗口4-窗口3
        
        // 处理相位跳变
        if (phase_diff_21 > 180.0f) phase_diff_21 -= 360.0f;
        if (phase_diff_21 < -180.0f) phase_diff_21 += 360.0f;
        if (phase_diff_32 > 180.0f) phase_diff_32 -= 360.0f;
        if (phase_diff_32 < -180.0f) phase_diff_32 += 360.0f;
        if (phase_diff_43 > 180.0f) phase_diff_43 -= 360.0f;
        if (phase_diff_43 < -180.0f) phase_diff_43 += 360.0f;
        
        // 只有当三个相位差都超过阈值时才调整频率
        if (fabsf(phase_diff_21) > PHASE_DIFF_THRESHOLD && 
            fabsf(phase_diff_32) > PHASE_DIFF_THRESHOLD && 
            fabsf(phase_diff_43) > PHASE_DIFF_THRESHOLD) {
            // 使用最后一个相位差作为频率调整的依据
            float phase_diff = phase_diff_43; // 使用最新的相位差
            
            // 理论精确补偿 (Δf = Δφ / (360 × T_window))
            float freq_adjust = -phase_diff / (360.0f * WINDOW_TIME);
            printf("时间: %.2f s, 窗口1平均相位: %.6f, 窗口2平均相位: %.6f, 窗口3平均相位: %.6f, 窗口4平均相位: %.6f\n",
                   current_time, avg_phase1, avg_phase2, avg_phase3, avg_phase4);
            printf("相位差(2-1): %.6f, 相位差(3-2): %.6f, 相位差(4-3): %.6f, freq_adjust: %.6f\n", 
                   phase_diff_21, phase_diff_32, phase_diff_43, freq_adjust);
            
            // 限制调整范围
            if (freq_adjust > MAX_FREQ_ADJUST) freq_adjust = MAX_FREQ_ADJUST;
            if (freq_adjust < -MAX_FREQ_ADJUST) freq_adjust = -MAX_FREQ_ADJUST;
            
            // 更新参考频率
            float new_freq = g_ref_freq + freq_adjust;
            printf("new_freq: %.6f\n", new_freq);
            update_reference_frequency(new_freq);
            
            // 重置所有窗口数据
            g_window1_sum = 0.0f;
            g_window2_sum = 0.0f;
            g_window3_sum = 0.0f;
            g_window4_sum = 0.0f;
            g_window1_count = 0;
            g_window2_count = 0;
            g_window3_count = 0;
            g_window4_count = 0;
            
            return 1;  // 表示频率已调整
        }
        
        // 准备下一轮比较：将窗口数据前移
        g_window1_sum = g_window2_sum;
        g_window2_sum = g_window3_sum;
        g_window3_sum = g_window4_sum;
        g_window4_sum = 0.0f;
        g_window1_count = g_window2_count;
        g_window2_count = g_window3_count;
        g_window3_count = g_window4_count;
        g_window4_count = 0;
    }
    
    g_last_phase = current_phase;
    return 0;  // 表示频率未调整
}




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
