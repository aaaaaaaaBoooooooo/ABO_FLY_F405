/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "bmp390_task.h"
#include "icm42688.h"
#include "angle.h"
#include "control.h"
#include "W25QXX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
remote_type my_remote;//遥控器结构体
aircraft_type my_aircraft;//飞行器结构体
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DATA_FILE_NAME "fly_data.txt"
char DATA_FILE_BUFFER[65];//一次写入的格式  "time:data1,data2,data3,data4,data5,data6\r\n"
/*无人机飞行数据采集*/
void aircraft_fly_data_store()
{
	uint32_t time,time_0;
	static uint8_t aircraft_last_status=0;
	FIL file;     // 文件句柄
	FRESULT res;
  UINT bytes_written;  // 写入的字节数

	while(1)
	{
		if(aircraft_last_status == 0x00 && my_aircraft.status == 0x01 )
		{
			aircraft_last_status = 0x01;
			time_0 = HAL_GetTick();
			/***打开文件***/
			res = f_open(&file, DATA_FILE_NAME,FA_WRITE);	
		}
		else if(aircraft_last_status == 0x01 && my_aircraft.status ==0x00 )
		{
			aircraft_last_status = 0x00;
			/***关闭文件***/
			f_close(&file);
		}
		if(my_fatfs_init_success&&my_aircraft.status & 0x01)//飞机正在飞行
		{
			delay_us(550);
			if (res == FR_OK)
			{
				time = HAL_GetTick() - time_0;
				sprintf(DATA_FILE_BUFFER,"%-6d:%13f,%13f,%13f,%13f\r\n",time,my_ahrs.Angle_Data.pitch,AttitudeControl.pitch_target_angle,my_ahrs.Angle_Data.roll,AttitudeControl.roll_target_angle);
				if(f_size(&file) < (12*1024*1024))
				{
					/****移动文件读写指针到文件结束处，以便添加数据***/
					f_lseek(&file,f_size(&file));
					// 写入数据到文件
					f_write(&file, DATA_FILE_BUFFER, sizeof(DATA_FILE_BUFFER), &bytes_written);
				}
			}
			else
			{
				while(1)
				{
						
				}
			}
		}
	}
}
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
	
	delay_init(168); /* 初始化延迟函数 */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	LED(1);//点亮LED		

 	while(icm42688_init())	//陀螺仪初始化
	{
		printf("ICM42688 Init Failed!");
		LED_TOGGLE;
		delay_ms(100);

	}
	while(BMP390_Init()) 		//气压计初始化
	{
		printf("BMP390 Init Failed!");
		LED_TOGGLE;
		delay_ms(100);		
	}
	delay_ms(100);
	
	IMU_Calibration();//陀螺仪零漂校准
	
	pid_control_init();
	for(uint8_t i=0;i<9;i++)//闪烁LED代表传感器初始化完成
	{
		LED_TOGGLE;
		delay_ms(100);
	}
	HAL_UART_Receive_DMA(&huart3,uart3_rx_buff,2*REMOTE_DATA_NUM);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4,uart4_rx_buff,UART4_RXBUFFERSIZE);//开启UART4空闲中断，DMA接收测距数据
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,uart6_rx_buff,UART6_RXBUFFERSIZE);//开启UART6空闲中断，DMA接收光流数据
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim5);
	delay_ms(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		aircraft_fly_data_store();
		//delay_ms(5);
	
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void error_handler_msg_log(char *file_msg, const char *func_msg)
{
    /* Start peintf error message, don't use rtos_log  */
    printf("Error file      : %s\r\n", file_msg);
    printf("Error function  : %s\r\n", func_msg);
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
	printf("Error_Handler\n");//报告错误
	error_handler_msg_log(__FILE__, __func__);//打印出错文件和函数信息
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
