/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include "control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct 
{
	uint8_t Battery_Volt;//电池电量
	uint8_t YG_LEFT_UD;//左摇杆上下打杆量
	uint8_t YG_LEFT_LR;//左摇杆左右打杆量
	uint8_t YG_LEFT_KEY;//左摇杆按键
	uint8_t YG_RIGHT_UD;//右摇杆上下打杆量
	uint8_t YG_RIGHT_LR;//右摇杆左右打杆量
	uint8_t YG_RIGHT_KEY;//右摇杆按键
	uint8_t KEY_OK;
	uint8_t KEY_BACK;
	uint8_t KEY_UP;
	uint8_t KEY_DOWN;
	uint8_t KEY_LEFT;
	uint8_t KEY_RIGHT;
	
}remote_type;//遥控器类
typedef struct 
{
	uint8_t Battery_Volt;//电池电量
	uint8_t Throttle;//油门
	uint8_t Motor_PWM_duty[4];//电机PWM占空比
	int8_t YAW;//偏航角
	int8_t PITCH;//俯仰角
	int8_t ROLL;//横滚角
	uint16_t Height;//高度
	float Pressure;//气压
	float Altitude;//海拔
	float Temperature;//温度
	uint8_t status;//状态
}aircraft_type;//飞行器类

extern remote_type my_remote;
extern aircraft_type my_aircraft;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOC
#define BADC_in_Pin GPIO_PIN_3
#define BADC_in_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_8
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_INT1_EXTI_IRQn EXTI9_5_IRQn
#define IMU_INT2_Pin GPIO_PIN_9
#define IMU_INT2_GPIO_Port GPIOC
#define IMU_INT2_EXTI_IRQn EXTI9_5_IRQn
#define BMP_INT_Pin GPIO_PIN_5
#define BMP_INT_GPIO_Port GPIOB
#define BMP_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
