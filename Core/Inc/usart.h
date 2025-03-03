/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "math.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

typedef struct
{
	uint16_t distance_mm;
	float distance_cm;
	float distance_m;
	float d_center_x_err_mm;//与无人机中心x轴的偏差距离
	float d_center_y_err_mm;//与无人机中心y轴的偏差距离
	float distance_offset_mm;//与真实高度的零偏
	uint16_t noise;//数据噪声
	uint16_t confidence;//TOF数据可信度
	
}TOF_TypeDef;

typedef struct
{
	int16_t flow_x_integral;//X像素点累计时间内的累加位移(radians*10000)
													//除以10000乘以高度(mm)后为实际位移(mm)
	int16_t flow_y_integral;//Y像素点累计时间内的累加位移(radians*10000)
													//除以10000乘以高度(mm)后为实际位移(mm)
	uint16_t integration_timespan;//上一次发送光流数据到本次发送的累计时间（us）
	
	uint16_t ground_distance;//预留，默认值999
	uint8_t valid;     //状态值：0(0x00)为光流数据不可用
	uint8_t version; //版本号
	
	float flow_x_integral_mm;
	float flow_y_integral_mm;
	
	float d_center_x_err_mm;//与无人机中心x轴的偏差距离
	float d_center_y_err_mm;//与无人机中心y轴的偏差距离	
	
	float last_yaw;//上一次的yaw角度
	float now_yaw;//这次的yaw角度
	float last_pitch;//上一次的pitch角度
	float now_pitch;//这次的pitch角度
	float last_roll;//上一次的roll角度
	float now_roll;//这次的roll角度
	
	float flow_yaw_x_com_mm;//光流因yaw角旋转的x累加位移补偿值
	float flow_yaw_y_com_mm;//光流因yaw角旋转的y累加位移补偿值
	float flow_pitch_x_com_mm;//光流因pitch角旋转的x累加位移补偿值
	float flow_pitch_y_com_mm;//光流因pitch角旋转的y累加位移补偿值
	float flow_roll_x_com_mm;//光流因roll角旋转的x累加位移补偿值
	float flow_roll_y_com_mm;//光流因roll角旋转的y累加位移补偿值	
	float flow_x_speed;//光流水平x速度 单位m/s
	float flow_y_speed;//光流水平y速度 单位m/s
	
	
	
}OpticalFlow_TypeDef;

void uart_printf(UART_HandleTypeDef *huart,const char *format, ...);
void aircraft_data_send(void);
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#define REMOTE_DATA_NUM 7
#define AIRCFAFT_DATA_NUM 22
#define UART3_RXBUFFERSIZE 128
#define UART6_RXBUFFERSIZE 128
#define UART4_RXBUFFERSIZE 128
#define UART1_RXBUFFERSIZE 128
extern uint8_t uart3_rx_buff[UART3_RXBUFFERSIZE];//串口3接收缓冲区
extern uint8_t uart6_rx_buff[UART6_RXBUFFERSIZE];//串口6接收缓冲区
extern uint8_t uart4_rx_buff[UART4_RXBUFFERSIZE];//串口4接收缓冲区
extern uint8_t uart1_rx_buff[UART1_RXBUFFERSIZE];//串口1接收缓冲区
extern uint8_t remote_data_flash[2];
extern TOF_TypeDef TOF;
extern OpticalFlow_TypeDef OpticalFlow;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

