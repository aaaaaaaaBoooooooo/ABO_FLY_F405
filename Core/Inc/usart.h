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

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
void uart_printf(UART_HandleTypeDef *huart,const char *format, ...);
void aircraft_data_send(void);
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#define REMOTE_DATA_NUM 7
#define AIRCFAFT_DARA_NUM 17
#define UART3_RXBUFFERSIZE 128
#define UART6_RXBUFFERSIZE 128
#define UART4_RXBUFFERSIZE 128
#define UART1_RXBUFFERSIZE 128
extern uint8_t uart3_rx_buff[UART3_RXBUFFERSIZE];//串口3接收缓冲区
extern uint8_t uart6_rx_buff[UART6_RXBUFFERSIZE];//串口6接收缓冲区
extern uint8_t uart4_rx_buff[UART4_RXBUFFERSIZE];//串口4接收缓冲区
extern uint8_t uart1_rx_buff[UART1_RXBUFFERSIZE];//串口1接收缓冲区
extern uint8_t remote_data_flash[2];
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

