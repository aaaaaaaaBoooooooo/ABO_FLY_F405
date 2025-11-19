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
	float d_center_x_err_mm;//�����˻�����x���ƫ�����
	float d_center_y_err_mm;//�����˻�����y���ƫ�����
	float distance_offset_mm;//����ʵ�߶ȵ���ƫ
	uint16_t noise;//��������
	uint16_t confidence;//TOF���ݿ��Ŷ�
	uint8_t is_valid;//tof is valid 
	
}TOF_TypeDef;

typedef struct
{
	int16_t flow_x_integral;//X���ص��ۼ�ʱ���ڵ��ۼ�λ��(radians*10000)
													//����10000���Ը߶�(mm)��Ϊʵ��λ��(mm)
	int16_t flow_y_integral;//Y���ص��ۼ�ʱ���ڵ��ۼ�λ��(radians*10000)
													//����10000���Ը߶�(mm)��Ϊʵ��λ��(mm)
	uint16_t integration_timespan;//��һ�η��͹������ݵ����η��͵��ۼ�ʱ�䣨us��
	
	uint16_t ground_distance;//Ԥ����Ĭ��ֵ999
	uint8_t valid;     //״ֵ̬��0(0x00)Ϊ�������ݲ�����
	uint8_t version; //�汾��
	
	float flow_x_integral_mm;
	float flow_y_integral_mm;
	
	float d_center_x_err_mm;//�����˻�����x���ƫ�����
	float d_center_y_err_mm;//�����˻�����y���ƫ�����	
	
	float last_yaw;//��һ�ε�yaw�Ƕ�
	float now_yaw;//��ε�yaw�Ƕ�
	float last_pitch;//��һ�ε�pitch�Ƕ�
	float now_pitch;//��ε�pitch�Ƕ�
	float last_roll;//��һ�ε�roll�Ƕ�
	float now_roll;//��ε�roll�Ƕ�
	
	float flow_yaw_x_com_mm;//������yaw����ת��x�ۼ�λ�Ʋ���ֵ
	float flow_yaw_y_com_mm;//������yaw����ת��y�ۼ�λ�Ʋ���ֵ
	float flow_pitch_x_com_mm;//������pitch����ת��x�ۼ�λ�Ʋ���ֵ
	float flow_pitch_y_com_mm;//������pitch����ת��y�ۼ�λ�Ʋ���ֵ
	float flow_roll_x_com_mm;//������roll����ת��x�ۼ�λ�Ʋ���ֵ
	float flow_roll_y_com_mm;//������roll����ת��y�ۼ�λ�Ʋ���ֵ	
	float flow_x_speed;//����ˮƽx�ٶ� ��λm/s
	float flow_y_speed;//����ˮƽy�ٶ� ��λm/s
	float flow_x_pos;//����ˮƽxλ�� ��λm
	float flow_y_pos;//����ˮƽyλ�� ��λm
	
	
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
#define AIRCFAFT_DATA_NUM 24
#define RC_SBUS_DATA_NUM 25
#define RC_SBUS_CH_MAX 1792
#define RC_SBUS_CH_MID 992
#define RC_SBUS_CH_MIN 192
#define RC_SBUS_CH_RANGE 1600
#define UART3_RXBUFFERSIZE 128
#define UART6_RXBUFFERSIZE 128
#define UART4_RXBUFFERSIZE 128
#define UART1_RXBUFFERSIZE 128
extern uint8_t uart3_rx_buff[UART3_RXBUFFERSIZE];//����3���ջ�����
extern uint8_t uart6_rx_buff[UART6_RXBUFFERSIZE];//����6���ջ�����
extern uint8_t uart4_rx_buff[UART4_RXBUFFERSIZE];//����4���ջ�����
extern uint8_t uart1_rx_buff[UART1_RXBUFFERSIZE];//����1���ջ�����
extern uint8_t remote_data_flash[2];
extern TOF_TypeDef TOF;
extern OpticalFlow_TypeDef OpticalFlow;

extern int rx_data_correct_cnt;
extern int rx_data_cnt;
extern int rx_data_tim_cnt;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

