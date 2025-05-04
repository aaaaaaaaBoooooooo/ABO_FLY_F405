/********************************Copyright (c)**********************************`
** 文件名称: filter.h
** 创建人员: aBo
** 创建日期: 2025-5-04
** 文档描述: 滤波器相关函数声明
********************************End of Head************************************/
#ifndef __filter_H
#define __filter_H
#include "main.h"

#define PI 3.1415927f
////////////////////////////////////////

typedef struct{
	float x;
	float p;
	float A;
	float H;
	float q;
	float r;
	float gain;
}kalman1_state;

void kalman1_init(kalman1_state *state, float init_x, float init_p);
float kalman1_filter(kalman1_state *state, float z_measure);

/*  LPF 1st filter   */
typedef struct{
		float old_data;
		float new_data;
		float factor;
}Filter_LPF_1;
float LPF_1st(Filter_LPF_1 *LPF_1,float dt);

typedef struct {
	
	int32_t cnt;
	int32_t input;
	int32_t *average;

	int32_t max_cnt;
}MovAverage;

int32_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage);

#endif
