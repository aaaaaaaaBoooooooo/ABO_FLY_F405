/********************************Copyright (c)**********************************`
** 文件名称: filter.c
** 创建人员: aBo
** 创建日期: 2025-5-04
** 文档描述: 滤波器相关函数实现
********************************End of Head************************************/
#include "filter.h"
#include "HANA_math.h"
#include <math.h>
#include <string.h>
#include "stdint.h"

/*一维卡尔曼滤波初始化*/
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}
/*一维卡尔曼滤波迭代*/
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;/*根据上一次最优估计值预测这一次的值*/
    state->p = state->A * state->A * state->p + state->q;  /*根据上一次最优估计值方差以及设定的预测方差求解当前预测值方差*/

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);/*根据预测值方差以及传感器方差计算卡尔曼增益*/
    state->x = state->x + state->gain * (z_measure - state->H * state->x);/*计算最优估计值*/
    state->p = (1 - state->gain * state->H) * state->p;/*计算估计值方差*/

    return state->x;
}

/*一阶离散低通滤波器*/
float LPF_1st(Filter_LPF_1 *LPF_1,float dt)
{
	 return LPF_1->old_data + (dt /( 1 / ( 2 * PI * LPF_1->factor ) + dt)) * (LPF_1->new_data - LPF_1->old_data);    
}

/*抗干扰滑动均值滤波器*/
int32_t AntiPulse_MovingAverage_Filter(MovAverage *_MovAverage)
{
		int32_t i;	
		int32_t sum=0;
		int32_t max=0;
		int32_t min=0xffff;
		//_MovAverage->max_cnt = 10; max_cnt 为窗口大小
		_MovAverage->average[_MovAverage->cnt] = _MovAverage->input;	
		_MovAverage->cnt++;			
		if(_MovAverage->cnt==_MovAverage->max_cnt)
		{
			_MovAverage->cnt=0;
		}	
		for(i=0;i<_MovAverage->max_cnt;i++)
		{
				if(_MovAverage->average[i]>max)
						max = _MovAverage->average[i];
				else if(_MovAverage->average[i]<min)
						min = _MovAverage->average[i];
				sum += _MovAverage->average[i];
		}
		return ((sum-max-min)/(_MovAverage->max_cnt-2));                                    
}

///////////////////////////            END        //////////////////////////////

