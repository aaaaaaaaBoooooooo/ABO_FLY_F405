#ifndef __BMP280_H
#define __BMP280_H

#include "main.h"
#include "math.h"
#include "string.h"
#include "stdio.h"

//#define us_num 2 //IIC时序间隔

//#define SCL_OUT_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)//SCL引脚定义
//#define SCL_OUT_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
//#define SDA_OUT_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)//SDA引脚定义
//#define SDA_OUT_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
//#define SDA_IN HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)//SDA写

/*
 *  BMP280 register address
 */
#define BMP280_REGISTER_DIG_T1      0x88
#define BMP280_REGISTER_DIG_T2      0x8A
#define BMP280_REGISTER_DIG_T3      0x8C

#define BMP280_REGISTER_DIG_P1      0x8E
#define BMP280_REGISTER_DIG_P2      0x90
#define BMP280_REGISTER_DIG_P3      0x92
#define BMP280_REGISTER_DIG_P4      0x94
#define BMP280_REGISTER_DIG_P5      0x96
#define BMP280_REGISTER_DIG_P6      0x98
#define BMP280_REGISTER_DIG_P7      0x9A
#define BMP280_REGISTER_DIG_P8      0x9C
#define BMP280_REGISTER_DIG_P9      0x9E

#define BMP280_REGISTER_CHIPID      0xD0
#define BMP280_REGISTER_VERSION     0xD1
#define BMP280_REGISTER_SOFTRESET   0xE0
#define BMP280_REGISTER_STATUS      0xF3
#define BMP280_REGISTER_CONTROL     0x1B
#define BMP280_REGISTER_CONFIG      0x1F

#define BMP280_TEMP_XLSB_REG        0xFC	    /*Temperature XLSB Register */
#define BMP280_TEMP_LSB_REG         0xFB        /*Temperature LSB Register  */
#define BMP280_TEMP_MSB_REG         0xFA        /*Temperature LSB Register  */
#define BMP280_PRESS_XLSB_REG       0x07		/*Pressure XLSB  Register   */
#define BMP280_PRESS_LSB_REG        0x08		/*Pressure LSB Register     */
#define BMP280_PRESS_MSB_REG        0x09		/*Pressure MSB Register     */

/*calibration parameters */
#define BMP280_DIG_T1_LSB_REG                0x88
#define BMP280_DIG_T1_MSB_REG                0x89
#define BMP280_DIG_T2_LSB_REG                0x8A
#define BMP280_DIG_T2_MSB_REG                0x8B
#define BMP280_DIG_T3_LSB_REG                0x8C
#define BMP280_DIG_T3_MSB_REG                0x8D
#define BMP280_DIG_P1_LSB_REG                0x8E
#define BMP280_DIG_P1_MSB_REG                0x8F
#define BMP280_DIG_P2_LSB_REG                0x90
#define BMP280_DIG_P2_MSB_REG                0x91
#define BMP280_DIG_P3_LSB_REG                0x92
#define BMP280_DIG_P3_MSB_REG                0x93
#define BMP280_DIG_P4_LSB_REG                0x94
#define BMP280_DIG_P4_MSB_REG                0x95
#define BMP280_DIG_P5_LSB_REG                0x96
#define BMP280_DIG_P5_MSB_REG                0x97
#define BMP280_DIG_P6_LSB_REG                0x98
#define BMP280_DIG_P6_MSB_REG                0x99
#define BMP280_DIG_P7_LSB_REG                0x9A
#define BMP280_DIG_P7_MSB_REG                0x9B
#define BMP280_DIG_P8_LSB_REG                0x9C
#define BMP280_DIG_P8_MSB_REG                0x9D
#define BMP280_DIG_P9_LSB_REG                0x9E
#define BMP280_DIG_P9_MSB_REG                0x9F

#define Ultrl_High_Re 0x57//超高精度0.16pa
#define High_Re 0x33//高精度0.33pa
#define Standard_Re 0x2f//标准精度0.66pa
#define config_Ultrl_High 0x1c//测量间隔0.5ms，滤波16
typedef struct {
	uint16_t T1; 		/*<calibration T1 data*/
	int16_t T2;  	 	/*<calibration T2 data*/
	int16_t T3;  		/*<calibration T3 data*/
	uint16_t P1;  	    /*<calibration P1 data*/
	int16_t P2;  		/*<calibration P2 data*/
	int16_t P3;  		/*<calibration P3 data*/
	int16_t P4;  		/*<calibration P4 data*/
	int16_t P5;  		/*<calibration P5 data*/
	int16_t P6;  		/*<calibration P6 data*/
	int16_t P7;  		/*<calibration P7 data*/
	int16_t P8;  		/*<calibration P8 data*/
	int16_t P9;			/*<calibration P9 data*/
	int32_t T_fine;	/*<calibration t_fine data*/
} BMP280_HandleTypeDef;

typedef struct
{
	uint8_t Index;
	int32_t AvgBuffer[8];
} BMP280_AvgTypeDef;

#define MSLP     101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)
#define ALTITUDE_OFFSET          1500

void PY_usDelayTest(void);
void PY_Delay_us_t(uint32_t Delay);//微秒延时函数
void PY_usDelayOptimize(void);//微秒延时函数优化
void PY_Delay_us(uint32_t Delay);//微秒延时函数
void I2C_Init(void);
uint8_t BMP280_Init(uint8_t ctrl,uint8_t config);
void BMP280_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude);
void BMP280_WriteReg(uint8_t WrAddr, uint8_t data);//写寄存器，WrAddr寄存器地址，data要写入数据

#endif /* __BMP280_H */
