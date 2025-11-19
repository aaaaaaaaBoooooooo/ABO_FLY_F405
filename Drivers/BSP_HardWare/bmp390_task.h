#ifndef BMP390_H__
#define BMP390_H__

#include <stdint.h>
#include "bmp3_defs.h"
#include "math.h"

#define USE_I2C_INTERFACE  
//#define USE_SPI_INTERFACE  


#define SEA_LEVEL_PRESSURE_HPA (1013.25)

#define SAMPLING_RATE		BMP3_OVERSAMPLING_4X

#define OUTPUT_RATE			BMP3_ODR_50_HZ
#define IIR_FILTER_COEFF 	BMP3_IIR_FILTER_COEFF_3

typedef struct
{
    struct bmp3_data data;
    struct bmp3_dev dev;
	uint8_t is_valid;//tof is valid 
	
}Barometer_TypeDef;

extern uint8_t bmp390_data_dma_buf[BMP3_LEN_P_T_DATA];

extern Barometer_TypeDef my_bmp390;


uint8_t BMP390_Init(void);
struct bmp3_data bmp390_getdata(void);
void bmp390_getregs_DMA(void);
struct bmp3_data bmp390_getdata_DMA(uint8_t *buf);
double convert_Pa_to_meter(double pressure_Pa);
double convert_mhPa_to_meter(int32_t pressure_mhPa);
int32_t convert_Pa_to_mhPa(double pressure_Pa);
struct bmp3_data bmp390_data_check(struct bmp3_data data);



#endif


