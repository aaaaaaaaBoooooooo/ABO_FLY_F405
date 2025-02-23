#include "BMP280.h"
#include "i2c.h"
//__IO float usDelayBase;
//void PY_usDelayTest(void)
//{
//  __IO uint32_t firstms, secondms;
//  __IO uint32_t counter = 0;

//  firstms = HAL_GetTick()+1;
//  secondms = firstms+1;

//  while(uwTick!=firstms) ;

//  while(uwTick!=secondms) counter++;

//  usDelayBase = ((float)counter)/1000;
//}

//void PY_Delay_us_t(uint32_t Delay)
//{
//  __IO uint32_t delayReg;
//  __IO uint32_t usNum = (uint32_t)(Delay*usDelayBase);

//  delayReg = 0;
//  while(delayReg!=usNum) delayReg++;
//}

//void PY_usDelayOptimize(void)
//{
//  __IO uint32_t firstms, secondms;
//  __IO float coe = 1.0;

//  firstms = HAL_GetTick();
//  PY_Delay_us_t(1000000) ;
//  secondms = HAL_GetTick();

//  coe = ((float)1000)/(secondms-firstms);
//  usDelayBase = coe*usDelayBase;
//}

//void PY_Delay_us(uint32_t Delay)//微秒延时函数
//{
//  __IO uint32_t delayReg;

//  __IO uint32_t msNum = Delay/1000;
//  __IO uint32_t usNum = (uint32_t)((Delay%1000)*usDelayBase);

//  if(msNum>0) HAL_Delay(msNum);

//  delayReg = 0;
//  while(delayReg!=usNum) delayReg++;
//}


////模拟IIC
//void I2C_Init(void)
//{

//	SCL_OUT_H;
//	SDA_OUT_H;
//	PY_Delay_us_t(100000);

//}

//void I2C_Start(void)
//{
//	PY_Delay_us_t(us_num) ;
//	SDA_OUT_H;
//	SCL_OUT_H;
//	PY_Delay_us_t(us_num/2) ;
//	SDA_OUT_L;
//	PY_Delay_us_t(us_num/2) ;
//	SCL_OUT_L;
//}

//void I2C_Stop(void)
//{
//	SCL_OUT_L;
//	PY_Delay_us_t(us_num) ;
//	SDA_OUT_L;
//	PY_Delay_us_t(us_num) ;
//	SCL_OUT_H;
//	PY_Delay_us_t(us_num) ;
//	SDA_OUT_H;
//	PY_Delay_us_t(us_num) ;
//}

//void I2C_Write_Ack(void)
//{

//    PY_Delay_us_t(us_num/2) ;
//	SDA_OUT_L;
//	PY_Delay_us_t(us_num/2) ;
//	SCL_OUT_H;
//	PY_Delay_us_t(us_num) ;
//	SCL_OUT_L;
//	SDA_OUT_H;

//}

//uint8_t I2C_Read_Ack(void)
//{
//	uint16_t time=0;
//	uint8_t status=0;

//	SCL_OUT_L;
//	PY_Delay_us_t(us_num/2) ;
//	SDA_OUT_H;

//	while(SDA_IN){
//	time++;
//		if(time >1000)break;
//	}
//	status = SDA_IN;
//	SCL_OUT_H;
//	PY_Delay_us_t(us_num) ;
//	SCL_OUT_L;
//	SDA_OUT_L;

//	return status;

//}


//void I2C_Send_Byte(uint8_t txd){


//    for(uint8_t i=0;i<8;i++)
//    {
//    	PY_Delay_us_t(us_num/2) ;
//        if((txd&0x80)>>7) SDA_OUT_H;
//        else SDA_OUT_L;
//        txd<<=1;
//        PY_Delay_us_t(us_num/2) ;
//        SCL_OUT_H;
//        PY_Delay_us_t(us_num) ;
//		SCL_OUT_L;
//    }

//    SDA_OUT_L;

//}

//uint8_t I2C_Read_Byte(unsigned char rdack)
//{
//	uint8_t rxd=0;


//    for(uint8_t i=0;i<8;i++ )
//	{
//    	SCL_OUT_L;
//    	PY_Delay_us_t(us_num/2) ;
//    	SDA_OUT_H;
//    	PY_Delay_us_t(us_num/2) ;
//    	SCL_OUT_H;
//        rxd<<=1;
//        if(SDA_IN) rxd++;
//        PY_Delay_us_t(us_num) ;
//    }

//    SCL_OUT_L;
//    SDA_OUT_H;

//    if (rdack) I2C_Write_Ack();

//    return rxd;
//}

//BMP280

#define BMP280_I2C_ADDR_SEL 1//IIC地址选择

BMP280_HandleTypeDef bmp280;//保存BMP内部校准值的结构体变量
#define dig_T1 bmp280.T1
#define dig_T2 bmp280.T2
#define dig_T3 bmp280.T3
#define dig_P1 bmp280.P1
#define dig_P2 bmp280.P2
#define dig_P3 bmp280.P3
#define dig_P4 bmp280.P4
#define dig_P5 bmp280.P5
#define dig_P6 bmp280.P6
#define dig_P7 bmp280.P7
#define dig_P8 bmp280.P8
#define dig_P9 bmp280.P9
#define t_fine bmp280.T_fine//温度系数

int32_t gs32Pressure0 = MSLP;

void BMP280_WriteReg(uint8_t WrAddr, uint8_t data)//写寄存器，WrAddr寄存器地址，data要写入数据
{
	  uint8_t daddr; //device address (0x1e<<1)
		uint8_t data_to_send[2];
	  if(BMP280_I2C_ADDR_SEL==0) daddr = 0xec ;//device address for SDO low status (0x76<<1)
	  else daddr = 0xee; //device address for SDO high status (0x77<<1)
	data_to_send[0] = WrAddr;
	data_to_send[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1,daddr,data_to_send,2,0x1000);
//	  I2C_Start();
//	  I2C_Send_Byte(daddr);
//	  I2C_Read_Ack();
//  	  I2C_Send_Byte(WrAddr);
//  	  I2C_Read_Ack();
//  	  I2C_Send_Byte(data);
//  	  I2C_Read_Ack();
//  	  I2C_Stop();

}

uint8_t BMP280_ReadReg(uint8_t RdAddr)//读寄存器值，RdAddr寄存器地址
{

	  uint8_t RegValue = 0;
	  uint8_t daddr;

	  if(BMP280_I2C_ADDR_SEL==0) daddr = 0xec; 
	  else daddr = 0xee; //device address for SDO high status (0x77<<1)

	HAL_I2C_Master_Transmit(&hi2c1,daddr,&RdAddr,1,0x1000);
	HAL_I2C_Master_Receive(&hi2c1,daddr+1,&RegValue,1,0x1000);
//	  I2C_Start();
//	  I2C_Send_Byte(daddr);
//	  I2C_Read_Ack();
//  	  I2C_Send_Byte(RdAddr);
//  	  I2C_Read_Ack();

//  	  I2C_Start();
//	  I2C_Send_Byte(daddr+1);
//	  I2C_Read_Ack();
//	  RegValue=I2C_Read_Byte(0);
//  	  I2C_Stop();

	  return RegValue;
}

/* Returns temperature in DegC, double precision. Output value of "1.23"equals 51.23 DegC. */
double BMP280_Compensate_Temperature(int32_t adc_T)
{
	double var1, var2, temperature;
	var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0) * ((double) dig_T2);
	var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)  * (((double) adc_T) / 131072.0
					- ((double) dig_T1) / 8192.0)) * ((double) dig_T3);
	t_fine = (int32_t) (var1 + var2);//
	temperature = (var1 + var2) / 5120.0;

	return temperature;
}


/* Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa */
double BMP280_Compensate_Pressure(int32_t adc_P)
{
	double var1, var2, pressure;

	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double) dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0  + ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);

	if (var1 == 0.0) {
		return 0; // avoid exception caused by division by zero
	}

	pressure = 1048576.0 - (double) adc_P;
	pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * pressure * pressure / 2147483648.0;
	var2 = pressure * ((double) dig_P8) / 32768.0;
	pressure = pressure + (var1 + var2 + ((double) dig_P7)) / 16.0;

	return pressure;
}

int32_t BMP280_Get_Pressure(void)//获取气压现实值
{
	uint8_t lsb, msb, xlsb;
	int32_t adc_P;

	xlsb = BMP280_ReadReg(BMP280_PRESS_XLSB_REG);
	lsb = BMP280_ReadReg(BMP280_PRESS_LSB_REG);
	msb = BMP280_ReadReg(BMP280_PRESS_MSB_REG);
	adc_P = (msb << 16) | (lsb << 8) | (xlsb);
	//adc_P = 51988;
	//return BMP280_Compensate_Pressure(adc_P);
	return adc_P;
}

void BMP280_Get_Temperature_And_Pressure(double *temperature, double *pressure)//获取气压和温度的现实值
{
	uint8_t lsb, msb, xlsb;
	int32_t adc_P,adc_T;

	xlsb = BMP280_ReadReg(BMP280_TEMP_XLSB_REG);
	lsb = BMP280_ReadReg(BMP280_TEMP_LSB_REG);
	msb = BMP280_ReadReg(BMP280_TEMP_MSB_REG);
	adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4);
	//adc_T = 415148;
	* temperature = BMP280_Compensate_Temperature(adc_T);

	xlsb = BMP280_ReadReg(BMP280_PRESS_XLSB_REG);
	lsb = BMP280_ReadReg(BMP280_PRESS_LSB_REG);
	msb = BMP280_ReadReg(BMP280_PRESS_MSB_REG);
	adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4);
	//adc_P = 51988;
	* pressure = BMP280_Compensate_Pressure(adc_P);
}

#define BMP280_AVG_TIMES 4 //maximum: 8
void BMP280_CalAvgValue(uint8_t *pIndex, int32_t *pAvgBuffer, int32_t InVal, int32_t *pOutVal)
{
	uint8_t i;
	static uint8_t status = 0;

  	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex %= BMP280_AVG_TIMES;

  	if(status<=24) //skip average computation before getting pre-defined data times (24 times)
  	{
  		*pOutVal = InVal;
  		status++;
  	}
  	else //compute average value
  	{
  	  	*pOutVal = 0;
  		for(i = 0; i < BMP280_AVG_TIMES; i ++)
  	  	{
  	    	*pOutVal += *(pAvgBuffer + i);
  	  	}
  	  	*pOutVal /= BMP280_AVG_TIMES;

  	}
}

void BMP280_CalculateAbsoluteAltitude(int32_t *pAltitude, int32_t PressureVal)//计算海拔高度
{
	*pAltitude = 4433000 * (1 - pow((PressureVal / (float)gs32Pressure0), 0.1903));
}

void BMP280_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude)
{
    double CurPressure, CurTemperature;
    int32_t CurAltitude;
    static BMP280_AvgTypeDef BMP280_Filter[3];

    BMP280_Get_Temperature_And_Pressure(&CurTemperature, &CurPressure);
    BMP280_CalAvgValue(&BMP280_Filter[0].Index, BMP280_Filter[0].AvgBuffer, (int32_t)(CurPressure), pressure);

    BMP280_CalculateAbsoluteAltitude(&CurAltitude, (*pressure));
    BMP280_CalAvgValue(&BMP280_Filter[1].Index, BMP280_Filter[1].AvgBuffer, CurAltitude, Altitude);
    BMP280_CalAvgValue(&BMP280_Filter[2].Index, BMP280_Filter[2].AvgBuffer, (int32_t)CurTemperature*10, temperature);

    (*Altitude) += ALTITUDE_OFFSET;

}


void BMP280_Read_Calibration(void)//读BMP内部校准值
{
	uint8_t lsb, msb;

	/* read the temperature calibration parameters */
	lsb = BMP280_ReadReg(BMP280_DIG_T1_LSB_REG);
    msb = BMP280_ReadReg(BMP280_DIG_T1_MSB_REG);
	dig_T1 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_T2_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_T2_MSB_REG);
	dig_T2 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_T3_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_T3_MSB_REG);
	dig_T3 = msb << 8 | lsb;

	/* read the pressure calibration parameters */
	lsb = BMP280_ReadReg(BMP280_DIG_P1_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P1_MSB_REG);
	dig_P1 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P2_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P2_MSB_REG);
	dig_P2 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P3_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P3_MSB_REG);
	dig_P3 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P4_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P4_MSB_REG);
	dig_P4 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P5_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P5_MSB_REG);
	dig_P5 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P6_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P6_MSB_REG);
	dig_P6 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P7_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P7_MSB_REG);
	dig_P7 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P8_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P8_MSB_REG);
	dig_P8 = msb << 8 | lsb;
	lsb = BMP280_ReadReg(BMP280_DIG_P9_LSB_REG);
	msb = BMP280_ReadReg(BMP280_DIG_P9_MSB_REG);
	dig_P9 = msb << 8 | lsb;

}

uint8_t BMP280_Init(uint8_t ctrl,uint8_t config)//BMP280初始化，读取芯片ID，工作模式，工作状态，内部校准值，并初始化ctrl、config寄存器
{

    uint8_t u8ChipID, u8CtrlMod, u8Config;

    u8ChipID = BMP280_ReadReg(0x00);
    u8CtrlMod = BMP280_ReadReg(BMP280_REGISTER_CONTROL);
    u8Config = BMP280_ReadReg(BMP280_REGISTER_CONFIG);

    if(u8ChipID == 0x60)
    {
			  BMP280_WriteReg(BMP280_REGISTER_CONTROL, ctrl); //ctrl_meas register设定工作模式，
        BMP280_WriteReg(BMP280_REGISTER_CONFIG, config);  //config register
        BMP280_Read_Calibration();//读取内部校准值			
    		printf("\r\nBMP280 initial successful : ChipID [0x%x] CtrlMod [0x%x] Config [0x%x] \r\n", u8ChipID,u8CtrlMod,u8Config);    
				return 1 ;
    }
    else
    {   	
				printf("\r\nBMP280 initial failure : ChipID [0x%x] CtrlMod [0x%x] Status [0x%x] \r\n", u8ChipID,u8CtrlMod,u8Config);
				return 0; 
    }
}

