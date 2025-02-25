#include "control.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "pid.h"
#include "angle.h"
#include "math.h"
#include "stdlib.h"
#include "usart.h"
#include "cJSON_usr.h"
#include "fatfs.h"
PID_FC_TypeDef pid_AttitudeControl;//姿态控制pid
PID_TypeDef pid_HeightControl;//高度控制pid
uint8_t aircraft_state =0x00;//飞行器状态

/*空心杯电机控制begin*/
//电机转速设置
void motor_speed_set(int m1_duty,int m2_duty,int m3_duty,int m4_duty)
{
	/***限幅处理begin***/
	if(m1_duty>MOTOR_MAX_DUTY)		m1_duty = MOTOR_MAX_DUTY;
	else if(m1_duty<MOTOR_MIN_DUTY)		m1_duty = MOTOR_MIN_DUTY;
	if(m2_duty>MOTOR_MAX_DUTY)		m2_duty = MOTOR_MAX_DUTY;
	else if(m2_duty<MOTOR_MIN_DUTY)		m2_duty = MOTOR_MIN_DUTY;
	if(m3_duty>MOTOR_MAX_DUTY)		m3_duty = MOTOR_MAX_DUTY;
	else if(m3_duty<MOTOR_MIN_DUTY)		m3_duty = MOTOR_MIN_DUTY;
	if(m4_duty>MOTOR_MAX_DUTY)		m4_duty = MOTOR_MAX_DUTY;
	else if(m4_duty<MOTOR_MIN_DUTY)		m4_duty = MOTOR_MIN_DUTY;	
	/***限幅处理end***/	
	

	/***HAL库占空比设置***/
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,m1_duty);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,m2_duty);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,m3_duty);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,m4_duty);
}
//电机初始化
void motor_init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);	
}
//电机解除初始化
void motor_deinit()
{
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);	
}
/*空心杯电机控制end*/

/*电机油门控制begin*/
int aircraft_throttle=0;//油门
void motor_throttle_control(uint8_t type)
{
	if(type == 0)//默认手动油门控制
	{
		if(remote_data_flash[0]==1)
		{
			if(abs((int)my_remote.YG_LEFT_UD-128)>50)
				aircraft_throttle += ((int)my_remote.YG_LEFT_UD - 128)*0.3;
			else 
				aircraft_throttle+=0;
			if(aircraft_throttle>MOTOR_MAX_THROTTLE)
				aircraft_throttle = MOTOR_MAX_THROTTLE;
			else if(aircraft_throttle<MOTOR_MIN_THROTTLE)
				aircraft_throttle = MOTOR_MIN_THROTTLE;
			remote_data_flash[0] = 0;
		}
	}
	else//自动定高油门控制
	{
		
	}
		/***油门***/
	motor1_duty +=(int)aircraft_throttle;
	motor2_duty +=(int)aircraft_throttle;
	motor3_duty +=(int)aircraft_throttle;
	motor4_duty +=(int)aircraft_throttle;	
}
/*电机油门控制end*/

/*飞行器飞行方向控制begin*/
float roll_target_angle=0;
float pitch_target_angle=0;
float yaw_target_angle=0;
float roll_compensate=0;//补偿
float pitch_compensate=0;//补偿
void aircraft_flight_direction_control()
{
	if(remote_data_flash[1]==1)
	{
		if(abs((int)my_remote.YG_RIGHT_LR-128)>25)
			roll_target_angle = (float)(my_remote.YG_RIGHT_LR-128)*((float)ROLL_TARGET_MAX_ANGLE/128.0f);
		else
			roll_target_angle = 0;
		if(abs((int)my_remote.YG_RIGHT_UD-128)>25)	
			pitch_target_angle = -(float)(my_remote.YG_RIGHT_UD-128)*((float)PITCH_TARGET_MAX_ANGLE/128.0f);
		else
			pitch_target_angle = 0;
		if(abs((int)my_remote.YG_LEFT_LR-128)>100)
			yaw_target_angle += (float)(my_remote.YG_LEFT_LR-128)*0.01f;
		else
			yaw_target_angle +=0;
		/****外环输入begin****/
		if(roll_target_angle>ROLL_TARGET_MAX_ANGLE)
			roll_target_angle = ROLL_TARGET_MAX_ANGLE;
		else if(roll_target_angle<ROLL_TARGET_MIN_ANGLE)
			roll_target_angle = ROLL_TARGET_MIN_ANGLE;
		pid_AttitudeControl.external.pid_x.target = roll_target_angle+roll_compensate;
		
		if(pitch_target_angle>PITCH_TARGET_MAX_ANGLE)
			pitch_target_angle = PITCH_TARGET_MAX_ANGLE;
		else if(pitch_target_angle<PITCH_TARGET_MIN_ANGLE)
			pitch_target_angle = PITCH_TARGET_MIN_ANGLE;	
		pid_AttitudeControl.external.pid_y.target = pitch_target_angle+pitch_compensate;
		
		if(yaw_target_angle>YAW_TARGET_MAX_ANGLE)
			yaw_target_angle = YAW_TARGET_MAX_ANGLE;
		else if(yaw_target_angle<YAW_TARGET_MIN_ANGLE)
			yaw_target_angle = YAW_TARGET_MIN_ANGLE;	
		pid_AttitudeControl.external.pid_z.target = yaw_target_angle;
		/****外环输入end****/	
		remote_data_flash[1]=0;
	}
}
/*飞行器飞行方向控制end*/

void pid_control_init()
{
	pid_init(&pid_AttitudeControl.internal.pid_x);
	pid_init(&pid_AttitudeControl.internal.pid_y);
	pid_init(&pid_AttitudeControl.internal.pid_z);
	pid_AttitudeControl.internal.pid_x.f_param_init(&pid_AttitudeControl.internal.pid_x,PID_Position,MOTOR_MAX_DUTY,MOTOR_MAX_DUTY*0.1,0,0,0,30.0f,0.0f,18.0f);
	pid_enable(&pid_AttitudeControl.internal.pid_x,1);
	pid_AttitudeControl.internal.pid_y.f_param_init(&pid_AttitudeControl.internal.pid_y,PID_Position,MOTOR_MAX_DUTY,MOTOR_MAX_DUTY*0.1,0,0,0,30.0f,0.0f,18.0f);
	pid_enable(&pid_AttitudeControl.internal.pid_y,1);
	pid_AttitudeControl.internal.pid_z.f_param_init(&pid_AttitudeControl.internal.pid_z,PID_Position,MOTOR_MAX_DUTY,MOTOR_MAX_DUTY*0.1,0,0,0,60.0f,0.0f,15.0f);
	pid_enable(&pid_AttitudeControl.internal.pid_z,1);
	
	pid_init(&pid_AttitudeControl.external.pid_x);
	pid_init(&pid_AttitudeControl.external.pid_y);
	pid_init(&pid_AttitudeControl.external.pid_z);
	pid_AttitudeControl.external.pid_x.f_param_init(&pid_AttitudeControl.external.pid_x,PID_Position,9999,100,0,0,0,2.5f,0.025f,20.0f);
	pid_enable(&pid_AttitudeControl.external.pid_x,1);
	pid_AttitudeControl.external.pid_y.f_param_init(&pid_AttitudeControl.external.pid_y,PID_Position,9999,100,0,0,0,2.5f,0.025f,20.0f);
	pid_enable(&pid_AttitudeControl.external.pid_y,1);
	pid_AttitudeControl.external.pid_z.f_param_init(&pid_AttitudeControl.external.pid_z,PID_Position,1000,5,0,0,0,6.0f,0.0f,0.0f);	
	pid_enable(&pid_AttitudeControl.external.pid_z,1);

	fatfs_params_read();
	motor_init();
}

int motor1_duty=0;
int motor2_duty=0;
int motor3_duty=0;
int motor4_duty=0;
void pid_internal_control()
{ 
	motor1_duty=0;
	motor2_duty=0;
	motor3_duty=0;
	motor4_duty=0;	
	/****pid计算begin****/
	pid_AttitudeControl.internal.pid_x.f_cal_pid(&pid_AttitudeControl.internal.pid_x,IMU_Data.gyro.x*Gyro_Gain);//X轴
	motor1_duty +=(int)pid_AttitudeControl.internal.pid_x.output;
	motor2_duty -=(int)pid_AttitudeControl.internal.pid_x.output;
	motor3_duty +=(int)pid_AttitudeControl.internal.pid_x.output;
	motor4_duty -=(int)pid_AttitudeControl.internal.pid_x.output;
	pid_AttitudeControl.internal.pid_y.f_cal_pid(&pid_AttitudeControl.internal.pid_y,IMU_Data.gyro.y*Gyro_Gain);//Y轴
	motor1_duty -=(int)pid_AttitudeControl.internal.pid_y.output;
	motor2_duty -=(int)pid_AttitudeControl.internal.pid_y.output;
	motor3_duty +=(int)pid_AttitudeControl.internal.pid_y.output;
	motor4_duty +=(int)pid_AttitudeControl.internal.pid_y.output;
	pid_AttitudeControl.internal.pid_z.f_cal_pid(&pid_AttitudeControl.internal.pid_z,IMU_Data.gyro.z*Gyro_Gain);//Z轴
	motor1_duty -=(int)pid_AttitudeControl.internal.pid_z.output;
	motor2_duty +=(int)pid_AttitudeControl.internal.pid_z.output;
	motor3_duty +=(int)pid_AttitudeControl.internal.pid_z.output;
	motor4_duty -=(int)pid_AttitudeControl.internal.pid_z.output;	
	/****pid计算end****/
	
	if(motor1_duty<MOTOR_MIN_DUTY) motor1_duty =MOTOR_MIN_DUTY;
	if(motor2_duty<MOTOR_MIN_DUTY) motor2_duty =MOTOR_MIN_DUTY;
	if(motor3_duty<MOTOR_MIN_DUTY) motor3_duty =MOTOR_MIN_DUTY;
	if(motor4_duty<MOTOR_MIN_DUTY) motor4_duty =MOTOR_MIN_DUTY;
	
	if(motor1_duty>MOTOR_MAX_DUTY) motor1_duty =MOTOR_MAX_DUTY;
	if(motor2_duty>MOTOR_MAX_DUTY) motor2_duty =MOTOR_MAX_DUTY;
  if(motor3_duty>MOTOR_MAX_DUTY) motor3_duty =MOTOR_MAX_DUTY;
	if(motor4_duty>MOTOR_MAX_DUTY) motor4_duty =MOTOR_MAX_DUTY;

	
}
void pid_external_control()
{    
	/****pid计算begin****/
	pid_AttitudeControl.external.pid_x.f_cal_pid(&pid_AttitudeControl.external.pid_x,Angle_Data.roll);//ROLL

	pid_AttitudeControl.external.pid_y.f_cal_pid(&pid_AttitudeControl.external.pid_y,Angle_Data.pitch);//PITCH

	pid_AttitudeControl.external.pid_z.f_cal_pid(&pid_AttitudeControl.external.pid_z,Angle_Data.yaw);//YAW

	/****pid计算end****/
	
	/****外环输出begin****/
	pid_AttitudeControl.internal.pid_x.target = pid_AttitudeControl.external.pid_x.output;
	
	pid_AttitudeControl.internal.pid_y.target = pid_AttitudeControl.external.pid_y.output;
	
	pid_AttitudeControl.internal.pid_z.target = pid_AttitudeControl.external.pid_z.output;
	/****外环输出end****/
}
/*飞行器保护措施begin*/
uint8_t aircraft_protection()
{
	static uint32_t Battery_Low_Power_cnt =0;
	static uint32_t Aircraft_IMU_Error_cnt =0;
	static uint32_t Motor_Duty_Error_cnt =0;
	
	if(my_aircraft.Battery_Volt <=32 )//电池低电量保护
	{
		Battery_Low_Power_cnt ++;
		if(Battery_Low_Power_cnt >= 50)//5s
		{
			motor_deinit();
			return 1;
		}
	}
	else 
	{
		Battery_Low_Power_cnt =0;
	}
	
	if(fabs(Angle_Data.roll)>60||fabs(Angle_Data.pitch)>60)//飞行器姿态错误保护
	{
		Aircraft_IMU_Error_cnt++;
		if(Aircraft_IMU_Error_cnt >= 2)//0.2s
		{
			motor_deinit();
			return 2;
		}		
	}
	else
	{
		Aircraft_IMU_Error_cnt=0;
	}
	
	if(motor1_duty>=MOTOR_MAX_DUTY*0.95f||motor2_duty>=MOTOR_MAX_DUTY*0.95f||motor3_duty>=MOTOR_MAX_DUTY*0.95f||motor4_duty>=MOTOR_MAX_DUTY*0.95f)//电机高占空比保护
	{
		Motor_Duty_Error_cnt++;
		if(Motor_Duty_Error_cnt >= 20)//2s
		{
			motor_deinit();
			return 3;
		}		
	}
	else
	{
		Motor_Duty_Error_cnt=0;
	}	
	return 0;
	
}
/*飞行器保护措施end*/

#define PARAMS_NAME "Param_s.txt"
#define PARAMS_MAX_READ_SIZE  4096
uint8_t fatfs_params_read()
{
	cJSON *json,*json_get;
	char read_buf[PARAMS_MAX_READ_SIZE];
	if(fatfs_read_file(PARAMS_NAME,read_buf,PARAMS_MAX_READ_SIZE))
	{
		return 1;
	}
	

	json = cJSON_Parse((const char *)read_buf); //将得到的字符串解析成json形式
    // 检查解析是否成功
	if (json == NULL) 
		{
			const char *error_ptr = cJSON_GetErrorPtr();
			if (error_ptr != NULL) {
					printf("JSON Parse error，pos: %s\n", error_ptr);
			}
			return 1; // 直接返回，避免操作空指针
		}
	/****************************/
	/*	  测试将JSON打印出来	*/
	/***************************/
//	char *out_data = cJSON_Print(json);   //将json形式打印成正常字符串形式
//	printf("%s",out_data);
	
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_x.kp" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_x.kp = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_x.ki" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_x.ki = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_x.kd" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_x.kd = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_y.kp" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_y.kp = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_y.ki" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_y.ki = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_y.kd" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_y.kd = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_z.kp" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_z.kp = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_z.ki" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_z.ki = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.internal.pid_z.kd" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.internal.pid_z.kd = json_get->valuedouble;
 	
	}	
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_x.kp" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_x.kp = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_x.ki" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_x.ki = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_x.kd" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_x.kd = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_y.kp" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_y.kp = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_y.ki" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_y.ki = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_y.kd" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_y.kd = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_z.kp" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_z.kp = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_z.ki" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_z.ki = json_get->valuedouble;
 	
	}
	json_get = cJSON_GetObjectItem( json ,"pid_AttitudeControl.external.pid_z.kd" );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
		pid_AttitudeControl.external.pid_z.kd = json_get->valuedouble;
 	
	}		
	cJSON_Delete(json);  //释放内存 
	cJSON_Delete(json_get);  //释放内存 		
	return 0;
}


