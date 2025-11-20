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
#include "bmp390_task.h"
AttiudeController AttitudeControl;//姿态控制
HeightController HeightControl;//高度控制
PositionController PositionControl;//位置控制
Motor_TypeDef aircraft_motor;//电机结构体


/*空心杯电机控制begin*/
//电机转速设置
void motor_speed_set()
{
	float power = 1.0f;
	/***电机电压补偿处理begin***/
	aircraft_motor.duty1 *=aircraft_motor.volt_k1;
	aircraft_motor.duty2 *=aircraft_motor.volt_k2;
	aircraft_motor.duty3 *=aircraft_motor.volt_k3;
	aircraft_motor.duty4 *=aircraft_motor.volt_k4;
	/***电机电压补偿处理end***/	

	/***限幅处理begin***/
	if(aircraft_motor.duty1>MOTOR_MAX_DUTY)		aircraft_motor.duty1 = MOTOR_MAX_DUTY;
	else if(aircraft_motor.duty1<MOTOR_MIN_DUTY)		aircraft_motor.duty1 = MOTOR_MIN_DUTY;
	if(aircraft_motor.duty2>MOTOR_MAX_DUTY)		aircraft_motor.duty2 = MOTOR_MAX_DUTY;
	else if(aircraft_motor.duty2<MOTOR_MIN_DUTY)		aircraft_motor.duty2 = MOTOR_MIN_DUTY;
	if(aircraft_motor.duty3>MOTOR_MAX_DUTY)		aircraft_motor.duty3 = MOTOR_MAX_DUTY;
	else if(aircraft_motor.duty3<MOTOR_MIN_DUTY)		aircraft_motor.duty3 = MOTOR_MIN_DUTY;
	if(aircraft_motor.duty4>MOTOR_MAX_DUTY)		aircraft_motor.duty4 = MOTOR_MAX_DUTY;
	else if(aircraft_motor.duty4<MOTOR_MIN_DUTY)		aircraft_motor.duty4 = MOTOR_MIN_DUTY;	
	/***限幅处理end***/	
	
	/*RC SBUS 电机功率控制*/
	if(my_remote.sbus.CH[7]==RC_SBUS_CH_MIN)//右拨杆上升
	{
	power = 1.0f;//最大功率
	}
	else if(my_remote.sbus.CH[7]==RC_SBUS_CH_MID)//右拨杆中间
	{
	power = 1.0f;//75%功率
	}
	else if(my_remote.sbus.CH[7]==RC_SBUS_CH_MAX)//右拨杆下降
	{
	power = 0.0f;//关闭电机
	}
	/***HAL库占空比设置***/
	if(my_aircraft.status&0x01 && !(my_aircraft.status&0x08))//解锁且无保护
	{
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,aircraft_motor.duty1*power);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,aircraft_motor.duty2*power);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,aircraft_motor.duty3*power);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,aircraft_motor.duty4*power);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,0);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,0);		
	}
}
//电机控制
void motor_control()
{
	/***占空比复位***/
	aircraft_motor.duty1=0;
	aircraft_motor.duty2=0;
	aircraft_motor.duty3=0;
	aircraft_motor.duty4=0;	

	/***姿态控制begin***/	
	aircraft_motor.duty1 +=(int)AttitudeControl.internal_pid.x.output;
	aircraft_motor.duty2 -=(int)AttitudeControl.internal_pid.x.output;
	aircraft_motor.duty3 +=(int)AttitudeControl.internal_pid.x.output;
	aircraft_motor.duty4 -=(int)AttitudeControl.internal_pid.x.output;	
	
	aircraft_motor.duty1 -=(int)AttitudeControl.internal_pid.y.output;
	aircraft_motor.duty2 -=(int)AttitudeControl.internal_pid.y.output;
	aircraft_motor.duty3 +=(int)AttitudeControl.internal_pid.y.output;
	aircraft_motor.duty4 +=(int)AttitudeControl.internal_pid.y.output;
	
	aircraft_motor.duty1 -=(int)AttitudeControl.internal_pid.z.output;
	aircraft_motor.duty2 +=(int)AttitudeControl.internal_pid.z.output;
	aircraft_motor.duty3 +=(int)AttitudeControl.internal_pid.z.output;
	aircraft_motor.duty4 -=(int)AttitudeControl.internal_pid.z.output;	
	
	/***姿态环输出限幅***/
	if(aircraft_motor.duty1<MOTOR_MIN_DUTY) aircraft_motor.duty1 =MOTOR_MIN_DUTY;
	if(aircraft_motor.duty2<MOTOR_MIN_DUTY) aircraft_motor.duty2 =MOTOR_MIN_DUTY;
	if(aircraft_motor.duty3<MOTOR_MIN_DUTY) aircraft_motor.duty3 =MOTOR_MIN_DUTY;
	if(aircraft_motor.duty4<MOTOR_MIN_DUTY) aircraft_motor.duty4 =MOTOR_MIN_DUTY;
	
	if(aircraft_motor.duty1>MOTOR_MAX_DUTY) aircraft_motor.duty1 =MOTOR_MAX_DUTY;
	if(aircraft_motor.duty2>MOTOR_MAX_DUTY) aircraft_motor.duty2 =MOTOR_MAX_DUTY;
  if(aircraft_motor.duty3>MOTOR_MAX_DUTY) aircraft_motor.duty3 =MOTOR_MAX_DUTY;
	if(aircraft_motor.duty4>MOTOR_MAX_DUTY) aircraft_motor.duty4 =MOTOR_MAX_DUTY;	
	/***姿态控制end***/	

	/***油门控制begin***/
	aircraft_motor.duty1 +=(int)aircraft_motor.throttle;
	aircraft_motor.duty2 +=(int)aircraft_motor.throttle;
	aircraft_motor.duty3 +=(int)aircraft_motor.throttle;
	aircraft_motor.duty4 +=(int)aircraft_motor.throttle;		
	/***油门控制end***/
	
	motor_speed_set();//电机驱动
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
void motor_throttle_control()
{
	if(HeightControl.auto_height_control_isEnable)//自动定高油门控制
	{
		if(remote_data_flash[0]==1)//定高高度控制 
		{
			if(abs((int)my_remote.YG_LEFT_UD-128)>40)
				HeightControl.target_height += ((int)my_remote.YG_LEFT_UD - 128)*0.00015f;
			else 
				HeightControl.target_height+=0;

			remote_data_flash[0] = 0;
		}	
		/*SBUS*/
		if(abs((int)my_remote.sbus.CH[2]-RC_SBUS_CH_MID)>200)
			 HeightControl.target_height += (float)((int)my_remote.sbus.CH[2]-RC_SBUS_CH_MID)*HeightControl.auto_height_climb_rate;
		
		aircraft_motor.throttle = (int)(HeightControl.hover_throttle*(MOTOR_MAX_THROTTLE/100.0f)) + HeightControl.pid.output;//油门输出
	
	}
	else//默认手动油门控制
	{
		if(remote_data_flash[0]==1)
		{
			if(abs((int)my_remote.YG_LEFT_UD-128)>40)
				aircraft_motor.throttle += ((int)my_remote.YG_LEFT_UD - 128)*0.3f;
			else 
				aircraft_motor.throttle+=0;
			remote_data_flash[0] = 0;
		}
		/*SBUS*/
		aircraft_motor.throttle = (float)MOTOR_MAX_THROTTLE * (float)(my_remote.sbus.CH[2]-RC_SBUS_CH_MIN)/(float)(RC_SBUS_CH_RANGE);
	}
	aircraft_motor.throttle += (int)(aircraft_motor.launch_throttle*(MOTOR_MAX_THROTTLE/100.0f)); //启动油门加成
	/***油门限幅***/
	if(aircraft_motor.throttle>MOTOR_MAX_THROTTLE)
		aircraft_motor.throttle = MOTOR_MAX_THROTTLE;
	else if(aircraft_motor.throttle<MOTOR_MIN_THROTTLE)
		aircraft_motor.throttle = MOTOR_MIN_THROTTLE;
}
/*电机油门控制end*/

/*飞行器飞行方向控制begin*/
void aircraft_flight_direction_control()
{
	static uint32_t poscontrol_speed_cnt = 0;
	static uint32_t poscontrol_postion_cnt = 0;
	if(PositionControl.auto_pos_control_isEnable)//开启自动定点
	{
		if(poscontrol_speed_cnt>=4)//内环50Hz
		{
			
			AttitudeControl.pitch_target_angle = PositionControl.internal_pid_x.f_cal_pid(&PositionControl.internal_pid_x,*PositionControl.sensor.speed_x);
			
			AttitudeControl.roll_target_angle = - PositionControl.internal_pid_y.f_cal_pid(&PositionControl.internal_pid_y,*PositionControl.sensor.speed_y);		
			
			poscontrol_speed_cnt = 0;
		}
		if(poscontrol_postion_cnt>=4)//外环50Hz
		{
			PositionControl.internal_pid_x.target = PositionControl.external_pid_x.f_cal_pid(&PositionControl.external_pid_x,*PositionControl.sensor.pos_x);
			
			PositionControl.internal_pid_y.target = PositionControl.external_pid_y.f_cal_pid(&PositionControl.external_pid_y,*PositionControl.sensor.pos_y);				
			poscontrol_postion_cnt = 0;
		}
		poscontrol_speed_cnt++;
		poscontrol_postion_cnt++;
	}
	else//手动方向控制
	{
		poscontrol_speed_cnt = 0;
		poscontrol_postion_cnt = 0;		
		if(remote_data_flash[1]==1)
		{
			if(abs((int)my_remote.YG_RIGHT_LR-128)>20)
			{
				if(my_remote.YG_RIGHT_LR>128)
				{
					AttitudeControl.roll_target_angle = +(float)(my_remote.YG_RIGHT_LR-128-20)*((float)ROLL_TARGET_MAX_ANGLE/107.0f);
				}
				else
				{
					AttitudeControl.roll_target_angle = +(float)(my_remote.YG_RIGHT_LR-128+20)*((float)ROLL_TARGET_MAX_ANGLE/108.0f);
				}
				
			}
			else
				AttitudeControl.roll_target_angle = 0;
			
			if(abs((int)my_remote.YG_RIGHT_UD-128)>20)	
			{
				if(my_remote.YG_RIGHT_UD>128)
				{
					AttitudeControl.pitch_target_angle = -(float)(my_remote.YG_RIGHT_UD-128-20)*((float)PITCH_TARGET_MAX_ANGLE/107.0f);
				}
				else
				{
					AttitudeControl.pitch_target_angle = -(float)(my_remote.YG_RIGHT_UD-128+20)*((float)PITCH_TARGET_MAX_ANGLE/108.0f);
				}
				
			}
			else
				AttitudeControl.pitch_target_angle = 0;
			
			if(abs((int)my_remote.YG_LEFT_LR-128)>100)
				AttitudeControl.yaw_target_angle += (float)(my_remote.YG_LEFT_LR-128)*0.008f;
			else
				AttitudeControl.yaw_target_angle -=0;
				
			remote_data_flash[1]=0;
		}
		/*SBUS*/
		if(abs((int)my_remote.sbus.CH[0]-RC_SBUS_CH_MID)>100)
			AttitudeControl.roll_target_angle = -                                                                                                              (float)((int)my_remote.sbus.CH[0]-RC_SBUS_CH_MID)*((float)ROLL_TARGET_MAX_ANGLE/(float)(RC_SBUS_CH_RANGE*0.5f));
		else
			AttitudeControl.roll_target_angle = 0;
		if(abs((int)my_remote.sbus.CH[1]-RC_SBUS_CH_MID)>100)
			AttitudeControl.pitch_target_angle = -(float)((int)my_remote.sbus.CH[1]-RC_SBUS_CH_MID)*((float)PITCH_TARGET_MAX_ANGLE/(float)(RC_SBUS_CH_RANGE*0.5f));
		else
			AttitudeControl.pitch_target_angle = 0;
		if(abs((int)my_remote.sbus.CH[3]-RC_SBUS_CH_MID)>100)
			AttitudeControl.yaw_target_angle -= (float)((int)my_remote.sbus.CH[3]-RC_SBUS_CH_MID)*AttitudeControl.yaw_turn_rate;

	}
		/****姿态外环输入begin****/
		if(AttitudeControl.roll_target_angle>ROLL_TARGET_MAX_ANGLE)
			AttitudeControl.roll_target_angle = ROLL_TARGET_MAX_ANGLE;
		else if(AttitudeControl.roll_target_angle<ROLL_TARGET_MIN_ANGLE)
			AttitudeControl.roll_target_angle = ROLL_TARGET_MIN_ANGLE;
		AttitudeControl.external_pid.x.target = AttitudeControl.roll_target_angle+AttitudeControl.roll_compensate;
		
		if(AttitudeControl.pitch_target_angle>PITCH_TARGET_MAX_ANGLE)
			AttitudeControl.pitch_target_angle = PITCH_TARGET_MAX_ANGLE;
		else if(AttitudeControl.pitch_target_angle<PITCH_TARGET_MIN_ANGLE)
			AttitudeControl.pitch_target_angle = PITCH_TARGET_MIN_ANGLE;	
		AttitudeControl.external_pid.y.target = AttitudeControl.pitch_target_angle+AttitudeControl.pitch_compensate;
		
//		if(AttitudeControl.yaw_target_angle>YAW_TARGET_MAX_ANGLE)
//			AttitudeControl.yaw_target_angle = YAW_TARGET_MAX_ANGLE;
//		else if(AttitudeControl.yaw_target_angle<YAW_TARGET_MIN_ANGLE)
//			AttitudeControl.yaw_target_angle = YAW_TARGET_MIN_ANGLE;	
		AttitudeControl.external_pid.z.target = AttitudeControl.yaw_target_angle;
		/****姿态外环输入end****/
}
/*飞行器飞行方向控制end*/

/*飞行器飞行高度控制begin*/

void aircraft_flight_Height_control()
{
	if(HeightControl.auto_height_control_isEnable)//开启自动定高
	{
		if(TOF.distance_m <3.0f&&TOF.confidence >80)//TOF测距小于3m 并且可信度高于80
		{
			HeightControl.mode = TOF_MODE;//TOF定高
		}
		else
		{
			HeightControl.mode = BARO_MODE;//气压计定高
	
		}
		if(HeightControl.mode == BARO_MODE)//气压计定高
		{
			HeightControl.pid.target = HeightControl.target_altitude + HeightControl.target_height;
			HeightControl.pid.MaxOutput = MOTOR_MAX_THROTTLE - (int)(HeightControl.hover_throttle*(MOTOR_MAX_THROTTLE/100.0f));
		}
		else if(HeightControl.mode == TOF_MODE)//TOF定高
		{
			HeightControl.pid.target =  HeightControl.target_height;
			HeightControl.pid.MaxOutput = MOTOR_MAX_THROTTLE - (int)(HeightControl.hover_throttle*(MOTOR_MAX_THROTTLE/100.0f));

		}
		HeightControl.pid.f_cal_pid(&HeightControl.pid,*HeightControl.sensor.tof_height);
	}
	else   
		return;
}
/*飞行器飞行高度控制end*/


void pid_control_init()
{
	/***姿态控制初始化begin***/
	pid_init(&AttitudeControl.internal_pid.x);
	AttitudeControl.sensor.gyro_x = &my_ahrs.IMU_Data.gyro.x;
	pid_init(&AttitudeControl.internal_pid.y);
	AttitudeControl.sensor.gyro_y = &my_ahrs.IMU_Data.gyro.y;
	pid_init(&AttitudeControl.internal_pid.z);
	AttitudeControl.sensor.gyro_z = &my_ahrs.IMU_Data.gyro.z;
	AttitudeControl.internal_pid.x.f_param_init(&AttitudeControl.internal_pid.x,PID_Position,MOTOR_MAX_DUTY,0,0,0,0,20.0f,0.0f,40.0f);
	pid_enable(&AttitudeControl.internal_pid.x,0);//未开启飞机就不使能pid
	AttitudeControl.internal_pid.y.f_param_init(&AttitudeControl.internal_pid.y,PID_Position,MOTOR_MAX_DUTY,0,0,0,0,20.0f,0.0f,40.0f);
	pid_enable(&AttitudeControl.internal_pid.y,0);//未开启飞机就不使能pid
	AttitudeControl.internal_pid.z.f_param_init(&AttitudeControl.internal_pid.z,PID_Position,MOTOR_MAX_DUTY,0,0,0,0,20.0f,0.0f,50.0f);
	pid_enable(&AttitudeControl.internal_pid.z,0);//未开启飞机就不使能pid
	
	pid_init(&AttitudeControl.external_pid.x);
	AttitudeControl.sensor.roll = &my_ahrs.Angle_Data.roll;
	pid_init(&AttitudeControl.external_pid.y);
	AttitudeControl.sensor.pitch = &my_ahrs.Angle_Data.pitch;
	pid_init(&AttitudeControl.external_pid.z);
	AttitudeControl.sensor.yaw = &my_ahrs.Angle_Data.yaw;
	AttitudeControl.external_pid.x.f_param_init(&AttitudeControl.external_pid.x,PID_Position,10000,1000,0,0,0,4.0f,0.0f,0.0f);
	pid_enable(&AttitudeControl.external_pid.x,0);//未开启飞机就不使能pid
	AttitudeControl.external_pid.y.f_param_init(&AttitudeControl.external_pid.y,PID_Position,10000,1000,0,0,0,4.0f,0.0f,0.0f);
	pid_enable(&AttitudeControl.external_pid.y,0);//未开启飞机就不使能pid
	AttitudeControl.external_pid.z.f_param_init(&AttitudeControl.external_pid.z,PID_Position,1000,100,0,0,0,6.0f,0.0f,0.0f);	
	pid_enable(&AttitudeControl.external_pid.z,0);//未开启飞机就不使能pid
	/***姿态控制初始化end***/
	
	/***高度控制初始化begin***/
	pid_init(&HeightControl.pid);
	HeightControl.sensor.tof_height = &TOF.distance_m;
	HeightControl.sensor.barometer_height = &my_aircraft.Altitude;
	HeightControl.pid.f_param_init(&HeightControl.pid,PID_Position,MOTOR_MAX_THROTTLE,MOTOR_MAX_THROTTLE,0,0,0,1500.0f,0.0f,30000.0f);
	pid_enable(&HeightControl.pid,0);//未开启定高就不使能pid
	HeightControl.auto_height_control_isEnable = 0;
	HeightControl.mode = OFF_MODE;//初始为关闭

	/***高度控制初始化end***/	
	
	/***位置控制初始化begin***/
	pid_init(&PositionControl.internal_pid_x);
	pid_init(&PositionControl.internal_pid_y);
	PositionControl.sensor.speed_x = &OpticalFlow.flow_x_speed;
	PositionControl.sensor.speed_y = &OpticalFlow.flow_y_speed;
	PositionControl.internal_pid_x.f_param_init(&PositionControl.internal_pid_x,PID_Position,ROLL_TARGET_MAX_ANGLE,0,0,0,0,0.0f,0.0f,0.0f);
	PositionControl.internal_pid_y.f_param_init(&PositionControl.internal_pid_y,PID_Position,PITCH_TARGET_MAX_ANGLE,0,0,0,0,0.0f,0.0f,0.0f);
	pid_enable(&PositionControl.internal_pid_x,0);//未开启定点就不使能pid
	pid_enable(&PositionControl.internal_pid_y,0);//未开启定点就不使能pid

	pid_init(&PositionControl.external_pid_x);
	pid_init(&PositionControl.external_pid_y);
	PositionControl.sensor.pos_x = &OpticalFlow.flow_x_pos;
	PositionControl.sensor.pos_y = &OpticalFlow.flow_y_pos;
	PositionControl.external_pid_x.f_param_init(&PositionControl.external_pid_x,PID_Position,X_TARGET_MAX_SPEED,X_TARGET_MAX_SPEED,0,0,0,0.0f,0.0f,0.0f);
	PositionControl.external_pid_y.f_param_init(&PositionControl.external_pid_y,PID_Position,Y_TARGET_MAX_SPEED,Y_TARGET_MAX_SPEED,0,0,0,0.0f,0.0f,0.0f);
	pid_enable(&PositionControl.external_pid_x,0);//未开启定点就不使能pid
	pid_enable(&PositionControl.external_pid_y,0);//未开启定点就不使能pid
	/***位置控制初始化end***/
	
	if(my_fatfs_init_success)
	{
		if(fatfs_PID_params_read())//pid参数读取
		{
			while(1)
			{
				LED_TOGGLE;
				HAL_Delay(500);
			}
		}

	}
	motor_init();//电机初始化
}

void pid_internal_control()
{ 

	/****pid计算begin****/
	AttitudeControl.internal_pid.x.f_cal_pid(&AttitudeControl.internal_pid.x,*AttitudeControl.sensor.gyro_x);//X轴

	AttitudeControl.internal_pid.y.f_cal_pid(&AttitudeControl.internal_pid.y,*AttitudeControl.sensor.gyro_y);//Y轴

	AttitudeControl.internal_pid.z.f_cal_pid(&AttitudeControl.internal_pid.z,*AttitudeControl.sensor.gyro_z);//Z轴

	/****pid计算end****/
	
}
void pid_external_control()
{    
	/****pid计算begin****/
	AttitudeControl.external_pid.x.f_cal_pid(&AttitudeControl.external_pid.x,*AttitudeControl.sensor.roll);//ROLL

	AttitudeControl.external_pid.y.f_cal_pid(&AttitudeControl.external_pid.y,*AttitudeControl.sensor.pitch);//PITCH

	AttitudeControl.external_pid.z.f_cal_pid(&AttitudeControl.external_pid.z,*AttitudeControl.sensor.yaw);//YAW

	/****pid计算end****/
	
	/****外环输出begin****/
	AttitudeControl.internal_pid.x.target = AttitudeControl.external_pid.x.output;
	
	AttitudeControl.internal_pid.y.target = AttitudeControl.external_pid.y.output;
	
	AttitudeControl.internal_pid.z.target = AttitudeControl.external_pid.z.output;
	/****外环输出end****/
}

/*飞行器保护措施begin*/
uint8_t aircraft_protection()
{
	static uint32_t Battery_Low_Power_cnt =0;
	static uint32_t Aircraft_IMU_Error_cnt =0;
	static uint32_t Motor_Duty_Error_cnt =0;
	static uint32_t lose_signal_cnt =0;


	if(my_aircraft.Battery_Volt <37 )//电池低电量保护
	{
		LED(0);
		Battery_Low_Power_cnt ++;
		if(Battery_Low_Power_cnt >= 300)//30s
		{
			motor_deinit();
			my_aircraft.status |=0x08;//set status bit high
			return 1;

		}
	}
	else 
	{
		Battery_Low_Power_cnt =0;
	}
	
	if(fabs(my_ahrs.Angle_Data.roll)>80||fabs(my_ahrs.Angle_Data.pitch)>80)//飞行器姿态错误保护
	{
		Aircraft_IMU_Error_cnt++;
		if(Aircraft_IMU_Error_cnt >= 10)//1s
		{
			motor_deinit();
			my_aircraft.status |=0x08;//set status bit high			
			return 2;
		}		
	}
	else
	{
		
		Aircraft_IMU_Error_cnt=0;
	}
	
	if(aircraft_motor.duty1>=MOTOR_MAX_DUTY*0.99f||aircraft_motor.duty2>=MOTOR_MAX_DUTY*0.99f||aircraft_motor.duty3>=MOTOR_MAX_DUTY*0.99f||aircraft_motor.duty4>=MOTOR_MAX_DUTY*0.99f)//电机高占空比保护
	{
		Motor_Duty_Error_cnt++;
		if(Motor_Duty_Error_cnt >= 20)//2s
		{
			motor_deinit();
			my_aircraft.status |=0x08;//set status bit high			
			return 3;
		}		
	}
	else
	{
	
		Motor_Duty_Error_cnt=0;
	}
	if(my_remote.sbus.signal){			
		my_remote.sbus.signal = 0;
		lose_signal_cnt =0;
	}
	else{
		lose_signal_cnt ++;
		if(lose_signal_cnt > 20){//超过2秒无遥控信号
			motor_deinit();
			my_aircraft.status |=0x08;//set status bit high			
			return 4;
		}
	}
	//all protection clear
	motor_init();
	my_aircraft.status &= ~0x08;//set status bit low
	return 0;
	
}
/*飞行器保护措施end*/

/*飞行器状态检测*/
void aircraft_status_check()
{
	if((my_remote.sbus.CH[5]==RC_SBUS_CH_MAX)&&(my_aircraft.status&0x01)==0x00)//解锁
	{
		/*油门摇杆限制*/
		if(my_remote.sbus.CH[2] > RC_SBUS_CH_MIN + (RC_SBUS_CH_RANGE/10)) //油门杆必须在最低点才能解锁
			return;
		my_aircraft.status = 0x01;//解锁
		my_aircraft.fly_start_time = HAL_GetTick();
		pid_enable(&AttitudeControl.internal_pid.x,1);//open pid
		pid_enable(&AttitudeControl.internal_pid.y,1);
		pid_enable(&AttitudeControl.internal_pid.z,1);
		pid_enable(&AttitudeControl.external_pid.x,1);
		pid_enable(&AttitudeControl.external_pid.y,1);
		pid_enable(&AttitudeControl.external_pid.z,1);
		AttitudeControl.yaw_target_angle = *AttitudeControl.sensor.yaw;		
	}                                                                                                    
	else if((my_remote.sbus.CH[5] == RC_SBUS_CH_MIN)&&(my_aircraft.status&0x01)==0x01)//上锁
	{
		my_aircraft.status = 0x00;//上锁
		pid_enable(&AttitudeControl.internal_pid.x,0);//close pid
		pid_enable(&AttitudeControl.internal_pid.y,0);
		pid_enable(&AttitudeControl.internal_pid.z,0);
		pid_enable(&AttitudeControl.external_pid.x,0);
		pid_enable(&AttitudeControl.external_pid.y,0);
		pid_enable(&AttitudeControl.external_pid.z,0);			
		pid_enable(&HeightControl.pid,0);//close height pid		
		pid_enable(&PositionControl.internal_pid_x,0);//close position pid
		pid_enable(&PositionControl.internal_pid_y,0);
		pid_enable(&PositionControl.external_pid_x,0);
		pid_enable(&PositionControl.external_pid_y,0);	

	}

	if(my_remote.sbus.CH[6] == RC_SBUS_CH_MAX && (my_aircraft.status & 0x02)==0x00 )//自动高度控制模式
	{
		if(TOF.is_valid ==0 && my_bmp390.is_valid==0)//TOF和气压计无效不能开启自动定高
			return;		
			
		HeightControl.auto_height_control_isEnable = 1; //open auto height hold
		my_aircraft.status |= 0x02;//自动定高标志位
		HeightControl.target_height = TOF.distance_m;//current TOF height as target height
		HeightControl.target_altitude = my_aircraft.Altitude;//current Baro altitude as target altitude
		pid_enable(&HeightControl.pid,1);//open height pid
	}
	else if(my_remote.sbus.CH[6] == RC_SBUS_CH_MIN && (my_aircraft.status & 0x02)==0x02 )//手动高度控制模式
	{
		HeightControl.auto_height_control_isEnable = 0; //close auto height hold
		my_aircraft.status &= ~0x02;//自动定高标志位清除
		HeightControl.mode = OFF_MODE;//height hold disabled
		HeightControl.target_height = 0;
		HeightControl.target_altitude =0;
		pid_enable(&HeightControl.pid,0);//close height pid		
	}

	
}



