#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
#include "pid.h"
#define LED(x)    			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,x)
#define LED_TOGGLE   		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
#define MOTOR_MAX_THROTTLE 5000				//电机最大油门
#define MOTOR_MIN_THROTTLE 0 					//电机最小油门
#define MOTOR_MAX_DUTY  7000          //电机最大占空比
#define MOTOR_MIN_DUTY  0							//电机最小占空比

#define ROLL_TARGET_MAX_ANGLE  15
#define ROLL_TARGET_MIN_ANGLE  -15
#define PITCH_TARGET_MAX_ANGLE  15
#define PITCH_TARGET_MIN_ANGLE -15
#define YAW_TARGET_MAX_ANGLE  360
#define YAW_TARGET_MIN_ANGLE  -360

typedef struct
{
	PID_TypeDef pid_x;
	PID_TypeDef pid_y;
	PID_TypeDef pid_z;
}PID_XYZ_TypeDef;

typedef struct
{
	PID_XYZ_TypeDef internal;//内环pid
	PID_XYZ_TypeDef external;//外环pid
	
	
}PID_FC_TypeDef;
extern PID_FC_TypeDef pid_AttitudeControl;
extern int motor1_duty;
extern int motor2_duty;
extern int motor3_duty;
extern int motor4_duty;
extern int aircraft_throttle;//油门
extern float roll_target_angle;
extern float pitch_target_angle;
extern float yaw_target_angle;
extern float roll_compensate;//补偿
extern float pitch_compensate;//补偿
extern uint8_t aircraft_state;
void pid_control_init(void);
void pid_internal_control(void);
void pid_external_control(void);
void motor_init(void);
void motor_deinit(void);
void motor_speed_set(int m1_duty,int m2_duty,int m3_duty,int m4_duty);
void motor_throttle_control(uint8_t type);
void aircraft_flight_direction_control(void);
uint8_t fatfs_params_read(void);
uint8_t aircraft_protection(void);

#endif
