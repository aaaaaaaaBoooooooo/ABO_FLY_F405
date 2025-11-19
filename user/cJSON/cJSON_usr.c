/*cJSON_usr.c by aBo*******cJSON的应用函数*/
#include "cJSON_usr.h"
#include "stdio.h"
#include "string.h"
#include "fatfs.h"
#include "control.h"
#include "flight_data_manager.h"

/*以json格式打包数字*/
void json_pack_number(const char *item,double number)
{
  cJSON * usr;
//	char *str_data;
  usr=cJSON_CreateObject();   //创建根数据对象
	
	cJSON_AddItemToObject(usr,item, cJSON_CreateNumber(number));  //根节点下添加数字
	
//	str_data = cJSON_Print(usr);   //将json形式打印成正常字符串形式(带有\r\n)
//	 str_data = cJSON_PrintUnformatted(usr);   //将json形式打印成正常字符串形式(没有\r\n)
//	printf("%s",str_data);			//通过串口打印出来
	cJSON_Delete(usr);
	
}
/*以json格式打包字符串*/
void json_pack_string(const char *item,char * string)
{
  cJSON * usr;
//	char *str_data;
  usr=cJSON_CreateObject();   //创建根数据对象
	
	cJSON_AddItemToObject(usr,item, cJSON_CreateString(string));  //根节点下添加数字
	
//	str_data = cJSON_Print(usr);   //将json形式打印成正常字符串形式(带有\r\n)
//	 data = cJSON_PrintUnformatted(usr);   //将json形式打印成正常字符串形式(没有\r\n)
//	printf("%s",data);			//通过串口打印出来
	cJSON_Delete(usr);
	
}
/*以json格式解析浮点数*/
double json_analysis_double(char *data,const char *item)
{

	cJSON *json,*json_get;
	double get_number=0;
	json = cJSON_Parse((const char *)data); //将得到的字符串解析成json形式
	/****************************/
	/*	  测试将JSON打印出来	*/
	/***************************/
 char *out_data = cJSON_Print(json);   //将json形式打印成正常字符串形式
 printf("%s",out_data);
	json_get = cJSON_GetObjectItem( json ,item );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
			get_number = json_get->valuedouble;
			cJSON_Delete(json);  //释放内存 
			cJSON_Delete(json_get);  //释放内存 	
		return get_number;
	}
	else 
	{
			cJSON_Delete(json);  //释放内存 
			cJSON_Delete(json_get);  //释放内存 			
		return 0;
	}

}
/*以json格式解析整形数*/
int json_analysis_int(const char *data,const char *item)
{

	cJSON *json,*json_get;
	int get_number=0;
	json = cJSON_Parse(data); //将得到的字符串解析成json形式
	/****************************/
	/*	  测试将JSON打印出来	*/
	/***************************/
 //char *out_data = cJSON_Print(json);   //将json形式打印成正常字符串形式
 //printf("%s",out_data);
	json_get = cJSON_GetObjectItem( json ,item );
	if(json_get->type == cJSON_Number)  //从json获取键值内容
	{
			get_number = json_get->valuedouble;
			cJSON_Delete(json);  //释放内存 
			cJSON_Delete(json_get);  //释放内存 	
		return get_number;
	}
	else 
	{
			cJSON_Delete(json);  //释放内存 
			cJSON_Delete(json_get);  //释放内存 			
		return 0;
	}
}

/*读取FLASH中保存的参数*/
#define PARAM_FILE_PATH "/param.txt"
#define PARAMS_MAX_READ_SIZE  4096
const char *PARAMS_DEFAULT_JSON =
		"{\n"
		"\"AttitudeControl.int_PID\":{\"X\":[25,0.1,30],\"Y\":[25,0.1,30],\"Z\":[15.0,0.0,15.0]},\n"
		"\"AttitudeControl.ext_PID\":{\"X\":[3.0,0.0,0.0],\"Y\":[3.0,0.0,0.0],\"Z\":[4.0,0.0,0.0]},\n"
		"\"HeightControl.PID\":[0.0,0.0,0.0],\n"
		"\"motor_volt_k\":[1.0,1.0,1.0,1.0],\n"
		"\"log_record_circle_ms\":20\n"
		"}\n";
uint8_t fatfs_PID_params_read()
{
	cJSON *json,*json_get_root,*json_get_child1;
	char read_buf[PARAMS_MAX_READ_SIZE];
	memset(read_buf,0,PARAMS_MAX_READ_SIZE);
	if(fatfs_read_file(PARAM_FILE_PATH,read_buf,PARAMS_MAX_READ_SIZE))//读取文件失败
	{
		FIL file;     //文件对象
		FRESULT res;
		UINT bytes_written;  // 实际写入的字节数
		strcpy(read_buf,PARAMS_DEFAULT_JSON);//将默认参数写入缓冲区
		res = f_open(&file, PARAM_FILE_PATH,FA_OPEN_ALWAYS|FA_WRITE);
		if (res == FR_OK)
		{
			printf("paramFile created successfully.\n");
			/****将默认参数写入文件***/
			res = f_write(&file, read_buf, strlen(read_buf), &bytes_written);
			if (res == FR_OK)
			{
					f_close(&file);
					printf("Param written successfully. Bytes written: %u\n", bytes_written);
			}
			else
			{
					f_close(&file);
					printf("paramWrite error: %d\n", res);
					return 1;
			}
		}
		else
		{
				printf("Failed to open/create file. Error: %d\n", res);
				return 1;
		}
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
	
	json_get_root = cJSON_GetObjectItem( json ,"AttitudeControl.int_PID" );
	json_get_child1 = cJSON_GetObjectItem(json_get_root,"X");
	if(json_get_child1->type == cJSON_Array)  //从json获取键值内容
	{
		AttitudeControl.internal_pid.x.kp = cJSON_GetArrayItem(json_get_child1,0)->valuedouble;
		AttitudeControl.internal_pid.x.ki = cJSON_GetArrayItem(json_get_child1,1)->valuedouble;
		AttitudeControl.internal_pid.x.kd = cJSON_GetArrayItem(json_get_child1,2)->valuedouble;
	}
	json_get_child1 = cJSON_GetObjectItem(json_get_root,"Y");
	if(json_get_child1->type == cJSON_Array)  //从json获取键值内容
	{
		AttitudeControl.internal_pid.y.kp = cJSON_GetArrayItem(json_get_child1,0)->valuedouble;
		AttitudeControl.internal_pid.y.ki = cJSON_GetArrayItem(json_get_child1,1)->valuedouble;
		AttitudeControl.internal_pid.y.kd = cJSON_GetArrayItem(json_get_child1,2)->valuedouble;
	}
	json_get_child1 = cJSON_GetObjectItem(json_get_root,"Z");
	if(json_get_child1->type == cJSON_Array)  //从json获取键值内容
	{
		AttitudeControl.internal_pid.z.kp = cJSON_GetArrayItem(json_get_child1,0)->valuedouble;
		AttitudeControl.internal_pid.z.ki = cJSON_GetArrayItem(json_get_child1,1)->valuedouble;
		AttitudeControl.internal_pid.z.kd = cJSON_GetArrayItem(json_get_child1,2)->valuedouble;
	}
	json_get_root = cJSON_GetObjectItem( json ,"AttitudeControl.ext_PID" );
	json_get_child1 = cJSON_GetObjectItem(json_get_root,"X");
	if(json_get_child1->type == cJSON_Array)  //从json获取键值内容
	{
		AttitudeControl.external_pid.x.kp = cJSON_GetArrayItem(json_get_child1,0)->valuedouble;
		AttitudeControl.external_pid.x.ki = cJSON_GetArrayItem(json_get_child1,1)->valuedouble;
		AttitudeControl.external_pid.x.kd = cJSON_GetArrayItem(json_get_child1,2)->valuedouble;
	}
	json_get_child1 = cJSON_GetObjectItem(json_get_root,"Y");
	if(json_get_child1->type == cJSON_Array)  //从json获取键值内容
	{
		AttitudeControl.external_pid.y.kp = cJSON_GetArrayItem(json_get_child1,0)->valuedouble;
		AttitudeControl.external_pid.y.ki = cJSON_GetArrayItem(json_get_child1,1)->valuedouble;
		AttitudeControl.external_pid.y.kd = cJSON_GetArrayItem(json_get_child1,2)->valuedouble;
	}
	json_get_child1 = cJSON_GetObjectItem(json_get_root,"Z");
	if(json_get_child1->type == cJSON_Array)  //从json获取键值内容
	{
		AttitudeControl.external_pid.z.kp = cJSON_GetArrayItem(json_get_child1,0)->valuedouble;
		AttitudeControl.external_pid.z.ki = cJSON_GetArrayItem(json_get_child1,1)->valuedouble;
		AttitudeControl.external_pid.z.kd = cJSON_GetArrayItem(json_get_child1,2)->valuedouble;
	}
	json_get_root = cJSON_GetObjectItem( json ,"HeightControl.PID");	
	if(json_get_root->type == cJSON_Array)  //从json获取键值内容
	{
		HeightControl.pid.kp = cJSON_GetArrayItem(json_get_root,0)->valuedouble;
		HeightControl.pid.ki = cJSON_GetArrayItem(json_get_root,1)->valuedouble;
		HeightControl.pid.kd = cJSON_GetArrayItem(json_get_root,2)->valuedouble;
	}
	json_get_root = cJSON_GetObjectItem( json ,"motor_volt_k");
	if(json_get_root->type == cJSON_Array)  //从json获取键值内容
	{
		aircraft_motor.volt_k1 = cJSON_GetArrayItem(json_get_root,0)->valuedouble;
		aircraft_motor.volt_k2 = cJSON_GetArrayItem(json_get_root,1)->valuedouble;
		aircraft_motor.volt_k3 = cJSON_GetArrayItem(json_get_root,2)->valuedouble;
		aircraft_motor.volt_k4 = cJSON_GetArrayItem(json_get_root,3)->valuedouble;

	}	
	json_get_root = cJSON_GetObjectItem( json ,"log_record_circle_ms");
	if(json_get_root->type == cJSON_Number)  //从json获取键值内容
	{	
		log_record_circle = json_get_root->valueint;
	}	
	
	printf("param_read_success:\n"
		   "%.2f,%.2f,%.2f\n"
		   "%.2f,%.2f,%.2f\n"
		   "%.2f,%.2f,%.2f\n"
		   "%.2f,%.2f,%.2f\n"
		   "%.2f,%.2f,%.2f\n"
		   "%.2f,%.2f,%.2f\n",
		AttitudeControl.internal_pid.x.kp, AttitudeControl.internal_pid.x.ki, AttitudeControl.internal_pid.x.kd,
		AttitudeControl.internal_pid.y.kp, AttitudeControl.internal_pid.y.ki, AttitudeControl.internal_pid.y.kd,
		AttitudeControl.internal_pid.z.kp, AttitudeControl.internal_pid.z.ki, AttitudeControl.internal_pid.z.kd,
		AttitudeControl.external_pid.x.kp, AttitudeControl.external_pid.x.ki, AttitudeControl.external_pid.x.kd,
		AttitudeControl.external_pid.y.kp, AttitudeControl.external_pid.y.ki, AttitudeControl.external_pid.y.kd,
		AttitudeControl.external_pid.z.kp, AttitudeControl.external_pid.z.ki, AttitudeControl.external_pid.z.kd
	);
	cJSON_Delete(json);  //释放内存 
	cJSON_Delete(json_get_root);  //释放内存 		
	return 0;
}

