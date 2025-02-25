/*cJSON_usr.c by aBo*******cJSON的应用函数*/
#include "cJSON_usr.h"
#include "stdio.h"
#include "string.h"
/*以json格式打包数字*/
void json_pack_number(const char *item,double number)
{
  cJSON * usr;
	char *str_data;
  usr=cJSON_CreateObject();   //创建根数据对象
	
	cJSON_AddItemToObject(usr,item, cJSON_CreateNumber(number));  //根节点下添加数字
	
	str_data = cJSON_Print(usr);   //将json形式打印成正常字符串形式(带有\r\n)
//	 str_data = cJSON_PrintUnformatted(usr);   //将json形式打印成正常字符串形式(没有\r\n)
//	printf("%s",str_data);			//通过串口打印出来
	cJSON_Delete(usr);
	
}
/*以json格式打包字符串*/
void json_pack_string(const char *item,char * string)
{
  cJSON * usr;
	char *str_data;
  usr=cJSON_CreateObject();   //创建根数据对象
	
	cJSON_AddItemToObject(usr,item, cJSON_CreateString(string));  //根节点下添加数字
	
	str_data = cJSON_Print(usr);   //将json形式打印成正常字符串形式(带有\r\n)
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

