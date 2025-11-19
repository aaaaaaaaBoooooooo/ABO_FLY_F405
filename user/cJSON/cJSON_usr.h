#ifndef __CJSON_USR_H_
#define __CJSON_USR_H_

#include "main.h"



void json_pack_number(const char *item,double number);
void json_pack_string(const char *item,char * string);
double json_analysis_double(char *data,const char *item);
int json_analysis_int(const char *data,const char *item);

uint8_t fatfs_PID_params_read(void);
#endif
