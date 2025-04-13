/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
uint8_t my_fatfs_init_success = 0;
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */ 
 	while(f_mount(&USERFatFS,"0:",1)!=FR_OK)	//尝试挂载文件系统
	{
		printf("Mount_Failed!");
		my_fatfs_init_success=0;
	}
	my_fatfs_init_success = 1;
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
uint8_t fatfs_read_file(TCHAR *path,char * read_buf,uint32_t data_size)
{
	FIL file;     // 文件句柄
	FRESULT res;
	UINT bytes_read;     // 读取的字节数
//	if(data_size>READ_FILE_MAX_SIZE)//读的数据太多
//	{
//		printf("SIZE OVERFLOW\n");
//		return ;
//	}
	res = f_open(&file, path, FA_READ);
	if (res == FR_OK)
	{
			printf("File opened for reading successfully.\n");

			// 读取文件内容
			if(data_size>file.obj.objsize)
				res = f_read(&file, read_buf,file.obj.objsize, &bytes_read);
			else
				res = f_read(&file, read_buf,data_size, &bytes_read);
			if (res == FR_OK)
			{
					// 关闭文件
					f_close(&file);
					printf("Data read successfully. Bytes read: %u\n", bytes_read);
					//printf("File content: %s\n", read_buf);
				return 0;
			}
			else
			{
					// 关闭文件
					f_close(&file);
					printf("Read error: %d\n", res);
					return 1;
			}


			
	}
	else
	{
			printf("Failed to open file for reading. Error: %d\n", res);
			return 1;
	}	
}

uint8_t fatfs_write_file(TCHAR *path,char * write_buf,uint32_t data_size)
{
	FIL file;     // 文件句柄
	FRESULT res;
  UINT bytes_written;  // 写入的字节数
	// 		打开文件（如果文件不存在，则创建）
	res = f_open(&file, path,FA_WRITE);
	if (res == FR_OK)
	{
			
			//printf("File opened/created successfully.\n");

			/****移动文件读写指针到文件结束处，以便添加数据***/
			f_lseek(&file,f_size(&file));
			// 写入数据到文件
			res = f_write(&file, write_buf, data_size, &bytes_written);
			if (res == FR_OK)
			{
					// 关闭文件
					f_close(&file);
					//printf("Data written successfully. Bytes written: %u\n", bytes_written);
					return 0;
			}
			else
			{
					// 关闭文件
					f_close(&file);
					//printf("Write error: %d\n", res);
					return 1;
			}


	}
	else
	{
			printf("Failed to open/create file. Error: %d\n", res);
			return 1;
	}
}

/* USER CODE END Application */
