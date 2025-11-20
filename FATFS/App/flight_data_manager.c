#include "main.h"
#include "delay.h"
#include "fatfs.h"
#include "stdio.h"
#include "string.h"
#include "angle.h"
#include "usart.h"
#include "bmp390_task.h"
#include "control.h"
#include "flight_data_manager.h"
#define LOG_DIR "/FLY_LOG"//日志文件存放目录
#define LOG_PATH "/FLY_LOG/"//日志文件存放路径 
#define LOG_FILE_PREFIX "LOG_"//日志文件前缀
#define LOG_FILE_EXT ".TXT"//日志文件后缀(大小写敏感！！！)
#define MAX_LOG_FILES 100	//最大保留100条日志
#define MAX_TOTAL_SIZE (12 * 1024 * 1024) // 12MB
#define RESERVE_SPACE (512 * 1024) // 保留512KB给新日志
#define DATA_BUFFER_SIZE 256 // 每条日志数据缓冲区大小

char DATA_FILE_BUFFER[DATA_BUFFER_SIZE];// "time:data1,data2,data3,data4,data5,data6\r\n"
uint32_t log_record_circle = 20;//日志记录周期，单位ms
static void log_record_format_data(uint32_t time)
{
    // 格式化数据到缓冲区
    snprintf(DATA_FILE_BUFFER, DATA_BUFFER_SIZE, "%u:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%.2f,%.2f\r\n",
            time,
            my_ahrs.Angle_Data.roll,
            my_ahrs.Angle_Data.pitch,
            my_ahrs.Angle_Data.yaw,
            my_ahrs.IMU_Data.gyro.x,
            my_ahrs.IMU_Data.gyro.y,
            my_ahrs.IMU_Data.gyro.z,
            my_aircraft.Altitude,
            TOF.distance_m,
            my_aircraft.Temperature,
            my_aircraft.Battery_Volt*0.1f,
            my_aircraft.Throttle,
            AttitudeControl.roll_target_angle,
            AttitudeControl.pitch_target_angle,
            AttitudeControl.yaw_target_angle,
            HeightControl.target_height
                );

} 

/* 日志文件信息结构体 */
typedef struct {
    uint32_t index;
    uint32_t size;
    char filename[32];
} log_file_info_t;
log_file_info_t log_files[MAX_LOG_FILES];
uint32_t log_total_size = 0; // 当前日志文件总大小
uint32_t log_total_number = 0; // 当前日志文件总数量
/* 文件状态标志 */
static FIL current_log_file;

/* 获取当前最大的日志文件序号 */
uint32_t get_max_log_index()
{
    uint32_t max_index = 0;
    for(uint32_t i = 0; i < log_total_number; i++) {
        if (log_files[i].index > max_index) {
            max_index = log_files[i].index;
        }
    }
    return max_index;
}

/* 获取日志文件列表和总大小 */
void get_log_files_info(log_file_info_t *files, uint32_t *total_size)
{
    DIR dir;
    FILINFO fno;
    uint32_t file_count = 0;
    char *ext;
    FRESULT res;
    
    *total_size = 0;
    
    res = f_opendir(&dir, LOG_DIR);
    if (res == FR_OK) {
        while (1) {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            
            // 检查是否为日志文件
            if (strstr(fno.fname, LOG_FILE_PREFIX) != NULL && 
                (ext = strstr(fno.fname, LOG_FILE_EXT)) != NULL) {
                // 提取序号
                char *underscore = strstr(fno.fname, "_");
                if (underscore != NULL && underscore < ext) {
                    uint32_t index = (uint32_t)atoi(underscore + 1);
                    
                    // 使用f_stat获取文件信息
                    FILINFO file_info;
                    char file_path[64];
                    strcpy(file_path,LOG_PATH);
                    strcat(file_path,fno.fname);
                    res = f_stat(file_path, &file_info);
                    if (res == FR_OK) {
                        files[file_count].index = index;
                        files[file_count].size = file_info.fsize;
                        strncpy(files[file_count].filename, fno.fname, sizeof(files[file_count].filename) - 1);
                        files[file_count].filename[sizeof(files[file_count].filename) - 1] = '\0';
                        
                        *total_size += files[file_count].size;
                        file_count++;
                    }
                }
            }
        }
        f_closedir(&dir);
    }
    else
		{

		}
    log_total_number = file_count;
}

/* 按序号排序的比较函数 */
int compare_log_files(const void *a, const void *b)
{
    const log_file_info_t *file_a = (const log_file_info_t *)a;
    const log_file_info_t *file_b = (const log_file_info_t *)b;
    
    if (file_a->index < file_b->index) return -1;
    if (file_a->index > file_b->index) return 1;
    return 0;
}

/* 删除最旧的日志文件直到满足限制 */
void cleanup_old_logs()
{
    uint32_t i;
    char file_path[64];

    // 获取所有日志文件信息
    get_log_files_info(log_files ,&log_total_size);
    printf("Current log files: %u, total size: %u bytes\n", log_total_number, log_total_size);
    // 如果文件数量或总大小超过限制，需要删除最旧的文件
    if (log_total_number > 0 && (log_total_number >= MAX_LOG_FILES || log_total_size >= (MAX_TOTAL_SIZE - RESERVE_SPACE))) {
        // 按序号排序（最旧的在前）
        qsort(log_files, log_total_number, sizeof(log_file_info_t), compare_log_files);
        
        // 从最旧的文件开始删除，直到满足限制
        i = 0;
        while ( log_total_number >= MAX_LOG_FILES || log_total_size >= (MAX_TOTAL_SIZE - RESERVE_SPACE) ) {
            strcpy(file_path,LOG_PATH);
            strcat(file_path,log_files[i].filename);
            f_unlink(file_path);// 删除文件
            log_files[i].index = 0;// 标记为已删除
            printf("Deleted log file: %s, size: %u bytes\n", log_files[i].filename, log_files[i].size);
            log_total_size -= log_files[i].size;
			log_total_number--;
            i++;
		   
        }
        printf("After cleanup, Current log files: %u, total size: %u bytes\n",log_total_number, log_total_size);
    }
    
}

/* 创建新的日志文件 */
uint32_t create_new_log_file(char *filename, uint32_t filename_size)
{
    uint32_t new_index;
    FRESULT res;
    
    // 先清理旧文件，确保有足够空间
    cleanup_old_logs();
    
    // 获取当前最大序号并加1
    new_index = get_max_log_index() + 1;
    printf("Creating new log file with index: %u\n", new_index);
    // 生成文件名
    snprintf(filename, filename_size, "%s%s%u%s",LOG_PATH,LOG_FILE_PREFIX, new_index, LOG_FILE_EXT);
    
    // 创建文件并写入文件头
    res = f_open(&current_log_file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res == FR_OK) {
        // 写入文件头信息
        char header[64];
        snprintf(header, sizeof(header), "---- Flight Log %u -aBo ----\r\n", new_index);
        UINT bytes_written;
        f_write(&current_log_file, header, strlen(header), &bytes_written);
        f_sync(&current_log_file);
    }
    
    return new_index;

}


/* 飞行数据存储函数 */
void aircraft_fly_data_record()
{
    uint32_t time_current, time_start;// 记录当前时间和起始时间
    static uint8_t aircraft_last_status = 0;// 0: 未飞行，1: 飞行中
    static char current_filename[32] = {0};// 当前日志文件名
    static uint8_t stop_writing = 0; // 标志是否因空间不足停止写入
    FRESULT res;
    UINT bytes_written;
    DIR dir;
    if(f_opendir(&dir,LOG_DIR) != FR_OK)//目录不存在则创建
    {
        printf("Log directory not found, creating: %s\n", LOG_DIR);
        if(f_mkdir(LOG_DIR) != FR_OK)
        {
            printf("Failed to create log directory: %s\n", LOG_DIR);
            return;
        }
        else
        {
            printf("Log directory created: %s\n", LOG_DIR);
        }
        
    }
    else
    {
        f_closedir(&dir);
    }
    while(1)
    {
        if(aircraft_last_status == 0x00 && (my_aircraft.status & 0x01) && !(my_aircraft.status & 0x08))// 飞机开始飞行同时未处于保护状态
        {
            // 飞机开始飞行
            aircraft_last_status = 0x01;
            stop_writing = 0; // 重置停止写入标志
            
            create_new_log_file(current_filename, sizeof(current_filename));           
            printf("Created new log file: %s\n", current_filename);
            time_start = HAL_GetTick();
        }
        else if(aircraft_last_status == 0x01 && ((!(my_aircraft.status & 0x01)) || (my_aircraft.status & 0x08)))// 飞机停止飞行或进入保护状态
        {
            // 飞机停止飞行
            aircraft_last_status = 0x00;
            stop_writing = 1; // 停止写入  
            char finish_msg[32];
            snprintf(finish_msg, sizeof(finish_msg),
                    "---- Flight Log End:0x%x ----\r\n", 
                    my_aircraft.status);            
            f_write(&current_log_file,finish_msg, sizeof(finish_msg), &bytes_written);
            f_sync(&current_log_file);
            f_close(&current_log_file);
            printf("Saved new log file: %s, size: %u \n", current_filename, f_size(&current_log_file));
        }
        
        if(my_fatfs_init_success && aircraft_last_status && !stop_writing)
        {
            // 准备数据
            time_current = HAL_GetTick() - time_start;

            log_record_format_data(time_current);
            uint32_t data_length = strlen(DATA_FILE_BUFFER);
            // 如果当前文件大小加上已保存的日志大小超过限制，停止写入
            if (f_size(&current_log_file) +log_total_size > MAX_TOTAL_SIZE) {
                // 停止写入，关闭文件
                stop_writing = 1;
                f_sync(&current_log_file);
                f_close(&current_log_file);
                printf("Log storage limit reached, stopped writing to file: %s\n", current_filename);
                continue;
            }
            
            // 写入数据
            res = f_write(&current_log_file, DATA_FILE_BUFFER, data_length, &bytes_written);
            if (res == FR_OK) {
            } else {
                // 写入失败，关闭文件，设置停止写入标志
                f_sync(&current_log_file);
                f_close(&current_log_file);
                stop_writing = 1;
            }
            delay_ms(log_record_circle);
        }
    }
}
