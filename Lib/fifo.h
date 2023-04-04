#ifndef __FIFO_H
#define __FIFO_H

#include "fatfs.h"

static int16_t file_count = 0; // check the number of previous file 
static int16_t folder_count = 0;
static FRESULT res;	
static FATFS SDFatFs;
static FIL ECG_fp;
static FIL PPG_fp;
static FIL IMU_fp;
static DIR dir;
static FILINFO fno;
static char SD_msg[100];

bool sd_init(void);
bool FIFO_Init_W(void);
bool Directory_check(void);
bool folder_WR(int16_t folder_N, IWDG_HandleTypeDef hiwdg);
bool FIFO_Close(void);
void fifo_ppg(int32_t ppg);
void fifo_ecg(int32_t ecg);
void fifo_imu(int32_t gyro, int32_t acc);
void fifo_imu2(int32_t imu[6]);
bool ecg_openR(uint8_t file_number);
bool ppg_openR(uint8_t file_number);
bool imu_openR(uint8_t file_number);
void fifo_ppg2(int32_t ppg[3]);
int16_t get_folderN(void);
#endif