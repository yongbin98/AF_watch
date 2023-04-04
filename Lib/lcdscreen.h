#ifndef __SCREEN_H
#define __SCREEN_H

#include "st7789/st7789.h"
#include "MAX17262/MAX17262.h"

//FLAG
static char lcd_str[40]; // lcd message

//PPG ECG layout
static uint8_t line_x = 0; // now x_line loc
static int32_t line_y1 = 0, line_y2=0; // previous y_line , next y_line
static int32_t base_line_y=0, max_line_y=0, min_line_y=0; // y_line threshold (mean max min)

void screen_init(TIM_HandleTypeDef *htim9);
bool main_screen(bool left_btn, bool ok_btn, bool right_btn, int16_t acc, int16_t gyro, int16_t ppg, int16_t ecg);
void menu_screen(int8_t menu_select,bool left_btn, bool ok_btn, bool right_btn);
void reset_screen(LCD_SCREEN_TYPE screen_status);
void ppg_screen(uint16_t max86171_step, MAX86171 data[256]);
void ecg_screen(uint16_t max30003_step, MAX30003 max30003_data[25]);
bool diagnose_screen(int8_t screen_flag, uint16_t time_flag, int8_t af_select);
void showAFdata(double AF_shannon, double AF_sample, double AF_rmssd, uint8_t AF_rri, int16_t IMU_data);
void dot_screen_moving(char dot_msg[], uint16_t time_flag);
void save_screen(bool file_init, uint16_t time_flag2, int16_t imu, int32_t ppg, int32_t ecg);
void server_screen(int8_t screen_flag, int8_t server_select_count);
void reset_line(void);

#endif