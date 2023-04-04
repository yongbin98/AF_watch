#include "lcdscreen.h"

extern bool sd_cnt; // sd init -> true
extern bool bat_cnt;
extern bool ble_cnt;

void screen_init(TIM_HandleTypeDef *htim9){
	ST7789_Init();
	
	sprintf(lcd_str, "Loading...");
	ST7789_WriteString(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	HAL_TIM_PWM_Start(htim9, TIM_CHANNEL_1);
	for(int i=0; i<500; i=i+10){ TIM9->CCR1 = i;	HAL_Delay(1);}
}

bool main_screen(bool left_btn, bool ok_btn, bool right_btn, int16_t acc, int16_t gyro, int16_t ppg, int16_t ecg){
	bool main_screen_flag = false;	
	
	bat_cnt = !HAL_GPIO_ReadPin(BAT_CG_POK_GPIO_Port,BAT_CG_POK_Pin);
	
	if(ble_cnt == false)
		ST7789_DrawImage(1, 1, 37, 37, (uint16_t *)IMG_37x37_BT_NC);
	else
		ST7789_DrawImage(1, 1, 37, 37, (uint16_t *)IMG_37x37_BT_CN);
	
	if(sd_cnt == false){
		if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){ 
			main_screen_flag = true;
		}
		ST7789_DrawImage(40, 1, 37, 37, (uint16_t *)IMG_37x37_SD_UM);
	}
	else{
		if((HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){			
			NVIC_SystemReset();
		}
		ST7789_DrawImage(40, 1, 37, 37, (uint16_t *)IMG_37x37_SD_MT);
	}
	
	if(bat_cnt == false)
		ST7789_DrawImage(210, 1, 24, 37, (uint16_t *)IMG_24x37_BAT_CAN);
	else
		ST7789_DrawImage(210, 1, 24, 37, (uint16_t *)IMG_24x37_BAT_CG);
	
	uint8_t battery_value=MAX17262_Get_BatPercent();
	sprintf(lcd_str, "%3d%%", battery_value);
	ST7789_WriteString(140, 10, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	
	sprintf(lcd_str, "<<");
	if(left_btn == false)
		ST7789_WriteString(5, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString(5, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "OK");
	if(ok_btn == false)
		ST7789_WriteString(100, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString(100, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, ">>");
	if(right_btn == false)
		ST7789_WriteString(200, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString(200, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
			
	sprintf(lcd_str, "ACC : %6d", acc);
	ST7789_WriteString(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "GYRO : %6d", gyro);
	ST7789_WriteString(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "PPG : %6d", ppg);
	ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "ECG : %6d", ecg);
	ST7789_WriteString(0, 90, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	return main_screen_flag;
}

void menu_screen(int8_t menu_select,bool left_btn, bool ok_btn, bool right_btn){
	sprintf(lcd_str, "<<");
	if(left_btn == false)
		ST7789_WriteString(5, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString(5, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "OK");
	if(ok_btn == false)
		ST7789_WriteString(100, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString(100, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, ">>");
	if(right_btn == false)
		ST7789_WriteString(200, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString(200, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "View PPG signal");
	ST7789_WriteString(0, 60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "View ECG signal");
	ST7789_WriteString(0, 90, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "AF test");
	ST7789_WriteString(0, 120, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "Save data");
	ST7789_WriteString(0, 150, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "Send data");
	ST7789_WriteString(0, 180, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));	
	
	sprintf(lcd_str, "<<");
	ST7789_WriteString(200, menu_select*30 +60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
}

void ppg_screen(uint16_t max86171_step, MAX86171 data[]){
	
	// 1 -> 0 2 -> 6 3 -> 4 4 -> 2
	int i = (10 - (2*data[0].tag))%8;
	uint8_t PPG_Rescale = 20;

	if(line_x == 0 && base_line_y == 0){ base_line_y = data[i+1].data;}
	for(;i < max86171_step;i+=8){
		line_y2 = 150+round((data[i+1].data - base_line_y)/PPG_Rescale);
			
		if(line_y1 > 240 || line_y2 > 240 || line_y1 < 40 || line_y2 < 40) { 
			base_line_y = data[i+1].data; 
			line_y1 = 150+round((data[i+1].data - base_line_y)/PPG_Rescale); 
			line_y2 = 150+round((data[i+1].data - base_line_y)/PPG_Rescale);  
		}
		
		ST7789_DrawLine(line_x,line_y1,line_x+1,line_y2,FLIP_RGB(BLUE));
		HAL_Delay(1);
		line_x++;
		line_y1 = line_y2;
		
		if(line_x > 250)	{
			reset_screen(LCD_PPGSCREEN);
			line_x = 0;
			base_line_y = data[i+1].data;
		}
	}
}
void reset_screen(LCD_SCREEN_TYPE screen_status){
	ST7789_Fill_Color(0, 0, 240, 240, FLIP_RGB(BLACK));
	ST7789_Fill_Color(0, 0, 240, 40, FLIP_RGB(WHITE));
	
	if(screen_status == LCD_MENUSCREEN){
		sprintf(lcd_str, "Select action");
		ST7789_WriteString(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_PPGSCREEN){
		sprintf(lcd_str, "PPG Signal");
		ST7789_WriteString(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_ECGSCREEN){
		sprintf(lcd_str, "ECG Signal");
		ST7789_WriteString(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_AFSCREEN){
		sprintf(lcd_str, "AF Detect");
		ST7789_WriteString(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_SAVESCREEN){
		sprintf(lcd_str, "SAVE Signal");
		ST7789_WriteString(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_SERVERSCREEN){
		sprintf(lcd_str, "Send to Server");
		ST7789_WriteString(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
}
void ecg_screen(uint16_t max30003_step, MAX30003 max30003_data[]){
	uint16_t ECG_Rescale = 200;
	uint16_t middle_line = 180;
	
	if(line_x == 0 && base_line_y == 0){
		int max = max30003_data[0].data;
		for(int i=0;i<max30003_step;i++)
			if(max < max30003_data[i].data)
				max = max30003_data[i].data;
		base_line_y = max;
	}
	
	for(int i=0;i<max30003_step;i++)
	{
		line_y2 = middle_line+round((max30003_data[i].data - base_line_y)/ECG_Rescale);
				
		if(line_y2 < 40){
			line_y2 = 40;
			line_y1 = 40;
		}
		
		ST7789_DrawLine(line_x,line_y1,line_x+1,line_y2,FLIP_RGB(BLUE));
		HAL_Delay(1);
		line_x++;
		line_y1 = line_y2;
		
		if(line_x > 250){
			reset_screen(LCD_ECGSCREEN);
			line_x = 0;
			base_line_y = 0;
			break;
		}
	}
	
}
bool diagnose_screen(int8_t screen_flag, uint16_t time_flag, int8_t af_select){	
	bool spk_switch = 0;
	if(sd_cnt){
		if(screen_flag == -1){			
			sprintf(lcd_str, "detect using rri");
			ST7789_WriteString(0, 60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			
			sprintf(lcd_str, "detect using fft");
			ST7789_WriteString(0, 120, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			
			sprintf(lcd_str, "Send 30s data");
			ST7789_WriteString(0, 180, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			
			sprintf(lcd_str, "<<");
			ST7789_WriteString(200, af_select*60 +60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));			
		}
		else if(screen_flag == 0) {
			dot_screen_moving("AF detecting",time_flag);
			HAL_Delay(1);
		}
		else if(screen_flag == 1){
			spk_switch = 1;
			sprintf(lcd_str, "  AF detected  ");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			
			sprintf(lcd_str, "put your finger");
			ST7789_WriteString(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			
			sprintf(lcd_str, " on the metal ");
			ST7789_WriteString(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		else if(screen_flag == 2){		
			sprintf(lcd_str, "Wait Data");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			sprintf(lcd_str, "stabilization");
			ST7789_WriteString(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		else if(screen_flag == 3){
			if((time_flag%10) == 0){
				sprintf(lcd_str, "wait 30 sec");
				ST7789_WriteString(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
				sprintf(lcd_str, "time : %d",(time_flag/10));
				ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
			}
		}		
		else if(screen_flag == 4){
			sprintf(lcd_str, "Wait bluetooth");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			sprintf(lcd_str, "connection");
			ST7789_WriteString(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		if(screen_flag == 5){
			sprintf(lcd_str, "Sending data  ");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		if(screen_flag == 6){
			sprintf(lcd_str, "Finish");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
	}
	else {
		sprintf(lcd_str, "Insert SD card");
		ST7789_WriteString(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	}
	return spk_switch;
}
void showAFdata(double AF_shannon, double AF_sample, double AF_rmssd, uint8_t AF_rri, int16_t IMU_data){
	if(AF_rri == 0 && IMU_data == 0){
		sprintf(lcd_str, "  shaE   smpE   RMSSD");
		ST7789_WriteString(0, 200, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		HAL_Delay(1);
		sprintf(lcd_str, "%  1.02f   %1.02f   %1.02f",  AF_shannon, AF_sample, AF_rmssd);
		ST7789_WriteString(0, 220, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		HAL_Delay(1);
	}
	else{
		sprintf(lcd_str, "shaE smpE RMSSD rri");
		ST7789_WriteString(0, 200, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		HAL_Delay(1);
		sprintf(lcd_str, "%1.02f %1.02f %1.02f  %02d",  AF_shannon, AF_sample, AF_rmssd,AF_rri);
		ST7789_WriteString(0, 220, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		HAL_Delay(1);

		if(IMU_data > 2000){
			sprintf(lcd_str, "ACC : %5d ",IMU_data);
			ST7789_WriteString(50, 170, lcd_str, Font_11x18, FLIP_RGB(RED), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		else{
			sprintf(lcd_str, "ACC : %5d ",IMU_data);
			ST7789_WriteString(50, 170, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
	}
}
void save_screen(bool file_init, uint16_t time_flag2, int16_t imu, int32_t ppg, int32_t ecg){
	if(sd_cnt){
		// SD out
		if((HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){ NVIC_SystemReset();}
		
		if(file_init){
			sprintf(lcd_str, "Save time : %2ds", (time_flag2/10));
			ST7789_WriteString(0, 200, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "IMU : %6d", imu );
			ST7789_WriteString(0, 160, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "PPG : %6d", ppg);
			ST7789_WriteString(0, 130, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "ECG : %6d", ecg);
			ST7789_WriteString(0, 100, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "save data");
			ST7789_WriteString(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		}
	}else {
		sprintf(lcd_str, "Insert SD card");
		ST7789_WriteString(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));		
	}
}
void server_screen(int8_t screen_flag, int8_t server_select_count){
	extern uint8_t server_dataN;
	if(sd_cnt){
		if(screen_flag == 0){			
			sprintf(lcd_str, "Send last 1 data");
			ST7789_WriteString(0, 60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "Send last N data");
			ST7789_WriteString(0, 120, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "Send whole data");
			ST7789_WriteString(0, 180, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));	
			
			sprintf(lcd_str, "<<");
			ST7789_WriteString(200, server_select_count*60 +60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		}
		else if(screen_flag == 1){
			switch(server_select_count){
				case 0 : 
					sprintf(lcd_str, "Send last 1 data");
					break;
				case 1 : 
					sprintf(lcd_str, "Send last %d data", server_dataN);
					break;
				case 2 : 
					sprintf(lcd_str, "Send whole data");
					break;
				default : 
					sprintf(lcd_str, "Error???");
					break;
			}
			ST7789_WriteString(30, 60, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		else if(screen_flag == 2){
			sprintf(lcd_str, "Connected");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			sprintf(lcd_str, "Send Data...");
			ST7789_WriteString(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		else if(screen_flag == 3){
			sprintf(lcd_str, "Finish");
			ST7789_WriteString(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
	}
	else {
		sprintf(lcd_str, "Insert SD card");
		ST7789_WriteString(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));		
	}
}

void dot_screen_moving(char dot_msg[], uint16_t time_flag){	
	strcpy(lcd_str,dot_msg);
	switch(time_flag){
		case 1 : 
			ST7789_WriteString(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
		case 50 : 
			strcat(lcd_str,".");
			ST7789_WriteString(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
		case 100 : 
			strcat(lcd_str,"..");
			ST7789_WriteString(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
		case 150 : 
			strcat(lcd_str,"...");
			ST7789_WriteString(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
	}
	
	if(time_flag >= 200){
		ST7789_Fill_Color(0, 120, 240, 30, FLIP_RGB(BLACK));
		time_flag = 1;
	}
}
void reset_line(){
	line_x = 0;
	line_y1 = 0;
	base_line_y = 0;
	max_line_y = 0;
	min_line_y = 0;
}