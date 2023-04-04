#include "fifo.h"

extern int fputc(int ch, FILE *f);

bool sd_init()
{
	if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin)))
	{
		HAL_Delay(1000);
		
		//SD카드 초기화
		res = BSP_SD_Init();
		if(res != FR_OK)
				return false;
		
		//SD 마운트
		res = f_mount(&SDFatFs,"", 0);
		if(res != FR_OK)
				return false;
		
		return true;
	}
	return false;
}

bool FIFO_Init_W(){	
	//f_open 파일 열기	
	if(file_count == 0)
		Directory_check();
	else
		file_count++;
	
	sprintf(SD_msg,"/data/ECG_%d.csv",file_count);
	res = f_open(&ECG_fp, SD_msg, FA_OPEN_ALWAYS | FA_WRITE );
	if(res != FR_OK)
		return true;
	
	memset(SD_msg,0,sizeof(SD_msg));
	
	sprintf(SD_msg,"/data/PPG_%d.csv",file_count);
	res = f_open(&PPG_fp, SD_msg, FA_OPEN_ALWAYS | FA_WRITE);
	if(res != FR_OK)
		return true;
	
	memset(SD_msg,0,sizeof(SD_msg));
	
	sprintf(SD_msg,"/data/IMU_%d.csv",file_count);
	res = f_open(&IMU_fp, SD_msg, FA_OPEN_ALWAYS | FA_WRITE);
	if(res != FR_OK)
		return true;
	
	memset(SD_msg,0,sizeof(SD_msg));

	return false;
}

bool Directory_check() {
	uint8_t data_flag = 0;
	
	// data folder exist?
	res = f_mkdir("/data");
	if(res != FR_OK && res != FR_EXIST){
		return true;
	}
	
	res = f_opendir(&dir, "/data");
	if(res != FR_OK){
		if(f_mkdir("/data") != FR_OK){
			return true;
		}
	}
							
	memset(&fno, 0, sizeof(FILINFO));
	res = f_readdir(&dir, &fno);
	if(res != FR_OK){
		return true;
	}	
	if(fno.fattrib== AM_DIR || fno.fattrib== AM_ARC){ data_flag = 2; }
	else if(fno.fattrib == 0x00){	data_flag = 1; }
	else {
		return true;
	}
	f_closedir(&dir);
	
	// find old directory
	f_opendir(&dir,"/");
	while(true){
		memset(&fno, 0, sizeof(FILINFO));
		f_findnext(&dir,&fno);
		if(fno.fname[0] == NULL){
			do{
				folder_count++;
				
				memset(SD_msg,0,sizeof(SD_msg));
				sprintf(SD_msg,"/OLD%d",folder_count);
				
				res = f_rename("/data",SD_msg);
			} while(res == FR_EXIST);
			
			if(res != FR_OK){
				return true;	
			}
			
			if(data_flag == 2){
				res = f_mkdir("data");
				if(res != FR_OK){
					return true;
				}
			}
			else if(data_flag == 1) {
				folder_count--;
				res = f_rename(SD_msg,"/data");
				if(res != FR_OK){
					return true;
				}
			}
			break;
		}
	}
	f_closedir(&dir);
	file_count++;
		
	/* read file name
	int count_name=-1;
	for(int i=strlen(fno.fname)-1;i>=0;i--)	{
		if(fno.fname[i] == '_')
			count_name = -1;
		
		if(count_name != -1){
			file_count = (fno.fname[i] - '0')*pow(
	10,count_name);
			count_name++;
		}
		
		if(fno.fname[i] == '.')
			count_name = 0;		
	}
	*/
	
	return false;
}
bool ecg_openR(uint8_t file_number){
	sprintf(SD_msg,"/OLD%d/ECG_%d.csv",folder_count,file_number);
	res = f_open(&ECG_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
	if(res != FR_OK){
		return false;
	}
	memset(SD_msg,0,sizeof(SD_msg));
	return true;
}
bool ppg_openR(uint8_t file_number){
	sprintf(SD_msg,"/OLD%d/PPG_%d.csv",folder_count,file_number);
	res = f_open(&PPG_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
	if(res != FR_OK){
		return false;
	}
	memset(SD_msg,0,sizeof(SD_msg));	
	return true;
}
bool imu_openR(uint8_t file_number){
	sprintf(SD_msg,"/OLD%d/IMU_%d.csv",folder_count,file_number);
	res = f_open(&IMU_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
	if(res != FR_OK){
		return false;
	}
	memset(SD_msg,0,sizeof(SD_msg));
	return true;
}


bool FIFO_Close(){
	res = f_close(&ECG_fp);
	if(res != FR_OK){  }
	res = f_close(&PPG_fp);
	if(res != FR_OK){ }
	res = f_close(&IMU_fp);
	if(res != FR_OK){ }
	
	return false;
}

void fifo_ppg(int32_t ppg){
	sprintf(SD_msg,"%d\n",ppg);
	f_puts(SD_msg,&PPG_fp);
	memset(SD_msg,0,sizeof(SD_msg));
}
void fifo_ecg(int32_t ecg){
	sprintf(SD_msg,"%d\n",ecg); //18bits int -> uint = 131072
	f_puts(SD_msg,&ECG_fp);
	memset(SD_msg,0,sizeof(SD_msg));
}
void fifo_imu(int32_t gyro, int32_t acc){
	sprintf(SD_msg,"%d,%d\n",gyro, acc);
	f_puts(SD_msg,&IMU_fp);
	memset(SD_msg,0,sizeof(SD_msg));
}
void fifo_imu2(int32_t imu[6]){
	sprintf(SD_msg,"%d,%d,%d,%d,%d,%d\n",imu[0],imu[1],imu[2],imu[3],imu[4],imu[5]);
	f_puts(SD_msg,&IMU_fp);
	memset(SD_msg,0,sizeof(SD_msg));
}

bool folder_WR(int16_t folder_N, IWDG_HandleTypeDef hiwdg){
	char read_str[10];	
	
	if(folder_count < folder_N || folder_N < 1)
		return true;

	while(true){
		sprintf(read_str,"/OLD%d",folder_count - folder_N + 1);
		res = f_opendir(&dir,read_str);
		if(res == FR_NO_FILE)
			folder_count--;
		else if(res != FR_OK){
			return true;
		}
		else
			break;
	}
	
	memset(&fno, 0, sizeof(FILINFO));
	res = f_readdir(&dir, &fno);
	if(res != FR_OK){
		return true;
	}
	
	if(fno.fattrib== AM_DIR || fno.fattrib== AM_ARC)// file or directory o
	{
		int16_t fileN = 0;
		while(true){
			char *file_tmp = strtok(fno.fname+1,".");
			char *file_ptr = strchr(file_tmp,'_')+1;
			
			// find the number of files
			int16_t tmp_number = 0;
			for(int16_t ii = strlen(file_ptr)-1;ii>=0;ii--){
				if(file_ptr[ii] - '0' > 9)
					return true;
				tmp_number += (file_ptr[ii] - '0')*pow(10,strlen(file_ptr)-ii-1);
			}
			if(tmp_number > fileN)
				fileN = tmp_number;
			
			if(fno.fname[0] == NULL){
				TCHAR read_buf[100];
				unsigned int read_bytes=0;
				int fileOpened = 0;
				
				for(int16_t ii = 1;ii<=fileN;ii++){
					if(ecg_openR(ii)){
						f_read(&ECG_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
						if(read_bytes != 0)
							fileOpened++;
					}
					if(ppg_openR(ii)){
						f_read(&PPG_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
						if(read_bytes != 0)
							fileOpened++;
					}
					if(imu_openR(ii)){
						f_read(&IMU_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
						if(read_bytes != 0)
							fileOpened++;
					}
					if(FIFO_Close())
						return true;					
				}
				printf("L%04d",fileOpened);
				
				// read & send whole files
				for(int16_t ii = 1;ii<=fileN;ii++){
					
					if(ecg_openR(ii))
					{
						f_read(&ECG_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
						if(read_bytes != 0){
							printf("E");
							HAL_IWDG_Refresh(&hiwdg);
							read_buf[read_bytes] = NULL;
							printf("%s",read_buf);
							while(read_bytes) {
								f_read(&ECG_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
								if(read_bytes)
									HAL_IWDG_Refresh(&hiwdg);
								read_buf[read_bytes] = NULL;
								printf("%s",read_buf);
							}
							printf("N");
						}
					}
					
					if(ppg_openR(ii))
					{
						f_read(&PPG_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
						if(read_bytes != 0){
							printf("P");
							HAL_IWDG_Refresh(&hiwdg);
							read_buf[read_bytes] = NULL;
							printf("%s",read_buf);
							while(read_bytes) {
								f_read(&PPG_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
								if(read_bytes)
									HAL_IWDG_Refresh(&hiwdg);
								read_buf[read_bytes] = NULL;
								printf("%s",read_buf);
							}
							printf("N");
						}
					}
					
					if(imu_openR(ii))
					{
						f_read(&IMU_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
						if(read_bytes != 0){
							printf("I");
							HAL_IWDG_Refresh(&hiwdg);
							read_buf[read_bytes] = NULL;
							printf("%s",read_buf);
							while(read_bytes) {
								f_read(&IMU_fp,read_buf,sizeof(read_buf)-1,&read_bytes);
								if(read_bytes)
									HAL_IWDG_Refresh(&hiwdg);
								read_buf[read_bytes] = NULL;
								printf("%s",read_buf);
							}
							printf("N");
						}
					}
					
					if(FIFO_Close()){
						return true;
					}
				}
				break;
			}
			else
				f_findnext(&dir,&fno);
		}		
	}
	else {
		return true;
	}
	return false;	
}
int16_t get_folderN(){
	return folder_count;
}

void fifo_ppg2(int32_t ppg[3]){
	sprintf(SD_msg,"%d,%d,%d\n",ppg[0],ppg[1],ppg[2]);
	f_puts(SD_msg,&PPG_fp);
	memset(SD_msg,0,sizeof(SD_msg));
}