
#include "LSM6DS/LSM6DS.h"


void LSM6DS_GetBeing(LSM6DS_AccelSetting *LSM6DS, uint8_t* getDataPointer)
{	
	uint8_t settingData[8];
	uint8_t dataToWrite[1]={0};
	
	settingData[0]=LSM6DS->accelEnabled;
	settingData[1]=LSM6DS->accelBandWidth;
	settingData[2]=LSM6DS->accelRange;
	settingData[3]=LSM6DS->accelSampleRate;
	settingData[4]=LSM6DS->gyroEnabled;
	settingData[5]=LSM6DS->gyroRange;
	settingData[6]=LSM6DS->gyroSampleRate;
	settingData[7]=LSM6DS->tempEnabled;

	
	switch (LSM6DS->accelBandWidth) {
		case 50:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
			break;
		case 100:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
			break;
		case 200:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
			break;
		default:  //set default case to max passthrough
		case 400:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
			break;
		}
	//Next, patch in full scale
		switch (LSM6DS->accelRange) {
		case 2:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_2g;
			break;
		case 4:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_4g;
			break;
		case 8:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_8g;
			break;
		default:  //set default case to 16(max)
		case 16:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_16g;
			break;
		}
		//Lastly, patch in accelerometer ODR
		switch (LSM6DS->accelSampleRate) {
		case 13:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
			break;
		case 26:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
			break;
		case 13330:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
			break;
		}
		while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_WHO_AM_I_REG, 1, getDataPointer, 1, 100)!=HAL_OK){return;}
		HAL_Delay(10);
		
		while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL1_XL, 1, dataToWrite, 1, 1000)!=HAL_OK){return;}
		HAL_Delay(10);
		//ORD
		
		//Setup the gyroscope**********************************************
		dataToWrite[0] = 0; //Start Fresh!
		switch (LSM6DS->gyroRange) {
		case 125:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
			break;
		case 245:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_245dps;
			break;
		case 500:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_500dps;
			break;
		case 1000:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
			break;
		default:  //Default to full 2000DPS range
		case 2000:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
			break;
		}
		
		switch (LSM6DS->gyroSampleRate) {
		case 13:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
			break;
		case 26:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
			break;
		case 52:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
			break;
		case 208:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
			break;
		case 416:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
			break;
		case 833:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
			break;
		case 1660:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
			break;
		}
		
		while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL2_G, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
		HAL_Delay(10);
		
	//LSM6DS
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_WHO_AM_I_REG, 1, getDataPointer, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
		
	// FIFO 50Hz, Continueous FIFO
	dataToWrite[0] = 0;
	dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz;
	dataToWrite[0] |= LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM_2;	
	while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_FIFO_CTRL5, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
	
	// gyro acc no decimation (same Hz)
	dataToWrite[0] = 0;
	dataToWrite[0] |= LSM6DS3_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION;
	dataToWrite[0] |= LSM6DS3_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION;	
	while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_FIFO_CTRL3, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
	
	// gyro acc rounding
	dataToWrite[0] = 0;
	dataToWrite[0] |= 0x60;
	while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL5_C , 1, dataToWrite, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
	
	// I2C multiple byte access, LSB MSB sampled at the same moment
	dataToWrite[0] = 0;
	dataToWrite[0] |= LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
	dataToWrite[0] |= LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE;
	while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL3_C, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
}

void LSM6DS_GetBeing_polling(LSM6DS_AccelSetting *LSM6DS, uint8_t* getDataPointer)
{
	uint8_t qdata_CFG_REG[1];
	uint8_t qdata_CTRL_REG1[1];
	uint8_t qdata_CTRL_REG4[1];
	
	uint8_t settingData[8];
	uint8_t dataToWrite[1]={0};
	
	settingData[0]=LSM6DS->accelEnabled;
	settingData[1]=LSM6DS->accelBandWidth;
	settingData[2]=LSM6DS->accelRange;
	settingData[3]=LSM6DS->accelSampleRate;
	settingData[4]=LSM6DS->gyroEnabled;
	settingData[5]=LSM6DS->gyroRange;
	settingData[6]=LSM6DS->gyroSampleRate;
	settingData[7]=LSM6DS->tempEnabled;

	
	switch (LSM6DS->accelBandWidth) {
		case 50:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
			break;
		case 100:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
			break;
		case 200:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
			break;
		default:  //set default case to max passthrough
		case 400:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
			break;
		}
	//Next, patch in full scale
		switch (LSM6DS->accelRange) {
		case 2:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_2g;
			break;
		case 4:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_4g;
			break;
		case 8:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_8g;
			break;
		default:  //set default case to 16(max)
		case 16:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_XL_16g;
			break;
		}
		//Lastly, patch in accelerometer ODR
		switch (LSM6DS->accelSampleRate) {
		case 13:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
			break;
		case 26:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
			break;
		case 13330:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
			break;
		}
		while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_WHO_AM_I_REG, 1, getDataPointer, 1, 100)!=HAL_OK){return;}
		HAL_Delay(10);
		
		while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL1_XL, 1, dataToWrite, 1, 1000)!=HAL_OK){return;}
		HAL_Delay(10);
		//ORD
		
		//Setup the gyroscope**********************************************
		dataToWrite[0] = 0; //Start Fresh!
		switch (LSM6DS->gyroRange) {
		case 125:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
			break;
		case 245:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_245dps;
			break;
		case 500:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_500dps;
			break;
		case 1000:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
			break;
		default:  //Default to full 2000DPS range
		case 2000:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
			break;
		}
		
		switch (LSM6DS->gyroSampleRate) {
		case 13:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
			break;
		case 26:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
			break;
		case 52:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
			break;
		case 208:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
			break;
		case 416:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
			break;
		case 833:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
			break;
		case 1660:
			dataToWrite[0] |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
			break;
		}
		
		while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL2_G, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
		HAL_Delay(10);
		
	//LSM6DS
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_WHO_AM_I_REG, 1, getDataPointer, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
	

}
void LSM6DS_Reset(void){
	uint8_t dataToWrite[1];
	dataToWrite[0] = LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE;
	while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL3_C, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
	HAL_Delay(10);
}

void LSM6DS_Reboot(void){
	uint8_t dataToWrite[1];
	dataToWrite[0] = LSM6DS3_ACC_GYRO_BOOT_REBOOT_MODE;
	while(HAL_I2C_Mem_Write(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_CTRL3_C, 1, dataToWrite, 1, 100)!=HAL_OK){return;}
	HAL_Delay(15);
}

void LSM6DS_GetAccelDataX(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[2];	
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_OUTX_L_XL, 1, myBuffer, 2, 10)!=HAL_OK){return;}
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	*getDataPointer = output;
}

void LSM6DS_GetAccelDataY(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[2];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_OUTY_L_XL, 1, myBuffer, 2, 10)!=HAL_OK){return;}
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	*getDataPointer = output;
}

void LSM6DS_GetAccelDataZ(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[2];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_OUTZ_L_XL, 1, myBuffer, 2, 10)!=HAL_OK){return;}
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	*getDataPointer = output;
}

void LSM6DS_GetGyroDataX(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[2];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_OUTX_L_G, 1, myBuffer, 2, 10)!=HAL_OK){return;}
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	*getDataPointer = output;
}

void LSM6DS_GetGyroDataY(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[2];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_OUTY_L_G, 1, myBuffer, 2, 10)!=HAL_OK){return;}
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	*getDataPointer = output;
}

void LSM6DS_GetGyroDataZ(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[2];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_OUTZ_L_G, 1, myBuffer, 2, 10)!=HAL_OK){return;}
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	*getDataPointer = output;
}

void LSM6DS_FIFO_DATA_NUM(int16_t* getDataPointer)	//
{
	uint8_t myBuffer[1];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_FIFO_STATUS1 , 1, myBuffer, sizeof(myBuffer), 10)!=HAL_OK){return;}
	getDataPointer[0] = (int16_t)myBuffer[0];
	while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_FIFO_STATUS2 , 1, myBuffer, sizeof(myBuffer), 10)!=HAL_OK){return;}
	getDataPointer[0] |= (int16_t)myBuffer[0]<<8;
}

void LSM6DS_FIFO_DATA_OUT(int16_t getDataPointer[6], int16_t count)	//
{	
	uint8_t myBuffer[2];
	for(int i=0;i<count;i++){
		while(HAL_I2C_Mem_Read(&LSM6DS_I2C_PORT, LSM6DS_DIVECE_ADDR, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L, 1, myBuffer, sizeof(myBuffer), 10)!=HAL_OK){return;}
		getDataPointer[i] = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	}
}


