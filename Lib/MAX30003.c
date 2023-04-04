
#include "max30003.h"


void max30003_RegWrite (uint8_t WRITE_ADDRESS, uint32_t data)
{
	
    uint8_t dataToSend = (WRITE_ADDRESS<<1) | WREG;
		uint8_t TxBuffer[4];
		TxBuffer[0] = dataToSend;
		TxBuffer[1] = data>>16;
		TxBuffer[2] = data>>8;
		TxBuffer[3] = data;
	
    MAX30003_Select();
	
		while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){return;}
		if(HAL_SPI_Transmit(&MAX30003_SPI_PORT, (uint8_t*)TxBuffer, 4,10) != HAL_OK){Error_Handler();}
    while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){return;}

		MAX30003_Deselect();
		
}


void max30003_SwReset(void)
{
    max30003_RegWrite(SW_RST,0x000000);
    HAL_Delay(100);
}


void max30003_RegRead(uint8_t Reg_address, uint8_t* RxBuffer)
{
	uint8_t dataToSend;
	uint8_t TxBuffer[4];
	
	dataToSend = (Reg_address<<1 ) | RREG;
	TxBuffer[0] = dataToSend;

	MAX30003_Select();
	while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){}
	if(HAL_SPI_TransmitReceive(&MAX30003_SPI_PORT, (uint8_t*)TxBuffer,(uint8_t*)RxBuffer, 4,10) != HAL_OK){Error_Handler();}
	while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){}

	MAX30003_Deselect();
	
}

void max30003_Synch(void)
{
    max30003_RegWrite(SYNCH,0x000000);
}


MAX30003_RETURN MAX30003_init()
{
	uint8_t RxBuffer[4];
	
	//init setting
	max30003_SwReset();
	
	max30003_RegWrite(EN_INT, 0xF00F00);  
	HAL_Delay(100);
	max30003_RegWrite(MNGR_INT, 0x800004); // FIFO interrupt 16bits
	HAL_Delay(100);
	max30003_RegWrite(CNFG_GEN, 0x081013); // ECG Enable, R(bias) 50MF
	HAL_Delay(100);
	max30003_RegWrite(CNFG_CAL, 0x601000); // Calibration enable, 1 Hz
	HAL_Delay(100);
	max30003_RegWrite(CNFG_EMUX,0x0B0000); // ECG inverted, ECGP N connect to VCAL P N, 
	HAL_Delay(100);
	max30003_RegWrite(CNFG_ECG, 0x834000); // 128sps, 20v/v, 0.5hz HPF
	HAL_Delay(100);

	max30003_RegWrite(CNFG_RTOR1,0x3FA300); // R to R EN, 96ms Auto scaling, 
	max30003_Synch();
	HAL_Delay(100);
	
	//check init	
	max30003_RegRead(STATUS,RxBuffer);
	
	return MAX30003_SUCCESS;
}

int32_t max30003_getEcgSamples(void)
{
    uint8_t regReadBuff[4];
		int32_t ecgdata;
	
    max30003_RegRead(ECG_FIFO, regReadBuff);

		ecgdata = (int16_t) (regReadBuff[1]<<10) | (int16_t) (regReadBuff[2]<<2) | (int16_t) (regReadBuff[3]>>6);
	
    return ecgdata;
}
int32_t max30003_getEcgSamples_burst(MAX30003 *max30003, uint8_t max30003_step)
{
	uint8_t TxBuffer[12];
	uint8_t RxBuffer[12];
	int sample;
	uint32_t word;
	uint16_t step = 0x0000;
	bool tag_err = false;
	
	TxBuffer[0] = (ECG_FIFO_BURST<<1) | RREG;
	
	MAX30003_Select();
	while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){}
		
	if(HAL_SPI_TransmitReceive(&MAX30003_SPI_PORT, (uint8_t*)TxBuffer,(uint8_t*)RxBuffer, 1,10) != HAL_OK){Error_Handler();}
	TxBuffer[0] = 0x00;
	
	do 
	{	
			/* get and process samples */
		if (!(step % 4)) {
					if(HAL_SPI_TransmitReceive(&MAX30003_SPI_PORT, (uint8_t*)TxBuffer,(uint8_t*)RxBuffer, 12,10) != HAL_OK){Error_Handler();}  
		}
		
		sample = (step % 4)*3;
		
		word = RxBuffer[0+sample] & 0x80 ? 0xFF000000 : 0x00000000;
		word |= ((uint32_t)(RxBuffer[0+sample]) << 16 );
		word |= ((uint32_t)(RxBuffer[1+sample]) << 8 );
		word |= ((uint32_t)(RxBuffer[2+sample]) );		
		max30003[step].tag = (word & 0x38)>>3;
		
		switch (max30003[step].tag) 
		{
			case ETAG_VALID_EOF:
			case ETAG_FAST_EOF:
				tag_err = true; /* exit, but save the sample as a valid sample */
			case ETAG_VALID:
			case ETAG_FAST:             
				/* get next sample */
				max30003[step].step = step;
				max30003[step].data = (int32_t)(word & 0xFFFFFFC0);
				step++;
				break;
			case ETAG_FIFO_OVERFLOW:
				MAX30003_Deselect();
				while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){}
				max30003_Synch(); /* or FIFO RST */
			case ETAG_FIFO_EMPTY:
				tag_err = true;
				step = 0;
				break;
			default : 
				MAX30003_Deselect();
				while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){}
				max30003_Synch();
				break; /* TODO error handling */
		}
	} while (!tag_err && step < max30003_step);
	
	MAX30003_Deselect();	
	while(HAL_SPI_GetState(&MAX30003_SPI_PORT) != HAL_SPI_STATE_READY){}
	return step;
}
