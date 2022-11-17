
#include "MAX86171.h"

uint8_t spi_end_flag;
uint16_t count;

void MAX86171_writeReg(uint8_t address, uint8_t data)
{
	HAL_Delay(100);
	uint8_t TxBuffer[3];

	TxBuffer[0] = address;
	TxBuffer[1] = WRITE_EN;
	TxBuffer[2] = data;
	
	MAX86171_Select();
	while(HAL_SPI_GetState(&MAX86171_SPI_PORT) != HAL_SPI_STATE_READY){}
	spi_end_flag = 1;
	if(HAL_SPI_Transmit_DMA(&MAX86171_SPI_PORT, (uint8_t*)TxBuffer, 3) != HAL_OK){Error_Handler();}
	while(HAL_SPI_GetState(&MAX86171_SPI_PORT) != HAL_SPI_STATE_READY){}
	MAX86171_Deselect();
	spi_end_flag = 0;
	//while(spi_end_flag){}
	
}


uint8_t MAX86171_readReg(uint8_t address)
{
	uint8_t TxBuffer[3];
	uint8_t RxBuffer[3];
	TxBuffer[0] = address;
	TxBuffer[1] = READ_EN;
	TxBuffer[2] = 0x00;
	
	MAX86171_Select();
	while(HAL_SPI_GetState(&MAX86171_SPI_PORT) != HAL_SPI_STATE_READY){}
	spi_end_flag = 1;
	HAL_SPI_TransmitReceive_DMA(&MAX86171_SPI_PORT, (uint8_t*)TxBuffer,(uint8_t*)RxBuffer, 3);
	while(HAL_SPI_GetState(&MAX86171_SPI_PORT) != HAL_SPI_STATE_READY){ }
	MAX86171_Deselect();
	spi_end_flag = 0;
	//while(spi_end_flag){}
	
	
	return RxBuffer[2];
	
}

uint16_t MAX86171_read_fifo(MAX86171 max86171[])
{
	//fifo flush and tag
	//23~20 tag = 1~9 MEASn
	//0xA = dark data captured, 0xB ALC overflow, 0xC Exposure overflow, 0xD picket fence detected, 0xE empty FIFO
	//19~0 adc , sequence = MEAS1 channel 1,2 -> MEAS2 channel 1,2 ......
	
	count = MAX86171_readReg(PPG_FIFO_DATA_COUNT);
	if(MAX86171_readReg(PPG_OVF_COUNTER)>>7)
		count = 256;
	
	uint8_t data_buffer[256*3+2]; //first 2byte resister address and 3byte data maximum 256
	data_buffer[0] = PPG_FIFO_DATA;
	data_buffer[1] = READ_EN;
	
	if(count !=0)
	{
		MAX86171_Select();
		while(HAL_SPI_GetState(&MAX86171_SPI_PORT) != HAL_SPI_STATE_READY){	}
		HAL_SPI_TransmitReceive_DMA(&MAX86171_SPI_PORT, (uint8_t*)data_buffer,(uint8_t*)data_buffer, count*3+2);
		while(HAL_SPI_GetState(&MAX86171_SPI_PORT) != HAL_SPI_STATE_READY){	}
		MAX86171_Deselect();
	}
	count = MAX86171_decode_fifo(count, data_buffer, max86171);
	return count;
}

uint16_t MAX86171_decode_fifo(uint16_t count, uint8_t data_buffer[], MAX86171 max86171[])
{
	for(int i=0; i<count; i++) // run
	{
		max86171[i].tag = (data_buffer[i*3+2] >> 4 ) & 0x0f;
		max86171[i].led = ((data_buffer[i*3+2] << 16 ) | (data_buffer[i*3+3] << 8 ) | (data_buffer[i*3+4])) & 0x0fffff;
				
		switch(max86171[i].tag){
			case PPG_DARK_DATA: //Dark data captured 
			case PPG_ALC_OVF_DATA: // ALC overflow
			case PPG_EXP_OVF_DATA: // Exposure Overflow
			case PPG_PF_DATA: //Picket Fence detected
				max86171[i].led = 0;
				break;
			case PPG_INVALID_DATA: // Empty FIFO				
				count = i;
				break;
		}
	}
	return count;
}

MAX86171_RETURN MAX86171_init()
{
	uint8_t result = 0;
	
	// PPG init
	MAX86171_init_setup();
	
	// Check Status
	result += MAX86171_read_status_1();
	HAL_Delay(100);
	result += MAX86171_read_status_2();
	HAL_Delay(100);
	result += MAX86171_read_status_3();
	HAL_Delay(100);
	if(MAX86171_readReg(PPG_PART_ID) != 0){ result++; }
	
	if(result == 4)
		return MAX86171_SUCCESS;
	else
		return MAX86171_FAILURE;
}

void MAX86171_init_setup()
{
	//FIFO Interrupt
	MAX86171_writeReg(PPG_FIFO_CONFIG_1, 0x80); // empty 128 bits interrupt
	MAX86171_writeReg(PPG_FIFO_CONFIG_2, 0x1E); // Full Interrupt, FIFO Interrupt, Flush FIFO, FIFO roll over
	
	//System
	MAX86171_writeReg(PPG_SYS_CONFIG_1, 0x00); //Internal SYNC, PD1 PD2 En, No Power-save mod
	MAX86171_writeReg(PPG_SYS_CONFIG_2, 0x0F); //MEAS1 2 3 4 EN
	MAX86171_writeReg(PPG_SYS_CONFIG_3, 0x00); //Ambient light cancelation EN, Raw X Computed Data, MEASn_Config = MEAS1_CONFIG
	
	//Pin Interrupt
	MAX86171_writeReg(PPG_PD_BIAS, 0x55); // PD1 2 3 4 0pF to 125pF (POR default)
	MAX86171_writeReg(PPG_PIN_CONFIG_1, 0x00); // INT2 disable TRIG active low
	MAX86171_writeReg(PPG_PIN_CONFIG_2, 0x00); // INT1 2 Open drain, active low
	
	//Clock
	MAX86171_writeReg(PPG_FR_CLK_FREQ, 0x20); // Internal Clock 32768Hz, No clock fine tune
	MAX86171_writeReg(PPG_FR_CLK_MSB, 0x01); // FR_CLK_DIV_MSB (256) Clock Divided 32768Hz / 256 = 128fps
	MAX86171_writeReg(PPG_FR_CLK_LSB, 0x00); // FR_CLK_DIV_LSB (0) , If, status_2 INVALID_CFG == 1 -> More Divide
	
	//PPG Measure 1 PPG 1 2 Green LED
	MAX86171_writeReg(PPG_MEAS1_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
	MAX86171_writeReg(PPG_MEAS1_CONFIG_1, 0x18); // channel 1,2 = PD1,PD2 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
	MAX86171_writeReg(PPG_MEAS1_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
	MAX86171_writeReg(PPG_MEAS1_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
	MAX86171_writeReg(PPG_MEAS1_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
	MAX86171_writeReg(PPG_MEAS1_DRVB, 0x0A); // LED current 5mA
	MAX86171_writeReg(PPG_MEAS1_DRVC, 0x0A); // LED current 5mA
	
	//PPG Measure 2 PPG 3 4 Green LED
	MAX86171_writeReg(PPG_MEAS2_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
	MAX86171_writeReg(PPG_MEAS2_CONFIG_1, 0x78); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
	MAX86171_writeReg(PPG_MEAS2_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
	MAX86171_writeReg(PPG_MEAS2_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
	MAX86171_writeReg(PPG_MEAS2_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
	MAX86171_writeReg(PPG_MEAS2_DRVB, 0x0A); // LED current 5mA
	MAX86171_writeReg(PPG_MEAS2_DRVC, 0x0A); // LED current 5mA
	
	//PPG Measure 3 PPG 1 2 Ambient measure
	MAX86171_writeReg(PPG_MEAS3_SEL, 0x40); // LED DRV C B A -> LED 9 3 4 EN, Ambient Measurement
	MAX86171_writeReg(PPG_MEAS3_CONFIG_1, 0x18); // channel 1,2 = PD1,PD2 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
	MAX86171_writeReg(PPG_MEAS3_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
	MAX86171_writeReg(PPG_MEAS3_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
	MAX86171_writeReg(PPG_MEAS3_DRVA, 0x00); // LED current 5mA, interval = 0.5mA
	MAX86171_writeReg(PPG_MEAS3_DRVB, 0x00); // LED current 5mA
	MAX86171_writeReg(PPG_MEAS3_DRVC, 0x00); // LED current 5mA
	
	//PPG Measure 3 PPG 3 4 Ambient measure
	MAX86171_writeReg(PPG_MEAS4_SEL, 0x40); // LED DRV C B A -> LED 9 3 4 EN, Ambient Measurement
	MAX86171_writeReg(PPG_MEAS4_CONFIG_1, 0x78); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
	MAX86171_writeReg(PPG_MEAS4_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
	MAX86171_writeReg(PPG_MEAS4_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
	MAX86171_writeReg(PPG_MEAS4_DRVA, 0x00); // LED current 5mA, interval = 0.5mA
	MAX86171_writeReg(PPG_MEAS4_DRVB, 0x00); // LED current 5mA
	MAX86171_writeReg(PPG_MEAS4_DRVC, 0x00); // LED current 5mA
	
//PPG Measure 5~9
//	MAX86171_writeReg(PPG_MEAS5_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
//	MAX86171_writeReg(PPG_MEAS5_CONFIG_1, 0x18); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
//	MAX86171_writeReg(PPG_MEAS5_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
//	MAX86171_writeReg(PPG_MEAS5_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
//	MAX86171_writeReg(PPG_MEAS5_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
//	MAX86171_writeReg(PPG_MEAS5_DRVB, 0x0A); // LED current 5mA
//	MAX86171_writeReg(PPG_MEAS5_DRVC, 0x0A); // LED current 5mA
//	
//	MAX86171_writeReg(PPG_MEAS6_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
//	MAX86171_writeReg(PPG_MEAS6_CONFIG_1, 0x18); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
//	MAX86171_writeReg(PPG_MEAS6_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
//	MAX86171_writeReg(PPG_MEAS6_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
//	MAX86171_writeReg(PPG_MEAS6_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
//	MAX86171_writeReg(PPG_MEAS6_DRVB, 0x0A); // LED current 5mA
//	MAX86171_writeReg(PPG_MEAS6_DRVC, 0x0A); // LED current 5mA
//	
//	MAX86171_writeReg(PPG_MEAS7_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
//	MAX86171_writeReg(PPG_MEAS7_CONFIG_1, 0x18); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
//	MAX86171_writeReg(PPG_MEAS7_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
//	MAX86171_writeReg(PPG_MEAS7_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
//	MAX86171_writeReg(PPG_MEAS7_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
//	MAX86171_writeReg(PPG_MEAS7_DRVB, 0x0A); // LED current 5mA
//	MAX86171_writeReg(PPG_MEAS7_DRVC, 0x0A); // LED current 5mA
//	
//	MAX86171_writeReg(PPG_MEAS8_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
//	MAX86171_writeReg(PPG_MEAS8_CONFIG_1, 0x18); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
//	MAX86171_writeReg(PPG_MEAS8_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
//	MAX86171_writeReg(PPG_MEAS8_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
//	MAX86171_writeReg(PPG_MEAS8_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
//	MAX86171_writeReg(PPG_MEAS8_DRVB, 0x0A); // LED current 5mA
//	MAX86171_writeReg(PPG_MEAS8_DRVC, 0x0A); // LED current 5mA
//	
//	MAX86171_writeReg(PPG_MEAS9_SEL, 0x36); // LED DRV C B A -> LED 9 3 4 EN
//	MAX86171_writeReg(PPG_MEAS9_CONFIG_1, 0x18); // channel 1,2 = PD3,PD4 , Integration Time = 117.1us, Exposure Led pulses = 1 (improve SNR ambient light cancelation??)
//	MAX86171_writeReg(PPG_MEAS9_CONFIG_2, 0x3A); // no SINC3 filter (improve ADC?), LED Range 128mA, ADC Range 16uA(?)
//	MAX86171_writeReg(PPG_MEAS9_CONFIG_3, 0x50); // PD Setling 12.1us, LED Setling 12us, offset DAC 0
//	MAX86171_writeReg(PPG_MEAS9_DRVA, 0x0A); // LED current 5mA, interval = 0.5mA
//	MAX86171_writeReg(PPG_MEAS9_DRVB, 0x0A); // LED current 5mA
//	MAX86171_writeReg(PPG_MEAS9_DRVC, 0x0A); // LED current 5mA
	
	//Threshold Interrupts -> not used
	MAX86171_writeReg(PPG_TH_SEL, 0x00); // 0x21 -> Threshold 1 2 = MEAS1 2 EN
	MAX86171_writeReg(PPG_TH_HYST, 0x00); // 0x94 -> Channel 1 2 = Threshold mea 1 2, 4 sample 16 HYSTERESIS
	MAX86171_writeReg(PPG_TH_HI_1, 0xff); // used Upper Threshold 1 MAX
	MAX86171_writeReg(PPG_TH_LO_1, 0x00); // Not used Lower Threshold 1
	MAX86171_writeReg(PPG_TH_HI_2, 0xff); // used Upper Threshold 2 MAX
	MAX86171_writeReg(PPG_TH_LO_2, 0x00); // Not used Lower Threshold 2
	
	//Picket Fence
	MAX86171_writeReg(PPG_PF_SEL,0x00); // 0x21 -> Picket fence detect in MEAS1, MEAS2
	MAX86171_writeReg(PPG_PF_CONFIG,0x40); // Enumeration 4 points, Config default
	
	//Interrupt Enable -> not used 
}


void MAX86171_adjust_clock(uint8_t clock_normal ,uint32_t rate){
	
	uint32_t clock_rate;
	uint8_t protocol = 0x00;
	
	protocol |= (clock_normal << 5);
	MAX86171_writeReg(PPG_FR_CLK_FREQ, protocol); // Internal Clock, No clock fine tune
	protocol = 0x00;
	printf("%d",protocol);
	
	// Clock / MSB+LSB = sps
	if(clock_normal == 1){		clock_rate = 32768/rate;	}
	else{		clock_rate = 32700/rate;	}
	printf("%d",clock_rate);
	
	protocol |= (uint8_t) (clock_rate/256); // FR_CLK_DIV_MSB
	MAX86171_writeReg(PPG_FR_CLK_MSB, protocol); 
	protocol = 0x00;
	printf("%d",protocol);
	
	protocol |= (uint8_t) (clock_rate%256);
	MAX86171_writeReg(PPG_FR_CLK_LSB, protocol); // FR_CLK_DIV_MSB (256) Clock Divided 32768Hz / 256 = 128fps
	protocol = 0x00;
	printf("%d",protocol);
}
	
MAX86171_RETURN MAX86171_read_status_1()
{
	uint8_t result = 0;
	uint8_t INIT_STATUS_1 = MAX86171_readReg(PPG_STAT_1);
	if(INIT_STATUS_1 & MAX86171_A_FULL) { 	} // FIFO Full error (too many data waiting(256)) 
	if(INIT_STATUS_1 & MAX86171_FRAME_RDY) {  } // Frame Ready
	if(INIT_STATUS_1 & MAX86171_FIFO_DATA_RDY){ } // new data in FIFO
	if(INIT_STATUS_1 & MAX86171_ALC_OVF){ result++; } // Overflow dark current
	if(INIT_STATUS_1 & MAX86171_EXP_OVF){ result++; } // Overflow exposure current
	if(INIT_STATUS_1 & MAX86171_THRESH2_HILO){ } // Threshold 2 active
	if(INIT_STATUS_1 & MAX86171_THRESH1_HILO){ } // Threshold 1 active
	if(INIT_STATUS_1 & MAX86171_PWR_RDY){ /* result++; */ } // VDD out of range
	
	if(result > 0)
		return MAX86171_FAILURE;
	else
		return MAX86171_SUCCESS;
	
}
MAX86171_RETURN MAX86171_read_status_2()
{
	uint8_t result = 0;
	uint8_t INIT_STATUS_2 = MAX86171_readReg(PPG_STAT_2);
	if(INIT_STATUS_2 & MAX86171_LED9_COMPB) { 
	} // LED9 voltage drop
	if(INIT_STATUS_2 & MAX86171_VDD_OOR) { 
		result++; 
	} // VDD_DIG out of range
	if(INIT_STATUS_2 & MAX86171_INVALID_CFG) { 
		result++; 
	} // Configureation timing is not valid
	
	if(result > 0)
		return MAX86171_FAILURE;
	else
		return MAX86171_SUCCESS;
}
MAX86171_RETURN MAX86171_read_status_3()
{
	uint8_t result = 0;
 	uint8_t INIT_STATUS_3 = MAX86171_readReg(PPG_STAT_3);
	if(INIT_STATUS_3 & MAX86171_LED8_COMPB) { 	} // LED8 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED7_COMPB) { 	} // LED7 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED6_COMPB) { 	} // LED6 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED5_COMPB) { 	} // LED5 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED4_COMPB) { /*result++;*/	} // LED4 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED3_COMPB) { /*result++;*/	} // LED3 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED2_COMPB) { 	} // LED2 voltage drop
	if(INIT_STATUS_3 & MAX86171_LED1_COMPB) { 	} // LED1 voltage drop
	
	if(result > 0)
		return MAX86171_FAILURE;
	else
		return MAX86171_SUCCESS;
}

