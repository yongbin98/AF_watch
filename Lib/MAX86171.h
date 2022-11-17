#ifndef __MAX86171_H
#define __MAX86171_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

//USER SETTING START***********************************************************************
//============================================================================================================================
#define MAX86171_SPI_PORT  			hspi2
extern SPI_HandleTypeDef 				MAX86171_SPI_PORT;
#define MAX86171_CS_GPIO_Port		PPG_CS_GPIO_Port
#define MAX86171_CS_Pin				 	PPG_CS_Pin

//#define FIFO_SAMPLES						128
//============================================================================================================================
//USER SETTING END***********************************************************************

#define MAX86171_Select()    HAL_GPIO_WritePin(MAX86171_CS_GPIO_Port, MAX86171_CS_Pin, GPIO_PIN_RESET)
#define MAX86171_Deselect()  HAL_GPIO_WritePin(MAX86171_CS_GPIO_Port, MAX86171_CS_Pin, GPIO_PIN_SET)

//MAX86171 Registry addresses

#define READ_EN 0x80
#define WRITE_EN 0x00

// Status
#define PPG_STAT_1         (0x00)      //PPG Status 1
#define PPG_STAT_2         (0x01)      //PPG Status 2
#define PPG_STAT_3         (0x02)      //PPG Status 3

// FIFO
#define PPG_FIFO_WR_PTR        (0x04)      //FIFO Buffer Write Pointer
#define PPG_FIFO_RD_PTR        (0x05)      //FIFO Buffer Read Pointer
#define PPG_OVF_COUNTER        (0x06)      //Over Flow Counter
#define PPG_FIFO_DATA_COUNT    (0x07)      //FIFO Data Counter
#define PPG_FIFO_DATA          (0x08)      //FIFO Data Register
#define PPG_FIFO_CONFIG_1      (0x09)      //FIFO Configuration 1
#define PPG_FIFO_CONFIG_2      (0x0A)      //FIFO Configuration 2

//System Control
#define PPG_SYS_CONFIG_1       (0x0C)      //System Control 1
#define PPG_SYS_CONFIG_2       (0x0D)      //System Control 2
#define PPG_SYS_CONFIG_3       (0x0E)      //System Control 3
#define PPG_PD_BIAS       		 (0x0F)      //Photodiode Bias
#define PPG_PIN_CONFIG_1       (0x10)      //Pin Functional Configuration
#define PPG_PIN_CONFIG_2       (0x11)      //Output Pin Configuration

//Frame Rate Clock
#define PPG_FR_CLK_FREQ      	 (0x15)      //FR Clock Frequency Select
#define PPG_FR_CLK_MSB	   		 (0x16)      //FR Clock Divider MSB
#define PPG_FR_CLK_LSB  	 		 (0x17)      //FR Clock Divider LSB

//MEAS1 Setup
#define PPG_MEAS1_SEL          (0x18)      //MEAS1 Selects
#define PPG_MEAS1_CONFIG_1     (0x19)      //MEAS1 Configuration 1
#define PPG_MEAS1_CONFIG_2     (0x1A)      //MEAS1 Configuration 2
#define PPG_MEAS1_CONFIG_3     (0x1B)      //MEAS1 Configuration 3
#define PPG_MEAS1_DRVA     		 (0x1C)      //MEAS1 DRVA Current
#define PPG_MEAS1_DRVB     		 (0x1D)      //MEAS1 DRVB Current
#define PPG_MEAS1_DRVC    	 	 (0x1E)      //MEAS1 DRVC Current

//MEAS2 Setup
#define PPG_MEAS2_SEL          (0x20)      //MEAS2 Selects
#define PPG_MEAS2_CONFIG_1     (0x21)      //MEAS2 Configuration 1
#define PPG_MEAS2_CONFIG_2     (0x22)      //MEAS2 Configuration 2
#define PPG_MEAS2_CONFIG_3     (0x23)      //MEAS2 Configuration 3
#define PPG_MEAS2_DRVA     		 (0x24)      //MEAS2 DRVA Current
#define PPG_MEAS2_DRVB     		 (0x25)      //MEAS2 DRVB Current
#define PPG_MEAS2_DRVC    	 	 (0x26)      //MEAS2 DRVC Current

//MEAS3 Setup
#define PPG_MEAS3_SEL          (0x28)      //MEAS3 Selects
#define PPG_MEAS3_CONFIG_1     (0x29)      //MEAS3 Configuration 1
#define PPG_MEAS3_CONFIG_2     (0x2A)      //MEAS3 Configuration 2
#define PPG_MEAS3_CONFIG_3     (0x2B)      //MEAS3 Configuration 3
#define PPG_MEAS3_DRVA     		 (0x2C)      //MEAS3 DRVA Current
#define PPG_MEAS3_DRVB     		 (0x2D)      //MEAS3 DRVB Current
#define PPG_MEAS3_DRVC    	 	 (0x2E)      //MEAS3 DRVC Current

//MEAS4 Setup
#define PPG_MEAS4_SEL          (0x30)      //MEAS4 Selects
#define PPG_MEAS4_CONFIG_1     (0x31)      //MEAS4 Configuration 1
#define PPG_MEAS4_CONFIG_2     (0x32)      //MEAS4 Configuration 2
#define PPG_MEAS4_CONFIG_3     (0x33)      //MEAS4 Configuration 3
#define PPG_MEAS4_DRVA     		 (0x34)      //MEAS4 DRVA Current
#define PPG_MEAS4_DRVB     		 (0x35)      //MEAS4 DRVB Current
#define PPG_MEAS4_DRVC    	 	 (0x36)      //MEAS4 DRVC Current

//MEAS5 Setup
#define PPG_MEAS5_SEL          (0x38)      //MEAS5 Selects
#define PPG_MEAS5_CONFIG_1     (0x39)      //MEAS5 Configuration 1
#define PPG_MEAS5_CONFIG_2     (0x3A)      //MEAS5 Configuration 2
#define PPG_MEAS5_CONFIG_3     (0x3B)      //MEAS5 Configuration 3
#define PPG_MEAS5_DRVA     		 (0x3C)      //MEAS5 DRVA Current
#define PPG_MEAS5_DRVB     		 (0x3D)      //MEAS5 DRVB Current
#define PPG_MEAS5_DRVC    	 	 (0x3E)      //MEAS5 DRVC Current

//MEAS6 Setup
#define PPG_MEAS6_SEL          (0x40)      //MEAS6 Selects
#define PPG_MEAS6_CONFIG_1     (0x41)      //MEAS6 Configuration 1
#define PPG_MEAS6_CONFIG_2     (0x42)      //MEAS6 Configuration 2
#define PPG_MEAS6_CONFIG_3     (0x43)      //MEAS6 Configuration 3
#define PPG_MEAS6_DRVA     		 (0x44)      //MEAS6 DRVA Current
#define PPG_MEAS6_DRVB     		 (0x45)      //MEAS6 DRVB Current
#define PPG_MEAS6_DRVC    	 	 (0x46)      //MEAS6 DRVC Current

//MEAS7 Setup
#define PPG_MEAS7_SEL          (0x48)      //MEAS7 Selects
#define PPG_MEAS7_CONFIG_1     (0x49)      //MEAS7 Configuration 1
#define PPG_MEAS7_CONFIG_2     (0x4A)      //MEAS7 Configuration 2
#define PPG_MEAS7_CONFIG_3     (0x4B)      //MEAS7 Configuration 3
#define PPG_MEAS7_DRVA     		 (0x4C)      //MEAS7 DRVA Current
#define PPG_MEAS7_DRVB     		 (0x4D)      //MEAS7 DRVB Current
#define PPG_MEAS7_DRVC    	 	 (0x4E)      //MEAS7 DRVC Current

//MEAS8 Setup
#define PPG_MEAS8_SEL          (0x50)      //MEAS8 Selects
#define PPG_MEAS8_CONFIG_1     (0x51)      //MEAS8 Configuration 1
#define PPG_MEAS8_CONFIG_2     (0x52)      //MEAS8 Configuration 2
#define PPG_MEAS8_CONFIG_3     (0x53)      //MEAS8 Configuration 3
#define PPG_MEAS8_DRVA     		 (0x54)      //MEAS8 DRVA Current
#define PPG_MEAS8_DRVB     		 (0x55)      //MEAS8 DRVB Current
#define PPG_MEAS8_DRVC    	 	 (0x56)      //MEAS8 DRVC Current

//MEAS9 Setup
#define PPG_MEAS9_SEL          (0x58)      //MEAS9 Selects
#define PPG_MEAS9_CONFIG_1     (0x59)      //MEAS9 Configuration 1
#define PPG_MEAS9_CONFIG_2     (0x5A)      //MEAS9 Configuration 2
#define PPG_MEAS9_CONFIG_3     (0x5B)      //MEAS9 Configuration 3
#define PPG_MEAS9_DRVA     		 (0x5C)      //MEAS9 DRVA Current
#define PPG_MEAS9_DRVB     		 (0x5D)      //MEAS9 DRVB Current
#define PPG_MEAS9_DRVC    	 	 (0x5E)      //MEAS9 DRVC Current

//Threshold Interrupts
#define PPG_TH_SEL          	 (0x68)      //Threshold MEAS Select
#define PPG_TH_HYST          	 (0x69)      //Threshold HYST
#define PPG_TH_HI_1          	 (0x6A)      //PPG High Threshold 1
#define PPG_TH_LO_1          	 (0x6B)      //PPG Low Threshold 1
#define PPG_TH_HI_2          	 (0x6C)      //PPG High Threshold 2
#define PPG_TH_LO_2            (0x6D)      //PPG Low Threshold 2

//Picket Fence
#define PPG_PF_SEL             (0x70)      //Picket Fence Measurement Select
#define PPG_PF_CONFIG          (0x71)      //Picket Fence Configuration

//Interuppt Enables
#define PPG_INT1_EN_1          (0x78)      //Interrupt1 Enable 1
#define PPG_INT1_EN_2          (0x79)      //Interrupt1 Enable 2
#define PPG_INT1_EN_3          (0x7A)      //Interrupt1 Enable 3
#define PPG_INT2_EN_1          (0x7C)      //Interrupt2 Enable 1
#define PPG_INT2_EN_2          (0x7D)      //Interrupt2 Enable 2
#define PPG_INT2_EN_3          (0x7E)      //Interrupt2 Enable 3

//Part ID
#define PPG_PART_ID            (0xFF)      //PPG Part ID

//other
#define MAX_MEAS_CHANNEL			6

typedef enum
{
	MAX86171_A_FULL				 = 0x80,
	MAX86171_FRAME_RDY		 = 0x40,
	MAX86171_FIFO_DATA_RDY = 0x20,
	MAX86171_ALC_OVF			 = 0x10,
	MAX86171_EXP_OVF			 = 0x08,
	MAX86171_THRESH2_HILO	 = 0x04,
	MAX86171_THRESH1_HILO	 = 0x02,
	MAX86171_PWR_RDY			 = 0x01
}MAX86171_STATUS_1;

typedef enum
{
	MAX86171_LED9_COMPB	 = 0x80,
	MAX86171_VDD_OOR		 = 0x40,
	MAX86171_INVALID_CFG = 0x04
}MAX86171_STATUS_2;

typedef enum
{
	MAX86171_LED8_COMPB	 = 0x80,
	MAX86171_LED7_COMPB	 = 0x40,
	MAX86171_LED6_COMPB	 = 0x20,
	MAX86171_LED5_COMPB	 = 0x10,
	MAX86171_LED4_COMPB	 = 0x08,
	MAX86171_LED3_COMPB	 = 0x04,
	MAX86171_LED2_COMPB	 = 0x02,
	MAX86171_LED1_COMPB	 = 0x01
}MAX86171_STATUS_3;

typedef enum
{
	PPG_DARK_DATA	 			= 0xA,
	PPG_ALC_OVF_DATA		= 0xB,
	PPG_EXP_OVF_DATA		= 0xC,
	PPG_PF_DATA	 				= 0xD,
	PPG_INVALID_DATA	 	= 0xE
}MAX86171_TAG;

typedef enum
{
	MAX86171_SUCCESS	 = 1,
	MAX86171_FAILURE	 = 0
}MAX86171_RETURN;

typedef struct __attribute__((__packed__)) {
	uint32_t tag : 4;
	int32_t led : 20;
}MAX86171;

void MAX86171_writeReg(uint8_t address, uint8_t data);
uint8_t MAX86171_readReg(uint8_t address);
uint16_t MAX86171_read_fifo(MAX86171 *max86171);
uint16_t MAX86171_decode_fifo(uint16_t count, uint8_t data_buffer[], MAX86171 *max86171);
MAX86171_RETURN MAX86171_init(void);
void MAX86171_init_setup(void);
void MAX86171_adjust_clock(uint8_t clock_normal ,uint32_t rate);
MAX86171_RETURN MAX86171_read_status_1(void);
MAX86171_RETURN MAX86171_read_status_2(void);
MAX86171_RETURN MAX86171_read_status_3(void);

#endif
