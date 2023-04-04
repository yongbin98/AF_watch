#ifndef __MAX30003_H
#define __MAX30003_H

#include "main.h"

//USER SETTING START***********************************************************************
//============================================================================================================================
#define MAX30003_SPI_PORT  			hspi4
extern SPI_HandleTypeDef 				MAX30003_SPI_PORT;
#define MAX30003_CS_GPIO_Port		ECG_CS_GPIO_Port
#define MAX30003_CS_Pin				 	ECG_CS_Pin

//============================================================================================================================
//USER SETTING END***********************************************************************

#define MAX30003_Select()    HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_RESET)
#define MAX30003_Deselect()  HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_SET)

#define WREG 0x00
#define RREG 0x01

#define   NO_OP           0x00
#define   STATUS          0x01
#define   EN_INT          0x02
#define   EN_INT2         0x03
#define   MNGR_INT        0x04
#define   MNGR_DYN        0x05
#define   SW_RST          0x08
#define   SYNCH           0x09
#define   FIFO_RST        0x0A
#define   INFO            0x0F
#define   CNFG_GEN        0x10
#define   CNFG_CAL        0x12
#define   CNFG_EMUX       0x14
#define   CNFG_ECG        0x15
#define   CNFG_RTOR1      0x1D
#define   CNFG_RTOR2      0x1E
#define   ECG_FIFO_BURST  0x20
#define   ECG_FIFO        0x21
#define   RTOR            0x25
#define   NO_OP_2         0x7F

#define RTOR_INTR_MASK     0x04

typedef enum
{
	MAX30003_SUCCESS	 = 1,
	MAX30003_FAILURE	 = 0
}MAX30003_RETURN;

typedef enum {
	ETAG_VALID			= 0,
	ETAG_FAST			= 1,
	ETAG_VALID_EOF		= 2,
	ETAG_FAST_EOF		= 3,
	_ETAG_RESERVED1		= 4,
	_ETAG_RESERVED2		= 5,
	ETAG_FIFO_EMPTY		= 6,
	ETAG_FIFO_OVERFLOW	= 7
} ECGFIFO_ETAG_VAL;


void max30003_RegWrite (uint8_t WRITE_ADDRESS, uint32_t data);
void max30003_SwReset(void);
void max30003_RegRead(uint8_t Reg_address, uint8_t* RxBuffer);
void max30003_Synch(void);
void max30003_ReadData(uint8_t * RxBuffer);
MAX30003_RETURN MAX30003_init(void);
int32_t max30003_getEcgSamples(void);
int32_t max30003_getEcgSamples_burst(MAX30003 *max30003, uint8_t max30003_step);


#endif

