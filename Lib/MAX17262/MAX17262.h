#ifndef __MAX17262_H
#define __MAX17262_H

#include "main.h"

#define MAX17262_I2C_PORT hi2c1
extern I2C_HandleTypeDef MAX17262_I2C_PORT;


#define MAX17262_DIVECE_ADDR		0x6C			//0x36  //0x6C


#define MAX17262_REPCAP					0x05				//
#define MAX17262_ATAVCAP				0xDF
#define MAX17262_REPSOC					0x06				//잔량 백분율 퍼센트
#define MAX17262_ATAVSOC				0xDE
#define MAX17262_FULLCAPREP			0x10
#define MAX17262_CURRENT				0x0A				//현재 사용중 전류

//모델셋팅관련
#define MAX17262_DESIGN_CAP			0x18 			//배터리 용량 정의
#define MAX17262_V_EMPTY				0x3A				//배터리 최소전압 및 회복전압 정의
#define MAX17262_ICHG_TERM			0x1E				//배터리 충전 종료 전류 정의
#define MAX17262_MODEL_CFG			0xDB			//모델 셋팅관련, 모델리셋

//저전력모드관련
#define MAX17262_SOFT_WAKEUP		0x60			//웨이크업
#define MAX17262_HIB_CFG					0xBA		//저전력모드 세부 셋팅(초기값 0x8400)

//배터리 상태 확인 레지스터


#define MAX17043_COMMAND				0xfe




void MAX17262_GetData(uint16_t MAX17262_target, int16_t* getDataPointer);
void MAX17262_SetData(uint16_t MAX17262_target, uint16_t MAX17262_target_data, int16_t* getDataPointer);
void MAX17262_init(void);
uint8_t MAX17262_Get_BatPercent(void);



#endif
