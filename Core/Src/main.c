/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"

#include "constants.h"
#include "fir_constants.h"
#include "lcdscreen.h"
#include "fifo.h"
#include "LSM6DS/LSM6DS.h"
#include "AF_detect.h"
#include "MAX30003.h"
#include "MAX86171.h"

//#include "weights.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;
DMA_HandleTypeDef hdma_spi5_tx;
DMA_HandleTypeDef hdma_spi5_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_tx;

/* USER CODE BEGIN PV */


//extern
bool sd_cnt = false;
bool bat_cnt = false; // bat init -> true
bool ble_cnt = false;
int16_t server_dataN = 1;

//MAX30003
uint8_t max30003_RxData[4]; // ECG temporary RxData
MAX30003 max30003_data[25]; // ECG data
uint8_t max30003_step = 0; // the number of received data

//MAX86171
MAX86171 max86171_data[256]; // PPG data
int32_t max86171_buffer[6]; // PPG buffer (0, 1, 2 : LED data / 3, 4, 5 : Ambient light)
uint16_t max86171_step = 0; // the number of received data
uint8_t max86171_stat; // PPG status (buffer > 128 interrupt)

//BLUETOOTH
char Rx_indx=0; //1바이트씩 수신된 데이터의 배열위치를 변경하여 문자열을 받기위한 변수
char Rx_data[2]; // received data
char Rx_Buffer[25]; // received data buffer
char Rx_read_data[25]; // previous received data buffer (when finished)
uint8_t BT_rx_flag = 0; // ble received

//SWITCH
uint8_t sw_flag=0;  // 0 : btn pressed, 1 : btn pressing, 2 : btn released
uint8_t sw_L_flag=0; //the number of times the left button was pressed in time
uint8_t sw_R_flag=0; //the number of times the right button was pressed in time
uint8_t sw_C_flag=0; //the number of times the center button was pressed in time

//IMU

LSM6DS_AccelSetting LSM6DSM_IMUSetting={
	1,					//accelEnabled
	50,				//accelBandWidth
	8,					//accelRange
	52,			//accelSampleRate
	1,					//gyroEnabled
	500,			//gyroRange
	52,			//gyroSampleRate
	1,					//tempEnabled
};

//uint8_t I2C_TxData[2]={0x0F};
uint8_t I2C_RxData[3] = {0,}; // I2c_RxData (use only init)
//int16_t I2C_BurstData[240]; // buffer[40] -> acc x, acc y, acc z, gyro x, gyro y, gyro z 
//int16_t I2C_Stat[1]; // I2c status
int16_t I2C_Polling_data[1];
int16_t I2C_Polling_buffer[6] = {0};

//FLAG
uint8_t spk_switch = 0; // OFF : 0, ON : 1, OFF ~ing : 2, ON ~ing : 3
uint16_t time_flag2 = 0; // 0.1s time check
uint16_t error_code = 0; // error code
char error_msg[30]; // error message
uint8_t AF_rri = 0; // the number of rri
uint8_t PPI_idx = 0; // the number of bpm(fft)

//layout
bool file_init = false; // file init -> true
bool left_btn = false; // left btn interrupt -> true
bool right_btn = false; // right btn interrupt -> true
bool ok_btn = false; // ok btn interrupt -> true
bool btn_flag = false; // btn interrupt -> true
uint16_t pull_btn_time = 0; // how long pull btn ( passing one second -> reset)
uint16_t btn_pressing = 0; // how long btn pressing
LCD_SCREEN_TYPE screen_status = LCD_MAINSCREEN; // now screen
int8_t menu_select = 0; // menu screen 5 btn
int8_t server_select = 0; // server menu 3 btn
int8_t screen_flag = 0;
int8_t af_select = 0;

//filter
arm_fir_instance_f32 S; // fir filter init
arm_rfft_fast_instance_f32 S2; 
float32_t *firStateF32; 
float32_t *newPpgDataFN; // circular queue
int16_t *PPI; // save recent 16 rri data
int16_t *Peak_Interval; // received rri
float32_t *fft_inputDataF;
float32_t *fft_outputDataF;
uint16_t Filter_Idx = 0; // check data number

//fft
int32_t bufferACC[500] = {0};
int32_t bufferGYRO[500] = {0};
int32_t fifoACC[250][3] = {0};
int32_t fifoGYRO[250][3] = {0};
uint16_t iterator_IMU = 0;
uint16_t iteratorFifoIMU = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM9_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

void HPF(int32_t *Input, float SamplingFrequency, float CutOffFrequency);
void Data_Update(void);
void UF_speaker_ON(void);
void UF_speaker_OFF(void);
void DNN_Run(void);
void AF_Algorithm(void);
void AF_Layer_rescale(float *AF_Rescale);
void save_algorithm(void);
void server_algorithm(void);
void SwitchFunction(void);
void Bluetooth_init(void);
void Set_StandbyMod(void);
void btn_flag_reset(void);
void ECG_PPG_IMU_reset(void);
void ECG_Update(void);
void PPG_Update(void);
void Data_Send(void);
void data_fifo(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//printf
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  uint8_t temp[1]={ch};
	while(HAL_UART_Transmit(&huart7, (uint8_t *)&temp, 1, 1000) != HAL_OK);
  return ch;
}

//Bluetooth Timer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART7) //current UART	AMP보드와의 연결
	{
		Rx_Buffer[Rx_indx++]=Rx_data[0];	//수신 받은 데이터를 Rx_Buffer에 입력
		HAL_UART_Receive_IT(&huart7, (uint8_t*)Rx_data, 1); //Rx_data에 1바이트의 데이터를 수신받음 Rx_data[0]으로 들어감
		if((Rx_data[0]==0x00)||(Rx_data[0]==0x0A))
		{
			BT_rx_flag=1;
			memset(Rx_read_data, 0, sizeof(Rx_read_data));
			memcpy(Rx_read_data, Rx_Buffer, sizeof(char)*Rx_indx);
			Rx_indx=0;
		}
	}
}

/*==================== GPIO EXTI 인터럽트 서비스 루틴======================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_5)
	{
		left_btn = true;
	}
	
	else if (GPIO_Pin == GPIO_PIN_4)
	{
		ok_btn = true;
	}
	
	else if (GPIO_Pin == GPIO_PIN_7)
	{
		right_btn = true;
	}
	btn_flag = true;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	MX_IWDG_Init();
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == SET){
		Set_StandbyMod();
	}
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_TIM9_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_TIM3_Init();
  MX_UART7_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	/* VPPEN */
	HAL_GPIO_WritePin(VPP_EN_GPIO_Port,VPP_EN_Pin,GPIO_PIN_SET);
	if(!HAL_GPIO_ReadPin(BAT_CG_POK_GPIO_Port,BAT_CG_POK_Pin))
		Set_StandbyMod();
  HAL_Delay(100);
	
	/* watch dog */
	__HAL_DBGMCU_FREEZE_IWDG();
	
	/* LCD init */
	screen_init(&htim9);
	
	/* Battery Monitoring */
	MAX17262_init();
	bat_cnt = !HAL_GPIO_ReadPin(BAT_CG_POK_GPIO_Port,BAT_CG_POK_Pin);
			
	/* BLUETOOTH EN */	
	HAL_UART_Receive_IT(&huart7, (uint8_t*)Rx_data, 1); // UART7수신 시작	
	Bluetooth_init();
		
	/* ECG PPG Init */
	max30003_RegRead(INFO,max30003_RxData);
	if(MAX30003_init() == 0) { 
		error_code=100;
		Error_Handler(); 
	}
	HAL_IWDG_Refresh(&hiwdg);
	
	if(MAX86171_init() == 0) { 
		error_code=101;
		Error_Handler(); 
	}
	
	/* SD Init */
	HAL_IWDG_Refresh(&hiwdg);
	sd_cnt = sd_init();
	
	/* LCD Screen */	
	reset_screen(screen_status);
	
	/*interrupt (TIMER, BLUETOOTH) */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);	
		
	/* IMU init */
	LSM6DS_Reset();
	LSM6DS_GetBeing_polling(&LSM6DSM_IMUSetting ,I2C_RxData);	//자이로 가속도 센서 활성화
		
	/* start ECG PPG */
	max30003_Synch();
	MAX86171_writeReg(PPG_FIFO_CONFIG_2, 0x1E);
	time_flag2++;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		HAL_IWDG_Refresh(&hiwdg);
				
		// get data
		Data_Update();
		
		//spk
		if(spk_switch == 1) { UF_speaker_ON(); spk_switch += 2;}
		else if(spk_switch == 0) { UF_speaker_OFF(); spk_switch += 2;}
		
				
		//LCD update
		switch (screen_status){
			case LCD_MAINSCREEN: 
				if(time_flag2%2 == 0){
					int16_t acc = bufferACC[iterator_IMU-1];
					int16_t gyro = bufferGYRO[iterator_IMU-1];
					int32_t ppg = max86171_data[0].data;
					int32_t ecg = max30003_data[0].data;
					if(main_screen(left_btn, ok_btn, right_btn, acc, gyro, ppg, ecg)){
						sd_cnt = sd_init();			
					}
				}
				break;
			case LCD_MENUSCREEN: 
				menu_screen(menu_select, left_btn, ok_btn, right_btn);
				break;
			case LCD_PPGSCREEN:
				if(max86171_step != 0){
					ppg_screen(max86171_step, max86171_data);
					max86171_step = 0;
				}
				break;
			case LCD_ECGSCREEN:
				if(max30003_step != 0){
					ecg_screen(max30003_step, max30003_data);
					max30003_step = 0;					
				}
				break;
			case LCD_AFSCREEN:
				AF_Algorithm();
				if(diagnose_screen(screen_flag, time_flag2, af_select))
					spk_switch = 1;
				break;
			case LCD_SAVESCREEN:
				if((time_flag2%10) == 0){					
					int32_t ppg = max86171_data[0].data;
					int32_t ecg = max30003_data[0].data;					
					save_screen(file_init, time_flag2, bufferGYRO[iterator_IMU-1], ppg, ecg);
				}
				save_algorithm();
				
				break;
			case LCD_SERVERSCREEN:
				server_algorithm();
				server_screen(screen_flag, server_select);
				break;
		}
		
		
		//Switch func
		if(btn_flag == true || sw_flag > 0) {	
			SwitchFunction(); 
			if(spk_switch == 3)
				spk_switch = 0;
		}
						
		// fifo data
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 5;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_1LINE;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 250;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 500-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 24;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ECG_CS_Pin|PPG_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLE_MOD_Pin|VPP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ECG_CS_Pin PPG_CS_Pin */
  GPIO_InitStruct.Pin = ECG_CS_Pin|PPG_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_CG_POK_Pin */
  GPIO_InitStruct.Pin = BAT_CG_POK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BAT_CG_POK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_MOD_Pin VPP_EN_Pin */
  GPIO_InitStruct.Pin = BLE_MOD_Pin|VPP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RES_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SW_Pin */
  GPIO_InitStruct.Pin = SD_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void Bluetooth_init(void)
{
	//AT-Mode
	HAL_GPIO_WritePin(BLE_MOD_GPIO_Port, BLE_MOD_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	
	//Operation check
	printf("AT");
	HAL_Delay(100);
	while(BT_rx_flag==0){return;}
	BT_rx_flag=0;
	
	//장비 정보 변경 여부 확인 
	printf("AT+DEVSWVER?");
	HAL_Delay(100);
	while(BT_rx_flag==0){return;}
	BT_rx_flag=0;
	
	if(strncmp(Rx_Buffer, "+OK=V1.0", 8)==0)
	{
		//sprintf(lcd_str, "BT_Init...");
		//ST7789_WriteString(1, 1, lcd_str, Font_16x26, FLIP_RGB(BLUE), FLIP_RGB(WHITE));
		//제조업체 정보 변경
		printf("AT+DEVMANUF=BAMI");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		//시리얼번호 변경
		printf("AT+DEVSERIAL=0001");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		//장비이름 변경
		printf("AT+DEVMODEL=Rehab01-001");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		//블루투스 장비 고유번호 변경
		printf("AT+DEVID=0001");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		//블루투스 스캔 이름 변경
		printf("AT+NAME=Rehab01");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		//블루투스 소프트웨어 버전 수정
		printf("AT+DEVSWVER=V1.1");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		//블루투스 리셋
		printf("AT+RESET");
		HAL_Delay(100);
		while(BT_rx_flag==0){return;}
		BT_rx_flag=0;
		if(strncmp(Rx_Buffer, "+OK", 3)!=0){error_code=300; Error_Handler();}
		HAL_Delay(1000);
		NVIC_SystemReset();
	}
	else if(strncmp(Rx_Buffer, "+OK=V1.1", 8)==0)
	{
//		printf("AT+NAME=Rehab02");
//		while(BT_rx_flag==0){}
//		BT_rx_flag=0;
//		printf("AT+RESET");
//		while(BT_rx_flag==0){}
//		BT_rx_flag=0;
		//이미 설정 완료됨
		//아이디 노출 간격 설정
//		printf("AT+ADVINTV=500");
//		while(BT_rx_flag==0){}
//		BT_rx_flag=0;
	}	
	else
	{
		error_code=300;
		Error_Handler();
	}
	//수신모드 활성
	HAL_GPIO_WritePin(BLE_MOD_GPIO_Port, BLE_MOD_Pin, GPIO_PIN_SET);
	ble_cnt = true;
}

void SwitchFunction(void){
	if(sw_flag == 0 & btn_flag == true){ // sw_flag 0 -> 1 == when pressed
		sw_flag++;
		// screen
		if(screen_status != LCD_MENUSCREEN){
			if(left_btn){
				if(screen_status == LCD_ECGSCREEN || screen_status == LCD_PPGSCREEN){
					reset_line();
				}
				if(screen_status == LCD_SERVERSCREEN){
					server_dataN = 1;
				}
				if(screen_status == LCD_AFSCREEN){
					MAX86171_adjust_clock(1,128);
					AF_rri = 0;
					if(screen_flag != -1){
						free(firStateF32);
						free(newPpgDataFN);
						free(PPI);
						if(af_select){
							free(fft_inputDataF);
							free(fft_outputDataF);
						}
						else
							free(Peak_Interval);
					}
					PPI_idx = 0;
					Filter_Idx = 0;
				}
				screen_flag = 0;
				memset(Rx_read_data,0,sizeof(Rx_read_data));
				if(file_init){	
					if(FIFO_Close()){
						error_code = 202;
						Error_Handler();							
					}
					file_init = false;	
				}
				time_flag2 = 0;				
				screen_status = LCD_MENUSCREEN;
				reset_screen(screen_status);
				btn_flag_reset();
			}
		}
		if(screen_status == LCD_MENUSCREEN){
			if(left_btn){
				time_flag2=1;
				screen_status = LCD_MAINSCREEN;
				reset_screen(screen_status);
				btn_flag_reset();
			}
			if(right_btn){
				sprintf(lcd_str, "<<");
				ST7789_WriteString(200, menu_select*30 +60, lcd_str, Font_11x18, FLIP_RGB(BLACK), FLIP_RGB(BLACK));
				if(menu_select >= 4)
					menu_select = 0;
				else
					menu_select++;
				right_btn = false;
			}
			if(ok_btn){ // status select				
				btn_flag_reset();
				switch(menu_select){
					case 0:
						//PPG
						ECG_PPG_IMU_reset();
						screen_status = LCD_PPGSCREEN;
						break;
					case 1:
						//ECG
						max30003_Synch();
						screen_status = LCD_ECGSCREEN;
						break;
					case 2:
						//AF
						screen_flag = -1;
						screen_status = LCD_AFSCREEN;
//						if(FIFO_Init_W()){
//							error_code = 202;
//							Error_Handler();
//						}
//						file_init = true;
						break;
					case 3:
						//SAVE
						if(sd_cnt){
							if(FIFO_Init_W()){
								error_code = 202;
								Error_Handler();
							}
							file_init = true;
							reset_screen(screen_status);
							time_flag2++;
							ECG_PPG_IMU_reset();
						}
						screen_status = LCD_SAVESCREEN;
						break;
					case 4:
						//SERVER
						screen_status = LCD_SERVERSCREEN;
						break;
				}
				reset_screen(screen_status);
				ok_btn = false;
			}
		}
		else if(screen_status == LCD_AFSCREEN){
			if(screen_flag == -1){
				if(ok_btn){
					if(sd_cnt){
						time_flag2=1;
						ECG_PPG_IMU_reset();
					}
					if(af_select == 0){
						// sample rate 50
						firStateF32 = (float32_t *)calloc((BLOCK_SIZE + NUM_TAPS - 1), sizeof(float32_t));
						arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)PPG_coeffs, (float32_t *)&firStateF32[0], BLOCK_SIZE);
						newPpgDataFN = (float32_t *)calloc((BUFF_SIZE), sizeof(float32_t));
						Peak_Interval = (int16_t *)calloc((3*BLOCK_SIZE/SAMPLE_RATE), sizeof(int16_t));
						MAX86171_adjust_clock(1,SAMPLE_RATE);
					}else if(af_select == 1){
						// sample rate 25
						firStateF32 = (float32_t *)calloc((BLOCK_SIZE_FFT + NUM_TAPS_FFT - 1), sizeof(float32_t));
						arm_fir_init_f32(&S, NUM_TAPS_FFT, (float32_t *)PPG_coeffs2, (float32_t *)&firStateF32[0], BLOCK_SIZE_FFT);
						newPpgDataFN = (float32_t *)calloc((BUFF_SIZE_FFT), sizeof(float32_t));
						fft_inputDataF = (float32_t *)calloc(NFFT*2, sizeof(float32_t));
						fft_outputDataF = (float32_t *)calloc(NFFT, sizeof(float32_t));
						MAX86171_adjust_clock(1,SAMPLE_RATE_FFT);
						iterator_IMU = 0;
					}
					PPI = (int16_t *)calloc(RRI_COUNT,sizeof(int16_t));
					screen_flag++;
					reset_screen(screen_status);
					ok_btn = false;
				}
				if(right_btn){
					sprintf(lcd_str, "<<");
					ST7789_WriteString(200, af_select*60 +60, lcd_str, Font_11x18, FLIP_RGB(BLACK), FLIP_RGB(BLACK));
					if(af_select >= 2)
						af_select = 0;
					else
						af_select++;					
					right_btn = false;
				}
			}
		}
		else if(screen_status == LCD_SAVESCREEN){
//			if(ok_btn){
//				
//			}
		}
		else if(screen_status == LCD_SERVERSCREEN){
			if(screen_flag == 0){
				if(ok_btn){
					screen_flag++;
					time_flag2 = 1;
					ok_btn = false;
					reset_screen(screen_status);
				}
				if(right_btn){
					sprintf(lcd_str, "<<");
					ST7789_WriteString(200, server_select*60 +60, lcd_str, Font_11x18, FLIP_RGB(BLACK), FLIP_RGB(BLACK));
					if(server_select >= 2)
						server_select = 0;
					else
						server_select++;
					right_btn = false;
				}
			}
			if(screen_flag == 1 && server_select == 1){
				if(ok_btn){
					if(server_dataN < 255)
						server_dataN++;
				}
			}
		}
	}
	else if(sw_flag == 1){  // sw_flag 1 == when pressing
		if(btn_flag == false){ // when button is released
			sw_flag++;
			if(pull_btn_time == 0)
				pull_btn_time = 1;
			btn_pressing = 0;
		}
		else{ //pressing
			if(btn_pressing == 0)
				btn_pressing = 1;
		}
	}
	else if(sw_flag == 2 & btn_flag == true & pull_btn_time <= 10){	// sw_flag 2 == release the button
		if(left_btn == true)
			sw_L_flag++;
		if(ok_btn == true)
			sw_C_flag++;
		if(right_btn == true)
			sw_R_flag++;
		
		sw_flag = 0;
		pull_btn_time = 0;
	}
	
	if(pull_btn_time > 10){
		btn_flag_reset();
	}
	
	
	if(left_btn == true)
	{
		//L button on
		if(!HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin))
			left_btn = false;
	}
	if(right_btn == true)
	{
		//R button on
		if(!HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin))
			right_btn = false;		
	}
	if(ok_btn == true)
	{
		//C button on
		if(!HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin))
			ok_btn = false;		
	}
	if(!left_btn & !right_btn & !ok_btn)
		btn_flag = false;
	if(btn_pressing >= 30 && screen_status == LCD_MAINSCREEN){
		Set_StandbyMod();
	}
}
void AF_Algorithm(){
	if(sd_cnt){
		if(af_select == 0 && screen_flag == 0){
			if(max86171_step != 0)
			{
				uint8_t count_86171 =0;
				int32_t max86171_input[3];
				for(int i=0;i<max86171_step;i+=2)
				{
					if(max86171_data[i].tag == 1)
					{
						max86171_buffer[0] = max86171_data[i].data;
						max86171_buffer[1] = max86171_data[i+1].data;
					}
					if(max86171_data[i].tag == 2)
					{
						max86171_buffer[2] = max86171_data[i].data;
					}
					if(max86171_data[i].tag == 3)
					{
						max86171_buffer[3] = max86171_data[i].data;
						max86171_buffer[4] = max86171_data[i+1].data;
					}
					if(max86171_data[i].tag == 4)
					{
						max86171_buffer[5] = max86171_data[i].data;
						count_86171++;
					}
					if(count_86171 == 1)
					{
						max86171_input[0] = max86171_buffer[0]-max86171_buffer[3];
						max86171_input[1] = max86171_buffer[1]-max86171_buffer[4];
						max86171_input[2] = max86171_buffer[2]-max86171_buffer[5];
						
						// data
						float32_t data = (float32_t) (max86171_input[0]+max86171_input[1]+max86171_input[2])/3;
								
						newPpgDataFN[Filter_Idx] = data;				
						if (Filter_Idx >= BLOCK_SIZE)
							newPpgDataFN[Filter_Idx - BLOCK_SIZE] = data;				
						
						// filter		
						++Filter_Idx;
						if((time_flag2) > FILTERING_RATE && Filter_Idx >= BLOCK_SIZE){
							float32_t filtered_ppgDataF[BLOCK_SIZE]={0,};
							float32_t raw_ppgData[BLOCK_SIZE] = {0,};
							time_flag2 = 1;
							
							for(int normalize = 0; normalize < BLOCK_SIZE ;normalize++){
								raw_ppgData[normalize] = newPpgDataFN[Filter_Idx - BLOCK_SIZE + normalize] - newPpgDataFN[Filter_Idx - BLOCK_SIZE];
							}
							
							arm_fir_f32(&S, raw_ppgData, filtered_ppgDataF, BLOCK_SIZE);
						
							// PEAK
							AF_rri = AF_Peak_detection(filtered_ppgDataF,Peak_Interval);
							
							/* matching rri */
							
							// save first rri
							if(PPI[0] == 0)
								for(int j=0;j<AF_rri;j++)
									PPI[j] = Peak_Interval[j];
							
							// matching rri idx									
							uint8_t max_idx_count = 0;
							uint8_t last_rri_idx = 0;			
							float64_t AF_shannon=0, AF_sample=0, AF_rmssd=0; // save rmssd, sample entropy, shannon entropy						
							
							for(int j=RRI_COUNT-1;j>=0;j--){
								uint8_t idx_count = 0;						
								do {
									if(Peak_Interval[0+idx_count] == PPI[j+idx_count]){
										if(max_idx_count < idx_count){
											max_idx_count = idx_count;
											last_rri_idx = j;
										}
										idx_count++;
									}
									else
										break;
								}while(idx_count < AF_rri-1 && idx_count > 0 && j+idx_count < RRI_COUNT);
							}
							
							//sort & check AF
							int8_t sort_num = AF_rri+last_rri_idx-RRI_COUNT;
							if(sort_num < 0){
								for(int j=last_rri_idx;j<AF_rri+last_rri_idx;j++)
									PPI[j] = Peak_Interval[j-last_rri_idx];
							}
							else{
								for(int j=0;j<RRI_COUNT-AF_rri;j++)
									PPI[j] = PPI[sort_num+j];
								for(int j=RRI_COUNT-AF_rri;j<RRI_COUNT;j++)
									PPI[j] = Peak_Interval[j-RRI_COUNT+AF_rri];

								float32_t normalizePPI[16];
								float32_t mean_Val;
								
								for(int norm_cpy = 0; norm_cpy<SAMPLE_N;norm_cpy++){
									normalizePPI[norm_cpy] = (float32_t)PPI[norm_cpy]/(float32_t)SAMPLE_RATE;
								}
								arm_mean_f32(normalizePPI, SAMPLE_N, &mean_Val);
								
								AF_shannon = AF_Shannon_Entropy(normalizePPI);
								AF_sample = AF_Sample_Entropy(normalizePPI,mean_Val);
								AF_rmssd = AF_Normlaized_RMSSD(normalizePPI,mean_Val);
								
								// LCD
								if(AF_shannon > 0.6 && AF_sample > 1.5 && AF_rmssd > 0.2){
									reset_screen(screen_status);
									screen_flag++;								
									MAX86171_adjust_clock(1,128);
									ECG_PPG_IMU_reset();
									time_flag2 = 0;
								}
							}
							int16_t IMU_data = 0;
							LSM6DS_GetGyroDataX(I2C_Polling_data);
							IMU_data += abs(I2C_Polling_data[0]);
							LSM6DS_GetGyroDataY(I2C_Polling_data);
							IMU_data += abs(I2C_Polling_data[0]);
							LSM6DS_GetGyroDataZ(I2C_Polling_data);
							IMU_data += abs(I2C_Polling_data[0]);
							
							showAFdata(AF_shannon, AF_sample, AF_rmssd, AF_rri, IMU_data);
							
						}
						// primary swap
						if (Filter_Idx >= BUFF_SIZE)
							Filter_Idx = BLOCK_SIZE;			
					}
					
				}
				max86171_step = 0;
			}
		}
		else if(af_select == 1 && screen_flag == 0){
			if(max86171_step != 0)
			{
				int32_t max86171_input[3];
				uint8_t count_86171 = 0;
				for(int i=0;i<max86171_step;i+=2)
				{
					if(max86171_data[i].tag == 1)
					{
						max86171_buffer[0] = max86171_data[i].data;
						max86171_buffer[1] = max86171_data[i+1].data;
					}
					if(max86171_data[i].tag == 2)
					{
						max86171_buffer[2] = max86171_data[i].data;
					}
					if(max86171_data[i].tag == 3)
					{
						max86171_buffer[3] = max86171_data[i].data;
						max86171_buffer[4] = max86171_data[i+1].data;
					}
					if(max86171_data[i].tag == 4)
					{
						max86171_buffer[5] = max86171_data[i].data;
						count_86171++;
					}
					if(count_86171 == 1)
					{
						count_86171 = 0;
						max86171_input[0] = max86171_buffer[0]-max86171_buffer[3];
						max86171_input[1] = max86171_buffer[1]-max86171_buffer[4];
						max86171_input[2] = max86171_buffer[2]-max86171_buffer[5];
						
						// data
						float32_t data = (float32_t) (max86171_input[0]+max86171_input[1]+max86171_input[2])/3;
								
						newPpgDataFN[Filter_Idx] = data;
						if (Filter_Idx >= BLOCK_SIZE_FFT)
							newPpgDataFN[Filter_Idx - BLOCK_SIZE_FFT] = data;				
						
						// filter		
						++Filter_Idx;
						if(time_flag2 > FILTERING_RATE && Filter_Idx >= BLOCK_SIZE_FFT && iterator_IMU >= BLOCK_SIZE_FFT){
							time_flag2 = 1;
							float32_t filtered_ppgDataF[BLOCK_SIZE_FFT]={0,};
							float32_t filtered_gyroDataF[BLOCK_SIZE_FFT]={0,};
							float32_t filtered_accDataF[BLOCK_SIZE_FFT]={0,};
							float32_t fft_ppgDataF[USING_FFT]={0,};
//							float32_t fft_accDataF[USING_FFT]={0,};
							float32_t fft_gyroDataF[USING_FFT]={0,};
							
							//fir
							{
								float32_t raw_ppgData[BLOCK_SIZE_FFT] = {0,};
								float32_t raw_accData[BLOCK_SIZE_FFT] = {0,};
								float32_t raw_gyroData[BLOCK_SIZE_FFT] = {0,};
								
								for(int normalize = 0; normalize < BLOCK_SIZE_FFT ;normalize++){
									raw_ppgData[normalize] = newPpgDataFN[Filter_Idx - BLOCK_SIZE_FFT + normalize] - newPpgDataFN[Filter_Idx - BLOCK_SIZE_FFT];
									raw_gyroData[normalize] = bufferGYRO[iterator_IMU - BLOCK_SIZE_FFT + normalize] - bufferGYRO[iterator_IMU - BLOCK_SIZE_FFT];
									raw_accData[normalize] = bufferACC[iterator_IMU - BLOCK_SIZE_FFT + normalize] - bufferACC[iterator_IMU - BLOCK_SIZE_FFT];
								}
								
								arm_fir_f32(&S, raw_ppgData, filtered_ppgDataF, BLOCK_SIZE_FFT);
								arm_fir_f32(&S, raw_gyroData, filtered_gyroDataF, BLOCK_SIZE_FFT);
								arm_fir_f32(&S, raw_accData, filtered_accDataF, BLOCK_SIZE_FFT);								
							}
							
														
							// ppg complex input & fft							
							for(int zzz = 0; zzz<BLOCK_SIZE_FFT - 50;zzz+=2){
								fft_inputDataF[zzz] = filtered_ppgDataF[(zzz/2) + 50];
								fft_inputDataF[zzz+1] = 0;
							}
							arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputDataF, 0, 1);
							arm_cmplx_mag_f32(fft_inputDataF, fft_outputDataF, NFFT);
							memset(fft_inputDataF,0,sizeof(float32_t) * NFFT*2);
							memcpy(fft_ppgDataF,&fft_outputDataF[49],sizeof(float32_t)*USING_FFT);
														
							// ppg normalize
							float32_t min_Val, max_Val;
							arm_min_f32(fft_ppgDataF, USING_FFT, &min_Val, 0);
							arm_offset_f32(fft_ppgDataF, -min_Val, fft_ppgDataF, USING_FFT);
							arm_max_f32(fft_ppgDataF, USING_FFT, &max_Val, 0);
							arm_scale_f32(fft_ppgDataF, 1/max_Val, fft_ppgDataF, USING_FFT);
							
							// gyro complex input & fft
							for(int zzz = 0; zzz<BLOCK_SIZE_FFT - 50;zzz+=2){
								fft_inputDataF[zzz] = filtered_gyroDataF[(zzz/2) + 50];
								fft_inputDataF[zzz+1] = 0;
							}							
							arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_inputDataF, 0, 1);
							arm_cmplx_mag_f32(fft_inputDataF, fft_outputDataF, NFFT);
							memset(fft_inputDataF,0,sizeof(float32_t) * NFFT*2);
							memcpy(fft_gyroDataF,&fft_outputDataF[49],sizeof(float32_t)*222);
							
							// gyro normalize
							arm_min_f32(fft_gyroDataF, USING_FFT, &min_Val, 0);
							arm_offset_f32(fft_gyroDataF, -min_Val, fft_gyroDataF, USING_FFT);
							arm_max_f32(fft_gyroDataF, USING_FFT, &max_Val, 0);
							arm_scale_f32(fft_gyroDataF, 1/max_Val, fft_gyroDataF, USING_FFT);
							
							// motion artifact
							float32_t mean_gyro;
							arm_mean_f32(fft_gyroDataF, USING_FFT, &mean_gyro);
							
							if(mean_gyro < 0.3){
								// ppg - gyro
								arm_sub_f32(fft_ppgDataF,fft_gyroDataF,fft_ppgDataF,USING_FFT);
								
								//log scale
								arm_mult_f32(fft_ppgDataF,(float32_t *)log_curve,fft_ppgDataF,USING_FFT);
							}
							
							//gaussian
							if(PPI_idx != 0){
								int32_t bpm_to_num = PPI[PPI_idx-1];
								float adaptive_gaussian_array[222];
								
								bpm_to_num = ((bpm_to_num*1024)/(60*12.5))-49;
								if(bpm_to_num<0){bpm_to_num=0;}
								
								memcpy(adaptive_gaussian_array, gaussian_dis2+250-bpm_to_num, sizeof(float32_t)*USING_FFT);
								arm_mult_f32(fft_ppgDataF,adaptive_gaussian_array,fft_ppgDataF,USING_FFT);
							}
								
							uint32_t max_Index;
							float32_t normalizePPI[16];
							arm_max_f32(fft_ppgDataF, USING_FFT, &max_Val, &max_Index);
							
							PPI[PPI_idx] = (max_Index+49)*(12.5/1024)*60;
							float64_t AF_shannon=0, AF_sample=0, AF_rmssd=0;
								
							if(PPI_idx < 16)
								PPI_idx++;
							else {
								for(int idx_sort = 0;idx_sort < 16;idx_sort++){
									PPI[idx_sort] = PPI[idx_sort+1];
									normalizePPI[idx_sort] = (float32_t)PPI[idx_sort+1]/(float32_t)60;
								}
								float32_t mean_Val;
								PPI[16] = 0;								
								arm_mean_f32(normalizePPI, SAMPLE_N, &mean_Val);
								
								AF_shannon = AF_Shannon_Entropy(normalizePPI);
								AF_sample = AF_Sample_Entropy(normalizePPI,mean_Val);
								AF_rmssd = AF_Normlaized_RMSSD(normalizePPI,mean_Val);
								
								// next
								if(AF_shannon > 0.7 && AF_sample > 0.5 && AF_rmssd > 0.06){
									reset_screen(screen_status);
									screen_flag++;									
									MAX86171_adjust_clock(1,128);
									ECG_PPG_IMU_reset();
									time_flag2 = 0;
								}
							}
							
							if(file_init){
								fifo_ppg(PPI[0]);
							}
							
							// LCD
							showAFdata(AF_shannon, AF_sample, AF_rmssd, 0,0);
							
							// FSM
							
						}
					// primary swap
						if (Filter_Idx >= BUFF_SIZE_FFT)
							Filter_Idx = BLOCK_SIZE_FFT;			
					}
					
				}
				max86171_step = 0;
			}
		}
		else {
			if(screen_flag == 1){ // ecg touch check
				if(max30003_step != 0)
				{
					for(int i=0;i<max30003_step;i++)
					{
						if(abs(max30003_data[i].data) > 10000){
							if(sd_cnt){
								if(FIFO_Init_W()){
									error_code = 202;
									Error_Handler();
								}
								file_init = true;
							}
							reset_screen(screen_status);
							screen_flag++;
							time_flag2 = 1;
							spk_switch = 0;
							break;
						}
					}
					max30003_step = 0;
				}
			}
			else if(screen_flag == 2){ // data stabilization
				max30003_step = 0;
				if(time_flag2 >= 50){
					reset_screen(screen_status);
					ECG_PPG_IMU_reset();
					time_flag2 = 1;
					screen_flag++;
				}
			}
			else if(screen_flag == 3){ // 30s data fifo
				if(file_init)
					data_fifo();
				if(time_flag2 >= 300){
					reset_screen(screen_status);
					if(file_init){
						if(FIFO_Close()){
							error_code = 202;
							Error_Handler();							
						}
						file_init = false;	
					}
					screen_flag++;
				}
			}	
			else if(screen_flag == 4){ // stfp check
				if(strncmp(Rx_read_data,"SCS",3) == 0){
					reset_screen(screen_status);
					BT_rx_flag = 0;
					screen_flag++;
				}
			}
			else if(screen_flag == 5){ // send data
				Directory_check();
				Data_Send();
				reset_screen(screen_status);
				screen_flag++;
			}
			else if(screen_flag == 6){ // finish
				if(Directory_check()){
					error_code = 205;
					Error_Handler();
				}
			}			
		}
	}
	else {
		sd_cnt = sd_init();
		if(sd_cnt){
			reset_screen(screen_status);
			time_flag2=1;
			ECG_PPG_IMU_reset();
		}
	}
}
void save_algorithm(){
	if(time_flag2 >= 600) { // 1 min data save
		time_flag2=1;					
		if(FIFO_Close()){
			error_code = 202;
			Error_Handler();							
		}
		file_init = false;		
		if(sd_cnt){
			if(FIFO_Init_W()){
				error_code = 202;
				Error_Handler();
			}
			file_init = true;
		}
	}
	
	if(file_init)
		data_fifo();
	else{ // wait sd connection
		sd_cnt = sd_init();
		if(sd_cnt){
			if(FIFO_Init_W()){
				error_code = 202;
				Error_Handler();
			}
			file_init = true;
			reset_screen(screen_status);
			time_flag2++;
			ECG_PPG_IMU_reset();
		}
	}
}

void server_algorithm(){
	if(sd_cnt){ //wait ble connection
		if(screen_flag == 1){			
			dot_screen_moving("looking for BLE",time_flag2);			
			if(strncmp(Rx_read_data,"SCS",3) == 0){
				BT_rx_flag = 0;
				reset_screen(screen_status);
				screen_flag++;
				time_flag2 = 0;
			}
		}
		else if(screen_flag == 2){			
			if(Directory_check()){
				error_code = 205;
				Error_Handler();
			}
			Data_Send();
			screen_flag++;
			reset_screen(screen_status);
		}
	}
	else{ //wait sd connection
		sd_cnt = sd_init();
		if(sd_cnt){
			reset_screen(screen_status);
		}
	}
}

void data_fifo(){
	if(max86171_step != 0) // ppg fifo
	{
		uint8_t count_86171 =0;
		int32_t max86171_input[3];
		for(int i=0;i<max86171_step;i+=2)
		{
			if(max86171_data[i].tag == 1)
			{
				max86171_buffer[0] = max86171_data[i].data;
				max86171_buffer[1] = max86171_data[i+1].data;
			}
			if(max86171_data[i].tag == 2)
			{
				max86171_buffer[2] = max86171_data[i].data;
			}
			if(max86171_data[i].tag == 3)
			{
				max86171_buffer[3] = max86171_data[i].data;
				max86171_buffer[4] = max86171_data[i+1].data;
			}
			if(max86171_data[i].tag == 4)
			{
				max86171_buffer[5] = max86171_data[i].data;
				count_86171++;
			}
			if(count_86171 == 1)
			{
				max86171_input[0] = max86171_buffer[0]-max86171_buffer[3];
				max86171_input[1] = max86171_buffer[1]-max86171_buffer[4];
				max86171_input[2] = max86171_buffer[2]-max86171_buffer[5];
				
//				int32_t ppg = max86171_input[0] + max86171_input[1] + max86171_input[2];
				fifo_ppg2(max86171_input);
			}
		}
		max86171_step = 0;
	}
	if(max30003_step != 0) // ecg fifo
	{
		for(int i=0;i<max30003_step;i++)
		{
			fifo_ecg(max30003_data[i].data);
			
		}
		max30003_step = 0;
	}	
	int16_t tmp_iterator_IMU = iterator_IMU;
	if(tmp_iterator_IMU >= BLOCK_SIZE_FFT)
		tmp_iterator_IMU = iterator_IMU-BLOCK_SIZE_FFT;
	
	if(iteratorFifoIMU != tmp_iterator_IMU){ // imu fifo circular queue
		if(iteratorFifoIMU > tmp_iterator_IMU){
			for(int i=iteratorFifoIMU;i<BLOCK_SIZE_FFT;i++){
				int32_t imu[6] = {fifoGYRO[i][0],fifoGYRO[i][1],fifoGYRO[i][2],fifoACC[i][0],fifoACC[i][1],fifoACC[i][2]};
				fifo_imu2(imu);
			}
			if(tmp_iterator_IMU != 0){
				for(int i=0;i<tmp_iterator_IMU;i++){
					int32_t imu[6] = {fifoGYRO[i][0],fifoGYRO[i][1],fifoGYRO[i][2],fifoACC[i][0],fifoACC[i][1],fifoACC[i][2]};
					fifo_imu2(imu);
				}
			}
		}
		else {
			for(int i=iteratorFifoIMU;i<tmp_iterator_IMU;i++){
				int32_t imu[6] = {fifoGYRO[i][0],fifoGYRO[i][1],fifoGYRO[i][2],fifoACC[i][0],fifoACC[i][1],fifoACC[i][2]};
				fifo_imu2(imu);
			}
		}
		iteratorFifoIMU = tmp_iterator_IMU;
	}
}

void Data_Update() {
	if(screen_status == LCD_PPGSCREEN || screen_status == LCD_MAINSCREEN || screen_status == LCD_SAVESCREEN || screen_status == LCD_AFSCREEN){		
		PPG_Update();
	}
		
	if(screen_status == LCD_ECGSCREEN || screen_status == LCD_MAINSCREEN || screen_status == LCD_SAVESCREEN || screen_status == LCD_AFSCREEN){
		ECG_Update();
	}
		
	if(screen_status == LCD_MAINSCREEN || screen_status == LCD_SAVESCREEN || screen_status == LCD_AFSCREEN){
//		IMU_Update(); buffer -> polling (for stability)
	}	
}

void Data_Send(void) {
	// send protocol O + the number of files
	if(screen_status == LCD_AFSCREEN){
		printf("O0001");
		
		if(folder_WR(1, hiwdg)){
			error_code = 202;
			Error_Handler();
		}
	}
	else if(screen_status == LCD_SERVERSCREEN){
		if(server_select == 0){
			printf("O0001");
			folder_WR(1, hiwdg);
		}
		if(server_select == 1){
			int16_t folderN = get_folderN();
			if(folderN < server_dataN)
				server_dataN = folderN;			
			printf("O%04d",server_dataN);
			
			while(server_dataN){
				if(folder_WR(server_dataN, hiwdg)){
					error_code = 202;
					Error_Handler();
				}
				server_dataN--;
			}
		}
		if(server_select == 2){
			int16_t folderN = get_folderN();
			printf("O%04d",folderN);
			while(folderN){
				if(folder_WR(folderN, hiwdg)){
					error_code = 202;
					Error_Handler();
				}
				folderN--;
			}
		}
	}
	printf("F");
}
void PPG_Update(void) {
	max86171_stat = MAX86171_readReg(PPG_STAT_1);
	if((max86171_stat & 0xC0)) { //128bits interrupt
		max86171_step = MAX86171_read_fifo(max86171_data); // 128fps		
	}
	if(max86171_step == 256){
		ECG_PPG_IMU_reset();
		max86171_step = 0;
	}
}

void ECG_Update(void) {
	max30003_RegRead(STATUS,max30003_RxData);
	if(max30003_RxData[1] & 0x80) // 16bits interrupt
	{
		max30003_step = max30003_getEcgSamples_burst(max30003_data, 16);// 128sps gain 20 0.5hz ~ 40hz cutoff
		if(max30003_step == 0)	
		{
			max30003_step = 16;
			for(int i=0;i<16;i++)
				max30003_data[i].data = 0;				
		}
	}
	if(max30003_RxData[1] & 0x40){ // Fifo Overflow
		ECG_PPG_IMU_reset();
	}
}

//void IMU_Update(void) {
//	
//	LSM6DS_FIFO_DATA_NUM(I2C_Stat);
//	if(I2C_Stat[0] & 0x6000){
//		spk_switch = 1;
//		ECG_PPG_IMU_reset();
//	}
//	else if(I2C_Stat[0] & 0x07ff)	{
//		I2C_Stat[0] = (int16_t)(I2C_Stat[0] & 0x07ff);	
//		if(I2C_Stat[0] >= 240)
//			LSM6DS_FIFO_DATA_OUT(I2C_BurstData, 240);
//		else
//			LSM6DS_FIFO_DATA_OUT(I2C_BurstData, I2C_Stat[0]);
//	}
//	else
//	{
//		I2C_Stat[0] = (int16_t)(I2C_Stat[0] & 0x07ff);	
//	}
//	
//}


void btn_flag_reset(void) {
	// btn flag reset
	left_btn = false;
	sw_flag = 0;
	sw_L_flag = 0;
	sw_R_flag = 0;
	sw_C_flag = 0;
	pull_btn_time = 0;
	btn_pressing = 0;
}

void ECG_PPG_IMU_reset(void) {
	// ppg ecg imu buffer reset
	max30003_Synch();
	LSM6DS_Reset();
	MAX86171_writeReg(PPG_FIFO_CONFIG_2, 0x1E);
	LSM6DS_GetBeing_polling(&LSM6DSM_IMUSetting ,I2C_RxData);
	iterator_IMU = 0;
	iteratorFifoIMU = 0;
}

void UF_speaker_ON(void) {
	// speaker sound on
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	TIM3->ARR=250;
	//TIM2->CCR2=(int)(325/2);
	TIM3->CCR4=(int)(200/2);
}

void UF_speaker_OFF(void) {
	// speaker sound off
	TIM3->CCR4=0;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}	

void Set_StandbyMod(void) {
	// turn off the watch
	while(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin));
	HAL_Delay(1000);
//	HAL_GPIO_WritePin(BLE_MOD_GPIO_Port, BLE_MOD_Pin, GPIO_PIN_RESET);
//  printf("AT+SLEEP");
//   
//  HAL_GPIO_WritePin(BLE_TX_GPIO_Port, BLE_TX_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(BLE_RX_GPIO_Port, BLE_RX_Pin, GPIO_PIN_RESET);

	HAL_IWDG_Refresh(&hiwdg);
		
	//사용 된 모든 웨이크 업 소스를 비활성화
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	
	// ALL FLAG RESET
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	
	//사용 된 모든 웨이크 업 소스를 다시 사용 가능하게 설정 
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
	
	HAL_GPIO_WritePin(VPP_EN_GPIO_Port,VPP_EN_Pin,GPIO_PIN_RESET);
	
	//스탠바이모드 진입
	HAL_PWR_EnterSTANDBYMode();
}
	

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	// interrupt
	if(htim -> Instance == TIM6) // 25Hz
	{
		LSM6DS_GetGyroAccAll(I2C_Polling_buffer);
		bufferGYRO[iterator_IMU] = I2C_Polling_buffer[0] + I2C_Polling_buffer[1] + I2C_Polling_buffer[2];
		bufferACC[iterator_IMU] = I2C_Polling_buffer[3] + I2C_Polling_buffer[4] + I2C_Polling_buffer[5];
		if(iterator_IMU >= BLOCK_SIZE_FFT){
			bufferGYRO[iterator_IMU - BLOCK_SIZE_FFT] = I2C_Polling_buffer[0] + I2C_Polling_buffer[1] + I2C_Polling_buffer[2];
			bufferACC[iterator_IMU - BLOCK_SIZE_FFT] = I2C_Polling_buffer[3] + I2C_Polling_buffer[4] + I2C_Polling_buffer[5];
			fifoGYRO[iterator_IMU - BLOCK_SIZE_FFT][0] = I2C_Polling_buffer[0];
			fifoGYRO[iterator_IMU - BLOCK_SIZE_FFT][1] = I2C_Polling_buffer[1];
			fifoGYRO[iterator_IMU - BLOCK_SIZE_FFT][2] = I2C_Polling_buffer[2];
			fifoACC[iterator_IMU - BLOCK_SIZE_FFT][0] = I2C_Polling_buffer[3];
			fifoACC[iterator_IMU - BLOCK_SIZE_FFT][1] = I2C_Polling_buffer[4];
			fifoACC[iterator_IMU - BLOCK_SIZE_FFT][2] = I2C_Polling_buffer[5];			
			
			if(iterator_IMU == BLOCK_SIZE_FFT*2 - 1)
				iterator_IMU=BLOCK_SIZE_FFT-1;
		}else {			
			fifoGYRO[iterator_IMU][0] = I2C_Polling_buffer[0];
			fifoGYRO[iterator_IMU][1] = I2C_Polling_buffer[1];
			fifoGYRO[iterator_IMU][2] = I2C_Polling_buffer[2];
			fifoACC[iterator_IMU][0] = I2C_Polling_buffer[3];
			fifoACC[iterator_IMU][1] = I2C_Polling_buffer[4];
			fifoACC[iterator_IMU][2] = I2C_Polling_buffer[5];
		}
		iterator_IMU++;
	}
	else if(htim->Instance == TIM7) // 10Hz
	{
		if(time_flag2)
			time_flag2++;
		if(pull_btn_time)
			pull_btn_time++;
		if(btn_pressing)
			btn_pressing++;
	}
	

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	switch (error_code) { // error code
			case 100:
				sprintf(error_msg,"ECG err");
				break;
			case 101:
				sprintf(error_msg,"PPG err");
				break;
			case 102:
				sprintf(error_msg,"PPG FIFO err");
				break;
			case 200:
				sprintf(error_msg,"SD bsf err");
				break;
			case 201:
				sprintf(error_msg,"SD Mount err");
				break;
			case 202:
				sprintf(error_msg,"SD FIFO err");
				break;
			case 204:
				sprintf(error_msg,"SD close err");	
				break;
			case 205: 
				sprintf(error_msg,"SD Dir err");	
				break;
			case 300:
				sprintf(error_msg,"Ble err");	
				break;
			case 401:
				sprintf(error_msg,"IMU err");	
				break;			
	}
	ST7789_WriteString(10, 100, error_msg, Font_16x26, FLIP_RGB(GREEN), FLIP_RGB(BLACK));
	HAL_Delay(3000);	
  __disable_irq();
  while (1)
  {
		NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
