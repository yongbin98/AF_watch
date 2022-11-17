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
#include "fir_constants.h"

#include "MAX17262/MAX17262.h"
#include "LSM6DS/LSM6DS.h"
#include "st7789/st7789.h"
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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_tx;

/* USER CODE BEGIN PV */

//SD 관련 구조체
FRESULT res;	
FATFS SDFatFs;
FIL ECG_fp;
FIL PPG_fp;
FIL IMU_fp;
DIR dir;
FILINFO fno;
char SD_msg[100];

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

//Battery
uint8_t battery_value=0;
 
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
int16_t I2C_BurstData[240]; // buffer[40] -> acc x, acc y, acc z, gyro x, gyro y, gyro z 
int16_t I2C_Stat[1]; // I2c status
int16_t I2C_polling_data[1];

//LCD
st7789 st7789_PFP; // lcd init address
char lcd_str[40]; // lcd message

//FLAG
uint8_t file_count = 0; // check the number of previous file 
uint8_t folder_count = 0; // check the number of previous folder 
uint8_t spk_switch = 0; // OFF : 0, ON : 1, OFF ~ing : 2, ON ~ing : 3
uint16_t time_flag2 = 0; // 0.1s time check
uint16_t error_code = 0; // error code
char error_msg[30]; // error message
float64_t AF_shannon=0, AF_sample=0, AF_rmssd=0; // save rmssd, sample entropy, shannon entropy
uint8_t AF_rri = 0; // the number of rri 
uint8_t PPI[RRI_COUNT] = {0,}; // save recent 16 rri data
uint8_t Peak_Interval[10] = {0,}; // received rri

//layout
bool ble_cnt = false; // ble init -> true
bool bat_cnt = false; // bat init -> true
bool sd_cnt = false; // sd init -> true
bool file_init = false; // file init -> true
bool left_btn = false; // left btn interrupt -> true
bool right_btn = false; // right btn interrupt -> true
bool ok_btn = false; // ok btn interrupt -> true
bool btn_flag = false; // btn interrupt -> true
uint16_t pull_btn_time = 0; // how long pull btn ( passing one second -> reset)
uint16_t btn_pressing = 0; // how long btn pressing
LCD_SCREEN_TYPE screen_status = LCD_MAINSCREEN; // now screen
uint8_t menu_btn_count = 0; // menu screen 5 btn
uint8_t server_btn_count = 0; // server menu 3 btn
uint8_t server_layout = 0; // server layout : 0 = send amount, 1 = ble wait, 2 = send data
uint16_t screen_dot = 0; // server connect
uint8_t AF_layout = 0;
uint8_t screen_flag = 0;

//PPG ECG layout
uint8_t line_x = 0; // now x_line loc
int32_t line_y1 = 0, line_y2=0; // previous y_line , next y_line
int32_t base_line_y=0, max_line_y=0, min_line_y=0; // y_line threshold (mean max min)

//filter
arm_fir_instance_f32 S; // fir filter init
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1]; // fir filter init
float32_t newPpgDataFN[BUFF_SIZE] = {0,}; // circular queue
float32_t fir_LCD_data[FILTERING_RATE] = {0,}; // LCD PPG data
uint16_t Filter_Idx = 0; // check data number
uint16_t fir_threshold = 0; // fir threshold > filtering rate -> do it

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
void FIFO_Init_W(void);
void FIFO_Init_R(uint8_t file_number);
void Data_Update(void);
void FIFO_Close(void);
void DATA_FIFO(void);
void LCD_Init(void);
void LCD_Reset_Screen(void);
void LCD_AF_Screen(void);
void LCD_MAIN_Screen(void);
void LCD_MENU_Screen(void);
void LCD_PPG_Screen(void);
void LCD_ECG_Screen(void);
void LCD_SAVE_Screen(void);
void LCD_SERVER_Screen(void);
void UF_speaker_ON(void);
void UF_speaker_OFF(void);
void SD_Init(void);
void DNN_Run(void);
void AF_Algorithm(void);
void AF_Layer_rescale(float *AF_Rescale);
void AF_Show_Data(void);
void SwitchFunction(void);
void Bluetooth_init(void);
void Directory_check(void);
void Set_StandbyMod(void);
void btn_flag_reset(void);
void ECG_PPG_IMU_reset(void);
void dot_screen_moving(char dot_msg[]);
void acc_warning(void);
void ECG_Update(void);
void PPG_Update(void);
void IMU_Update(void);
void IMU_Polling_Update(void);
void Data_Send(void);
void Folder_Read_Write(uint8_t folder_N);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//printf
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  uint8_t temp[1]={ch};
	while(HAL_UART_Transmit(&huart7, (uint8_t *)&temp, 1, HAL_MAX_DELAY) != HAL_OK);
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
  HAL_Delay(100);
	
	/* watch dog */
	__HAL_DBGMCU_FREEZE_IWDG();
	
	/* LCD init */
	LCD_Init();
	
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
	
	if(MAX86171_init() == 0) { 
		error_code=101;
		Error_Handler(); 
	}
	
	/* SD Init */
	SD_Init();
	
	/* LCD Screen */	
	ST7789_DrawFilledRectangle2(&st7789_PFP, 0, 0, 240, 240, FLIP_RGB(BLACK));
	ST7789_DrawFilledRectangle2(&st7789_PFP, 0, 0, 240, 40, FLIP_RGB(WHITE));
	battery_value=MAX17262_Get_BatPercent();
	LCD_MAIN_Screen();
	
	/*interrupt (TIMER, BLUETOOTH) */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);	
	
	/* FIR init */
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&PPG_coeffs[0], (float32_t *)&firStateF32[0], BLOCK_SIZE);
	
	/* IMU init */
	LSM6DS_Reset();
	LSM6DS_GetBeing(&LSM6DSM_IMUSetting ,I2C_RxData);	//자이로 가속도 센서 활성화
		
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
				if(time_flag2%2 == 0)
					LCD_MAIN_Screen();
				break;
			case LCD_MENUSCREEN: 
				LCD_MENU_Screen();
				break;
			case LCD_PPGSCREEN:
				if(max86171_step != 0)
					LCD_PPG_Screen();
				break;
			case LCD_ECGSCREEN:
				if(max30003_step != 0)
					LCD_ECG_Screen();
				break;
			case LCD_AFSCREEN:
				LCD_AF_Screen();
				AF_Algorithm();
				break;
			case LCD_SAVESCREEN:
				if((time_flag2%10) == 0)
					LCD_SAVE_Screen();
				if(file_init)
					DATA_FIFO();
				break;
			case LCD_SERVERSCREEN:
				LCD_SERVER_Screen();
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
  htim6.Init.Prescaler = 125-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1563-1;
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

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 14, 0);
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
		//ST7789_WriteString2(1, 1, lcd_str, Font_16x26, FLIP_RGB(BLUE), FLIP_RGB(WHITE));
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

void SwitchFunction(void)
{
	if(sw_flag == 0 & btn_flag == true){ // sw_flag 0 -> 1 == when pressed
		sw_flag++;
		// screen
		if(screen_status != LCD_MENUSCREEN){
			if(left_btn){
				if(screen_status == LCD_SAVESCREEN){
					if(file_init){	FIFO_Close();}					
				}
				if(screen_status == LCD_SERVERSCREEN){
					server_layout = 0;
					memset(Rx_read_data,0,sizeof(Rx_read_data));
					if(file_init){	FIFO_Close();}
				}
				if(screen_status == LCD_ECGSCREEN || screen_status == LCD_PPGSCREEN || screen_status == LCD_AFSCREEN){
					line_x = 0;
					base_line_y = 0;
					memset(Rx_read_data,0,sizeof(Rx_read_data));
				}
				if(screen_status == LCD_AFSCREEN){
					MAX86171_adjust_clock(1,128);
					memset(newPpgDataFN,0,sizeof(newPpgDataFN));
					memset(PPI,0,sizeof(PPI));
					Filter_Idx = 0;
					fir_threshold = 0;
					AF_layout = 0;
					if(file_init)
						FIFO_Close();
				}
				time_flag2 = 0;
				screen_status = LCD_MENUSCREEN;
				LCD_Reset_Screen();
				btn_flag_reset();
			}
		}
		if(screen_status == LCD_MENUSCREEN){
			if(left_btn){
				time_flag2=1;
				screen_status = LCD_MAINSCREEN;
				LCD_Reset_Screen();
				btn_flag_reset();
			}
			if(right_btn){
				sprintf(lcd_str, "<<");
				ST7789_WriteString2(200, menu_btn_count*30 +60, lcd_str, Font_11x18, FLIP_RGB(BLACK), FLIP_RGB(BLACK));
				menu_btn_count++;
				if(menu_btn_count == 5)
					menu_btn_count = 0;
				right_btn = false;
			}
			if(ok_btn){ // status select				
				btn_flag_reset();
				switch(menu_btn_count){
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
						if(sd_cnt){
							FIFO_Init_W();							
							LCD_Reset_Screen();
							screen_flag = 0;
							time_flag2=1;
						}
						screen_status = LCD_AFSCREEN;
						MAX86171_adjust_clock(0,SAMPLE_RATE); // sample rate 50
						ECG_PPG_IMU_reset();
						break;
					case 3:
						//SAVE
						screen_status = LCD_SAVESCREEN;
						break;
					case 4:
						//SERVER
						if(!sd_cnt)
							server_layout = 99;
						else
							server_layout = 0;
						screen_status = LCD_SERVERSCREEN;
						break;
				}
				LCD_Reset_Screen();
				ok_btn = false;		
			}
		}
		else if(screen_status == LCD_SAVESCREEN){
			if(ok_btn && sd_cnt){
				LCD_Reset_Screen();
				if(file_init){
					FIFO_Close();
					time_flag2=0;
				}
				else {
					//reset
					ECG_PPG_IMU_reset();
					FIFO_Init_W();
					time_flag2=1;
				}
				ok_btn = false;
			}
		}
		else if(screen_status == LCD_SERVERSCREEN){
			if(server_layout == 0){
				if(ok_btn){
					server_layout++;
					ok_btn = false;
					LCD_Reset_Screen();
				}
				if(right_btn){
					sprintf(lcd_str, "<<");
					ST7789_WriteString2(200, server_btn_count*60 +60, lcd_str, Font_11x18, FLIP_RGB(BLACK), FLIP_RGB(BLACK));
					if(server_btn_count >= 2)
						server_btn_count = 0;
					else
						server_btn_count++;
					right_btn = false;
				}
			}
			else if(server_layout == 1){
				
			}
			else if(server_layout == 2){
				
			}
			
		}
	}
	else if(sw_flag == 1){  // sw_flag 1 == when pressing
		if(btn_flag == false){ // when button is released
			sw_flag++;
			pull_btn_time++;
			btn_pressing = 0;
		}
		else{ //pressing
			if(btn_pressing <= 65000)
				btn_pressing++;
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
	if(sw_C_flag >= 3 && screen_status == LCD_MAINSCREEN){
		Set_StandbyMod();
	}
}
void SD_Init()
{	
	if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin)))
	{
		//SD카드 초기화
		res = BSP_SD_Init();
		if(res != FR_OK){
				error_code=200;
				Error_Handler();	
		}

		//SD 마운트
		res = f_mount(&SDFatFs,"", 0);
		if(res != FR_OK){
				error_code=201;
				Error_Handler();	
		}
	}
	sd_cnt = !(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin));
}

void LCD_Init()
{
	ST7789_Init(&st7789_PFP);
	
	ST7789_DrawFilledRectangle2(&st7789_PFP, 0, 0, 240, 40, FLIP_RGB(WHITE));
	
	sprintf(lcd_str, "Loading...");
	ST7789_WriteString2(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	for(int i=0; i<500; i=i+10){ TIM9->CCR1 = i;	HAL_Delay(1);}
}
void LCD_MAIN_Screen()
{
	if(ble_cnt == false)
		ST7789_DrawImage_Flip(1, 1, 37, 37, (uint16_t *)IMG_37x37_BT_NC);
	else
		ST7789_DrawImage_Flip(1, 1, 37, 37, (uint16_t *)IMG_37x37_BT_CN);
	
	if(sd_cnt == false){
		// sd card O
		if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){ 
			HAL_Delay(1000);
			SD_Init();
		}
		ST7789_DrawImage_Flip(40, 1, 37, 37, (uint16_t *)IMG_37x37_SD_UM);
	}
	else{
		if((HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){			
			NVIC_SystemReset();
		}
		ST7789_DrawImage_Flip(40, 1, 37, 37, (uint16_t *)IMG_37x37_SD_MT);
	}
	
	if(bat_cnt == false)
		ST7789_DrawImage_Flip(210, 1, 24, 37, (uint16_t *)IMG_24x37_BAT_CAN);
	else
		ST7789_DrawImage_Flip(210, 1, 24, 37, (uint16_t *)IMG_24x37_BAT_CG);
	
	sprintf(lcd_str, "%3d%%", battery_value);
	ST7789_WriteString2(140, 10, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	
	sprintf(lcd_str, "<<");
	if(left_btn == false)
		ST7789_WriteString2(5, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString2(5, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "OK");
	if(ok_btn == false)
		ST7789_WriteString2(100, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString2(100, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, ">>");
	if(right_btn == false)
		ST7789_WriteString2(200, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString2(200, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
			
	sprintf(lcd_str, "ACC : %6d", abs(I2C_BurstData[0])+abs(I2C_BurstData[1])+abs(I2C_BurstData[2]) );
	ST7789_WriteString2(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "GYRO : %6d", I2C_BurstData[3]+I2C_BurstData[4]+I2C_BurstData[5] );
	ST7789_WriteString2(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "PPG : %6d", max86171_data[0].led);
	ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "ECG : %6d", max30003_data[0].data);
	ST7789_WriteString2(0, 90, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		
	bat_cnt = !HAL_GPIO_ReadPin(BAT_CG_POK_GPIO_Port,BAT_CG_POK_Pin);
	
}

void LCD_MENU_Screen()
{	
	sprintf(lcd_str, "<<");
	if(left_btn == false)
		ST7789_WriteString2(5, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString2(5, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "OK");
	if(ok_btn == false)
		ST7789_WriteString2(100, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString2(100, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, ">>");
	if(right_btn == false)
		ST7789_WriteString2(200, 214, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	else
		ST7789_WriteString2(200, 214, lcd_str, Font_16x26, FLIP_RGB(RED), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "View PPG signal");
	ST7789_WriteString2(0, 60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "View ECG signal");
	ST7789_WriteString2(0, 90, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "AF test");
	ST7789_WriteString2(0, 120, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "Save data");
	ST7789_WriteString2(0, 150, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
	sprintf(lcd_str, "Send data");
	ST7789_WriteString2(0, 180, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));	
	
	sprintf(lcd_str, "<<");
	ST7789_WriteString2(200, menu_btn_count*30 +60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	
}

void LCD_ECG_Screen(){
	uint8_t ECG_Rescale = 40;
	uint8_t middle_line = 180;
	
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
			LCD_Reset_Screen();
			line_x = 0;
			base_line_y = 0;
			break;
		}
	}
	max30003_step = 0;
}
void LCD_PPG_Screen(){	
	// 1 -> 0 2 -> 6 3 -> 4 4 -> 2
	int i = (10 - (2*max86171_data[0].tag))%8;
	uint8_t PPG_Rescale = 20;

	if(line_x == 0 && base_line_y == 0){ base_line_y = max86171_data[i+1].led;}
	for(;i < max86171_step;i+=8){
		line_y2 = 150+round((max86171_data[i+1].led - base_line_y)/PPG_Rescale);
			
		if(line_y1 > 240 || line_y2 > 240 || line_y1 < 40 || line_y2 < 40) { 
			base_line_y = max86171_data[i+1].led; 
			line_y1 = 150+round((max86171_data[i+1].led - base_line_y)/PPG_Rescale); 
			line_y2 = 150+round((max86171_data[i+1].led - base_line_y)/PPG_Rescale);  
		}
		
		ST7789_DrawLine(line_x,line_y1,line_x+1,line_y2,FLIP_RGB(BLUE));
		HAL_Delay(1);
		line_x++;
		line_y1 = line_y2;
		
		if(line_x > 250)	{
			LCD_Reset_Screen();
			line_x = 0;
			base_line_y = max86171_data[i+1].led;
		}
	}
	max86171_step = 0;
}

void AF_Algorithm(){
	
	// acc check
		
	
	// PPG filtering
	if(max86171_step != 0 && AF_layout == 0)
	{
		uint8_t count_86171 =0;
		int32_t max86171_input[3];
		for(int i=0;i<max86171_step;i+=2)
		{
			if(max86171_data[i].tag == 1)
			{
				max86171_buffer[0] = max86171_data[i].led;
				max86171_buffer[1] = max86171_data[i+1].led;
			}
			if(max86171_data[i].tag == 2)
			{
				max86171_buffer[2] = max86171_data[i].led;
			}
			if(max86171_data[i].tag == 3)
			{
				max86171_buffer[3] = max86171_data[i].led;
				max86171_buffer[4] = max86171_data[i+1].led;
			}
			if(max86171_data[i].tag == 4)
			{
				max86171_buffer[5] = max86171_data[i].led;
				count_86171++;
			}
			if(count_86171 == 1)
			{
				max86171_input[0] = max86171_buffer[0]-max86171_buffer[3];
				max86171_input[1] = max86171_buffer[1]-max86171_buffer[4];
				max86171_input[2] = max86171_buffer[2]-max86171_buffer[5];
				
				// data
				float32_t data = (float32_t) (max86171_input[0]+max86171_input[1]+max86171_input[2])/3;
							
				//FIFO raw
//				if(sd_cnt){
//					sprintf(SD_msg,"%f, %d\n",data);
//					f_puts(SD_msg,&PPG_fp);
//					memset(SD_msg,0,sizeof(SD_msg));
//				}				
				
				//flag reset
				if(time_flag2 >= 1000)
					time_flag2=0;
				
				newPpgDataFN[Filter_Idx] = data;				
				if (Filter_Idx >= BLOCK_SIZE)
					newPpgDataFN[Filter_Idx - BLOCK_SIZE] = data;				
				
				// filter		
				++Filter_Idx;
				if(++fir_threshold >= FILTERING_RATE && Filter_Idx >= BLOCK_SIZE){
					float32_t filtered_ppgDataF[BLOCK_SIZE]={0,};
					float32_t raw_ppgData[BLOCK_SIZE] = {0,};
					
					for(int normalize = 0; normalize < BLOCK_SIZE ;normalize++){
						raw_ppgData[normalize] = newPpgDataFN[Filter_Idx - BLOCK_SIZE + normalize] - newPpgDataFN[Filter_Idx - BLOCK_SIZE];
					}
					
					arm_fir_f32(&S, raw_ppgData, filtered_ppgDataF, BLOCK_SIZE);
					fir_threshold = 0;
					
					// save last 2 sec (screen)
					for(int j = 0;j<FILTERING_RATE;j++)
						fir_LCD_data[j] = filtered_ppgDataF[BLOCK_SIZE-FILTERING_RATE+j];
										
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
						
						AF_shannon = AF_Shannon_Entropy(PPI);
						AF_sample = AF_Sample_Entropy(PPI);
						AF_rmssd = AF_Normlaized_RMSSD(PPI);
					}
					// LCD
					AF_Show_Data();
					
				}
			// primary swap
				if (Filter_Idx >= BUFF_SIZE)
					Filter_Idx = BLOCK_SIZE;			
			}
			
		}
		max86171_step = 0;
	}
	
	if( AF_layout == 1){
		if(screen_flag == 1){
			if(max30003_step != 0)
			{
				for(int i=0;i<max30003_step;i++)
				{
					if(abs(max30003_data[i].data) > 10000){
						LCD_Reset_Screen();
						sprintf(lcd_str, "data saving...");
						ST7789_WriteString2(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
						HAL_Delay(1);
						screen_flag = 2;
						time_flag2 = 1;
						spk_switch = 0;
						break;
					}
				}
				max30003_step = 0;
			}
		}
		if(screen_flag == 2){
			max30003_step = 0;
			if(time_flag2 >= 50){	
				LCD_Reset_Screen();
				ECG_PPG_IMU_reset();
				time_flag2 = 1;
				screen_flag = 3;
				sprintf(lcd_str, "wait 30 sec");
				ST7789_WriteString2(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
			}		
		}
		else if(screen_flag == 3){
			DATA_FIFO();
			if(time_flag2 >= 300){
				AF_layout = 2;
				LCD_Reset_Screen();
				if(file_init)
					FIFO_Close();
			}
		}
	}
	else if(AF_layout == 2){
		if(strncmp(Rx_read_data,"SCS",3) == 0){
			BT_rx_flag = 0;
			AF_layout = 3;
		}
	}
	else if(AF_layout == 3){
		Data_Send();
		AF_layout = 4;
	}
	else if(AF_layout == 4){
		Directory_check();
	}
}

void LCD_AF_Screen(){
	/* View PPG FIR */
	/*
	float AF_Rescale = (float)(max_line_y-min_line_y)/(float)100;
	if(line_x == 0)
		AF_Layer_rescale(&AF_Rescale);
	
	for (int i = 0; i < FILTERING_RATE; i++){
		line_y1 = line_y2;
		line_y2 = 180 - ((fir_LCD_data[i] - base_line_y)/AF_Rescale);
		if(line_y2 < 40 || line_y2 > 200)
			AF_Layer_rescale(&AF_Rescale);
		
		ST7789_DrawLine(line_x,line_y1,line_x+1,line_y2,FLIP_RGB(BLUE));			
		line_x++;
		if(line_x > 250){			
			line_x = 0;
			LCD_Reset_Screen();
			AF_Show_Data();
		}
	}
	*/
	if(sd_cnt){
		if(AF_layout == 0) {
			dot_screen_moving("AF detecting");
			HAL_Delay(1);
		}
		if(AF_layout == 1){
			if(screen_flag == 0){
				LCD_Reset_Screen();
				spk_switch = 1;
				sprintf(lcd_str, "  AF detected  ");
				ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
				
				sprintf(lcd_str, "put your finger");
				ST7789_WriteString2(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
				
				sprintf(lcd_str, " on the metal ");
				ST7789_WriteString2(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
				
				screen_flag++;
			}
			else if(screen_flag == 2){		
				sprintf(lcd_str, "Wait Data");
				ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
				sprintf(lcd_str, "stabilization");
				ST7789_WriteString2(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
			}
			else if(screen_flag == 3){
				if((time_flag2%10) == 0){
					sprintf(lcd_str, "time : %d",(time_flag2/10));
					ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
					HAL_Delay(1);
				}
			}
		}
		if(AF_layout == 2){
			sprintf(lcd_str, "Fin AF detect");
			ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			sprintf(lcd_str, "Wait BLE cnt");
			ST7789_WriteString2(0, 150, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		if(AF_layout == 3){
			LCD_Reset_Screen();
			sprintf(lcd_str, "Sending data  ");
			ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
		if(AF_layout == 4){
			sprintf(lcd_str, "Sending fin  ");
			ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
		}
	}
	else {
		if(screen_flag == 0){
			LCD_Reset_Screen();
			sprintf(lcd_str, "Insert SD card");
			ST7789_WriteString2(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			screen_flag++;
		}
		else{
			if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){
				HAL_Delay(1000);
				SD_Init();
				FIFO_Init_W();
				LCD_Reset_Screen();
				screen_flag = 0;
				time_flag2=1;
				ECG_PPG_IMU_reset();
			}
		}
	}
	
}
void AF_Layer_rescale(float *AF_Rescale){
	LCD_Reset_Screen();
	AF_Show_Data();
	line_x = 0;
	max_line_y = fir_LCD_data[0];
	min_line_y = fir_LCD_data[0];
	for(int i =0;i<FILTERING_RATE;i++){
		if(fir_LCD_data[i] > max_line_y)
			max_line_y = fir_LCD_data[i];
		if(fir_LCD_data[i] < min_line_y)
			min_line_y = fir_LCD_data[i];
	}
	*AF_Rescale = (float)(max_line_y-min_line_y)/(float)100;
	line_y2 = 180;
	base_line_y = min_line_y;
}
void AF_Show_Data(){	
//	sprintf(lcd_str, "%d %d %d %d %d %d %d",PPI[9],PPI[10],PPI[11],PPI[12],PPI[13],PPI[14],PPI[15]);
//	ST7789_WriteString2(0, 180, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
//	HAL_Delay(1);
	sprintf(lcd_str, "shaE smpE RMSSD rri");
	ST7789_WriteString2(0, 200, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
	HAL_Delay(1);
	
	if(AF_shannon > 0.6 && AF_sample > 1.5 && AF_rmssd > 0.25){
		LCD_Reset_Screen();		
		AF_layout = 1;
		time_flag2 = 0;
	}
	else{
		sprintf(lcd_str, "%1.02f %1.02f %1.02f   %d",  AF_shannon, AF_sample, AF_rmssd,AF_rri);
		ST7789_WriteString2(0, 220, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		HAL_Delay(1);
	}
	
	acc_warning();
}

void LCD_SAVE_Screen(){	
	if(sd_cnt){
		// SD out
		if((HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){ NVIC_SystemReset();}
				
		//LCD Reset
		if(time_flag2 >= 600) {
			time_flag2=1;
			FIFO_Close();
			FIFO_Init_W();
		}
		
		if(file_init){		
			sprintf(lcd_str, "Save time : %2ds", (time_flag2/10));
			ST7789_WriteString2(0, 200, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "IMU : %6d", I2C_Stat[0] );
			ST7789_WriteString2(0, 160, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "PPG : %6d", max86171_data[0].led);
			ST7789_WriteString2(0, 130, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "ECG : %6d", max30003_data[0].data);
			ST7789_WriteString2(0, 100, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "save data");
			ST7789_WriteString2(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		} else {
			sprintf(lcd_str, "Press ok button");
			ST7789_WriteString2(40, 120, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		}
		
	}else {
		sprintf(lcd_str, "Insert SD card");
		ST7789_WriteString2(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));		
		if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){
			HAL_Delay(1000);
			SD_Init();
			FIFO_Init_W();
			LCD_Reset_Screen();
			time_flag2++;
			ECG_PPG_IMU_reset();
		}
	}
}
void LCD_SERVER_Screen(){
	if(sd_cnt){
		if(server_layout == 0){			
			sprintf(lcd_str, "Send last 1 data");
			ST7789_WriteString2(0, 60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "Send last 2 data");
			ST7789_WriteString2(0, 120, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			
			sprintf(lcd_str, "Send whole data");
			ST7789_WriteString2(0, 180, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));	
			
			sprintf(lcd_str, "<<");
			ST7789_WriteString2(200, server_btn_count*60 +60, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		}
		else if(server_layout == 1){
			switch(server_btn_count){
				case 0 : 
					sprintf(lcd_str, "Send last 1 data");
					break;
				case 1 : 
					sprintf(lcd_str, "Send last 2 data");
					break;
				case 2 : 
					sprintf(lcd_str, "Send whole data");
					break;
				default : 
					sprintf(lcd_str, "Error???");
					break;
			}
			ST7789_WriteString2(30, 60, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
						
			dot_screen_moving("looking for BLE");
			
			if(strncmp(Rx_read_data,"SCS",3) == 0){
				BT_rx_flag = 0;
				server_layout = 2;
				LCD_Reset_Screen();
				sprintf(lcd_str, "Connected");
				ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
				sprintf(lcd_str, "Send Data...");
				ST7789_WriteString2(0, 180, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
				HAL_Delay(1);
			}
		}
		else if(server_layout == 2){
			Directory_check();
			Data_Send();
			server_layout = 3;
		}
		else if(server_layout == 3){
			LCD_Reset_Screen();
			sprintf(lcd_str, "Finished");
			ST7789_WriteString2(0, 120, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			HAL_Delay(1);
			server_layout = 4;
		}
	}
	else {
		sprintf(lcd_str, "Insert SD card");
		ST7789_WriteString2(10, 50, lcd_str, Font_16x26, FLIP_RGB(WHITE), FLIP_RGB(BLACK));		
		if(!(HAL_GPIO_ReadPin(SD_SW_GPIO_Port,SD_SW_Pin))){
			HAL_Delay(1000);
			SD_Init();
			LCD_Reset_Screen();
			server_layout = 0;
		}
	}
}
void LCD_Reset_Screen(){
	ST7789_DrawFilledRectangle2(&st7789_PFP, 0, 0, 240, 240, FLIP_RGB(BLACK));
	ST7789_DrawFilledRectangle2(&st7789_PFP, 0, 0, 240, 40, FLIP_RGB(WHITE));
	battery_value=MAX17262_Get_BatPercent();
	
	if(screen_status == LCD_MENUSCREEN){
		sprintf(lcd_str, "Select action");
		ST7789_WriteString2(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_PPGSCREEN){
		sprintf(lcd_str, "PPG Signal");
		ST7789_WriteString2(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_ECGSCREEN){
		sprintf(lcd_str, "ECG Signal");
		ST7789_WriteString2(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_AFSCREEN){
		sprintf(lcd_str, "AF Detect");
		ST7789_WriteString2(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_SAVESCREEN){
		sprintf(lcd_str, "SAVE Signal");
		ST7789_WriteString2(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
	if(screen_status == LCD_SERVERSCREEN){
		sprintf(lcd_str, "Send to Server");
		ST7789_WriteString2(10, 7, lcd_str, Font_16x26, FLIP_RGB(BLACK), FLIP_RGB(WHITE));
	}
}

void DATA_FIFO(){
	if(sd_cnt == true){
		if(max86171_step != 0)
		{
			uint8_t count_86171 =0;
			int32_t max86171_input[3];
			for(int i=0;i<max86171_step;i+=2)
			{
				if(max86171_data[i].tag == 1)
				{
					max86171_buffer[0] = max86171_data[i].led;
					max86171_buffer[1] = max86171_data[i+1].led;
				}
				if(max86171_data[i].tag == 2)
				{
					max86171_buffer[2] = max86171_data[i].led;
				}
				if(max86171_data[i].tag == 3)
				{
					max86171_buffer[3] = max86171_data[i].led;
					max86171_buffer[4] = max86171_data[i+1].led;
				}
				if(max86171_data[i].tag == 4)
				{
					max86171_buffer[5] = max86171_data[i].led;
					count_86171++;
				}
				if(count_86171 == 1)
				{
					max86171_input[0] = max86171_buffer[0]-max86171_buffer[3];
					max86171_input[1] = max86171_buffer[1]-max86171_buffer[4];
					max86171_input[2] = max86171_buffer[2]-max86171_buffer[5];
					
					sprintf(SD_msg,"%d,%d,%d\n",max86171_input[0],max86171_input[1],max86171_input[2]);
					f_puts(SD_msg,&PPG_fp);
					memset(SD_msg,0,sizeof(SD_msg));				
				}
			}
			max86171_step = 0;
		}		
		if(max30003_step != 0)
		{
			for(int i=0;i<max30003_step;i++)
			{
				if(max30003_data[i].data > 0)
					max30003_data[i].data = max30003_data[i].data*-1;
				sprintf(SD_msg,"%d\n",max30003_data[i].data); //18bits int -> uint = 131072
				f_puts(SD_msg,&ECG_fp);
				memset(SD_msg,0,sizeof(SD_msg));
			}
			max30003_step = 0;
		}
		if(I2C_Stat[0] & 0x07ff){
			for(int i=0;i<I2C_Stat[0]/6;i++){
				sprintf(SD_msg,"%d,%d,%d,%d,%d,%d\n",I2C_BurstData[0+(i*6)],I2C_BurstData[1+(i*6)],I2C_BurstData[2+(i*6)],I2C_BurstData[3+(i*6)],I2C_BurstData[4+(i*6)],I2C_BurstData[5+(i*6)]);
				f_puts(SD_msg,&IMU_fp);
				memset(SD_msg,0,sizeof(SD_msg));
			}
		}
		I2C_Stat[0] = 0;		
	}
}

void FIFO_Init_W(){	
	//f_open 파일 열기	
	if(sd_cnt == true){
		file_init = true;
		if(file_count == 0)
			Directory_check();
		else
			file_count++;
		
		sprintf(SD_msg,"/data/ECG_%d.csv",file_count);
		res = f_open(&ECG_fp, SD_msg, FA_OPEN_ALWAYS | FA_WRITE );
		if(res != FR_OK){
			error_code=202;
			Error_Handler();
		}
		memset(SD_msg,0,sizeof(SD_msg));
		sprintf(SD_msg,"\n");
		f_puts(SD_msg,&ECG_fp);
		memset(SD_msg,0,sizeof(SD_msg));
		
		sprintf(SD_msg,"/data/PPG_%d.csv",file_count);
		res = f_open(&PPG_fp, SD_msg, FA_OPEN_ALWAYS | FA_WRITE);
		if(res != FR_OK){
			error_code=202;
			Error_Handler();
		}
		memset(SD_msg,0,sizeof(SD_msg));
		sprintf(SD_msg,"\n");
		f_puts(SD_msg,&PPG_fp);
		memset(SD_msg,0,sizeof(SD_msg));
		
		sprintf(SD_msg,"/data/IMU_%d.csv",file_count);
		res = f_open(&IMU_fp, SD_msg, FA_OPEN_ALWAYS | FA_WRITE);
		if(res != FR_OK){
			error_code=202;
			Error_Handler();
		}
		memset(SD_msg,0,sizeof(SD_msg));
		sprintf(SD_msg,"\n");
		f_puts(SD_msg,&IMU_fp);
		memset(SD_msg,0,sizeof(SD_msg));
	}
}

void FIFO_Init_R(uint8_t file_number){
	//f_open 파일 열기	
	if(sd_cnt){
		file_init = true;
		
		if(screen_status == LCD_SERVERSCREEN){
			sprintf(SD_msg,"/OLD%d/ECG_%d.csv",folder_count,file_number);
			res = f_open(&ECG_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
			if(res != FR_OK){
				error_code=202;
				Error_Handler();
			}
			memset(SD_msg,0,sizeof(SD_msg));
			
			sprintf(SD_msg,"/OLD%d/PPG_%d.csv",folder_count,file_number);
			res = f_open(&PPG_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
			if(res != FR_OK){
				error_code=202;
				Error_Handler();
			}
			memset(SD_msg,0,sizeof(SD_msg));
			
			sprintf(SD_msg,"/OLD%d/IMU_%d.csv",folder_count,file_number);
			res = f_open(&IMU_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
			if(res != FR_OK){
				error_code=202;
				Error_Handler();
			}
			memset(SD_msg,0,sizeof(SD_msg));
		}
		else if(screen_status == LCD_AFSCREEN){
			sprintf(SD_msg,"/data/ECG_1.csv");
			res = f_open(&ECG_fp, SD_msg, FA_OPEN_ALWAYS | FA_READ );
			if(res != FR_OK){
				error_code=202;
				Error_Handler();
			}
			memset(SD_msg,0,sizeof(SD_msg));
		}
		
	}
	
}

void FIFO_Close(){
	file_init = false;
	res = f_close(&ECG_fp);
	if(res != FR_OK){ error_code=204; Error_Handler();}
	if(screen_status != LCD_AFSCREEN){
		res = f_close(&PPG_fp);
		if(res != FR_OK){ error_code=204; Error_Handler();}
		res = f_close(&IMU_fp);
		if(res != FR_OK){ error_code=204; Error_Handler();}
	}
}

void Data_Update(){
	if(screen_status == LCD_PPGSCREEN || screen_status == LCD_MAINSCREEN || screen_status == LCD_SAVESCREEN || screen_status == LCD_AFSCREEN){		
		PPG_Update();
	}
		
	if(screen_status == LCD_ECGSCREEN || screen_status == LCD_MAINSCREEN || screen_status == LCD_SAVESCREEN || AF_layout == 1){
		ECG_Update();
	}
		
	if(screen_status == LCD_MAINSCREEN || screen_status == LCD_SAVESCREEN || AF_layout == 1){
		IMU_Update();
	}
	
}

void Data_Send(void){
	if(screen_status == LCD_AFSCREEN){
		Folder_Read_Write(0);
	}
	else if(screen_status == LCD_SERVERSCREEN){
		if(server_btn_count == 0){
			Folder_Read_Write(folder_count);
		}
		if(server_btn_count == 1){
			Folder_Read_Write(folder_count-1);
			Folder_Read_Write(folder_count);
		}
		if(server_btn_count == 2){
			for(uint8_t i=1;i<=folder_count;i++){
				Folder_Read_Write(i);
			}
		}
	}
}
void Folder_Read_Write(uint8_t folder_N){
	char read_str[10];
		
	//open dir
	if(screen_status == LCD_SERVERSCREEN){
		sprintf(read_str,"/OLD%d",folder_N);
		res = f_opendir(&dir,read_str);
		if(res != FR_OK){
			error_code=205;
			Error_Handler();
		}
	}
	else if(screen_status == LCD_AFSCREEN){
		sprintf(read_str,"data");
		res = f_opendir(&dir,read_str);
		if(res != FR_OK){
			error_code=205;
			Error_Handler();
		}
	}
	
	//read dir
	memset(&fno, 0, sizeof(FILINFO));
	res = f_readdir(&dir, &fno);
	if(res != FR_OK){
		error_code = 205;
		Error_Handler();
	}
	
	if(fno.fattrib== AM_DIR || fno.fattrib== AM_ARC)// file or directory o
	{
		if(screen_status == LCD_SERVERSCREEN){
			while(true){
				f_findnext(&dir,&fno);
				if(fno.fname[0] == NULL){
					char *file_tmp = strtok(fno.fname+1,".");
					char *file_ptr = strchr(file_tmp,'_')+1;
					
					// find the number of files
					file_count = 0;
					for(int16_t ii = strlen(file_ptr)-1;ii>=0;ii--){
						file_count += (file_ptr[ii] - '0')*pow(10,strlen(file_ptr)-ii-1);
					}				
					
					// read & send whole files
					for(int16_t ii = 1;ii<=file_count;ii++){
						FIFO_Init_R(ii);
						uint8_t read_buf[50];
						
						printf("E");
						while(f_gets(read_buf,sizeof(read_buf),&ECG_fp)){
							HAL_IWDG_Refresh(&hiwdg);
							printf("%s",read_buf);
						}
						printf("F");
						
						printf("P");
						while(f_gets(read_buf,sizeof(read_buf),&PPG_fp)){
							HAL_IWDG_Refresh(&hiwdg);
							printf("%s",read_buf);
						}
						printf("F");
						
						printf("I");
						while(f_gets(read_buf,sizeof(read_buf),&IMU_fp)){
							HAL_IWDG_Refresh(&hiwdg);
							printf("%s",read_buf);
						}
						printf("F");
						
						FIFO_Close();
					}
					break;
				}
			}
		}
		else if(screen_status == LCD_AFSCREEN){
			FIFO_Init_R(0);
			uint8_t read_buf[20];
			
			printf("E\n");
			while(f_gets(read_buf,sizeof(read_buf),&ECG_fp)){
				HAL_IWDG_Refresh(&hiwdg);
				printf("%s",read_buf);
			}
			printf("F\n");
			HAL_Delay(1000);
			
			FIFO_Close();
		}
	}
	else {
		
	}
	//find file
	
}
void PPG_Update(void){
	
	max86171_stat = MAX86171_readReg(PPG_STAT_1);
	if((max86171_stat & 0xC0)) { //128bits interrupt
		max86171_step = MAX86171_read_fifo(max86171_data); // 128fps		
	}
	if(max86171_step == 256){
		ECG_PPG_IMU_reset();
		max86171_step = 0;
	}
	
}

void ECG_Update(void){
	
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

void IMU_Update(void){
	
	LSM6DS_FIFO_DATA_NUM(I2C_Stat);
	if(I2C_Stat[0] & 0x6000){
		spk_switch = 1;
		ECG_PPG_IMU_reset();
	}
	else if(I2C_Stat[0] & 0x07ff)	{
		I2C_Stat[0] = (int16_t)(I2C_Stat[0] & 0x07ff);	
		if(I2C_Stat[0] >= 240)
			LSM6DS_FIFO_DATA_OUT(I2C_BurstData, 240);
		else
			LSM6DS_FIFO_DATA_OUT(I2C_BurstData, I2C_Stat[0]);
		
	}
	else
	{
		I2C_Stat[0] = (int16_t)(I2C_Stat[0] & 0x07ff);	
	}
	
}

void Directory_check(void)
{
	uint8_t data_flag = 0;
	
	// data folder exist?
	res = f_mkdir("/data");
	if(res != FR_OK && res != FR_EXIST){
		error_code=205;
		Error_Handler();
	}
	
	res = f_opendir(&dir, "/data");
	if(res != FR_OK){
		if(f_mkdir("/data") != FR_OK){
			error_code=205;
			Error_Handler();
		}
	}
							
	memset(&fno, 0, sizeof(FILINFO));
	res = f_readdir(&dir, &fno);
	if(res != FR_OK){
		error_code = 205;
		Error_Handler();
	}	
	if(fno.fattrib== AM_DIR || fno.fattrib== AM_ARC){ data_flag = 2; }
	else if(fno.fattrib == 0x00){	data_flag = 1; }
	else {
		error_code=205;
		Error_Handler();
	}
	f_closedir(&dir);
	
	// find old directory
	f_opendir(&dir,"/");
	while(true){
		memset(&fno, 0, sizeof(FILINFO));
		f_findnext(&dir,&fno);
		if(fno.fname[0] == NULL){
			// dir does not exist
			strncpy(fno.fname,"O",1);
			if(strstr(fno.fname,"OLD")==NULL) {					
				res = f_rename("/data","/OLD1");
				if(f_mkdir("data") != FR_OK || res != FR_OK){
					error_code=205;
					Error_Handler();
				}
			}
			else {
				//find old dir name
				char *oldname = strtok(fno.fname," ");
				char *dataPath = malloc((sizeof(oldname)+2));
				uint8_t str_old = strlen(oldname);
				
				folder_count = 0;
				for(uint8_t ii = strlen(oldname)-1;ii>2;ii--){
					folder_count += (oldname[ii] - '0')*pow(10,strlen(oldname)-ii-1);
				}
				
				do{
					folder_count++;
					
					memset(dataPath,0,sizeof(dataPath));
					sprintf(dataPath,"/OLD%d",folder_count);
					
					res = f_rename("/data",dataPath);
				} while(res == FR_EXIST);
				
				if(res != FR_OK){
					error_code=205;
					Error_Handler();				
				}
				
				if(data_flag == 2){
					res = f_mkdir("data");
					if(res != FR_OK){
						error_code=205;
						Error_Handler();
					}
				}
				else if(data_flag == 1) {
					folder_count--;
					res = f_rename(dataPath,"/data");
					if(res != FR_OK){
						error_code=205;
						Error_Handler();
					}
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
	
}
void btn_flag_reset(void){
	left_btn = false;
	sw_flag = 0;
	pull_btn_time = 0;
	sw_L_flag = 0;
	sw_R_flag = 0;
	sw_C_flag = 0;
}

void ECG_PPG_IMU_reset(void){
	max30003_Synch();
	LSM6DS_Reset();
	MAX86171_writeReg(PPG_FIFO_CONFIG_2, 0x1E);
	if(screen_status == LCD_AFSCREEN && AF_layout == 0)
		LSM6DS_GetBeing_polling(&LSM6DSM_IMUSetting ,I2C_RxData);
	else
		LSM6DS_GetBeing(&LSM6DSM_IMUSetting ,I2C_RxData);
}
void dot_screen_moving(char dot_msg[]){
	
	strcpy(lcd_str,dot_msg);
	switch(screen_dot){
		case 0 : 
			ST7789_WriteString2(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
		case 500 : 
			strcat(lcd_str,".");
			ST7789_WriteString2(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
		case 1000 : 
			strcat(lcd_str,"..");
			ST7789_WriteString2(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
		case 1500 : 
			strcat(lcd_str,"...");
			ST7789_WriteString2(30, 120, lcd_str, Font_11x18 , FLIP_RGB(WHITE), FLIP_RGB(BLACK));
			break;
	}
	
	if(screen_dot >= 2000){
		ST7789_DrawFilledRectangle2(&st7789_PFP, 0, 120, 240, 30, FLIP_RGB(BLACK));
		screen_dot = 0;
	}
	else
		screen_dot++;
}
void acc_warning(void){	
	int16_t IMU_data = 0;
	LSM6DS_GetGyroDataX(I2C_polling_data);
	IMU_data += abs(I2C_polling_data[0]);
	LSM6DS_GetGyroDataY(I2C_polling_data);
	IMU_data += abs(I2C_polling_data[0]);
	LSM6DS_GetGyroDataZ(I2C_polling_data);
	IMU_data += abs(I2C_polling_data[0]);	
	
	if(IMU_data > 2000){
		sprintf(lcd_str, "  ACC : %5d ",IMU_data);
		ST7789_WriteString2(50, 170, lcd_str, Font_11x18, FLIP_RGB(RED), FLIP_RGB(BLACK));
		HAL_Delay(1);
	}
	else{
		sprintf(lcd_str, "  ACC : %5d ",IMU_data);
		ST7789_WriteString2(50, 170, lcd_str, Font_11x18, FLIP_RGB(WHITE), FLIP_RGB(BLACK));
		HAL_Delay(1);
	}
}
void UF_speaker_ON(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	TIM3->ARR=250;
	//TIM2->CCR2=(int)(325/2);
	TIM3->CCR4=(int)(200/2);
}

void UF_speaker_OFF(void)
{
	TIM3->CCR4=0;
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}	

void Set_StandbyMod(void)
{
	while(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin));
	HAL_Delay(1000);
	HAL_GPIO_WritePin(BLE_MOD_GPIO_Port, BLE_MOD_Pin, GPIO_PIN_RESET);
  printf("AT+SLEEP");
   
  HAL_GPIO_WritePin(BLE_TX_GPIO_Port, BLE_TX_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLE_RX_GPIO_Port, BLE_RX_Pin, GPIO_PIN_RESET);

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
	
	HAL_GPIO_WritePin(VPP_EN_GPIO_Port,VPP_EN_Pin,GPIO_PIN_RESET);
	
	__HAL_RCC_CLEAR_RESET_FLAGS();
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
	
	if(htim -> Instance == TIM6) // 127.96 Hz
	{
	}
	else if(htim->Instance == TIM7) // 10Hz
	{
		if(time_flag2 != 0)
			time_flag2++;
		if(pull_btn_time != 0)
			pull_btn_time++;
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
	switch (error_code) {
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
	ST7789_WriteString2(10, 100, error_msg, Font_16x26, FLIP_RGB(GREEN), FLIP_RGB(BLACK));
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
