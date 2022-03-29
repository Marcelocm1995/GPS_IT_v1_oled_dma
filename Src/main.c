/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
	
	//https://stm32withoutfear.blogspot.com/2018/04/stm32-oled-display-ssd1306-i2c.html
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ssd1306.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define gps_serial &huart1
#define pc_serial &huart2
#define COMMA 0X2C //VIRGULA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char GPS_BAUD_115200[] = "$PMTK251,115200*1F";

char ONLY_GPRMC[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29";
char GPRMC_GPGSA[] = "$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28";

char HOT_START[] = "$PMTK101*32";
char WARM_START[] = "$PMTK102*31";
char COLD_START[] = "$PMTK103*30";

char GPS_2s[]   = "$PMTK300,2000,0,0,0,0*1F";
char GPS_1Hz[]  = "$PMTK300,1000,0,0,0,0*1C";
char GPS_2Hz[]  = "$PMTK300,500,0,0,0,0*28";
char GPS_5Hz[]  = "$PMTK300,200,0,0,0,0*2F";
char GPS_10Hz[] = "$PMTK300,100,0,0,0,0*2C";

char CRLF[2] = {0x0D,0x0A};

char NMEA_type[6];
char NMEA_time[12];
char NMEA_fix[2];
char NMEA_latitude[10];
char NMEA_latitude_pole[2];
char NMEA_longitude[10];
char NMEA_longitude_side[2];
char NMEA_speed[8];
char NMEA_course[8];
char NMEA_date[8];
char NMEA_variation[8];
char NMEA_variation_side[2];

char rawHOUR[2];
char rawMINUTE[2];
char rawSECOND[6];

int HOUR;
int MINUTE;
float SECOND;
float START_TIME_0_100 = 0;
float END_TIME_0_100 = 0;
float TOTAL_TIME_0_100 = 0;
int DRAG = 0;

char OLD_NMEA_latitude[10];
char OLD_NMEA_longitude[10];
char OLD_NMEA_latitude_pole[2];
char OLD_NMEA_longitude_side[2];

int NMEA_type_length;
int NMEA_time_length;
int NMEA_fix_length;
int NMEA_latitude_length;
int NMEA_latitude_pole_length;
int NMEA_longitude_length;
int NMEA_longitude_side_length;
int NMEA_speed_length;
int NMEA_course_length;
int NMEA_date_length;
int NMEA_variation_length;
int NMEA_variation_side_length;

char myNMEA[80];//  

char Rx_data[80];
char Tx_data[80];

char strbff[30];

unsigned int i = 0;

int TIMER_MS;
int k = 0;
int j = 0;
int result;
int NMEA_length = 0;
int COMMA_COUNT = 0;
int NMEA_length_pause = 0;

int READY_MESSAGE = 0;

float DISTANCE_METERS = 0.0;
float DISTANCE_KM = 0.0;

__IO uint32_t received_char;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void Configure_USART(void);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);
float distance_between (float lat1, float long1, float lat2, float long2);
float radians(float val);
float GpsToDecimalDegrees(const char* nmeaPos, char quadrant);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	
char *position_ptr;

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	ssd1306_Init();
  ssd1306_FlipScreenVertically();
  ssd1306_Clear();
  ssd1306_SetColor(White);
	ssd1306_SetCursor (2,0);
	ssd1306_WriteString( "GPS" , Font_16x26);
	ssd1306_UpdateScreen();
	

//	HAL_UART_Transmit(gps_serial, (uint8_t *)ONLY_GPRMC, strlen(ONLY_GPRMC),300);
//	HAL_UART_Transmit(gps_serial, (uint8_t *)CRLF, 2,10);
//	HAL_Delay(200);
	
	HAL_UART_Transmit(gps_serial, (uint8_t *)GPS_BAUD_115200, strlen(GPS_BAUD_115200),300);
	HAL_UART_Transmit(gps_serial, (uint8_t *)CRLF, 2,10);
	HAL_Delay(200);
	/* Configure USARTx (USART IP configuration and related GPIO initialization) */
  Configure_USART();
	HAL_Delay(200);
		
	HAL_UART_Transmit(gps_serial, (uint8_t *)GPRMC_GPGSA, strlen(GPRMC_GPGSA),300);
	HAL_UART_Transmit(gps_serial, (uint8_t *)CRLF, 2,10);
	HAL_Delay(200);
	
	HAL_UART_Transmit(gps_serial, (uint8_t *)GPS_10Hz, strlen(GPS_10Hz),300);
	HAL_UART_Transmit(gps_serial, (uint8_t *)CRLF, 2,10);
	HAL_Delay(200);
	
//	HAL_UART_Transmit(gps_serial, (uint8_t *)HOT_START, strlen(HOT_START),300);
//	HAL_UART_Transmit(gps_serial, (uint8_t *)CRLF, 2,10);
//	HAL_Delay(200);		
			
	ssd1306_Clear();
	
	/* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
  LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(READY_MESSAGE == 1)
		{
			//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			
			memcpy(myNMEA, Rx_data, 80);
			memset(Rx_data,0x00,80);
			
			if((myNMEA[1]=='G') && (myNMEA[2]=='P') && (myNMEA[3]=='R') && (myNMEA[4]=='M') && (myNMEA[5]=='C'))
			{
				HAL_TIM_Base_Stop(&htim1);
				TIMER_MS =__HAL_TIM_GetCounter(&htim1);
				__HAL_TIM_SetCounter(&htim1, 0);
				HAL_TIM_Base_Start(&htim1);
				result = 1;
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			}
			
			else
			{
				result = 0;
				memset(myNMEA,0x00,80);
			}
			
			//Example $GPRMC,151802.962,A,2338.944,S,04639.370,W,208.0,219.1,240320,000.0,W*7E
			//vector  0123456789
			if(result)
			{	
				memcpy(OLD_NMEA_latitude, NMEA_latitude, sizeof(NMEA_latitude));
				memcpy(OLD_NMEA_longitude, NMEA_longitude, sizeof(NMEA_longitude));				
				memcpy(OLD_NMEA_latitude_pole, NMEA_latitude_pole, sizeof(NMEA_latitude_pole));
				memcpy(OLD_NMEA_longitude_side, NMEA_longitude_side, sizeof(NMEA_longitude_side));
				
				//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				NMEA_length_pause = 7;
				j = 7;
				while(myNMEA[j] != COMMA)
				{
					NMEA_time[j-NMEA_length_pause] = myNMEA[j];
					j++;
				}
				NMEA_time_length = j - NMEA_length_pause  - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_fix[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_fix_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_latitude[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_latitude_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_latitude_pole[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_latitude_pole_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_longitude[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_longitude_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_longitude_side[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_longitude_side_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_speed[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_speed_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_course[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_course_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_date[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_date_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != COMMA)
				{
					NMEA_variation[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_variation_length = j - NMEA_length_pause - 1;
				NMEA_length_pause = j;
				
				j++;
			
				while(myNMEA[j] != '*')
				{
					NMEA_variation_side[j-NMEA_length_pause-1] = myNMEA[j];
					j++;
				}
				NMEA_variation_side_length = j - NMEA_length_pause - 1;
			}
			
			j=0;
			k=0;		
			result = 0;	
			NMEA_length_pause = 0;
			//memset(myNMEA,0x00,80);
			
						
			int SPEED = (atoi(NMEA_speed)) * 1.852;
			sprintf(strbff, "%03iKm/h ", SPEED);
			ssd1306_SetCursor (2,0);
			ssd1306_WriteString (strbff, Font_16x26);
			
			rawHOUR[0] = NMEA_time[0];
			rawHOUR[1] = NMEA_time[1];
			HOUR = atoi(rawHOUR);
			HOUR = HOUR - 3;
			if(HOUR < 0)
				HOUR += 24;
			sprintf(strbff, "%02i", HOUR);
			
			ssd1306_SetCursor (2,25);
			ssd1306_WriteString (strbff, Font_7x10);
			
			//ssd1306_SetCursor (2,30);		
			//ssd1306_WriteChar (NMEA_time[0], Font_7x10);ssd1306_WriteChar (NMEA_time[1], Font_7x10); 
			ssd1306_WriteChar (0x3A, Font_7x10);
			ssd1306_WriteChar (NMEA_time[2], Font_7x10);ssd1306_WriteChar (NMEA_time[3], Font_7x10); //MINUTES
			ssd1306_WriteChar (0x3A, Font_7x10);
			ssd1306_WriteChar (NMEA_time[4], Font_7x10);ssd1306_WriteChar (NMEA_time[5], Font_7x10); //SECONDS
			ssd1306_WriteChar (NMEA_time[6], Font_7x10);ssd1306_WriteChar (NMEA_time[7], Font_7x10); //MILISSECOND
			
			if((NMEA_fix[0] == 'A') && (SPEED > 0))
			{
				float lat1  = GpsToDecimalDegrees(OLD_NMEA_latitude,  OLD_NMEA_latitude_pole[0]);
				float long1 = GpsToDecimalDegrees(OLD_NMEA_longitude, OLD_NMEA_longitude_side[0]);
				float lat2  = GpsToDecimalDegrees(NMEA_latitude,  NMEA_latitude_pole[0]);
				float long2 = GpsToDecimalDegrees(NMEA_longitude, NMEA_longitude_side[0]);
    
				float delta = distance_between(lat1, long1, lat2, long2);
				DISTANCE_METERS = DISTANCE_METERS + delta;
				DISTANCE_KM = DISTANCE_METERS / 1000.0;
			}
			if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0)
			{
				DISTANCE_METERS = 0;
				DISTANCE_KM = 0;
			}
			
			if((SPEED == 0) && (DRAG == 0))
			{
				rawSECOND[0] = NMEA_time[4]; //second
				rawSECOND[1] = NMEA_time[5]; //second
				rawSECOND[2] = NMEA_time[6]; //dot
				rawSECOND[3] = NMEA_time[7]; //milissecond
				START_TIME_0_100 = atof(rawSECOND);
				DRAG = 1;
			}
			
			if((SPEED >= 100) && (DRAG == 1))
			{
				rawSECOND[0] = NMEA_time[4]; //second
				rawSECOND[1] = NMEA_time[5]; //second
				rawSECOND[2] = NMEA_time[6]; //dot
				rawSECOND[3] = NMEA_time[7]; //milissecond
				END_TIME_0_100 = atof(rawSECOND);
				DRAG = 0;
				TOTAL_TIME_0_100 = END_TIME_0_100 - START_TIME_0_100;
				
			}
			sprintf(strbff, "0-100: %04.1f", TOTAL_TIME_0_100);
			ssd1306_SetCursor (2,38);
			ssd1306_WriteString(strbff, Font_7x10);
			
			memset(strbff,0x00,30);
			sprintf(strbff, "D %07.2fKm   ", DISTANCE_KM);
			ssd1306_SetCursor (30,53);
			ssd1306_WriteString(strbff, Font_7x10);
			
			ssd1306_SetCursor (2,53);
			ssd1306_WriteChar(NMEA_fix[0], Font_7x10);
			ssd1306_UpdateScreen ();
			
			READY_MESSAGE = 0;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
						
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6400-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

float GpsToDecimalDegrees(const char* nmeaPos, char quadrant)
{
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant=='W' || quadrant=='S')
      v= -v;
  }
  return v;
}

float radians(float val)
{
    float radians_val = (val * 71) / 4068;
    return radians_val;
    //float rad = val * 1000.0 / 57296.0;
    //return rad;
}

float distance_between (float lat1, float long1, float lat2, float long2)
{	
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = delta * delta; 
  delta += ((clat2 * sdlong)*(clat2 * sdlong)); 
  delta = sqrt(delta); 
  float denom = ((slat1 * slat2) + (clat1 * clat2 * cdlong)); 
  delta = atan2(delta, denom); 
  delta =  delta * 6372795;
    
  return delta;
}

void USART_CharReception_Callback(void)
{

  /* Read Received character. RXNE flag is cleared by reading of DR register */
  Rx_data[i] = LL_USART_ReceiveData8(USART1);
	
	/* Echo received character on TX */
  LL_USART_TransmitData8(USART2, Rx_data[i]);
	
	if(Rx_data[i] == 0x0A)
	{
		i = 0;
		READY_MESSAGE = 1;
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		i++;
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}
  
}

void Error_Callback(void)
{
  __IO uint32_t sr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART SR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  sr_reg = LL_USART_ReadReg(USARTx_INSTANCE, SR);
  if (sr_reg & LL_USART_SR_NE)
  {
    /* case Noise Error flag is raised : ... */
    //LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    //LED_Blinking(LED_BLINK_ERROR);
  }
}

void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  
    LL_mDelay(Period);
  }
}

void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);
}

void Configure_USART(void)
{

  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  USARTx_GPIO_CLK_ENABLE();

  /* Enable USART peripheral clock *******************************************/
  USARTx_CLK_ENABLE();

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Input Floating function, High Speed, Pull up */
  LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_FLOATING);
  LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USARTx_IRQn, 0);  
  NVIC_EnableIRQ(USARTx_IRQn);

  /* (3) Configure USART functional parameters ********************************/
  
  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

  /* Set Baudrate to 115200 using APB frequency set to 72000000/APB_Div Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 72000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
  */
  LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, 115200); 

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USARTx_INSTANCE);

  /* Enable RXNE and Error interrupts */
  //LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
  //LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
