/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define OLED_DISPLAY
#define USE_EEPROM
//#define FLOAT_CONVERSION

#define NUMBER_OF_MEASURE 48 //2days if 1 hour

#ifdef OLED_DISPLAY
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <math.h>
#endif
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_SIZE_BYTES     8192
#define EEPROM_MEASURECOUNTER 		DATA_EEPROM_START_ADDR
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t ADC_buf[6] = {9,9,9,9,9};
uint32_t ADC_VSolar,ADC_Vreg,ADC_Vref,ADC_Temp,ADC_Soil;
bool measureADCDone;
//uint8_t MeasADCindex =0;
bool SleepOccurs;
uint32_t MeasureCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void ftoa(float n, char *res, int afterpoint);
void UpdateDisplay();
void GotoStandby();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	if(hadc->Instance == ADC1)
	{

		//for L0
		//IN4 : Pin A3 Soil
		//IN5 : Pin A4
		//IN6 : Pin A5
		//Temp
		//Vref
		ADC_Soil = ADC_buf[0];
		ADC_VSolar = ADC_buf[1];
		ADC_Vreg = ADC_buf[2];
		ADC_Temp = ADC_buf[2];
		ADC_Vref = ADC_buf[3];

		measureADCDone = true;




	}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  measureADCDone = false;




  /* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
	{
	  /* Calibration Error */
	  Error_Handler();
	}



  HAL_GPIO_WritePin(LCD_VCC_D2_GPIO_Port, LCD_VCC_D2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(HUM_SENSOR_VCC_A2_GPIO_Port, HUM_SENSOR_VCC_A2_Pin, GPIO_PIN_SET);

#ifdef OLED_DISPLAY
  ssd1306_Init();
  HAL_Delay(100);
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  HAL_Delay(100);
  ssd1306_SetCursor(2,2);
#endif
  /* Check and handle if the system was resumed from StandBy mode */
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
	/* Clear Standby flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
#ifdef OLED_DISPLAY
	ssd1306_WriteString("Wake up",Font_7x10,White);
#endif
	measureADCDone = false;
	SleepOccurs = true;


	//if wake up increment measure counter and store it in EEPROM
//	MeasureCounter = *(uint32_t*) (DATA_EEPROM_START_ADDR);
//	MeasureCounter ++;
//	MeasureCounter = MeasureCounter % NUMBER_OF_MEASURE;
//#ifdef USE_EEPROM
//	if (HAL_GPIO_ReadPin(WRITE_EEPROM_D3_GPIO_Port,WRITE_EEPROM_D3_Pin))
//	{
//		HAL_FLASHEx_DATAEEPROM_Unlock();
//		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000, MeasureCounter);
//		HAL_FLASHEx_DATAEEPROM_Lock();
//	}
//#endif

  }
  else
  {
#ifdef OLED_DISPLAY
	  ssd1306_WriteString("First Init",Font_7x10,White);
#endif
	  SleepOccurs = false;
	  //MeasureCounter = 0 ;
#ifdef USE_EEPROM
	  if (HAL_GPIO_ReadPin(WRITE_EEPROM_D3_GPIO_Port,WRITE_EEPROM_D3_Pin))
	  	{

		ssd1306_SetCursor(0,13);
		ssd1306_WriteString("ERASE EEPROM",Font_7x10,White);

		  //Erase all Data in EEPROM
		  HAL_FLASHEx_DATAEEPROM_Unlock();
		  for (int i = 0; i < (NUMBER_OF_MEASURE *3)+1 ; i++)
		  {
			  HAL_FLASHEx_DATAEEPROM_Erase(DATA_EEPROM_START_ADDR + i*4);
		  }
		  HAL_FLASHEx_DATAEEPROM_Lock();
	  	}
#endif
  }



#ifdef OLED_DISPLAY
  ssd1306_UpdateScreen();
#endif

  HAL_Delay(1000);

  HAL_ADC_Start_DMA(&hadc,(uint32_t*)ADC_buf,sizeof(ADC_buf));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //Start ADC conversion
	if (measureADCDone == false)
	{

		HAL_GPIO_WritePin(HUM_SENSOR_VCC_A2_GPIO_Port, HUM_SENSOR_VCC_A2_Pin, GPIO_PIN_SET);
		HAL_ADC_Start_IT(&hadc);

	}

	else
	{
		HAL_GPIO_WritePin(HUM_SENSOR_VCC_A2_GPIO_Port, HUM_SENSOR_VCC_A2_Pin, GPIO_PIN_RESET);
		HAL_ADC_Stop_IT(&hadc);

//		if (HAL_GPIO_ReadPin(WRITE_EEPROM_D3_GPIO_Port,WRITE_EEPROM_D3_Pin))
//		{
			MeasureCounter = *(uint32_t*) (DATA_EEPROM_START_ADDR);
//		}
		MeasureCounter = MeasureCounter % NUMBER_OF_MEASURE + 1;

		//saved measure into EEPROM


#ifdef USE_EEPROM
//		if (HAL_GPIO_ReadPin(WRITE_EEPROM_D3_GPIO_Port,WRITE_EEPROM_D3_Pin))
//		{
			HAL_FLASHEx_DATAEEPROM_Unlock();
			HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000, MeasureCounter);
			//save SUN measure
			HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000 + ((MeasureCounter) *4), ADC_buf[1]);
			//save Vcc Measure
			HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000 + NUMBER_OF_MEASURE *4 + ((MeasureCounter) *4), ADC_buf[2]);
			//save Vref
			HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000 + NUMBER_OF_MEASURE *2*4 + ((MeasureCounter) *4), ADC_buf[4]);
			HAL_FLASHEx_DATAEEPROM_Lock();
//		}
#endif
#ifdef OLED_DISPLAY
		//Display information on OLED
		UpdateDisplay();
#endif

		//give some time to read LCD
		if (SleepOccurs == true)
		{
			HAL_Delay(2000);
			SleepOccurs =false;
		}


		MeasureCounter ++;
		measureADCDone = false;

		//EnterStandy
		if (HAL_GPIO_ReadPin(Sleep_D9_GPIO_Port,Sleep_D9_Pin))
		{
			GotoStandby();
		}


	}

    //blink LED
    uint32_t blinkTime = 100;
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	HAL_Delay(blinkTime);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_Delay(blinkTime);

	if(ADC_Soil < 500)
	{
		HAL_GPIO_WritePin(PUMP_SWITCH_D12_GPIO_Port, PUMP_SWITCH_D12_Pin,GPIO_PIN_SET);

		  ssd1306_Fill(Black);
		  ssd1306_UpdateScreen();

		uint16_t NumOfSec = 60;
		for (uint16_t i =0 ; i < NumOfSec ; i++)
		{
			HAL_ADC_Start_IT(&hadc);
			HAL_Delay(1000);
			char buffer[128];
			sprintf( buffer, "SOIL_A3=%4lu" ,ADC_Soil);
			ssd1306_SetCursor(0,0);
			ssd1306_WriteString(buffer,Font_7x10,White);

			sprintf( buffer, "Vref=%4lu" ,ADC_Vref);
			ssd1306_SetCursor(0,13);
			ssd1306_WriteString(buffer,Font_7x10,White);

			sprintf( buffer, "ARROSAGE %4u sec" ,NumOfSec -i);
			ssd1306_SetCursor(0,26);
			ssd1306_WriteString(buffer,Font_7x10,White);
			ssd1306_UpdateScreen();
		}

		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();

		HAL_GPIO_WritePin(PUMP_SWITCH_D12_GPIO_Port, PUMP_SWITCH_D12_Pin,GPIO_PIN_RESET);
	}


	//for L0 not programmable
			//IN4 : Pin A3
			//IN5 : Pin A4
			//IN6 : Pin A5
			//Temp
			//Vref





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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HUM_SENSOR_VCC_A2_Pin|LCD_VCC_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|PUMP_SWITCH_D12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HUM_SENSOR_VCC_A2_Pin LCD_VCC_D2_Pin */
  GPIO_InitStruct.Pin = HUM_SENSOR_VCC_A2_Pin|LCD_VCC_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : WRITE_EEPROM_D3_Pin */
  GPIO_InitStruct.Pin = WRITE_EEPROM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WRITE_EEPROM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Sleep_D9_Pin */
  GPIO_InitStruct.Pin = Sleep_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sleep_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin PUMP_SWITCH_D12_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|PUMP_SWITCH_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GotoStandby()
{
	//GPIO is use to enter or not in sleep mode : GPIOa8 -> pin D9

//		if (measureADCDone == true)
//		{
//		/* Insert 5 seconds delay */
//			  HAL_Delay(5000);

			//Power off LCD
			HAL_GPIO_WritePin(LCD_VCC_D2_GPIO_Port, LCD_VCC_D2_Pin, GPIO_PIN_RESET);

			  /* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
			    mainly  when using more than one wakeup source this is to not miss any wakeup event.
			    - Disable all used wakeup sources,
			    - Clear all related wakeup flags,
			    - Re-enable all used wakeup sources,
			    - Enter the Standby mode.
			  */
			  /* Disable all used wakeup sources*/
			  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

			  /* Clear all related wakeup flags */
			  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);


			    RTC_AlarmTypeDef salarmstructure;
			    /*##-3- Configure the RTC Alarm peripheral #################################*/
			  /* Set Alarm to 02:20:30
				 RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
			  salarmstructure.Alarm = RTC_ALARM_A;
			  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
			  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
			  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
			  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
			  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
			  salarmstructure.AlarmTime.Hours = 0x01;
			  salarmstructure.AlarmTime.Minutes = 0x00;
			  salarmstructure.AlarmTime.Seconds = 0x00;
			  salarmstructure.AlarmTime.SubSeconds = 0x0;

			  if(HAL_RTC_SetAlarm_IT(&hrtc,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
			  {
				/* Initialization Error */
				Error_Handler();
			  }

			  /* Enter the Standby mode */


			  HAL_PWR_EnterSTANDBYMode();
		//}
	//}
}
void UpdateDisplay()
{
	char buffer[128];
	    //Display ADC4 value
	    sprintf( buffer, "SOIL_A3=%4lu" ,ADC_Soil);
		ssd1306_SetCursor(0,0);
		ssd1306_WriteString(buffer,Font_7x10,White);

		ssd1306_SetCursor(100,0);
		if (ADC_Soil < 500)
			ssd1306_WriteString("DRY",Font_7x10,White);
		else if ((ADC_buf[0] > 500) && (ADC_buf[0] < 2000))
			ssd1306_WriteString("OK",Font_7x10,White);
		else
			ssd1306_WriteString("WET",Font_7x10,White);

		ssd1306_SetCursor(0,13);
		sprintf( buffer, "SUN_A4= %4lu" ,ADC_VSolar);
		ssd1306_WriteString(buffer,Font_7x10,White);

	//	uint32_t Vsolar =  (16.2/3129) * ADC_buf[1] ;
	//	sprintf( buffer, "%4lu" ,Vsolar);
	//#ifdef FLOAT_CONVERSION
	//	ftoa(Vsolar, buffer, 1);
	//#endif
	//	ssd1306_SetCursor(90,13);
	//	ssd1306_WriteString(buffer,Font_7x10,White);


		sprintf( buffer, "Vcc A5=%4lu" ,ADC_Vreg);
		ssd1306_SetCursor(0,26);
		ssd1306_WriteString(buffer,Font_7x10,White);

		//Display ADC temp
		sprintf( buffer, "Temp=%4lu" ,ADC_Temp);
		ssd1306_SetCursor(0,39);
		ssd1306_WriteString(buffer,Font_7x10,White);

		ssd1306_SetCursor(100,39);
		sprintf( buffer, "%2lu" ,MeasureCounter);
		ssd1306_WriteString(buffer,Font_7x10,White);

		//Display Vref
		sprintf( buffer, "Vref= %4lu" ,ADC_Vref);
		ssd1306_SetCursor(0,52);
		ssd1306_WriteString(buffer,Font_7x10,White);





		ssd1306_SetCursor(80,52);
		if (HAL_GPIO_ReadPin(WRITE_EEPROM_D3_GPIO_Port,WRITE_EEPROM_D3_Pin))
		{
			ssd1306_WriteString("SAVED",Font_7x10,White);
		}
		else
		{
			ssd1306_WriteString("     ",Font_7x10,White);
		}
	#if 0
	    //Display ADC4 value
	    sprintf( buffer, "SOIL_A3=%4lu" ,ADC_buf[0]);
		ssd1306_SetCursor(0,0);
		ssd1306_WriteString(buffer,Font_7x10,White);

		ssd1306_SetCursor(100,0);
		if (ADC_buf[0] < 500)
			ssd1306_WriteString("DRY",Font_7x10,White);
		else if ((ADC_buf[0] > 500) && (ADC_buf[0] < 2000))
			ssd1306_WriteString("OK",Font_7x10,White);
		else
			ssd1306_WriteString("WET",Font_7x10,White);

		ssd1306_SetCursor(0,13);
		sprintf( buffer, "SUN_A4= %4lu" ,ADC_buf[1]);
		ssd1306_WriteString(buffer,Font_7x10,White);

		float Vsolar =  (16.2/3129) * ADC_buf[1] ;
		ftoa(Vsolar, buffer, 1);
		ssd1306_SetCursor(90,13);
		ssd1306_WriteString(buffer,Font_7x10,White);

		//sprintf( buffer, "SUN [A4]= %.1f" ,Vsolar);

	//	char buffer1[] = "SUN_A4=";
	//	//memcpy(buffer ,  "SUN [A4]= ");
	//	ssd1306_WriteString(buffer1,Font_7x10,White);
	//
	//	float Vsolar =  (16.2/3129) * ADC_buf[1] ;
	//	ftoa(Vsolar, buffer, 1);
	//	ssd1306_SetCursor(60,13);
	//	ssd1306_WriteString(buffer,Font_7x10,White);
	//
	//	sprintf( buffer, "V:%4lu" ,ADC_buf[1]);
	//	ssd1306_SetCursor(81,13);
	//	ssd1306_WriteString(buffer,Font_7x10,White);

		//Calculate VBat from Vref
	//	float VBat=-0.00211*ADC_buf[3]+5.1357;
	//    uint16_t VBatbis = (uint16_t)(VBat*100);
		sprintf( buffer, "Vbat=%4lu" ,ADC_buf[2]);
		ssd1306_SetCursor(0,26);
		ssd1306_WriteString(buffer,Font_7x10,White);

		//Display ADC temp
		sprintf( buffer, "Vref=%4lu" ,ADC_buf[3]);
		ssd1306_SetCursor(0,39);
		ssd1306_WriteString(buffer,Font_7x10,White);

		//Display Temp
	//	sprintf( buffer, "ADCTemp= %4lu" ,ADC_buf[4]);
	//	ssd1306_SetCursor(0,52);
	//	ssd1306_WriteString(buffer,Font_7x10,White);

		//Display Temp
		sprintf( buffer, "ADC3V3= %4lu" ,ADC_buf[5]);
		ssd1306_SetCursor(0,52);
		ssd1306_WriteString(buffer,Font_7x10,White);
	#endif
		ssd1306_UpdateScreen();
}
// C program for implementation of ftoa()
#ifdef OLED_DISPLAY
#ifdef FLOAT_CONVERSION
// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[]. d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}
#endif
#endif
// driver program to test above funtion

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
