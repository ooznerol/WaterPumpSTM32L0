/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define HUM_SENSOR_VCC_A2_Pin GPIO_PIN_3
#define HUM_SENSOR_VCC_A2_GPIO_Port GPIOA
#define ADC_IN4_A3_Pin GPIO_PIN_4
#define ADC_IN4_A3_GPIO_Port GPIOA
#define ADC_IN5_A4_Pin GPIO_PIN_5
#define ADC_IN5_A4_GPIO_Port GPIOA
#define ADC_IN6_A5_Pin GPIO_PIN_6
#define ADC_IN6_A5_GPIO_Port GPIOA
#define WRITE_EEPROM_D3_Pin GPIO_PIN_0
#define WRITE_EEPROM_D3_GPIO_Port GPIOB
#define Sleep_D9_Pin GPIO_PIN_8
#define Sleep_D9_GPIO_Port GPIOA
#define I2C_SCL_D1_Pin GPIO_PIN_9
#define I2C_SCL_D1_GPIO_Port GPIOA
#define I2C_SDA_D0_Pin GPIO_PIN_10
#define I2C_SDA_D0_GPIO_Port GPIOA
#define LCD_VCC_D2_Pin GPIO_PIN_12
#define LCD_VCC_D2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define PUMP_SWITCH_D12_Pin GPIO_PIN_4
#define PUMP_SWITCH_D12_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
