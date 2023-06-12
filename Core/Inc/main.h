/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum MachineState {
	MS_IDLE = 0x00,
	MS_WAIT,
	MS_MEASURE,
	MS_END,
	MS_COVID_19,
	MS_ERROR,
	MS_EXERCISE
} MachineState;

typedef struct MachineData {
	float heartRate;
	float oxygen;
	float confidence;
	float temperature;
} MachineData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MAX_MEASURE_TIME	30 // seconds
#define EXERCISE_TIME		20 // seconds
#define TIME_RATIO			100 // counts per second (timer here has resolution 10ms)
#define PAUSE_TIME			5
#define OPT_MEASURES		30

#define HIGH_TEMP_THRES		36.0
#define LOW_OXY_THRES		94.0 //TODO: check reference
#define HIGH_HR_THRES		75.0 //TODO: check reference
#define MIN_UNCERT_THRES	0.1 // 1 = 100%
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define __EXPIRED(t, timeout)	((t) == (timeout) * TIME_RATIO)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define ERROR_Pin GPIO_PIN_7
#define ERROR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
