/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "hal_utils.h"

#include "b3950.h"
#include "ds1307rtc.h"
#include "ssd1306.h"
#include "max32664.h"
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

/* USER CODE BEGIN PV */
GPIO_Line PC0 = { .port = GPIOC, .pin = GPIO_PIN_0 };
GPIO_Line PC1 = { .port = GPIOC, .pin = GPIO_PIN_1 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
B3950_Handle *temp_sens;
MAX32664_Handle *pox;

// conditions
uint8_t flag_enable = 0;
uint8_t flag_finger_on_p = 0;
uint8_t flag_high_hr = 0;

// state
MachineState state = MS_IDLE;

// other
uint8_t oled_written = 0;
uint8_t led_dir = 0;
uint32_t led_pulse = 0;

// data
uint32_t measureCount = 0;
uint32_t timeCount = 0;
uint32_t cicleCount = 0;
MachineData average;
MachineData maximum;
MachineData minimum;
char buf[1] = { '\0' };
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_I2C3_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_TIM10_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	USART_PRINT("\r\nSystem init...");

	uint32_t adc_buffer[1];
	HAL_ADC_Start_DMA(&hadc1, adc_buffer, 1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim10);

	// devices creation
	temp_sens = B3950_Init(adc_buffer);
	pox = MAX32664(&hi2c1, &PC0, &PC1, 0x55);

	// devices init/start
	ssd1306_Init();
	ds1307rtc_init();
	MAX32664_Begin(pox);

	date_time_t dt = { .year = 23, .month = 6, .date = 12, .hours = 8,
			.minutes = 21, .seconds = 0 };
	ds1307rtc_set_date_time(&dt);

	date_time_t test;
	ds1307rtc_get_date_time(&test);
	USART_PRINT("\r\nDatetime: %u/%u/%d %u:%u:%u", test.date, test.month,
			(int )test.year, test.hours, test.minutes, test.seconds);

	uint8_t error = MAX32664_ConfigBpm(pox, MODE_ONE); // Configuring just the BPM settings.
	if (error == SB_SUCCESS) {
		USART_PRINT("\r\nSensor configured correctly");
	} else {
		USART_PRINT("\r\nError during configuration with status code %u", error);
	}

	// Data lags a bit behind the sensor, if you're finger is on the sensor when
	// it's being configured this delay will give some time for the data to catch
	// up.
	USART_PRINT("\r\nLoading sensor data...");
	HAL_Delay(4000);
	USART_PRINT("\r\nOk, sensor ready");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		switch (state) {
		case MS_WAIT: {
			bioData poxData = MAX32664_ReadBpm(pox);
			flag_finger_on_p = (poxData.status == 3);
			if (flag_finger_on_p) {
				ssd1306_Fill(Black);
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Measuring", Font_7x10, White);
				ssd1306_UpdateScreen();
				USART_PRINT("\r\nOk, measuring");
				state = MS_MEASURE;
			}
			break;
		}
		case MS_MEASURE: {
			bioData poxData = MAX32664_ReadBpm(pox);
			if ((float) poxData.heartRate < 10.0
					|| (float) poxData.oxygen < 10.0)
				break;
			float tmp = B3950_Read(temp_sens);
			average.oxygen += poxData.oxygen;
			average.heartRate += poxData.heartRate;
			average.confidence += poxData.confidence;
			average.temperature += tmp;

			if (poxData.heartRate > maximum.heartRate)
				maximum.heartRate = poxData.heartRate;
			if (poxData.heartRate < minimum.heartRate)
				minimum.heartRate = poxData.heartRate;

			if (poxData.oxygen > maximum.oxygen)
				maximum.oxygen = poxData.oxygen;
			if (poxData.oxygen < minimum.oxygen)
				minimum.oxygen = poxData.oxygen;

			if (poxData.confidence > maximum.confidence)
				maximum.confidence = poxData.confidence;
			if (poxData.confidence < minimum.confidence)
				minimum.confidence = poxData.confidence;

			if (tmp > maximum.temperature)
				maximum.temperature = tmp;
			if (tmp < minimum.temperature)
				minimum.temperature = tmp;

			measureCount += 1;
			break;
		}
		default:
			break;
		}

		// pox sensor delay
		HAL_Delay(40);
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim10) {

		if (state == MS_MEASURE) {
			if (__EXPIRED(timeCount, MAX_MEASURE_TIME)) {
				timeCount = 0;
				state = MS_END;
				date_time_t curr = { 0 };
				ds1307rtc_get_date_time(&curr);
				USART_PRINT("\r\nReport [%d-%d-%dT%d:%d:%d]", curr.year,
						curr.month, curr.date, curr.hours, curr.minutes,
						curr.seconds);
				if (measureCount < OPT_MEASURES) {
					USART_PRINT("\r\nobtained %lu/%lu good samples -> discard",
							measureCount, (uint32_t)OPT_MEASURES);
				} else {
					USART_PRINT("\r\nobtained %lu/%lu good samples -> accept",
							measureCount, (uint32_t)OPT_MEASURES);
				}

				if (measureCount < OPT_MEASURES) {
					state = MS_ERROR;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Invalid measure", Font_7x10, White);
					ssd1306_SetCursor(0, 15);
					ssd1306_WriteString("Repeat", Font_7x10, White);
					ssd1306_SetCursor(0, 0);
					ssd1306_UpdateScreen();
					return;
				}

				average.oxygen /= measureCount;
				average.heartRate /= measureCount;
				average.confidence /= measureCount;
				average.temperature /= measureCount;

				USART_PRINT("\r\nHr: %.1f, Ox: %.0f, Conf: %.0f, T: %.1f",
						(float )average.heartRate, (float )average.oxygen,
						average.confidence, average.temperature);

				// uncertainty computation
				MachineData unc;
				// max errors
				unc.heartRate = (maximum.heartRate - minimum.heartRate) / 2;
				unc.oxygen = (maximum.oxygen - minimum.oxygen) / 2;
				unc.temperature = (maximum.temperature - minimum.temperature) / 2;

				unc.heartRate = unc.heartRate / average.heartRate;
				unc.oxygen = unc.oxygen / average.oxygen;
				unc.temperature = unc.temperature / average.temperature;

				if (unc.heartRate <= MIN_UNCERT_THRES
						|| unc.oxygen <= MIN_UNCERT_THRES
						|| unc.temperature <= MIN_UNCERT_THRES) {
					state = MS_ERROR;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Invalid measure", Font_7x10, White);
					ssd1306_SetCursor(0, 15);
					ssd1306_WriteString("Repeat", Font_7x10, White);
					ssd1306_SetCursor(0, 0);
					ssd1306_UpdateScreen();
					return;
				}

				if (average.heartRate > HIGH_HR_THRES) {
					state = MS_EXERCISE;
					USART_PRINT("\r\nBreath exercise mode");
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Exercise mode", Font_7x10, White);
					ssd1306_UpdateScreen();
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					return;
				}
				if (average.oxygen < LOW_OXY_THRES
						&& average.temperature > HIGH_TEMP_THRES) {
					state = MS_COVID_19;
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Take a COVID test", Font_7x10, White);
					ssd1306_UpdateScreen();
					return;
				}

				// write pox data
				char hr[30];
				char ox[30];
				char cf[30];
				char tmp_msg[30];
				sprintf(hr, "Hr: %.1f bpm", average.heartRate);
				sprintf(ox, "Ox: %.0f perc", average.oxygen);
				sprintf(cf, "Cf: %.0f perc", average.confidence);
				sprintf(tmp_msg, "T: %.1f C", average.temperature);
				ssd1306_Fill(Black);
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString(hr, Font_7x10, White);
				ssd1306_SetCursor(0, 15);
				ssd1306_WriteString(ox, Font_7x10, White);
				ssd1306_SetCursor(0, 30);
				ssd1306_WriteString(cf, Font_7x10, White);
				ssd1306_SetCursor(0, 45);
				ssd1306_WriteString(tmp_msg, Font_7x10, White);
				ssd1306_SetCursor(0, 0);
				ssd1306_UpdateScreen();
			}

			else {
				timeCount += 1;
			}
			return;
		}

		if (state == MS_EXERCISE) {
			if (__EXPIRED(timeCount, EXERCISE_TIME)) {
				timeCount = 0;
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
				state = MS_WAIT;
				return;
			}

			if (led_dir == 0)
				led_pulse += 4;
			else
				led_pulse -= 4;

			if ((led_pulse == 0) || (led_pulse >= 999))
				led_dir = !led_dir;

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, led_pulse);
			timeCount += 1;
			return;
		}

		if (state == MS_ERROR || state == MS_COVID_19) {
			if (timeCount == PAUSE_TIME * 100) {
				timeCount = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				state = MS_WAIT;
				return;
			}

			timeCount += 1;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		if (state == MS_IDLE) {
			USART_PRINT("\r\nDevice is on");
			state = MS_WAIT;
			ssd1306_Fill(Black);
			ssd1306_WriteString("Put finger", Font_7x10, White);
			ssd1306_SetCursor(0, 15);
			ssd1306_WriteString("on sensors", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
