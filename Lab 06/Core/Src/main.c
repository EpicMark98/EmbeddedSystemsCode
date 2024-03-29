/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	SystemClock_Config(); //Configure the system clock
	
	// Initialize GPIO and ADC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	// Configure PA1 and PA4 for analog mode
	GPIOA->MODER = 0x30C;
	
	// Initialize LEDs
	GPIOC->MODER = 0x55000;
	
	// Calibrate ADC
	if ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0){	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0){	}
	
	// Configure ADC
	ADC1->CFGR1 |= ADC_CFGR1_CONT | (0x2 << 3);
	ADC1->CHSELR = ADC_CHSELR_CHSEL1;
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
	
	// Enable the ADC
	ADC1->CR |= 0x1;
	ADC1->CR |= ADC_CR_ADSTART;
	
	// Configure DAC
	DAC->CR |= (7 << 3) | (1 << 2);	// Sets software trigger
	DAC->CR |= 0x1;	// Enables the DAC
	
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	uint8_t index = 0;
		
	while (1) {
		
		// Read the ADC value and change the LEDs accordingly
		uint8_t value = ADC1->DR & 0xFF;
		uint32_t output = 0;
		if(value > 0x30) {
			output |= (1 << 6);
		}
		if(value > 0x50) {
			output |= (1 << 9);
		}
		if(value > 0xA0) {
			output |= (1 << 7);
		}
		if(value > 0xD0) {
			output |= (1 << 8);
		}
		GPIOC->ODR = output;
		
		// Write value to DAC and wait 1 ms
		DAC->DHR8R1 = triangle_table[index];
		DAC->SWTRIGR |= 0x1;
		index = (index + 1) % 32;
		HAL_Delay(1);
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
