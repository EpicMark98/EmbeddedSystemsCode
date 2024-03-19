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

// Helper function to perform an I2C read
uint8_t I2C_Read(uint8_t slaveAddr, uint8_t registerAddr) {
	I2C2->CR2 = (1 << 16) | (slaveAddr << 1);	// Setup write transaction
	I2C2->CR2 |= I2C_CR2_START;	// Set start bit
	while(!(I2C2->ISR & 0x12));	// Wait until bit 4 or 1 is set
	I2C2->TXDR = registerAddr;	// Address of register
	while(!(I2C2->ISR & 0x40));	// Wait until bit 6 is set
	
	I2C2->CR2 = (1 << 16) | (1 << 10) | (slaveAddr << 1);	// Setup read transaction
	I2C2->CR2 |= (1 << 13);	// Set start bit
	while(!(I2C2->ISR & 0x14));	// Wait until bit 4 or 2 is set
	uint8_t data = I2C2->RXDR;	// Read the data
	while(!(I2C2->ISR & 0x40));	// Wait until bit 6 is set
	I2C2->CR2 |= (1 << 14);	// Set STOP bit
	
	return data;
}

// Helper function to perform an I2C write
void I2C_Write(uint8_t slaveAddr, uint8_t data[], uint8_t numBytes) {
	I2C2->CR2 = (numBytes << 16) | (slaveAddr << 1);	// Setup write transaction
	I2C2->CR2 |= (1 << 13);	// Set start bit
	for(uint8_t bytesWritten = 0; bytesWritten < numBytes; ++bytesWritten) {
		while(!(I2C2->ISR & 0x12));	// Wait until bit 4 or 1 is set
		I2C2->TXDR = data[bytesWritten];	// Next data byte
	}
	while(!(I2C2->ISR & 0x40));	// Wait until bit 6 is set
	I2C2->CR2 |= (1 << 14);	// Set STOP bit
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	SystemClock_Config(); //Configure the system clock
	
	// Initialize GPIO and I2C
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Initialize PB13 and PB11 to AF mode and PB14 to output mode
	GPIOB->MODER = 0x18800000;
	GPIOB->OTYPER = 0x2800;
	
	// Select alternate functions (PB13 set to AF5 - I2C2 SCL, PB11 set to AF1 - I2C2 SDA)
	GPIOB->AFR[1] = 0x00501000;
	
	// Initialize PC0 to output mode (as well as PC6-9 for LEDs)
	GPIOC->MODER = 0x00055001;
	
	// Turn all LEDs off and set PC0 high
	GPIOC->ODR = 0x1;
	
	// Set PB14 high
	GPIOB->ODR |= 0x4000;
	
	// Configure I2C (PRESC = 1, SCLL = 0x13, SCLH = 0xF, SDADEL = 0x2, SCLDEL = 0x4)
	I2C2->TIMINGR = (0x1 << 28) | (0x13) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);
	I2C2->CR1 |= I2C_CR1_PE;
	
	// Read the WHO_AM_I register
	uint8_t whoAmI = I2C_Read(0x69, 0xF);
	
	// Verify that WHO_AM_I returned 0xD3
	if(whoAmI != 0xD3) {
		GPIOC->ODR |= (1 << 6);
		while(1);
	}
	
	// Configure gyroscope
	uint8_t data[2] = {0x20, 0x0F};
	I2C_Write(0x69, data, 2);
	
	// Main gyroscope and LED operation loop
	while (1) {
		HAL_Delay(100);	//ADC Wait 100 ms
		
		// Read all the data
		uint8_t xl = I2C_Read(0x69, 0x28);
		uint8_t xh = I2C_Read(0x69, 0x29);
		uint8_t yl = I2C_Read(0x69, 0x2A);
		uint8_t yh = I2C_Read(0x69, 0x2B);
		
		int16_t x = (xh << 8) | xl;
		int16_t y = (yh << 8) | yl;
		
		// Turn on the LED that points in the direction of rotation
		GPIOC->ODR &= ~(0xF << 6);
		if(y > 1000) {
			GPIOC->ODR |= (1 << 6);
		}
		else if(y < -1000) {
			GPIOC->ODR |= (1 << 7);
		}
		else{
			if(x > 1000) {
				GPIOC->ODR |= (1 << 9);
			}
			else if(x < -1000) {
				GPIOC->ODR |= (1 << 8);
			}
		}
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
