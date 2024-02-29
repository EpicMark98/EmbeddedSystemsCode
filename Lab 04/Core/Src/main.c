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

//#define BLOCKING

volatile char receivedChar = 0;
volatile char receivedNum = '3';
volatile uint8_t ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Transmits one character over the USART interface
void TransmitChar(char c) {
	while(!(USART3->ISR & 0x80));	// wait until TXE (bit 7 of ISR) is 1
	USART3->TDR = c;	// write the character to the USART
}

// Transmits a string
void TransmitString(char s[]) {
	int i = 0;
	while(s[i]) {		// Loop until a null character is reached
		TransmitChar(s[i++]);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	SystemClock_Config(); //Configure the system clock
	
	// Initialize GPIO and Timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	
	// Initialize Pins 4 through 9
	GPIOC->MODER = 0x55A00;
	
	// Set alternate function mode
	GPIOC->AFR[0] |= 0x110000;
	
	// Turn all LEDS off
	GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	
	// Initialize the USART
	USART3->BRR = 8000000 / 115200;	// Set baud rate
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;	// 8 data bit, 1 start bit, 1 stop bit, no parity, reception and transmission mode
	
	
#ifndef BLOCKING
	// Enable interrupts and set priority
	USART3->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 2);	
#endif
	
	while (1) 
	{
#ifdef BLOCKING
		// Loop until RXNE is high, indicating data to be read
		while((USART3->ISR & 0x20) != 0x20);
		
		// Read the character and toggle the appropriate LED
		char c = (char)USART3->RDR;
		switch(c)
		{
			case 'r':
				GPIOC->ODR ^= (1 << 6);
				break;
			case 'b':
				GPIOC->ODR ^= (1 << 7);
				break;
			case 'o':
				GPIOC->ODR ^= (1 << 8);
				break;
			case 'g':
				GPIOC->ODR ^= (1 << 9);
				break;
			default:
				TransmitString("Invalid character recieved: ");
				TransmitChar(c);
				TransmitString("\r\n");
		}
#else
		TransmitString("CMD? ");
		
		while(!ready);
		ready = 0;
		
		// Validate character and go back to waiting if invalid
		if((receivedChar != 'r' && receivedChar != 'g' && receivedChar != 'b' && receivedChar != 'o') || (receivedNum >= '3' || receivedNum < '0'))
		{
			TransmitString("\r\n   Invalid character received\r\n");
			receivedNum = '3';
			continue;
		}
		TransmitString("\r\n   Command recognized: ");
		
		uint8_t gpioNum;
		if(receivedChar == 'r') {
			gpioNum = 6;
		}
		else if(receivedChar == 'g') {
			gpioNum = 9;
		}
		else if(receivedChar == 'b') {
			gpioNum = 7;
		}
		else if(receivedChar == 'o') {
			gpioNum = 8;
		}
		
		if(receivedNum == '0') {
			GPIOC->ODR &= ~(1 << gpioNum);
		}
		else if(receivedNum == '1') {
			GPIOC->ODR |= (1 << gpioNum);
		}
		else if(receivedNum == '2') {
			GPIOC->ODR ^= (1 << gpioNum);
		}
		
		TransmitChar(receivedChar);
		TransmitChar(receivedNum);
		TransmitString("\r\n");
		
		receivedNum = '3';
#endif
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
