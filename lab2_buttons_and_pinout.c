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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int Button1Cnt = 0;
int Button2Cnt = 0; // zmienne globalne dla latwiejszej obslugi
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		//static unsigned Button1Cnt = 0;
		//static unsigned Button2Cnt = 0;

		if( GPIO_Pin == Button1_Pin) Button1Cnt++;
		if( GPIO_Pin == Button2_Pin) Button2Cnt++;

		if (Button1Cnt > 99) // po przekroczeniu maksymalnej wartosci mozliwej do wyswietlenia ustawiam buttonCnt na 0
		{
			Button1Cnt = 0;
		}

		if (Button2Cnt > 99)
		{
			Button2Cnt = 0;
		}

		for(int i = 0; i < 5000; i++) // petla opozniajaca, alternatywna metoda to skorzystanie z timera
		{
			int x = i * 100;
		}

	}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  void seg_zero() // cyfra 0 na wyswietlaczu 7 segmentowym
  			  {
  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_SET);
  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_SET);
  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_RESET);
  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  			  }

  	  	  void seg_one() // cyfra 1 na wyswietlaczu 7 segmentowym
  	  	  {
  	  		  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_RESET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_RESET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_RESET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_RESET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_RESET);
  	  		  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  	  }

  	  	  void seg_two() // cyfra 2 na wyswietlaczu 7 segmentowym
  	  	  	  {
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_RESET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_SET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_RESET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  	  		  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  	  	  }

  	  	  void seg_three() // cyfra 3 na wyswietlaczu 7 segmentowym
  	  	  	  	  {
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_RESET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_RESET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  	  	  		  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  	  	  	  }
  	  	  void seg_four() // cyfra 4 na wyswietlaczu 7 segmentowym
  	  			  {
  	  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  			  }
  	  	   void seg_five() // cyfra 5 na wyswietlaczu 7 segmentowym
  	  			  {
  	  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  			  }

  	  	  void seg_six() // cyfra 6 na wyswietlaczu 7 segmentowym
  	  			  {
  	  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  			  }

  	  	  void seg_seven() // cyfra 7 na wyswietlaczu 7 segmentowym
  	  			  {
  	  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  			  }

  	   	  void seg_eight() // cyfra 8 na wyswietlaczu 7 segmentowym
  	  			  {
  	  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  			  }

  	  	  void seg_nine() // cyfra 9 na wyswietlaczu 7 segmentowym
  	  			  {
  	  				  HAL_GPIO_WritePin(GPIOC, S7A_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7B_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7C_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7D_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7E_Pin, GPIO_PIN_RESET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7F_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7G_Pin, GPIO_PIN_SET);
  	  				  HAL_GPIO_WritePin(GPIOC, S7Dp_Pin, GPIO_PIN_RESET);
  	  			  }



  	  	  void first_disp()
  	  	  		  {
  	  				HAL_GPIO_WritePin(GPIOC, S7Com1_Pin, GPIO_PIN_RESET); // wlaczam tylko 1 wyswietlacz
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com2_Pin, GPIO_PIN_SET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com3_Pin, GPIO_PIN_SET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com4_Pin, GPIO_PIN_SET);

  	  			  }

  	  	  void second_disp()
  	  	  		  {
  	  				HAL_GPIO_WritePin(GPIOC, S7Com1_Pin, GPIO_PIN_SET); // wlaczam tylko 2 wyswietlacz
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com2_Pin, GPIO_PIN_RESET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com3_Pin, GPIO_PIN_SET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com4_Pin, GPIO_PIN_SET);

  	  			  }

  	  	  void third_disp()
  	  	  		  {
  	  				HAL_GPIO_WritePin(GPIOC, S7Com1_Pin, GPIO_PIN_SET); // wlaczam tylko 3 wyswietlacz
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com2_Pin, GPIO_PIN_SET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com3_Pin, GPIO_PIN_RESET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com4_Pin, GPIO_PIN_SET);

  	  			  }

  	  	  void fourth_disp()
  	  	  		  {
  	  				HAL_GPIO_WritePin(GPIOC, S7Com1_Pin, GPIO_PIN_SET); // wlaczam tylko 2 wyswietlacz
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com2_Pin, GPIO_PIN_SET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com3_Pin, GPIO_PIN_SET);
  	  		  		HAL_GPIO_WritePin(GPIOC, S7Com4_Pin, GPIO_PIN_RESET);

  	  			  }

  	  	  void sw_1(digit) // switch do dobierania odpowiedniej funkcji wyswietlajacej cyfre odpowiadajaca wartosci liczbowej
  	  	  {
  	  		switch (digit){
  					case 0:
  						seg_zero();
  						break;

  					case 1:
  						seg_one();
  						break;

  					case 2:
  						seg_two();
  						break;

  					case 3:
  						seg_three();
  						break;

  					case 4:
  						seg_four();
  						break;

  					case 5:
  						seg_five();
  						break;

  					case 6:
  						seg_six();
  						break;

  					case 7:
  						seg_seven();
  						break;

  					case 8:
  						seg_eight();
  						break;

  					case 9:
  						seg_nine();
  						break;
  	  	  }
  	  	  }


  	  	void show_num(int* button1, int* button2)
  	  			  {

  	  			  	int digit4 = (*button1) / 10; // wyciagam cyfre dziesiatek liczby button1Cnt
  	  			  	int digit3 = (*button1) % 10; // wyciagam cyfre jednosci liczby button1Cnt

  	  			  	int digit2 = (*button2) / 10;// wyciagam cyfre dziesiatek liczby button2Cnt
  	  			  	int digit1 = (*button2) % 10;// wyciagam cyfre jednosci liczby button2Cnt

  	  				first_disp(); // wyswietlam pierwsza cyfre na pierwszym wyswietlaczu
  	  				sw_1(digit4);
  	  				HAL_Delay(2);


  	  				second_disp();// wyswietlam druga cyfre na drugim wyswietlaczu
  	  				sw_1(digit3);
  	  				HAL_Delay(2);

  	  				third_disp();// wyswietlam trzecia cyfre na trzecim wyswietlaczu
  	  				sw_1(digit2);
  	  				HAL_Delay(2);

  	  				fourth_disp();// wyswietlam czwarta cyfre na czwartym wyswietlaczu
  	  				sw_1(digit1);
  	  				HAL_Delay(2); // minimalne opoznienie potrzebne do zobaczenia wszystkich cyfr jednoczesnie na wyswietlaczu

  	  			  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
	  show_num(&Button1Cnt, &Button2Cnt);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOC, S7G_Pin|S7D_Pin|S7E_Pin|S7C_Pin
                          |S7B_Pin|S7F_Pin|S7A_Pin|S7Dp_Pin
                          |S7Com4_Pin|S7Com3_Pin|S7Com2_Pin|S7Com1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S7G_Pin S7D_Pin S7E_Pin S7C_Pin
                           S7B_Pin S7F_Pin S7A_Pin S7Dp_Pin
                           S7Com4_Pin S7Com3_Pin S7Com2_Pin S7Com1_Pin */
  GPIO_InitStruct.Pin = S7G_Pin|S7D_Pin|S7E_Pin|S7C_Pin
                          |S7B_Pin|S7F_Pin|S7A_Pin|S7Dp_Pin
                          |S7Com4_Pin|S7Com3_Pin|S7Com2_Pin|S7Com1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button1_Pin Button2_Pin */
  GPIO_InitStruct.Pin = Button1_Pin|Button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */



	int __io_putchar(int ch) {
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
		return ch;
	}

	int getchar(void) {
		uint8_t value;
		if( __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != SET )
			return( -1 );
		if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET)
			HAL_UART_Receive(&huart2, &value, 1, 100);
			__io_putchar( value );
		return ((int) value);
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
