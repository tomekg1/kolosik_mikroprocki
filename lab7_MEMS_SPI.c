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

#include <inttypes.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t ADCDMABuff[3];
#define BUFFER_LEN (3000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARM_CM_DEMCR (*(uint32_t *)0xE000EDFC)
#define ARM_CM_DWT_CTRL (*(uint32_t *)0xE0001000)
#define ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)
#define CPU_CLOCK_FREQ ((double)100000000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

int TimerCnt = 0;

int ADCRes;

int ClickinterruptCnt = 0;

uint32_t BufferT[BUFFER_LEN];
uint16_t BufferAD[BUFFER_LEN];

uint32_t Start;
uint32_t Stop;

uint32_t it1 = 0;
uint32_t it4 = 0;

uint8_t read;
double tim = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

  	  	  	  void sw_1(int digit) // switch do dobierania odpowiedniej funkcji wyswietlajacej cyfre odpowiadajaca wartosci liczbowej
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

  	  	  	    	  	void show_num(int TimerCnt)
  	  	  	    	  	  	  			  {

  	  	  	    	  	  	  			  	int digit1 = TimerCnt / 1000 % 10; // cyfra tysiecy
  	  	  	    	  	  	  			  	int digit2 = TimerCnt / 100 % 10; // cyfra setek

  	  	  	    	  	  	  			  	int digit3 = (TimerCnt / 10) % 10; // cyfra dziesiatek
  	  	  	    	  	  	  			  	int digit4 = TimerCnt % 10; // cyfra jednosci

  	  	  	    	  	  	  				first_disp(); // wyswietlam pierwsza cyfre na pierwszym wyswietlaczu
  	  	  	    	  	  	  				sw_1(digit1);
  	  	  	    	  	  	  				HAL_Delay(2);


  	  	  	    	  	  	  				second_disp();// wyswietlam druga cyfre na drugim wyswietlaczu
  	  	  	    	  	  	  				sw_1(digit2);
  	  	  	    	  	  	  				HAL_Delay(2);

  	  	  	    	  	  	  				third_disp();// wyswietlam trzecia cyfre na trzecim wyswietlaczu
  	  	  	    	  	  	  				sw_1(digit3);
  	  	  	    	  	  	  				HAL_Delay(2);

  	  	  	    	  	  	  				fourth_disp();// wyswietlam czwarta cyfre na czwartym wyswietlaczu
  	  	  	    	  	  	  				sw_1(digit4);
  	  	  	    	  	  	  				HAL_Delay(2); // minimalne opoznienie potrzebne do zobaczenia wszystkich cyfr jednoczesnie na wyswietlaczu

  	  	  	    	  	  	  			  }

// komunikacja
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint32_t *)&ch, 1, 0xFFFF);
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

uint16_t SPITx[10];
  	uint16_t SPIRx[10];
  	uint8_t SPIRead( uint8_t Address ) {
  		SPITx[0] = (0x80 | Address) << 8;
  		HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_RESET);
  		HAL_SPI_TransmitReceive(&hspi1, SPITx, SPIRx, 1, 100);
  		HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_SET);
  		return( (uint8_t)(SPIRx[0] & 0xFF) );
  	}
  	void SPIWrite( uint8_t AddressAndAtributes, uint8_t Data ) {
  		SPITx[0] = (AddressAndAtributes << 8) + Data;
  		HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_RESET);
  		HAL_SPI_TransmitReceive(&hspi1, SPITx, SPIRx, 1, 100);
  		HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin, GPIO_PIN_SET);
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
  if (ARM_CM_DWT_CTRL != 0) { // See if DWT is available
	  ARM_CM_DEMCR |= 1 << 24; // Set bit 24
	  ARM_CM_DWT_CYCCNT = 0;
	  ARM_CM_DWT_CTRL |= 1 << 0; // Set bit 0
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCDMABuff, 3);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_ADC_Start_IT(&hadc1);
  Start = ARM_CM_DWT_CYCCNT;



  SPIWrite(0x38, 0b11000111);
	SPIWrite(0x3B, 0b00000000);
	SPIWrite(0x3C, 0b10000111);
	SPIWrite(0x3D, 0x88);
	SPIWrite(0x3E, 0x22);
	SPIWrite(0x3F, 0xff);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  show_num(ClickinterruptCnt);  // Sprawdzanie dzialania timera
	  //Start = ARM_CM_DWT_CYCCNT;
	  //HAL_Delay(1000);
	  //Stop = ARM_CM_DWT_CYCCNT;

	  //show_num(ADCRes);
	 // click_window



	//show_num(ADCRes); // sprawdzam wartosc ADCRes

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 19;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, SPI_NSS_Pin|LD2_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : SPI_NSS_Pin */
  GPIO_InitStruct.Pin = SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_NSS_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : Clickinterrupt_Pin */
  GPIO_InitStruct.Pin = Clickinterrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Clickinterrupt_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	ClickinterruptCnt++;

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // przerwanie zegarowe T = 2ms

		if (htim->Instance == TIM1)
		{
			TimerCnt++;

			SPIWrite(0x20, 0b11000111);
			SPIWrite(0x21, 0b01000000);
			SPIWrite(0x22, 0b10000111);
			uint8_t read1 = SPIRead(0x20);
			uint8_t read2 = SPIRead(0x21);
			uint8_t read3 = SPIRead(0x22);

			int8_t outx = SPIRead(0x29);
			int8_t outy = SPIRead(0x2B);
			int8_t outz = SPIRead(0x2D);

			//printf("%f %f %f\n\r", (double)read1, (double)read2, (double)read3);
			if (TimerCnt < 3000){
				printf("%f %f %f %f\n\r", (double)outx, (double)outy, (double)outz, tim);
				tim += 0.002;
			}

		}


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
