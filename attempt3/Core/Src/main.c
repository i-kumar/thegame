/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// this code runs when the data transfer is complete and stops it from repeating
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

// stores data for one led
typedef struct ledData{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

// storage for whole screen
#define NUM_LEDS 16*16
struct ledData storage[NUM_LEDS];

// buffer storage
#define RESET_PULSES 60 // need this because stm has done nothing but ruin my life and im tired of it
uint32_t pwmBuffer[RESET_PULSES + (NUM_LEDS * 24)]; // 24 bits per led (plus our 60 0s at the start)

// write all leds to buffer
void writeLedsToBuffer(){
	for (int i = 0; i < 60; i++){
		pwmBuffer[i] = 0; // i hate you stm. why.
	}

	for (int i = 0; i < NUM_LEDS; i++){
		// write g data
		for (int bit = 0; bit < 8; bit++){
			int temp = (storage[i].g >> bit) & 1; // take one bit at a time
			if (temp == 1){
				// index to spot in buffer
				pwmBuffer[RESET_PULSES + (i*24) + 7 - bit] = 64; // CCR value for high
			} else {
				pwmBuffer[RESET_PULSES + (i*24) + 7 - bit] = 32; // CCR value for low
			}
		}

		// write r data
		for (int bit = 0; bit < 8; bit++){
			int temp = (storage[i].r >> bit) & 1; // take one bit at a time
			if (temp == 1){
				// index to spot in buffer
				pwmBuffer[RESET_PULSES + (i*24) + 8 + 7 - bit] = 64; // CCR value for high
			} else {
				pwmBuffer[RESET_PULSES + (i*24) + 8 + 7 - bit] = 32; // CCR value for low
			}
		}

		// write b data
		for (int bit = 0; bit < 8; bit++){
			int temp = (storage[i].b >> bit) & 1; // take one bit at a time
			if (temp == 1){
				// index to spot in buffer
				pwmBuffer[RESET_PULSES + (i*24) + 16 + 7 - bit] = 64; // CCR value for high
			} else {
				pwmBuffer[RESET_PULSES + (i*24) + 16 + 7 - bit] = 32; // CCR value for low
			}
		}
	}
}

void showLeds(){
	writeLedsToBuffer();
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwmBuffer, RESET_PULSES + (NUM_LEDS * 24));
}

void clearScreen(){
	for (int i = 0; i < 256; i++){
		storage[i] = (struct ledData){0, 0, 0};
	}
}

// x and y -> 0 to 15 each.
// r, g, and b -> 0 to 255 each.
void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b){

  int realIndex = 0;

  if (y % 2 == 0) {
    // even row
    realIndex = y * 16 + x;
  } else {
    // odd row (reversed)
    realIndex = y * 16 + (16 - 1 - x);
  }

  storage[realIndex] = (struct ledData){r, g, b};
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwmData, 4);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //int currLimit = 0;
  //int currentColor = 0; //0 = red, 1 = green, 2 = blue


  while (1)
  {
    /* USER CODE END WHILE */
	  playPong();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// GAMES: -------------------------------------------------------------------------

void playPong(){
	// PONG - rewritten for the game
	// by Ishan Kumar: 11/7/2025

	int ballX = 1;
	int ballY = 4;

	/*
	STATE MACHINE FOR BALL DIRECTION
	> UP:
	0 = left fast
	1 = left
	2 = right
	3 = right fast
	> DOWN
	4 = left fast
	5 = left
	6 = right
	7 = right fast
	*/
	int ballStateMachine = 3;

	// we don't want to move the ball every frame. The prescaler will move the ball every n frames
	int ballPrescaler = 2;
	int ballCurrN = 0;

	int lose = 0;

	while (!lose){
		// read pot
		uint32_t ADC_VAL = 0;
		HAL_ADC_Start(&hadc1);//start conversion
		HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);//wait for conversion to finish
		ADC_VAL = HAL_ADC_GetValue(&hadc1);//retrieve value

		// turn value into x value from 0 to 12
		int pot = ADC_VAL / 16; // standardize
		pot = (pot / 8) - 6; // put it on the range from 0 to 12
		if (pot < 0) pot = 0;
		if (pot > 12) pot = 12;
		pot = 12 - pot; // guess i had the direction reversed oops

		// start drawing screen
		clearScreen();

		// draw paddle
		for (int i = pot; i < pot + 4; i++){
			setPixel(i, 1, 0, 50, 50);
		}

		// decide whether to move the ball
		if (ballCurrN < ballPrescaler){
			ballCurrN++;
			setPixel(ballX, ballY, 0, 50, 0);
		} else {
			ballCurrN = 0;

			// getting rid of edge cases
			if (ballX == 1){
				if (ballStateMachine == 0){
					ballStateMachine = 1;
				}
				if (ballStateMachine == 4){
					ballStateMachine = 5;
				}
			} else if (ballX == 14){
				if (ballStateMachine == 7){
					ballStateMachine = 6;
				}
				if (ballStateMachine == 3){
					ballStateMachine = 2;
				}
			}

			// left side bounce
			if (ballX == 0){
				if (ballStateMachine == 0) ballStateMachine = 3;
				if (ballStateMachine == 1) ballStateMachine = 2;
				if (ballStateMachine == 4) ballStateMachine = 7;
				if (ballStateMachine == 5) ballStateMachine = 6;
			}
			// right side bounce
			if (ballX == 15){
				if (ballStateMachine == 2) ballStateMachine = 1;
				if (ballStateMachine == 3) ballStateMachine = 0;
				if (ballStateMachine == 6) ballStateMachine = 5;
				if (ballStateMachine == 7) ballStateMachine = 4;
			}
			// top bounce
			if (ballY == 15){
				ballStateMachine += 4;
			}
			// bottom bounce
			if (ballY == 2){
				if (ballX == pot){
					ballStateMachine = 0;
				} else if (ballX == pot + 1){
					ballStateMachine = 1;
				} else if (ballX == pot + 2){
					ballStateMachine = 2;
				} else if (ballX == pot + 3){
					ballStateMachine = 3;
				} else {
					lose = 1; // ggs
				}
			}

			// animate ball
			if (ballStateMachine == 0){
				ballX -= 2;
				ballY += 1;
			} else if (ballStateMachine == 1){
				ballX -= 1;
				ballY += 1;
			} else if (ballStateMachine == 2){
				ballX += 1;
				ballY += 1;
			} else if (ballStateMachine == 3){
				ballX += 2;
				ballY += 1;
			} else if (ballStateMachine == 4){
				ballX -= 2;
				ballY -= 1;
			} else if (ballStateMachine == 5){
				ballX -= 1;
				ballY -= 1;
			} else if (ballStateMachine == 6){
				ballX += 1;
				ballY -= 1;
			} else if (ballStateMachine == 7){
				ballX += 2;
				ballY -= 1;
			}
			setPixel(ballX, ballY, 0, 50, 0);
		}
		showLeds();
		HAL_Delay(30);
	}

	// lose animation
	for (int j = 16; j >= 0; j--){
		for (int i = 0; i < 16; i++){
			setPixel(i, j, 50, 0, 0);
		}
		showLeds();
		HAL_Delay(50);
	}
}


// FULL SCREEN IMAGES: ------------------------------------------------------------

// shows "eecs 373" on the led display
void show373(){
	  storage[224] = (struct ledData){0, 0, 100};
	  storage[225] = (struct ledData){0, 0, 100};
	  storage[226] = (struct ledData){0, 0, 100};
	  storage[228] = (struct ledData){0, 0, 100};
	  storage[229] = (struct ledData){0, 0, 100};
	  storage[230] = (struct ledData){0, 0, 100};
	  storage[232] = (struct ledData){0, 0, 100};
	  storage[233] = (struct ledData){0, 0, 100};
	  storage[234] = (struct ledData){0, 0, 100};
	  storage[236] = (struct ledData){0, 0, 100};
	  storage[237] = (struct ledData){0, 0, 100};
	  storage[238] = (struct ledData){0, 0, 100};
	  storage[223] = (struct ledData){0, 0, 100};
	  storage[219] = (struct ledData){0, 0, 100};
	  storage[215] = (struct ledData){0, 0, 100};
	  storage[211] = (struct ledData){0, 0, 100};
	  storage[192] = (struct ledData){0, 0, 100};
	  storage[193] = (struct ledData){0, 0, 100};
	  storage[194] = (struct ledData){0, 0, 100};
	  storage[196] = (struct ledData){0, 0, 100};
	  storage[197] = (struct ledData){0, 0, 100};
	  storage[198] = (struct ledData){0, 0, 100};
	  storage[200] = (struct ledData){0, 0, 100};
	  storage[204] = (struct ledData){0, 0, 100};
	  storage[205] = (struct ledData){0, 0, 100};
	  storage[206] = (struct ledData){0, 0, 100};
	  storage[191] = (struct ledData){0, 0, 100};
	  storage[187] = (struct ledData){0, 0, 100};
	  storage[183] = (struct ledData){0, 0, 100};
	  storage[177] = (struct ledData){0, 0, 100};
	  storage[160] = (struct ledData){0, 0, 100};
	  storage[161] = (struct ledData){0, 0, 100};
	  storage[162] = (struct ledData){0, 0, 100};
	  storage[164] = (struct ledData){0, 0, 100};
	  storage[165] = (struct ledData){0, 0, 100};
	  storage[166] = (struct ledData){0, 0, 100};
	  storage[168] = (struct ledData){0, 0, 100};
	  storage[169] = (struct ledData){0, 0, 100};
	  storage[170] = (struct ledData){0, 0, 100};
	  storage[172] = (struct ledData){0, 0, 100};
	  storage[173] = (struct ledData){0, 0, 100};
	  storage[174] = (struct ledData){0, 0, 100};
	  storage[129] = (struct ledData){100, 0, 0};
	  storage[130] = (struct ledData){100, 0, 0};
	  storage[131] = (struct ledData){100, 0, 0};
	  storage[134] = (struct ledData){100, 0, 0};
	  storage[135] = (struct ledData){100, 0, 0};
	  storage[136] = (struct ledData){100, 0, 0};
	  storage[137] = (struct ledData){100, 0, 0};
	  storage[140] = (struct ledData){100, 0, 0};
	  storage[141] = (struct ledData){100, 0, 0};
	  storage[142] = (struct ledData){100, 0, 0};
	  storage[127] = (struct ledData){100, 0, 0};
	  storage[123] = (struct ledData){100, 0, 0};
	  storage[118] = (struct ledData){100, 0, 0};
	  storage[116] = (struct ledData){100, 0, 0};
	  storage[112] = (struct ledData){100, 0, 0};
	  storage[100] = (struct ledData){100, 0, 0};
	  storage[105] = (struct ledData){100, 0, 0};
	  storage[111] = (struct ledData){100, 0, 0};
	  storage[94] = (struct ledData){100, 0, 0};
	  storage[93] = (struct ledData){100, 0, 0};
	  storage[92] = (struct ledData){100, 0, 0};
	  storage[87] = (struct ledData){100, 0, 0};
	  storage[83] = (struct ledData){100, 0, 0};
	  storage[82] = (struct ledData){100, 0, 0};
	  storage[81] = (struct ledData){100, 0, 0};
	  storage[68] = (struct ledData){100, 0, 0};
	  storage[72] = (struct ledData){100, 0, 0};
	  storage[79] = (struct ledData){100, 0, 0};
	  storage[63] = (struct ledData){100, 0, 0};
	  storage[59] = (struct ledData){100, 0, 0};
	  storage[56] = (struct ledData){100, 0, 0};
	  storage[52] = (struct ledData){100, 0, 0};
	  storage[48] = (struct ledData){100, 0, 0};
	  storage[33] = (struct ledData){100, 0, 0};
	  storage[34] = (struct ledData){100, 0, 0};
	  storage[35] = (struct ledData){100, 0, 0};
	  storage[39] = (struct ledData){100, 0, 0};
	  storage[44] = (struct ledData){100, 0, 0};
	  storage[45] = (struct ledData){100, 0, 0};
	  storage[46] = (struct ledData){100, 0, 0};
}

void showLogo(){
  // t
  setPixel(0, 11, 204, 0, 0);
  setPixel(1, 12, 204, 0, 0);
  setPixel(1, 11, 204, 0, 0);
  setPixel(1, 10, 204, 0, 0);
  setPixel(1, 9, 204, 0, 0);
  setPixel(2, 11, 204, 0, 0);
  // h
  setPixel(4, 12, 204, 0, 0);
  setPixel(4, 11, 204, 0, 0);
  setPixel(4, 10, 204, 0, 0);
  setPixel(4, 9, 204, 0, 0);
  setPixel(5, 10, 204, 0, 0);
  setPixel(6, 10, 204, 0, 0);
  setPixel(6, 9, 204, 0, 0);
  // e
  setPixel(8, 11, 204, 0, 0);
  setPixel(8, 10, 204, 0, 0);
  setPixel(8, 9, 204, 0, 0);
  setPixel(9, 11, 204, 0, 0);
  setPixel(9, 10, 204, 0, 0);
  setPixel(9, 9, 204, 0, 0);
  setPixel(10, 11, 204, 0, 0);
  setPixel(10, 9, 204, 0, 0);

  // g
  setPixel(0, 7, 234, 24, 24);
  setPixel(0, 6, 234, 24, 24);
  setPixel(0, 5, 234, 24, 24);
  setPixel(0, 3, 234, 24, 24);
  setPixel(1, 7, 234, 24, 24);
  setPixel(1, 5, 234, 24, 24);
  setPixel(1, 3, 234, 24, 24);
  setPixel(2, 7, 234, 24, 24);
  setPixel(2, 6, 234, 24, 24);
  setPixel(2, 5, 234, 24, 24);
  setPixel(2, 4, 234, 24, 24);
  setPixel(2, 3, 234, 24, 24);
  // a
  setPixel(4, 7, 234, 24, 24);
  setPixel(4, 6, 234, 24, 24);
  setPixel(4, 5, 234, 24, 24);
  setPixel(5, 7, 234, 24, 24);
  setPixel(5, 5, 234, 24, 24);
  setPixel(6, 7, 234, 24, 24);
  setPixel(6, 6, 234, 24, 24);
  setPixel(6, 5, 234, 24, 24);
  setPixel(7, 5, 234, 24, 24);
  // m
  setPixel(8, 7, 234, 24, 24);
  setPixel(8, 6, 234, 24, 24);
  setPixel(8, 5, 234, 24, 24);
  setPixel(9, 7, 234, 24, 24);
  setPixel(10, 7, 234, 24, 24);
  setPixel(10, 6, 234, 24, 24);
  setPixel(10, 5, 234, 24, 24);
  setPixel(11, 7, 234, 24, 24);
  setPixel(12, 7, 234, 24, 24);
  setPixel(12, 6, 234, 24, 24);
  setPixel(12, 5, 234, 24, 24);
  // e
  setPixel(14, 7, 234, 24, 24);
  setPixel(14, 6, 234, 24, 24);
  setPixel(14, 5, 234, 24, 24);
  setPixel(15, 7, 234, 24, 24);
  setPixel(15, 5, 234, 24, 24);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
