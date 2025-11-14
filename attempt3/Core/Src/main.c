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
#define PS2_SELECT      0x0001
#define PS2_L3          0x0002
#define PS2_R3          0x0004
#define PS2_START       0x0008
#define PS2_UP          0x0010
#define PS2_RIGHT       0x0020
#define PS2_DOWN        0x0040
#define PS2_LEFT        0x0080
#define PS2_L2          0x0100
#define PS2_R2          0x0200
#define PS2_L1          0x0400
#define PS2_R1          0x0800
#define PS2_TRIANGLE    0x1000
#define PS2_CIRCLE      0x2000
#define PS2_CROSS       0x4000
#define PS2_SQUARE      0x8000

// CS control macros
#define PS2_CS_LOW()  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)
#define PS2_CS_HIGH() HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

// PS2 Controller variables
int32_t current_value = 50;
uint16_t prev_buttons = 0xFFFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

// PS2 Controller function prototypes
uint8_t PS2_TransferByte(uint8_t data);
uint8_t PS2_ReadController(uint16_t *buttons);
void PS2_Init(void);
void PS2_ProcessButtons(uint16_t buttons);
void PS2_MainTask(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int cursorX = 0;
int cursorY = 0;

uint8_t PS2_TransferByte(uint8_t data) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, 100);
    return rx_data;
}


//debugging below
uint8_t PS2_ReadController(uint16_t *buttons) {
    uint8_t data[9] = {0};

    // Pull ATT low to start communication
    PS2_CS_LOW();
    HAL_Delay(20);

    // Send command sequence
    uint8_t cmd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HAL_SPI_TransmitReceive(&hspi1, cmd, data, 9, 100);
    // Pull ATT high to end communication
    PS2_CS_HIGH();
    HAL_Delay(10);

   //debug
//    printf("Response bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);

     //DON'T validate - just always return success for debugging
    *buttons = (data[4] << 8) | data[3];
   // printf("button value: %x", *buttons);
    return 1;
}

void PS2_Init(void) {
    PS2_CS_HIGH();
    HAL_Delay(500);  // Let controller fully stabilize

    printf("\r\n========================================\r\n");
    printf("PS2 Controller Initialized\r\n");
    printf("========================================\r\n");

    // Flush initial garbage reads - READ BUT IGNORE
    uint16_t dummy;
    for (int i = 0; i < 10; i++) {
        PS2_ReadController(&dummy);
        HAL_Delay(50);
    }

    // Reset button state after flushing
    prev_buttons = 0xFFFF;  // All buttons released

    printf("Ready! Starting value: %ld\r\n\r\n", current_value);
}


void PS2_ProcessButtons(uint16_t buttons) {

    if (buttons == 0xFFFF) {
        return;
    }
    uint16_t pressed = (~buttons) & (prev_buttons);
    prev_buttons = buttons;
    if (pressed == 0) {
        return;
    }

    if (pressed & PS2_UP) {
        cursorY++;
        printf("UP pressed:    %ld\r\n", current_value);
    }

    if (pressed & PS2_DOWN) {
        cursorY--;
        printf("DOWN pressed:  %ld\r\n", current_value);
    }

    if (pressed & PS2_LEFT) {
        cursorX--;
        printf("LEFT pressed:  %ld\r\n", current_value);
    }

    if (pressed & PS2_RIGHT) {
        cursorX++;
        printf("RIGHT pressed: %ld\r\n", current_value);
    }
}


void PS2_MainTask(void){
    uint16_t buttons;

    PS2_ReadController(&buttons);
    PS2_ProcessButtons(buttons);
}

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
				pwmBuffer[RESET_PULSES + (i*24) + 7 - bit] = 32; // CCR value for high
			} else {
				pwmBuffer[RESET_PULSES + (i*24) + 7 - bit] = 16; // CCR value for low
			}
		}

		// write r data
		for (int bit = 0; bit < 8; bit++){
			int temp = (storage[i].r >> bit) & 1; // take one bit at a time
			if (temp == 1){
				// index to spot in buffer
				pwmBuffer[RESET_PULSES + (i*24) + 8 + 7 - bit] = 32; // CCR value for high
			} else {
				pwmBuffer[RESET_PULSES + (i*24) + 8 + 7 - bit] = 16; // CCR value for low
			}
		}

		// write b data
		for (int bit = 0; bit < 8; bit++){
			int temp = (storage[i].b >> bit) & 1; // take one bit at a time
			if (temp == 1){
				// index to spot in buffer
				pwmBuffer[RESET_PULSES + (i*24) + 16 + 7 - bit] = 32; // CCR value for high
			} else {
				pwmBuffer[RESET_PULSES + (i*24) + 16 + 7 - bit] = 16; // CCR value for low
			}
		}
	}
}

void showLeds(){
	writeLedsToBuffer();
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwmBuffer, RESET_PULSES + (NUM_LEDS * 24));
}

// test this
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
  MX_SPI1_Init();
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

    /* USER CODE BEGIN 3 */
	  setPixel(cursorX, cursorY, 0, 0, 0);
	  PS2_MainTask();
	  setPixel(cursorX, cursorY, 30, 30, 30);
	  showLeds();
	  HAL_Delay(30);
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Period = 49;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
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

  /*Configure GPIO pin : PS2_CS_Pin */
  GPIO_InitStruct.Pin = PS2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PS2_CS_GPIO_Port, &GPIO_InitStruct);

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
