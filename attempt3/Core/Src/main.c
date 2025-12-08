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
#include <math.h>
#include <stdlib.h>   // for random
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// stores data for one led
typedef struct ledData{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};


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
#define PS2_X       0x4000
#define PS2_SQUARE      0x8000

// CS control macros
#define PS2_CS_LOW()  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)
#define PS2_CS_HIGH() HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)

// Audio Settings
#define SAMPLE_RATE 20000   // Timer 6 will run at this fixed frequency
#define BUFFER_SIZE 256     // Size of the DMA buffer (processed in two halves of 128)
#define SINE_RES 2048       // Resolution of our sine wave table (larger = smoother)

// notes
#define B2 123.47
#define C3 130.81
#define C3S 138.59
#define D3 146.83
#define D3S 155.56
#define E3 164.81
#define F3 174.61
#define F3S 185.00
#define G3 196.00
#define G3S 207.65
#define A3 220.00
#define A3S 233.08
#define B3 246.94
#define C4 261.63
#define C4S 277.18
#define D4 293.66
#define D4S 311.13
#define E4 329.63
#define F4 349.23
#define F4S 369.99
#define G4 392.00
#define G4S 415.30
#define A4 440.00
#define A4S 466.16
#define B4 493.88
#define C5 523.25
#define C5S 554.37
#define D5 587.33
#define D5S 622.25
#define E5 659.25
#define F5 698.46
#define F5S 739.99
#define G5 783.99
#define G5S 830.61
#define A5 880.00
#define A5S 932.33
#define B5 987.77
#define C6 1046.50
#define C6S 1108.73
#define D6 1174.66
#define D6S 1244.51
#define E6 1318.51
#define F6 1396.91
#define F6S 1479.98
#define G6 1567.98

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

//LCD function definitions

//                   _         _   _
//                  (_)       | | | |
//   _ __ ___   __ _ _ _ __   | |_| |__   ___ _ __ ___   ___
//  | '_ ` _ \ / _` | | '_ \  | __| '_ \ / _ \ '_ ` _ \ / _ \
//  | | | | | | (_| | | | | | | |_| | | |  __/ | | | | |  __/
//  |_| |_| |_|\__,_|_|_| |_|  \__|_| |_|\___|_| |_| |_|\___|
//  ========  composed by Ishan Kumar - 11/25/2025  =========
				          // 1                            //2                             //3                             //4                             //5                             //6                             //7                             //8                             //9                             //10                            //11                            //12                            //13                            //14                            //15                            //16
int bass[128]          = {E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, E3, E4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, D3, D4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, C3, C4, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3, B2, B3 };
int melody[128]        = {B5, B5, G5, G5, E5, E5, E5, E5, 00, E5, 00, C6, B5, B5, A5, A5, B5, B5, G5, G5, E5, E5, E5, E5, 00, E5, 00, C6, B5, B5, A5, A5, B5, B5, G5, G5, E5, E5, E5, E5, 00, E5, 00, C6, B5, B5, A5, A5, B5, B5, C6, B5, B5, C6, B5, B5, C6, B5, B5, B5, B5, B5, B5, B5, B5, B5, G5, G5, E5, E5, E5, E5, 00, E5, 00, C6, B5, B5, A5, A5, B5, B5, G5, G5, E5, E5, E5, E5, 00, E5, 00, C6, B5, B5, A5, A5, B5, B5, G5, G5, E5, E5, E5, E5, 00, E5, 00, C6, B5, B5, A5, A5, B5, B5, C6, B5, B5, C6, B5, B5, C6, B5, B5, B5, B5, B5, B5, B5 };
int counterMelody[128] = {00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, D3S,00, D3S,00, D3S,00, D3S,00, D3S,00, D3S,E6, E6, E6, E6, E6, E6, E6, E6, E6, E6, E6, E6, G6, G6, G6, G6, F6S,F6S,F6S,F6S,F6S,F6S,F6S,F6S,F6S,F6S,F6S,F6S,D6S,D6S,D6S,D6S,E6, E6, E6, E6, E6, E6, E6, E6, E6, E6, E6, E6, D6S,D6S,D6S,D6S,00, 00, 00, 00, 00, D3S,00, D3S,00, D3S,00, D3S,00, D3S,00, D3S};
int chordSupport[128]  = {00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00 ,00, 00 ,00, F3S,00, F3S,00, F3S,00, F3S,00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00 ,00, 00 ,00, F3S,00, F3S,00, F3S,00, F3S};


//                   _         _   _
//  MICHIGAN THEME
				                    // 1                            //2                             //3                             //4                             //5                             //6                             //7                             //8                             //9                             //10                            //11                            //12                            //13                            //14                            //15                            //16
int michiganmelody[128]          = {C5, C5, C5, C5, G4S,G4S,A4S,A4S,C5, C5, G4S,G4S,A4S,A4S,C5, C5, C5S,C5S,C5S,C5S,A4S,A4S,C5, C5, C5S,C5S,A4S,A4S,C5, C5, C5S,C5S,D5S,D5S,D5S,D5S,F5, F5, F5, C5, C5, C5, C5S,C5S,G4S,G4S,A4S,A4S,C5, C5, C5, C5, A4S,A4S,G4S,G4S,D5S,D5S,D5S,D5S,D5S,D5S,D5S,D5S,C5, C5, C5, C5, G4S,G4S,A4S,A4S,C5, C5, G4S,G4S,A4S,A4S,C5, C5, C5S,C5S,C5S,C5S,A4S,A4S,C5, C5, C5S,C5S,A4S,A4S,C5, C5, C5S,C5S,D5S,D5S,D5S,D5S,F5, F5, F5, C5, C5, C5, C5S,C5S,G4S,G4S,A4S,A4S,C5, C5, D5S,D5S,C5, C5, C5, A4S,G4S,G4S,G4S,G4S,G4S,G4S,G4S,G4S};
int michigancounterMelody[128]   = {G4S,G4S,G4S,G4S,00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, A4S,A4S,A4S,A4S,00, 00, G4S,G4S,G4, G4, D4S,D4S,F4, F4, G4, G4, G4S,G4S,F4S,F4S,F4, F4, C4, C4, G3S,G3S,A3S,A3S,C4, C4, C4S,C4S,D4, D4, D4, D4, D4, D4, D4, D4, D4S,D4S,D4S,D4S,D4S,D4S,D4S,D4S,G4S,G4S,G4S,G4S,00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, A4S,A4S,A4S,A4S,00, 00, G4S,G4S,G4, G4, D4S,D4S,F4, F4, G4, G4, G4S,G4S,F4S,F4S,F4, F4, C4, C4, G3S,G3S,A3S,A3S,C4, C4, C4S,C4S,C4, C4, C4, C4, D4S,D4S,D4S,C4S,C4, C4, C4, C4, C4, C4, C4, C4};
int michiganchordSupport[128]    = {00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, G4, G4, G4, G4, G4, G4, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00};
int michiganchordSupportTwo[128] = {00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, A4S,A4S,B4, B4, C5, C5, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00};

//minesweeper variables
#define PANEL_W 16   // physical LEDs per row
#define PANEL_H 16   // physical rows
int GRID_W = 16;
int GRID_H = 16;
#define MAX_W 16
#define MAX_H 16

int bombCount = 20; //(default medium)

typedef enum {
    TILE_EMPTY = 0,
    TILE_1,
    TILE_2,
    TILE_3,
    TILE_BOMB
} TileContent;

typedef enum {
    STATE_HIDDEN = 0,
    STATE_REVEALED,
    STATE_FLAGGED
} TileState;

typedef enum {
    MODE_EASY = 0,
    MODE_MEDIUM,
    MODE_HARD
} GameMode;




TileContent board[MAX_H][MAX_W];
TileState  tileState[MAX_H][MAX_W];
int animRevealStep[MAX_H][MAX_W];
//this is gonna break bruh


int gameOver = 0;
int gameWon = 0;
int gameLoss = 0;
int powerCycle = 0;
int firstXPress = 0;
int gameStart = 0;
int winPulseTimer = 0;
int testMode = 0;
int flagsPlaced = 0;
int mineSweeper_initialized = 0;

int floodAnimating = 0;
int floodAnimFrame = 0;
int floodAnimTimer = 0;
int floodMaxStep = 0;

int cursorX = 0;
int cursorY = 0;

int flagSoundToggle = 0;
int playedLosingSoundEffect = 0;
int playedWinningSoundEffect = 0;
int writeMode = 0;
int writePlaying = 0;
int gameSelect = 1;
int minesweeper = 0;
int pong = 0;




//int bombsRemaining = bombCount - flagsPlaced; //FOR VARUN

//gamemode stuff

GameMode currentMode = MODE_MEDIUM;

// PS2 Controller variables
int32_t current_value = 50;
uint16_t prev_buttons = 0xFFFF;

// storage for whole screen
#define MAX_LEDS (MAX_W * MAX_H)
struct ledData storage[MAX_LEDS];
int NUM_LEDS =  MAX_LEDS;;  // default

// buffer storage
#define RESET_PULSES 60 // need this because stm has done nothing but ruin my life and im tired of it
uint32_t pwmBuffer[RESET_PULSES + (MAX_LEDS * 24)]; // 24 bits per led (plus our 60 0s at the start)

// audio stuffs
uint32_t dmaBuffer[BUFFER_SIZE];
int16_t sineLUT[SINE_RES];

// Voice Structure (Represents one "Note")
typedef struct {
    uint8_t active;
    float frequency;
    float phase;       // Current position in the sine wave
    float phaseStep;   // How much to step forward per sample
} Voice;

// 5th voice (index 4) reserved for sound effects
#define MAX_VOICES 5
Voice voices[MAX_VOICES];

// Initialization Flag
uint8_t audioInitialized = 0;

//brightness variable
uint8_t brightness = 40;

int playingTheme = 0; //0 = theme 1 = win music

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_game_win(void);
void LCD_game_loss(void);
void LCD_write_test(void);
void LCD_write_hello(int ms, int pong);
void LCD_set_bomb(GameMode mode);
void LCD_game_mode(GameMode mode);
void LCD_update_bomb(int bombCnt, GameMode mode);
void SetGameMode(GameMode mode);

// PS2 Controller function prototypes
uint8_t PS2_TransferByte(uint8_t data);
uint8_t PS2_ReadController(uint16_t *buttons);
void PS2_Init(void);
void PS2_ProcessButtons(uint16_t buttons);
void PS2_MainTask(void);


//minesweeper functions
void Minesweeper_Render(void);
void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
void revealTileAtCursor(void);
void toggleFlagAtCursor(void);
void onXPress(void);
void floodReveal(int startX, int startY);
void checkWin(void);
void Minesweeper_InitBoard(void);
//16 x 16 grid for now

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//gamemode stuff
void SetGameMode(GameMode mode) {
    currentMode = mode;

    if (mode == MODE_EASY) {
        GRID_W = 10;
        GRID_H = 10;
        bombCount = 10;
    }
    else if (mode == MODE_MEDIUM) {
        GRID_W = 16;
        GRID_H = 16;
        bombCount = 30;
    }
    else if (mode == MODE_HARD) {
        GRID_W = 24;
        GRID_H = 24;
        bombCount = 40;
    }

    NUM_LEDS = PANEL_W * PANEL_H;

    Minesweeper_InitBoard();
    cursorX = 0;
    cursorY = 0;
    firstXPress = 0;
    flagsPlaced = 0;
}

void InitAudio() {
    // 1. Generate a high-res sine table
    for(int i = 0; i < SINE_RES; i++) {
        // Range: -1900 to +1900
        sineLUT[i] = (int16_t)(1900.0f * sinf(2.0f * 3.14159f * (float)i / (float)SINE_RES));
    }

    // 2. Clear voices
    for(int i = 0; i < MAX_VOICES; i++) {
        voices[i].active = 0;
        voices[i].phase = 0;
    }

    // 3. Start Timer at FIXED frequency (e.g. 22kHz)
    // Note: Configure TIM6 in CubeMX to trigger at 22000Hz
    HAL_TIM_Base_Start(&htim6);

    // 4. Start DAC with Circular DMA
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dmaBuffer, BUFFER_SIZE, DAC_ALIGN_12B_R);

    audioInitialized = 1;
}

// Process a portion of the buffer (start to end)
void FillBuffer(uint32_t *buffer, int start, int length) {
    for (int i = 0; i < length; i++) {
        int32_t sampleAccumulator = 0;
        int activeCount = 0;

        // Loop through all voices (Polyphony!)
        for (int v = 0; v < MAX_VOICES; v++) {
            if (voices[v].active) {
                // 1. Get the integer part of the phase index
                int idx = (int)voices[v].phase;

                // 2. Add sample from LUT to accumulator
                sampleAccumulator += sineLUT[idx];

                // 3. Advance phase
                voices[v].phase += voices[v].phaseStep;

                // 4. Wrap phase if it exceeds table size
                if (voices[v].phase >= SINE_RES) {
                    voices[v].phase -= SINE_RES;
                }
                activeCount++;
            }
        }

        // 5. Handling Volume/Clipping
        // If we simply add two waves of 1900 amplitude, we get 3800.
        // 3800 + 2048 (center) = 5848 -> DAC Overflow!
        // Simple fix: Divide result by number of active voices or a fixed number.
        if (activeCount > 0) {
            sampleAccumulator /= 5; // Auto-gain
        }

        // 6. Write to buffer: Center Offset + Mixed Signal
        buffer[start + i] = (uint32_t)(2048 + sampleAccumulator);
    }
}

// Called when first half of buffer is played
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    FillBuffer(dmaBuffer, 0, BUFFER_SIZE / 2);
}

// Called when second half of buffer is played
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    FillBuffer(dmaBuffer, BUFFER_SIZE / 2, BUFFER_SIZE / 2);
}

void PlayNote(int voiceIndex, float freq) {
    if (voiceIndex >= MAX_VOICES) return;

    if (freq == 0) {
        voices[voiceIndex].active = 0;
    } else {
        // Calculate Step Size
        // Formula: Step = (LUT_Size * Freq) / Sample_Rate
        voices[voiceIndex].phaseStep = ((float)SINE_RES * freq) / (float)SAMPLE_RATE;
        voices[voiceIndex].active = 1;
    }
}

void Minesweeper_InitBoard(void) {
    // 0) Reset state
	mineSweeper_initialized = 1;
	flagsPlaced = 0;
    gameOver = 0;
    gameWon  = 0;
    gameLoss = 0;
    winPulseTimer = 0;
    playingTheme = 0;
    //animation stuff
    floodAnimating = 0;
    floodAnimFrame = 0;
    floodAnimTimer = 0;
    floodMaxStep   = 0;

    // clear state
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            board[y][x] = TILE_EMPTY;
            tileState[y][x] = STATE_HIDDEN;
        }
    }

    // bombs
    if (testMode) {
        //one bomb in middle test mode
        int cx = GRID_W / 2;   // for 16 to 8
        int cy = GRID_H / 2;   // for 16 to 8
        if (cx >= GRID_W) cx = GRID_W - 1;
        if (cy >= GRID_H) cy = GRID_H - 1;
        board[cy][cx] = TILE_BOMB;
    } else {
       //normal
        srand(HAL_GetTick());

        int numBombs = bombCount;      // adjutsable
        int placed = 0;

        while (placed < numBombs) {
            int rx = rand() % GRID_W;
            int ry = rand() % GRID_H;

            if (board[ry][rx] != TILE_BOMB) {
                board[ry][rx] = TILE_BOMB;
                placed++;
            }
        }
    }

    // adjacency computing
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {

            if (board[y][x] == TILE_BOMB)
                continue;

            int neighbors = 0;

            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
                        continue;

                    if (board[ny][nx] == TILE_BOMB)
                        neighbors++;
                }
            }

            // can add more with diffculty scaling, net steps
            switch (neighbors) {
                case 0: board[y][x] = TILE_EMPTY; break;
                case 1: board[y][x] = TILE_1;     break;
                case 2: board[y][x] = TILE_2;     break;
                default: board[y][x] = TILE_3;    break;
            }
        }
    }
}




//inprogress // see if i can make it a ripple

void checkWin(void){
	if(gameOver) return;
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            if (board[y][x] != TILE_BOMB &&
                tileState[y][x] != STATE_REVEALED) {

                return;
            }
        }
    }
    //if you go through the entire board, then we have a win
    //status check
    gameOver = 1;
    gameWon  = 1;
}



//not fulyl working yet
void floodReveal(int startX, int startY) {
    // clear animation
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            animRevealStep[y][x] = -1;
        }
    }

    int qx[GRID_W * GRID_H];
    int qy[GRID_W * GRID_H];
    int qd[GRID_W * GRID_H];
    int head = 0;
    int tail = 0;

    qx[tail] = startX;
    qy[tail] = startY;
    qd[tail] = 0;
    tail++;

    floodMaxStep = 0;

    while (head < tail) {
        int x = qx[head];
        int y = qy[head];
        int d = qd[head];
        head++;

        // bounds
        if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H)
            continue;

        // skip bombs
        if (board[y][x] == TILE_BOMB)
            continue;

        // skip flagged
        if (tileState[y][x] == STATE_FLAGGED ||
            tileState[y][x] == STATE_REVEALED)
            continue;

        // logical reveal
        tileState[y][x] = STATE_REVEALED;

        // rstore
        animRevealStep[y][x] = d;
        if (d > floodMaxStep) {
            floodMaxStep = d;
        }

        // problem here i think
        if (board[y][x] == TILE_EMPTY) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
                        continue;

                    if (board[ny][nx] == TILE_BOMB)
                        continue;

                    if (tileState[ny][nx] == STATE_HIDDEN) {
                        if (tail < GRID_W * GRID_H) {
                            qx[tail] = nx;
                            qy[tail] = ny;
                            qd[tail] = d + 1;
                            tail++;
                        }
                    }
                }
            }
        }
    }

    // enable
    floodAnimating = 1;
    floodAnimFrame = 0;
    floodAnimTimer = 0;
}


static inline uint8_t scaleColor(uint8_t c) {
    return (c * brightness) / 255;
}

static void colorForNumber(TileContent t, uint8_t *r, uint8_t *g, uint8_t *b) {
    switch (t) {
        case TILE_1: *r = 0;   *g = 0;   *b = 180; break; // blue
        case TILE_2: *r = 0;   *g = 180; *b = 0;   break; // green
        //case TILE_3: *r = 255; *g = 120;   *b = 0;   break; // orange
        case TILE_3: *r = 220; *g = 0;   *b = 140;   break; // pink
        default:      *r = 0;   *g = 0;   *b = 0;   break;
    }
}


void Minesweeper_Render(void) {
    for (int y = 0; y < PANEL_H; y++) {
        for (int x = 0; x < PANEL_W; x++) {

            uint8_t r = 0, g = 0, b = 0;

            // We are OUTSIDE the game grid
            if (x >= GRID_W || y >= GRID_H) {
            	r = 0; g = 0; b = 0;
            }
            else {

                if (gameOver && !gameWon && board[y][x] == TILE_BOMB) {
                    r = 200; g = 0; b = 0;
                } else {
                    switch (tileState[y][x]) {
                        case STATE_HIDDEN:
                            r = g = b = 0;
                            break;

                        case STATE_FLAGGED:
                            r = 200; g = 200; b = 0;
                            break;

                        case STATE_REVEALED:
                            if (floodAnimating &&
                                animRevealStep[y][x] >= 0 &&
                                animRevealStep[y][x] > floodAnimFrame)
                            {
                                r = g = b = 0;
                            }
                            else {
                                if (board[y][x] == TILE_BOMB) {
                                    r = 200; g = 0; b = 0;
                                } else if (board[y][x] == TILE_EMPTY) {
                                    r = g = b = 40;
                                } else {
                                    colorForNumber(board[y][x], &r, &g, &b);
                                }
                            }
                            break;
                    }
                }

                if (gameWon) {
                    int pulse = winPulseTimer % 100;
                    int bright = (pulse < 50) ? pulse : (100 - pulse);
                    r = 0; g = bright * 4; b = 0;
                }

                if (!gameWon && x == cursorX && y == cursorY) {
                    r = 255; g = 0; b = 0;
                }
                // ==============================================================
            }

            setPixel(x, y,
                     scaleColor(r),
                     scaleColor(g),
                     scaleColor(b));
        }
    }
}

void revealTileAtCursor(void) {
    if (gameOver) return;

    int x = cursorX;
    int y = cursorY;

    // don't reveal already revealed or flagged
    if (tileState[y][x] == STATE_REVEALED ||
        tileState[y][x] == STATE_FLAGGED) {
        return;
    }

    // bomb end game
    if (board[y][x] == TILE_BOMB) {
        tileState[y][x] = STATE_REVEALED;
        gameOver = 1;
        gameLoss = 1;
        return;
    }


    if (board[y][x] == TILE_EMPTY) {
        floodReveal(x, y);
        checkWin();
        return;
    }

    tileState[y][x] = STATE_REVEALED;
    checkWin();
}



void toggleFlagAtCursor(void) {

    if (gameOver) return;

    TileState *s = &tileState[cursorY][cursorX];

    // Don't allow flagging revealed tiles
    if (*s == STATE_REVEALED) {
        return;    // <-- early exit prevents crashes
    }

    int bombsRemaining;

    if (*s == STATE_HIDDEN) {
        *s = STATE_FLAGGED;
        flagsPlaced++;
    } else if (*s == STATE_FLAGGED) {
        *s = STATE_HIDDEN;
        flagsPlaced--;
    }

    bombsRemaining = flagsPlaced;

    writeMode = 0;
    if (!writeMode) {
        LCD_update_bomb(bombsRemaining, currentMode);
        writeMode = 1;
    }
}


void onXPress() {
	revealTileAtCursor();
}


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

    // Flush initial garbage reads - READ BUT IGNORE
    uint16_t dummy;
    for (int i = 0; i < 10; i++) {
        PS2_ReadController(&dummy);
        HAL_Delay(50);
    }

    // Reset button state after flushing
    prev_buttons = 0xFFFF;  // All buttons released
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
    if (pressed & PS2_START){
    	gameSelect = 1;
    	minesweeper = 0;
    	pong = 0;
    	clearLED();
    	//
    	showLogo();
    	LCD_select_game();

    	//clearLED();
		//showLogo();
    	//showLeds();
    }
    if(gameSelect){
    	showLeds();

    	if (pressed & PS2_X) {
    		//LCD_PrintStr("X:GM");
    		minesweeper = 1;
			pong = 0;
			gameSelect = 0;
			playMinesweeper();
			//LCD_write_hello(minesweeper, pong);

    	}

		if(pressed & PS2_CIRCLE){
			LCD_PrintStr("O:GM");
			minesweeper = 0;
			pong = 1;
			gameSelect = 0;
			//LCD_write_hello(minesweeper, pong);

		}

    }
    else if(minesweeper){
		if (pressed & PS2_UP) {
			if (cursorY == GRID_H - 1){
				cursorY = 0;
			} else {
				cursorY++;
			}
		}

		if (pressed & PS2_DOWN) {
			if (cursorY == 0){
				cursorY = GRID_H - 1;
			} else {
				cursorY--;
			}
		}

		if (pressed & PS2_LEFT) {
			if (cursorX == 0){
				cursorX = GRID_W - 1;
			} else {
				cursorX--;
			}
		}

		if (pressed & PS2_RIGHT) {
			if (cursorX == GRID_W - 1){
				cursorX = 0;
			} else {
				cursorX++;
			}
		}

		if (pressed & PS2_X) {
			writePlaying = 1;
			if(!firstXPress){
				LCD_set_bomb(currentMode);
				firstXPress = 1;

			}
			if (writePlaying) {             // executes only once per press
				writePlaying = 0;           // clear it immediately so no flicker
			}

			onXPress();
		}
					// always run the X action


		if (pressed & PS2_CIRCLE) { //minsweeper flag
			toggleFlagAtCursor();
			flagSoundToggle = 1;
		}

		if (pressed & PS2_TRIANGLE) {
			testMode = 0;
			Minesweeper_InitBoard();
			firstXPress = 0;
			cursorX = 0;
			cursorY = 0;
			playedLosingSoundEffect = 0;
			playedWinningSoundEffect = 0;
			writePlaying = 1;
			if(writePlaying){
				LCD_write_hello(minesweeper, pong);
				writePlaying = 0;
			}

		}
		if (pressed & PS2_SQUARE) {
			// tyoggle test
			if (testMode == 0) {
				testMode = 1;  // enter  test mode
			} else {
				testMode = 0;  // go back to normal random mode
			}
			writeMode = 0;
			if(!writeMode && testMode){
				LCD_write_test();
				writeMode = 1;
			}

			writeMode = 0;
			if(!writeMode && !testMode){
				LCD_game_mode(currentMode);
				writeMode = 1;
			}

			Minesweeper_InitBoard();  // rebuild board using new mode
			cursorX = 0;
			cursorY = 0;
		}
		//CHANGE TO OPEN THINGS
		if (pressed & PS2_L1) {
			writeMode = 0;// easy mode

			SetGameMode(MODE_EASY);
			if(!writeMode){
				LCD_game_mode(MODE_EASY);
				writeMode = 1;
			}

		}

		if (pressed & PS2_R1) {                // medium mode
			writeMode = 0;
			SetGameMode(MODE_MEDIUM);
			if(!writeMode){
				LCD_game_mode(MODE_MEDIUM);
				writeMode = 1;
			}
		}

		if (pressed & PS2_R2) {                // hard mode
			writeMode = 0;
			SetGameMode(MODE_HARD);
			if(!writeMode){
				LCD_game_mode(MODE_HARD);
				writeMode = 1;
			}
		}
    }
    if(pong){
    	//do something here
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

void clearLED(){
	for (int y = 0; y < GRID_H; y++) {
	        for (int x = 0; x < GRID_W; x++) {
	            setPixel(x, y, 0, 0, 0);
	        }
	    }

}
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
	  //realIndex = y * GRID_W + x;
	  realIndex = y * PANEL_W + x;
  } else {
    // odd row (reversed)
	  //realIndex = y * GRID_W + (GRID_W - 1 - x);
	  realIndex = y * PANEL_W + (PANEL_W - 1 - x);
  }

  storage[realIndex] = (struct ledData){r, g, b};
}

void playMinesweeper(){
	LCD_write_hello(minesweeper, pong);
	Minesweeper_InitBoard();
	Minesweeper_Render();  // draw entire board + cursor into storage[]
	showLeds();            // send storage[] out via DMAf

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
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init(&hi2c1);

  InitAudio();

  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwmData, 4);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  PS2_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int currNote = 0;
  int prescalerForMusic = 0;

  while (1)
  {
	// if game over, play sad trombone
	if (gameLoss && !playedLosingSoundEffect){
		LCD_game_loss();
	    PlayNote(0, D5);
	    PlayNote(1, 0);
	    PlayNote(2, 0);
	    PlayNote(3, 0);
	    HAL_Delay(600);
	    PlayNote(0, C5S);
	    HAL_Delay(600);
	    PlayNote(0, C5);
	    HAL_Delay(600);
	    PlayNote(0, B4);
	    HAL_Delay(3000);
	    playedLosingSoundEffect = 1;
	}

	if(gameWon && !playedWinningSoundEffect){
		playingTheme = 1;
		currNote = 0;
		LCD_game_win();
		playedWinningSoundEffect = 1;
	}

	// play music
	if (prescalerForMusic < 2){
		prescalerForMusic++;
	} else {
		prescalerForMusic = 0;

		// every 185 ms
		if (currNote == 128) {
			currNote = 0;
		} else if(!playingTheme){
		    PlayNote(0, bass[currNote]);
		    PlayNote(1, melody[currNote]);
		    PlayNote(2, counterMelody[currNote]);
		    PlayNote(3, chordSupport[currNote]);
		    PlayNote(4, 0);

		    if (flagSoundToggle){
			    PlayNote(4, C5);
			    flagSoundToggle = 0;
		    }

		    currNote++;
		} else {
			// playing michigan theme
		    PlayNote(0, michiganmelody[currNote]);
		    PlayNote(1, michigancounterMelody[currNote]);
		    PlayNote(2, michiganchordSupport[currNote]);
		    PlayNote(3, michiganchordSupportTwo[currNote]);
		    currNote++;

		}
	}
	//LCD_PrintStr(" PRESS START TO PLAY");
	PS2_MainTask();        // read controller, update cursor + buttons
//	if(gameSelect){
//		writeMode = 0;
//		if(writeMode){
//			LCD_select_game();
//			writeMode = 1;
//		}
//
//		//gameSelect = 0;
	if(gameSelect){
		//showLogo();
		showLeds();
	}
//	}
	if(minesweeper){
//		LCD_write_hello(minesweeper, pong);
//		if(!mineSweeper_initialized){
//			Minesweeper_InitBoard();
//		}
		Minesweeper_Render();  // draw entire board + cursor into storage[]
		showLeds();            // send storage[] out via DMAf
	}
//	if(pong){
//		LCD_write_hello(minesweeper, pong);
//	}


    if (gameWon) {
        winPulseTimer++;

    }
//    if(!firstXPress){
//    	if(!writeMode){
//    		LCD_write_hello(minesweeper, pong);
//    		writeMode = 1;
//    	}
//    }
    //anjimattion
    if (floodAnimating) {
        floodAnimTimer++;
                 if (floodAnimTimer >= 3) {
            floodAnimTimer = 0;
            floodAnimFrame++;

            if (floodAnimFrame > floodMaxStep) {
                floodAnimating = 0;
            }
        }
    }

    HAL_Delay(5);

    /* USER CODE END WHILE */

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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00D09BE3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 19;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
