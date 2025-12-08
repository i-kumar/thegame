#include "stm32l4xx_hal.h"
#include "game.h"
#include <stdio.h>
/* -------------------- I2C LCD Definitions ----------------------------------
 * The HD44780 LCD normally talks over parallel bus. The I2C breakout (PCF8574)
 * converts serial I2C → 8-bit GPIO pins:
 *
 *      PCF8574 Pin → LCD Function
 *      --------------------------
 *      P7 -> D7
 *      P6 -> D6
 *      P5 -> D5
 *      P4 -> D4
 *      P3 -> Backlight Control
 *      P2 -> Enable (E)
 *      P1 -> RW (unused, write always)
 *      P0 -> RS (0=Command, 1=Data)
 *
 * Because the LCD powers up in 8-bit mode, but we send only high nibble over
 * I2C, initialization requires a special sequence to switch to 4-bit mode.
 * --------------------------------------------------------------------------*/


#define DEVICE_ADDR        (0x27 << 1) // your module address
#define LCD_BACKLIGHT      0x08        // Backlight ON bit
#define LCD_NOBACKLIGHT    0x00

#define ENABLE_BIT         0x04        // Toggles LCD latch
#define RS_BIT             0x01        // 0 = command, 1 = data

/* LCD Commands */
#define LCD_CLEARDISPLAY 0x01        // Clears display & resets cursor to position 0
#define LCD_RETURNHOME 0x02          // Moves cursor to home without clearing display
#define LCD_ENTRYMODESET 0x04        // Base command for entry mode configuration
#define LCD_DISPLAYCONTROL 0x08      // Base command for turning display/cursor on/off
#define LCD_CURSORSHIFT 0x10         // Base command for shifting cursor/display
#define LCD_FUNCTIONSET 0x20         // Base for setting bus mode, font, line count
#define LCD_SETCGRAMADDR 0x40        // Sets address in CGRAM (custom chars)
#define LCD_SETDDRAMADDR 0x80        // Sets address in DDRAM (normal cursor position)

/* LCD Mode Flags */
#define LCD_4BITMODE       0x00
#define LCD_2LINE          0x08
#define LCD_5x8DOTS        0x00

/* Entry mode options */
#define LCD_ENTRYLEFT      0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Display config flags */
#define LCD_DISPLAYON      0x04
#define LCD_CURSORON       0x02
#define LCD_BLINKOFF       0x00
#define LCD_BLINKON 	   0x01

/*Cursor Shift*/
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00



/* ---- static config state ---- */
static I2C_HandleTypeDef *LCD_i2c;
static uint8_t _backlight = LCD_BACKLIGHT;
static uint8_t _displayControl;
static uint8_t _entryMode;
static uint8_t _rows = 2;




/* ------------ LOW-LEVEL I2C SEND FUNCTIONS ---------------- */

static void lcd_write_i2c(uint8_t data)
{
    HAL_I2C_Master_Transmit(LCD_i2c, DEVICE_ADDR, &data, 1, HAL_MAX_DELAY);
}

// Toggle "Enable" to latch bits into LCD
static void lcd_pulse_enable(uint8_t data)
{
    lcd_write_i2c(data | ENABLE_BIT);
    HAL_Delay(1);
    lcd_write_i2c(data & ~ENABLE_BIT);
    HAL_Delay(1);
}

// Send 4 bits (high nibble already shifted)
static void lcd_write4(uint8_t value, uint8_t mode)
{
    uint8_t data = (value & 0xF0) | mode | _backlight;
    lcd_pulse_enable(data);
}

// Send full 8-bit command by splitting into two 4-bit transmissions
static void lcd_send(uint8_t value, uint8_t mode)
{
	uint8_t highnib = value & 0xF0;
	uint8_t lownib = (value<<4) & 0xF0;

    lcd_write4(highnib, mode);
    lcd_write4(lownib, mode);
}


/* -------------------- PUBLIC LCD FUNCTIONS -------------------- */

void LCD_Clear()
{
    lcd_send(LCD_CLEARDISPLAY, 0);
    HAL_Delay(2);
}

void LCD_Home()
{
    lcd_send(LCD_RETURNHOME, 0);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row)
{
    uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send(LCD_SETDDRAMADDR | (col + offsets[row]), 0);
}

void LCD_Backlight()
{
    _backlight = LCD_BACKLIGHT;
    lcd_write_i2c(_backlight);
}

void LCD_NoBacklight()
{
    _backlight = LCD_NOBACKLIGHT;
    lcd_write_i2c(_backlight);
}

void LCD_Display()
{
    _displayControl |= LCD_DISPLAYON;
    lcd_send(LCD_DISPLAYCONTROL | _displayControl, 0);
}

void LCD_NoDisplay()
{
    _displayControl &= ~LCD_DISPLAYON;
    lcd_send(LCD_DISPLAYCONTROL | _displayControl, 0);
}

void LCD_Cursor()
{
    _displayControl |= LCD_CURSORON;
    lcd_send(LCD_DISPLAYCONTROL | _displayControl, 0);
}

void LCD_NoCursor()
{
    _displayControl &= ~LCD_CURSORON;
    lcd_send(LCD_DISPLAYCONTROL | _displayControl, 0);
}

void LCD_ResetDisplayShift(void)
{
    lcd_send(LCD_SETDDRAMADDR | 0x00, 0);  // Reset cursor to start
}


//void LCD_ScrollLeft()
//{
//    // Move whole display, cursor stays logically in place
//    lcd_send(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT, 0);
//}

//void LCD_ScrollText(const char *text, uint16_t delayMs)
//{
//    LCD_Clear();
//    LCD_Home();
//    LCD_SetCursor(0, 0);
//    LCD_PrintStr(text);
//
//    // Scroll the display left for each character
//    for (int i = 0; i < strlen(text); i++) {
//        HAL_Delay(delayMs);
//        LCD_ScrollDisplayLeft();
//    }
//}



void LCD_LeftToRight()
{
  //dpMode |= LCD_ENTRYLEFT;
  lcd_send(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
}


void LCD_ScrollDisplayRight()
{
    lcd_send(LCD_CURSORSHIFT | 0x0C, 0);
}

void LCD_ScrollDisplayLeft()
{
    lcd_send(LCD_CURSORSHIFT | 0x08, 0);
}


void LCD_PrintStr(const char *str)
{
    while (*str)
        lcd_send(*str++, RS_BIT);
}


void LCD_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7;
  lcd_send(LCD_SETCGRAMADDR | (location << 3), 0);
  for (int i=0; i<8; i++)
  {
    lcd_send(charmap[i], RS_BIT);
  }
}

void LCD_PrintSpecialChar(uint8_t index)
{
  lcd_send(index, RS_BIT);
}

/* -------------------- LCD INITIALIZATION -------------------- */

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    LCD_i2c = hi2c;
    _rows = 2;

    HAL_Delay(50);

    // Init sequence (LCD still in 8-bit mode)
    lcd_write4(0x30, 0);
    HAL_Delay(5);
    lcd_write4(0x30, 0);
    HAL_Delay(1);
    lcd_write4(0x30, 0);

    // Switch to 4-bit mode
    lcd_write4(0x20, 0);

    // Function set: 4-bit mode, lines, font
    lcd_send(LCD_FUNCTIONSET | LCD_4BITMODE |LCD_2LINE | LCD_5x8DOTS, 0);

    // Display ON, cursor off by default
    _displayControl = LCD_DISPLAYON;
    lcd_send(LCD_DISPLAYCONTROL | _displayControl, 0);

    // Entry mode: Left to right (requested)
    _entryMode = LCD_ENTRYLEFT;
    lcd_send(LCD_ENTRYMODESET | _entryMode, 0);

    LCD_Clear();
    LCD_Home();
	LCD_PrintStr("    WELCOME!   ");
	LCD_SetCursor(0, 1);
	LCD_PrintStr("  PRESS START  ");
}

void LCD_game_win(){
	LCD_Clear();
	LCD_Home();
	LCD_PrintStr("HUZZAH!");
	LCD_SetCursor(0, 1);
	LCD_PrintStr("YOU WIN!!!");


}

void LCD_game_loss(){
	LCD_Clear();
	LCD_Home();
	LCD_PrintStr("WOMP WOMP");
	LCD_SetCursor(0, 1);
	LCD_PrintStr("YOU LOST :(");
}

void LCD_game_mode(GameMode mode){
	LCD_Clear();
	LCD_Home();
	LCD_SetCursor(0, 0);
	if(mode == MODE_EASY){
		LCD_PrintStr("EASY MODE");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("~For John");
	}
	else if(mode == MODE_MEDIUM){
		LCD_PrintStr("MEDIUM MODE");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("~For Kannan");
	}else if(mode == MODE_HARD){
		LCD_PrintStr("HARD MODE");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("~For Matt&James");

	}
}
void LCD_write_test(){
	LCD_Clear();
	LCD_Home();
	LCD_SetCursor(0, 0);
	LCD_PrintStr("TEST MODE");
	LCD_SetCursor(0, 1);
	LCD_PrintStr("~For Vish");

}

void LCD_set_bomb(GameMode mode){
	LCD_Clear();
	LCD_Home();
	LCD_SetCursor(0, 0);
	LCD_PrintStr("BOMB COUNTER:");
	LCD_SetCursor(0, 1);
	if(mode == MODE_EASY){
		LCD_PrintStr("0 out of 10");
	}
	else if(mode == MODE_MEDIUM){
		LCD_PrintStr("0 out of 30");
	}
	else if(mode == MODE_HARD){
		LCD_PrintStr("0 out of 40");
	}
}

void LCD_update_bomb(int bombCnt, GameMode currentMode){
    LCD_SetCursor(0, 1);

    // Clear the full line first (optional but safest)
    LCD_PrintStr("                ");  // 16 spaces
    LCD_SetCursor(0, 1);

    char buffer[4];  // enough for numbers up to 999

    if (bombCnt < 10) {
        // One-digit number → print with leading space
        sprintf(buffer, " %d", bombCnt);
    } else {
        // Two-digit number → no leading space
        sprintf(buffer, "%d", bombCnt);
    }

    LCD_PrintStr(buffer);

    // Re-print the static text
    if (currentMode == MODE_EASY)
        LCD_PrintStr(" out of 10");
    else if (currentMode == MODE_MEDIUM)
        LCD_PrintStr(" out of 30");
    else if (currentMode == MODE_HARD)
        LCD_PrintStr(" out of 40");
}

//void LCD_write_hello(){
//	LCD_Clear();
//	LCD_Home();
//	LCD_SetCursor(0, 0);
//	LCD_PrintStr("Welcome to Minesweeper:");
//	LCD_SetCursor(0, 1);
//	LCD_PrintStr("Select Game Mode");
//}

void LCD_write_hello(int ms, int pong, int pongHS) {
	//int count = 0;
    LCD_Clear();
    LCD_Home();

    // Put full text on the first line (even if longer than 16 chars)
    // Print second line normally
    if(ms){
		LCD_SetCursor(0, 0);
		LCD_PrintStr("YOU ARE PLAYING:");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("MINESWEEPER");
		HAL_Delay(1500);
		LCD_Clear();
		LCD_Home();
		LCD_SetCursor(0, 0);
		LCD_PrintStr("SELECT GAME MODE:");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("L1, R1 or R2");
    }else if (pong){
    	LCD_SetCursor(0, 0);
		LCD_PrintStr("YOU ARE PLAYING:");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("PONG");
		LCD_Clear();
		LCD_Home();
		LCD_SetCursor(0, 0);
		LCD_PrintStr("USE DPAD TO PLAY");
		LCD_SetCursor(0, 1);
		LCD_PrintStr("GAME STARTING...");
		HAL_Delay(1000);
		LCD_Clear();
		LCD_Home();
		for(uint8_t i = 3; i > 0; --i){
			LCD_Clear();
			char buffer[8];  // enough for numbers up to 999
			sprintf(buffer, " %d", i);
			LCD_SetCursor(8, 0);
			LCD_PrintStr(buffer);
			HAL_Delay(500);

		}
		LCD_Clear();
		LCD_SetCursor(0, 0);
		LCD_PrintStr("YOUR SCORE: ");
		LCD_SetCursor(0, 1);
		char buffer2[128];
		sprintf(buffer2,"HIGH SCORE: %d", pongHS);
		LCD_PrintStr(buffer2);

    }

}

void LCD_select_game() {
	//int count = 0;
    LCD_Clear();
    LCD_Home();

    // Put full text on the first line (even if longer than 16 chars)
    // Print second line normally
    LCD_SetCursor(0, 0);
    LCD_PrintStr("SELECT GAME...");
	HAL_Delay(1150);
//    LCD_SetCursor(0, 1);
//    LCD_PrintStr("MINESWEEPER (X) OR PONG (O)");
//    for(int i = 0; i < 30; ++i){
//    	HAL_Delay(100);
//    	LCD_ScrollDisplayLeft();
//    }
    LCD_Clear();
	LCD_Home();

	// Put full text on the first line (even if longer than 16 chars)
	// Print second line no   srmally
	LCD_SetCursor(0, 0);
	LCD_PrintStr("MINESWEEPER (X)");
	LCD_SetCursor(0, 1);
	LCD_PrintStr("PONG (O)");


}
void LCD_update_score(int score) {
	//int count = 0;
    // Put full text on the first line (even if longer than 16 chars)
    // Print second line normally
	LCD_Home();
    LCD_SetCursor(11, 0);
    char buffer1[16];
    if (score < 10) {
            // One-digit number → print with leading space
            sprintf(buffer1, " %d", score);
        } else {
            // Two-digit number → no leading space
            sprintf(buffer1, "%d", score);
        }

    LCD_PrintStr(buffer1);

}

void LCD_game_over_pong(int score) {
    char buffer[20];
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_PrintStr("WOMP WOMP!");
    LCD_SetCursor(0, 1);
    sprintf(buffer, "Score: %d", score);
    LCD_PrintStr(buffer);
}

void LCD_new_high_score(int score) {
    char buffer[20];
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_PrintStr("NEW HIGH SCORE!");
    LCD_SetCursor(0, 1);
    sprintf(buffer, "Score: %d", score);
    LCD_PrintStr(buffer);
}


    // Scroll first line
    //while(count < 2){
		//		count ++;
//		if (count == 1){
//			LCD_SetCursor(0, 0);
//			LCD_Home();
//		}
   // }


