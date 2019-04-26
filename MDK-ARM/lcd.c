
/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2017 Kuba Ciekanski
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */

/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "lcd.h"

/* Private HD44780 structure */
typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t currentX;
	uint8_t currentY;
	uint8_t DataPinsMode;
} HD44780_CurrentState_t;

/* Private functions */
static void HD44780_SendInstructions(uint8_t cmd);
static void HD44780_Send4bits(uint8_t cmd);
static void HD44780_SendCharacter(uint8_t data);
static void HD44780_CursorSet(uint8_t col, uint8_t row);
static void HD44780_InitPinsAsOutput(void);
static void HD44780_Delay_us(float i);
static void HD44780_InitPinsAsInput(void);
static uint8_t HD44780_CheckBusyFlag(void);

/* Private variable */
static HD44780_CurrentState_t HD44780_CurrentState;

void HD44780_Init(uint8_t cols, uint8_t rows) {

	HD44780_InitPinsAsOutput();

	/* At least 40ms */
	delay(45000, &htim4);
	
	/* Set LCD width and height */
	HD44780_CurrentState.Rows = rows;
	HD44780_CurrentState.Cols = cols;
	
	/* Set cursor pointer to beginning for LCD */
	HD44780_CurrentState.currentX = 0;
	HD44780_CurrentState.currentY = 0;
	
	HD44780_CurrentState.DisplayFunction = HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_1LINE;
	if (rows > 1) {
		HD44780_CurrentState.DisplayFunction |= HD44780_2LINE;
	}
	
	/* Try to set 4bit mode */
	HD44780_Send4bits(0x03);
	delay(4500,&htim4);
	
	/* Second try */
	HD44780_Send4bits(0x03);
delay(4500,&htim4);
	
	/* Third goo! */
	HD44780_Send4bits(0x03);
	delay(4500,&htim4);
	
	/* Set 4-bit interface */
	HD44780_Send4bits(0x02);
	delay(100,&htim4);
	
	/* Set # lines, font size, etc. */
	HD44780_SendInstructions(HD44780_FUNCTIONSET | HD44780_CurrentState.DisplayFunction);

	/* Turn the display on with no cursor or blinking default */
	HD44780_CurrentState.DisplayControl = HD44780_DISPLAYON;
	HD44780_DisplayOn();

	/* Clear lcd */
	HD44780_Clear();

	/* Default font directions */
	HD44780_CurrentState.DisplayMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
	HD44780_SendInstructions(HD44780_ENTRYMODESET | HD44780_CurrentState.DisplayMode);

	/* Delay */
	delay(4500,&htim4);
}

void HD44780_Clear(void) {
	HD44780_SendInstructions(HD44780_CLEARDISPLAY);
	delay(30,&htim4);;
}

void HD44780_Puts(uint8_t x, uint8_t y, char* str) {
	HD44780_CursorSet(x, y);
	while (*str) {
		if (HD44780_CurrentState.currentX >= HD44780_CurrentState.Cols) {
			HD44780_CurrentState.currentX = 0;
			HD44780_CurrentState.currentY++;
			HD44780_CursorSet(HD44780_CurrentState.currentX, HD44780_CurrentState.currentY);
		}
		if (*str == '\n') {
			HD44780_CurrentState.currentY++;
			HD44780_CursorSet(HD44780_CurrentState.currentX, HD44780_CurrentState.currentY);
		} else if (*str == '\r') {
			HD44780_CursorSet(0, HD44780_CurrentState.currentY);
		} else {
			{	int timeout=0;
				while (HD44780_CheckBusyFlag()){
				timeout++;
					if (timeout>100) break;
				}
				
			}
			HD44780_SendCharacter(*str);
			HD44780_CurrentState.currentX++;
		}
		str++;
	}
}

void HD44780_DisplayOn(void) {
	HD44780_CurrentState.DisplayControl |= HD44780_DISPLAYON;
	HD44780_SendInstructions(HD44780_DISPLAYCONTROL | HD44780_CurrentState.DisplayControl);
}

void HD44780_DisplayOff(void) {
	HD44780_CurrentState.DisplayControl &= ~HD44780_DISPLAYON;
	HD44780_SendInstructions(HD44780_DISPLAYCONTROL | HD44780_CurrentState.DisplayControl);
}

void HD44780_BlinkOn(void) {
	HD44780_CurrentState.DisplayControl |= HD44780_BLINKON;
	HD44780_SendInstructions(HD44780_DISPLAYCONTROL | HD44780_CurrentState.DisplayControl);
}

void HD44780_BlinkOff(void) {
	HD44780_CurrentState.DisplayControl &= ~HD44780_BLINKON;
	HD44780_SendInstructions(HD44780_DISPLAYCONTROL | HD44780_CurrentState.DisplayControl);
}

void HD44780_CursorOn(void) {
	HD44780_CurrentState.DisplayControl |= HD44780_CURSORON;
	HD44780_SendInstructions(HD44780_DISPLAYCONTROL | HD44780_CurrentState.DisplayControl);
}

void HD44780_CursorOff(void) {
	HD44780_CurrentState.DisplayControl &= ~HD44780_CURSORON;
	HD44780_SendInstructions(HD44780_DISPLAYCONTROL | HD44780_CurrentState.DisplayControl);
}

void HD44780_ScrollLeft(void) {
	HD44780_SendInstructions(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void HD44780_ScrollRight(void) {
	HD44780_SendInstructions(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void HD44780_CreateChar(uint8_t location, uint8_t *data) {
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	HD44780_SendInstructions(HD44780_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) {
		HD44780_SendCharacter(data[i]);
	}
}

void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location) {
	HD44780_CursorSet(x, y);
	HD44780_SendCharacter(location);
}

/* Private functions */
static void HD44780_SendInstructions(uint8_t cmd) {
	/* Command mode */
	HD44780_InitPinsAsOutput();
	HD44780_RS_LOW;
	
	/* High nibble */
	HD44780_Send4bits(cmd >> 4);
	/* Low nibble */
	HD44780_Send4bits(cmd & 0x0F);
}

static void HD44780_SendCharacter(uint8_t data) {
	/* Data mode */
	HD44780_InitPinsAsOutput();
	HD44780_RS_HIGH;
	
	/* High nibble */
	HD44780_Send4bits(data >> 4);
	/* Low nibble */
	HD44780_Send4bits(data & 0x0F);
}

static void HD44780_Send4bits(uint8_t cmd) {
	/* Set output port */

	HAL_GPIO_WritePin(DB7_PORT, DB7_PIN, (cmd & 0x08));
	HAL_GPIO_WritePin(DB6_PORT, DB6_PIN, (cmd & 0x04));
	HAL_GPIO_WritePin(DB5_PORT, DB5_PIN, (cmd & 0x02));
	HAL_GPIO_WritePin(DB4_PORT, DB4_PIN, (cmd & 0x01));

	HD44780_E_BLINK;
}

static void HD44780_CursorSet(uint8_t col, uint8_t row) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	
	/* Go to beginning */
	if (row >= HD44780_CurrentState.Rows) {
		row = 0;
	}
	
	/* Set current column and row */
	HD44780_CurrentState.currentX = col;
	HD44780_CurrentState.currentY = row;
	
	/* Set location address */
	HD44780_SendInstructions(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}

static void HD44780_InitPinsAsOutput(void) {
	static uint8_t confPins=1;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if (confPins) {
	
		//Control pins configuration - ALWAYS as output!
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pin=RS_PIN;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RS_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=RW_PIN;
	HAL_GPIO_Init(RW_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=E_PIN;
	HAL_GPIO_Init(E_PORT,&GPIO_InitStruct);
	confPins=0;
	}
		
	if (HD44780_CurrentState.DataPinsMode!=HD44780_DATA_PINS_MODE_OUTPUT){
	
	//Data pins configuration
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;	
		
	GPIO_InitStruct.Pin=DB7_PIN;
	HAL_GPIO_Init(DB7_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=DB6_PIN;
	HAL_GPIO_Init(DB6_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=DB5_PIN;
	HAL_GPIO_Init(DB5_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=DB4_PIN;
	HAL_GPIO_Init(DB4_PORT,&GPIO_InitStruct);
	}
	
	HD44780_CurrentState.DataPinsMode=HD44780_DATA_PINS_MODE_OUTPUT;

	//After configuration set pin low
	HAL_GPIO_WritePin(RS_PORT,RS_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RW_PORT,RW_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E_PORT,E_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB7_PORT,DB7_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB6_PORT,DB6_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB5_PORT,DB5_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB4_PORT,DB4_PIN,GPIO_PIN_RESET);
	
}

static void HD44780_InitPinsAsInput(void){
	
	if (HD44780_CurrentState.DataPinsMode!=HD44780_DATA_PINS_MODE_INPUT){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//Data pins only
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Pin=DB7_PIN;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DB7_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=DB6_PIN;
	HAL_GPIO_Init(DB6_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=DB5_PIN;
	HAL_GPIO_Init(DB5_PORT,&GPIO_InitStruct);
	
	GPIO_InitStruct.Pin=DB4_PIN;
	HAL_GPIO_Init(DB4_PORT,&GPIO_InitStruct);
	}
	
	HD44780_CurrentState.DataPinsMode=HD44780_DATA_PINS_MODE_INPUT;
		
}

static void HD44780_Delay_us (float i){
	float k=((SystemCoreClock)/1000000)*i;
	for(int j=0; j<=k ; j++) ;
}

static uint8_t HD44780_CheckBusyFlag(void) {
	uint8_t data;
	HD44780_InitPinsAsInput();
	HD44780_RW_HIGH;
	HD44780_RS_LOW;
	HD44780_E_HIGH;
	delay(1,&htim4);
	data=HAL_GPIO_ReadPin(DB7_PORT,DB7_PIN);
	HD44780_E_LOW;
	delay(1,&htim4);
	HD44780_E_HIGH;
	delay(1,&htim4);
	HD44780_E_LOW;
	HD44780_RW_LOW;
	HD44780_RS_LOW;
	HD44780_InitPinsAsOutput();
	return data;
}


