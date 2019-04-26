
#include "LCD_RG1602A.h"
#include "Delay.h"

#define SET_LCD_E HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET)
#define RESET_LCD_E HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET)
//BORYS

#define SET_LCD_RS HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET)
#define RESET_LCD_RS HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET)

/*
#define SET_LCD_RS HAL_GPIO_WritePin(LCD_RS_TEMP_GPIO_Port, LCD_RS_TEMP_Pin, GPIO_PIN_SET)
#define RESET_LCD_RS HAL_GPIO_WritePin(LCD_RS_TEMP_GPIO_Port, LCD_RS_TEMP_Pin, GPIO_PIN_RESET)
*/

#ifdef USE_RW
#define SET_LCD_RW HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET)
#define RESET_LCD_RW HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET)
#endif

/*
 *
 * 	INSIDE FUNCTIONS
 *
 */

//
//	Set data port
//BORYS
extern volatile char argument;
static inline void LCD_SetDataPort(uint8_t data)
{
//	argument = data;
#ifdef LCD_4BIT
	if(data & (1<<0))
	{
		HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, GPIO_PIN_RESET);
	}

	if(data & (1<<1))
	{
		HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, GPIO_PIN_RESET);
	}

	if(data & (1<<2))
	{
		HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, GPIO_PIN_RESET);
	}

	if(data & (1<<3))
	{
		HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, GPIO_PIN_RESET);
	}
#endif
}

#ifdef USE_RW
static inline uint8_t LCD_GetDataPort()
{
	uint8_t result = 0;

#ifdef LCD_4BIT

	if(HAL_GPIO_ReadPin(LCD_D4_GPIO_Port, LCD_D4_Pin) == GPIO_PIN_SET) result |= (1<<0);
	if(HAL_GPIO_ReadPin(LCD_D5_GPIO_Port, LCD_D5_Pin) == GPIO_PIN_SET) result |= (1<<1);
	if(HAL_GPIO_ReadPin(LCD_D6_GPIO_Port, LCD_D6_Pin) == GPIO_PIN_SET) result |= (1<<2);
	if(HAL_GPIO_ReadPin(LCD_D7_GPIO_Port, LCD_D7_Pin) == GPIO_PIN_SET) result |= (1<<3);
#endif
	argument=result;
return result;
}
#endif

static void LCD_DataOut()
{
	  GPIO_InitTypeDef GPIO_InitStruct;
#ifdef LCD_4BIT
	  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
#endif
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void LCD_DataIn()
{	
	GPIO_InitTypeDef GPIO_InitStruct;
	 GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//
//	Write byte to LCD
//
uint8_t LCD_ReadByte(void)
{
	uint8_t result = 0;
	LCD_DataIn();

	SET_LCD_RW;
	delay(1,&htim4);
	SET_LCD_E;
	delay(5,&htim4);
	result = (LCD_GetDataPort() << 4);
	RESET_LCD_E;
	RESET_LCD_RW;
	delay(3,&htim4);
	
	RESET_LCD_RW;
	SET_LCD_E;
	delay(5,&htim4);
	result |= LCD_GetDataPort();
	RESET_LCD_E;
	delay(3,&htim4);
	argument=result;
	return result;
}

//
//	Check Busy Flag
//
uint8_t LCD_CheckBusyFlag()
{
	RESET_LCD_RS;
	delay(1,&htim4);
	return LCD_ReadByte();
}

//
//	Write byte to LCD
//
void LCD_WriteByte(uint8_t data)
{
#ifdef USE_RW // There is no need to change GPIO direction if RW is not used
	LCD_DataOut();

	RESET_LCD_RW;
#endif
#ifdef LCD_4BIT
	delay(1,&htim4);
	LCD_SetDataPort(data >> 4);
	SET_LCD_E;
	delay(1,&htim4);
	RESET_LCD_E;
	delay(2,&htim4);
#endif
	LCD_SetDataPort(data);
	SET_LCD_E;
	delay(1,&htim4);
	RESET_LCD_E;
	delay(2,&htim4);
/*#ifdef USE_RW
	while((LCD_CheckBusyFlag() & (1<<7))); // Wait for data processing
	argument=argument;
#else
		HAL_Delay(1);
		delay(120, &htim4); // Wait for data processing
#endif*/

//		HAL_Delay(1);
		delay(1200, &htim4); // Wait for data processing
}

//
//	Write command to LCD
//
void LCD_WriteCmd(uint8_t cmd)
{
	RESET_LCD_RS;
	LCD_WriteByte(cmd);
	
#ifndef USE_RW
	delay(1000, &htim4); //<<--- wait for command processing
#endif
}

//
//	Write data to LCD
//
void LCD_WriteData(uint8_t data)
{
	SET_LCD_RS;
	argument=data;
	LCD_WriteByte(data);
}

/*
 *
 * 	USER FUNCTIONS
 *
 */

//
//	Write one character to LCD
//
void LCD_Char(unsigned char c)
{
		LCD_WriteData(c);
//			LCD_WriteData(((c >= 0x80) && (c <= 0x87)) ? (c & 0x07) : c);
}

//
//	Write string to LCD
//
void LCD_String(char* str)
{
	while(*str)
	{
		argument=*(str+1);
		LCD_Char(*str);
		str++;
	}
		argument=argument;
}

//
// Print integer on LCD
//
void LCD_Int(int value)
{
	char buf[LCD_X+1];
	sprintf(buf, "%d", value);
	LCD_String(buf);
}

//
// Print hexadecimal on LCD
//
void LCD_Hex(int value, uint8_t upper_case)
{
	char buf[LCD_X+1];
	if(upper_case)
		sprintf(buf, "%X", value);
	else
		sprintf(buf, "%x", value);
	LCD_String(buf);
}

//
// Define custom char in LCD CGRAM in 'number' slot
//
void LCD_DefChar(uint8_t number, uint8_t *def_char)
{
	uint8_t i, c;
	LCD_WriteCmd(64+((number&0x7)*8));
	for(i = 0; i < 8; i++)
	{
		c = *(def_char++);
		LCD_WriteData(c);
	}
}

//
// Back to home position
//
void LCD_Home(void)
{
	LCD_WriteCmd(LCDC_CLS|LCDC_HOME);
}

//
// Control cursor visibility
//
void LCD_Cursor(uint8_t on_off)
{
	if(on_off == 0)
		LCD_WriteCmd(LCDC_ONOFF|LCDC_DISPLAYON);
	else
		LCD_WriteCmd(LCDC_ONOFF|LCDC_DISPLAYON|LCDC_CURSORON);
}

//
// Control cursor blinking
//
void LCD_Blink(uint8_t on_off)
{
	if(on_off == 0)
		LCD_WriteCmd(LCDC_ONOFF|LCDC_DISPLAYON);
	else
		LCD_WriteCmd(LCDC_ONOFF|LCDC_DISPLAYON|LCDC_CURSORON|LCDC_BLINKON);
}

//
// Set cursor for x-column, y-row
//
void LCD_Locate(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			y = LCD_LINE1;
			break;
#if (LCD_Y>1)
		case 1:
			y = LCD_LINE2;
			break;
#endif
#if (LCD_Y>2)
		case 2:
			y = LCD_LINE3;
			break;
#endif
#if (LCD_Y>3)
		case 3:
			y = LCD_LINE4;
			break;
#endif
	}

	LCD_WriteCmd((0x80 + y + x));
}

//
//	Clear LCD
//
void LCD_Cls(void)
{
	LCD_WriteCmd(LCDC_CLS);
}

//
//	Initialization
//
void LCD_Init(void)
{
	RESET_LCD_RS;
	RESET_LCD_E;
#ifdef USE_RW
	RESET_LCD_RW;
#endif
	LCD_DataOut();

	HAL_Delay(45);
/*
//	LCD_SetDataPort(LCDC_FUNC|LCDC_FUNC8B);
	LCD_SetDataPort(0x03);
//	delay(4100, &htim4);
	delay(4200, &htim4);
	//LCD_SetDataPort(LCDC_FUNC|LCDC_FUNC8B);
	LCD_SetDataPort(0x03);
//	delay(100, &htim4);
	delay(200, &htim4);
#ifdef LCD_4BIT
	//LCD_SetDataPort(LCDC_FUNC|LCDC_FUNC4B); //4-byte mode
	LCD_SetDataPort(0x2);
	delay(100, &htim4);
	LCD_WriteCmd(LCDC_FUNC|LCDC_FUNC4B|LCDC_FUNC2L|LCDC_FUNC5x7); // 4-bit, 2 lanes, 5x7 chars
#endif

	LCD_WriteCmd(LCDC_ONOFF|LCDC_CURSOROFF); // Cursor off
	LCD_WriteCmd(LCDC_ONOFF|LCDC_DISPLAYON); // LCD on
	LCD_WriteCmd(LCDC_ENTRY|LCDC_ENTRYR); // Data entry right

	LCD_Cls(); // Clear display
	delay(4500, &htim4);
	*/
	LCD_SetDataPort(0x03);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(5100, &htim4);
	
	LCD_SetDataPort(0x03);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(180, &htim4);
	
	LCD_SetDataPort(0x03);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(180, &htim4);
	
	LCD_SetDataPort(0x02);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(120, &htim4);
	
	LCD_SetDataPort(0x02);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(10, &htim4);
	LCD_SetDataPort(0x08);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(120, &htim4);
	
	LCD_SetDataPort(0x00);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(120, &htim4);
	LCD_SetDataPort(0x0F);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(120, &htim4);
	
	LCD_SetDataPort(0x00);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(120, &htim4);
	LCD_SetDataPort(0x06);
	SET_LCD_E;
	delay(1, &htim4);
	RESET_LCD_E;
	delay(120, &htim4);
	LCD_Cls();
	
	/*
	LCD_WriteCmd(LCDC_FUNC|LCDC_FUNC4B|LCDC_FUNC2L|LCDC_FUNC5x7); // 4-bit, 2 lanes, 5x7 chars
//	LCD_WriteCmd(LCDC_FUNC|LCDC_FUNC4B|LCDC_FUNC5x7|LCDC_FUNC2L);
	LCD_WriteCmd(LCDC_ONOFF|LCDC_CURSOROFF); // Cursor off
	LCD_WriteCmd(LCDC_ONOFF|LCDC_DISPLAYON); // LCD on
	LCD_WriteCmd(LCDC_ENTRY|LCDC_ENTRYR); // Data entry right
//	LCD_WriteCmd(0x02|0x00);
	LCD_Cls(); // Clear display
	*/

	delay(4500, &htim4);
	
}
