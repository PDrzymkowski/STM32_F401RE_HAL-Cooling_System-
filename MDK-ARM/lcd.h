/**
 * @author  Kuba Ciekanski
 * @email   kontakt@kubaciekanski.pl
 * @website http://kubaciekanski.pl
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   HD44780 LCD driver library for STM32F4xx
 *	
 * 
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2017 Kuba Ciekanski

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   HD44780 LCD driver library for STM32Fxxx
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */

#include "stm32f4xx_hal.h"
#include "Delay.h"

/*
	Ciekanski defines
 */
#define RS_PORT			GPIOA
#define RS_PIN 			GPIO_PIN_3

#define RW_PORT			GPIOA
#define RW_PIN			GPIO_PIN_2

#define E_PORT			GPIOA
#define E_PIN				GPIO_PIN_10

#define	DB7_PORT		GPIOB
#define DB7_PIN			GPIO_PIN_10

#define DB6_PORT		GPIOB
#define DB6_PIN			GPIO_PIN_4

#define DB5_PORT		GPIOB
#define DB5_PIN			GPIO_PIN_5

#define DB4_PORT		GPIOB
#define DB4_PIN			GPIO_PIN_3

#define HD44780_DATA_PINS_MODE_OUTPUT			1
#define HD44780_DATA_PINS_MODE_INPUT			0

/* Pin definitions */
#define HD44780_RS_LOW              HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_RESET)
#define HD44780_RS_HIGH             HAL_GPIO_WritePin(RS_PORT, RS_PIN, GPIO_PIN_SET)
#define HD44780_E_LOW               HAL_GPIO_WritePin(E_PORT, E_PIN, GPIO_PIN_RESET)
#define HD44780_E_HIGH              HAL_GPIO_WritePin(E_PORT, E_PIN, GPIO_PIN_SET)
#define HD44780_RW_LOW              HAL_GPIO_WritePin(RW_PORT, RW_PIN, GPIO_PIN_RESET)
#define HD44780_RW_HIGH             HAL_GPIO_WritePin(RW_PORT, RW_PIN, GPIO_PIN_SET)

#define HD44780_Delay(x)            delay_us(x, &htim4)
#define HD44780_E_BLINK             HD44780_E_HIGH; delay(10, &htim4); HD44780_E_LOW

/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

//extern UART_HandleTypeDef huart6;
/**
 * @defgroup HD44780_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes HD44780 LCD
 * @brief  cols: Width of lcd
 * @param  rows: Height of lcd
 * @retval None
 */
void HD44780_Init(uint8_t cols, uint8_t rows);

/**
 * @brief  Turn display on
 * @param  None
 * @retval None
 */
void HD44780_DisplayOn(void);

/**
 * @brief  Turn display off
 * @param  None
 * @retval None
 */
void HD44780_DisplayOff(void);

/**
 * @brief  Clears entire LCD
 * @param  None
 * @retval None
 */
void HD44780_Clear(void);

/**
 * @brief  Puts string on lcd
 * @param  x: X location where string will start
 * @param  y; Y location where string will start
 * @param  *str: pointer to string to display
 * @retval None
 */
void HD44780_Puts(uint8_t x, uint8_t y, char* str);

/**
 * @brief  Enables cursor blink
 * @param  None
 * @retval None
 */
void HD44780_BlinkOn(void);

/**
 * @brief  Disables cursor blink
 * @param  None
 * @retval None
 */
void HD44780_BlinkOff(void);

/**
 * @brief  Shows cursor
 * @param  None
 * @retval None
 */
void HD44780_CursorOn(void);

/**
 * @brief  Hides cursor
 * @param  None
 * @retval None
 */
void HD44780_CursorOff(void);

/**
 * @brief  Scrolls display to the left
 * @param  None
 * @retval None
 */
void HD44780_ScrollLeft(void);

/**
 * @brief  Scrolls display to the right
 * @param  None
 * @retval None
 */
void HD44780_ScrollRight(void);

/**
 * @brief  Creates custom character
 * @param  location: Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 * @param *data: Pointer to 8-bytes of data for one character
 * @retval None
 */
void HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * @brief  Puts custom created character on LCD
 * @param  x: X location where character will be shown
 * @param  y: Y location where character will be shown
 * @param  location: Location on LCD where character is stored, 0 - 7
 * @retval None
 */
void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif


