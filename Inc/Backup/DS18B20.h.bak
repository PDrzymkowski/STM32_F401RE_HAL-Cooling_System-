#include <stdio.h>
#include "stm32f4xx_hal.h"

#define DS18B20_INPUT_PIN_Pin GPIO_PIN_1
#define DS18B20_INPUT_PIN_GPIO_Port GPIOB
#define DS18B20_OUTPUT_PIN_Pin GPIO_PIN_2
#define DS18B20_OUTPUT_PIN_GPIO_Port GPIOB

//BORYS BEGIN
//#define DIODA_Pin GPIO_PIN_0
//#define DIODA_GPIO_Port GPIOB
//BORYS END

//void delay(uint16_t delay_time);
uint16_t DS18B20_ResetPulse(void);
void DS18B20_SendBit(uint16_t bit);
uint16_t DS18B20_ReadBit(void);
void DS18B20_SendByte(uint16_t byte);
uint16_t DS18B20_ReadByte(void);
float DS18B20_ReadTemperature(void);
//void DS18B20_DataIn(void);
//void DS18B20_DataOut(void);
