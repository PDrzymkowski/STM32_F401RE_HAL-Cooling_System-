#include "DS18B20.h"
#include "Delay.h"

//void delay(uint16_t delay_time){
//		&htim4->Inst
//}
volatile uint16_t temp1,temp2,temp3;


//void DS18B20_DataOut(){
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	GPIO_InitStruct.Pin = DS18B20_PIN_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(DS18B20_PIN_GPIO_Port, &GPIO_InitStruct);
//}
//void DS18B20_DataIn(){
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	GPIO_InitStruct.Pin = DS18B20_PIN_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
////  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(DS18B20_PIN_GPIO_Port, &GPIO_InitStruct);
//}

uint16_t DS18B20_ResetPulse(void){
		uint16_t PRESENCE=0;
	//GPIO_ResetBits(portW1, wire1); //ustawienie stanu niskiego
//	DS18B20_DataOut();
	HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_RESET);
	delay(480,&htim4); //odczekanie 480us
	//GPIO_SetBits(portW1, wire1); //odpuszczenie do stanu wysokiego
		HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_SET);
	delay(70,&htim4); //odczekanie ok. 70us na reakcje czujnika
 
	//sprawdzenie odpowiedzi
	//if (GPIO_ReadInputDataBit(portW1, wire1)==Bit_RESET)
//	DS18B20_DataIn();
	if(HAL_GPIO_ReadPin(DS18B20_INPUT_PIN_GPIO_Port,DS18B20_INPUT_PIN_Pin) == GPIO_PIN_RESET)
		PRESENCE++;
	delay(410,&htim4);
 
	//if (GPIO_ReadInputDataBit(portW1, wire1)==Bit_SET)
	if(HAL_GPIO_ReadPin(DS18B20_INPUT_PIN_GPIO_Port,DS18B20_INPUT_PIN_Pin) == GPIO_PIN_SET)
		PRESENCE++;
 
	if (PRESENCE == 2)
		return 1;
	else
		return 0;
}

void DS18B20_SendBit(uint16_t bit){
	
//		DS18B20_DataOut();
		if (bit==0)
	{
	//	GPIO_ResetBits(portW1, wire1);
		HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_RESET);
		delay(65,&htim4);
	//	GPIO_SetBits(portW1, wire1);
		HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_SET);
		delay(10,&htim4);
	}
	else
	{
	//	GPIO_ResetBits(portW1, wire1);
		HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_RESET);
		delay(10,&htim4);
	//	GPIO_SetBits(portW1, wire1);
		HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_SET);
		delay(65,&htim4);
	}
	//BORYS
//	DS18B20_DataIn();
}

uint16_t DS18B20_ReadBit(void){
	
	uint16_t bit=0;
//	DS18B20_DataOut();
	//GPIO_ResetBits(portW1, wire1);
	HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_RESET);
	delay(5,&htim4);
	//GPIO_SetBits(portW1, wire1);
	HAL_GPIO_WritePin(DS18B20_OUTPUT_PIN_GPIO_Port,DS18B20_OUTPUT_PIN_Pin,GPIO_PIN_SET);
//	DS18B20_DataIn();
	delay(5,&htim4);
 
	//warunek sprawdzajacy czy zostalo wyslane 1 czy 0
//	if (GPIO_ReadInputDataBit(portW1, wire1)==Bit_SET)
//	delay(5,&htim4);
	if(HAL_GPIO_ReadPin(DS18B20_INPUT_PIN_GPIO_Port,DS18B20_INPUT_PIN_Pin) == GPIO_PIN_SET)
		bit=1;
	else
		bit=0;
	temp3=bit;
	delay(55,&htim4);  	//przerwa czasowa miedzy bitami
	return bit;
}

void DS18B20_SendByte(uint16_t byte){
	
		uint16_t i,tmp;
	for (i = 0; i < 8; i++)
	{
		tmp = byte >> i; //przesuniecie bitowe
		tmp &= 0x01; //wyodrebnienie bitu do wyslania
		DS18B20_SendBit(tmp); //wyslanie bitu
}
	}

uint16_t DS18B20_ReadByte(void){
	
		uint16_t i,value=0;
	for (i = 0; i < 8; i++)
	{
		if(DS18B20_ReadBit()) //odczyt linii danych
			value |= 0x01 << i; //tworzenie osmiobitowej liczby
	}
	return value;
}

float DS18B20_ReadTemperature(void){
		uint16_t i, presence=0, memory[3];
	float temp=0;
 
	presence=DS18B20_ResetPulse();
 
	if (presence==1)
	{
		DS18B20_SendByte(0xCC); //Skip ROM
		DS18B20_SendByte(0x44); //Convert T
 
		
		for (i=0; i < 100; i++) //odczekanie 750ms na odczyt i konwersje 
		//delay(7500,&htim4); //temperatury
		delay(7500,&htim4); //temperatury
	}
//	HAL_GPIO_WritePin(D
 
	presence = DS18B20_ResetPulse();
 
	if (presence==1)
	{
		DS18B20_SendByte(0xCC);	//Skip ROM
		DS18B20_SendByte(0xBE);	//Read Scratchpad
 
		for (i=0;i<2;i++)
			memory[i] = DS18B20_ReadByte(); 		//odczyt 2 bajtów Scratchpad
		
		temp1=memory[0];
		temp2=memory[1];
 
		memory[2] = (240U & memory[1]) >>  7;
		memory[1] = (15U & memory[1] ) <<  8;
 
		if (memory[2] == 0)		//jesli dodatnia temperatura
		{
			temp = (memory[0] + memory[1]);
			temp = temp/16;
		}
		if (memory[2] >= 1)		//jesli ujemna temperatura
		{
			temp = (memory[0] + memory[1]-4095);
			temp = temp/16;
		}
	}
 
	presence = DS18B20_ResetPulse();
	return temp;
}
