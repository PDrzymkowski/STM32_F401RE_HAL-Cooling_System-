/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
	*/
	
	
	/*! \mainpage Projekt PMIK 18Z: Uklad chlodzacy
 *
 * \section 	  Opis
 *
 * Projekt jest ukladem chlodzacym ktorego dzialanie opiera sie na pomiarze temperatury
 * odpowiednim czujnikiem i na podstawie wynikow uruchomienie wiatraczkow chlodzacych.
 * <br> Uklad komunikuje sie z uzytkownikiem przy uzyciu wyswietlacza LCD oraz zestawu
 * przyciskow, jak rowniez za posrednictwem UART z komputerem.<br> Uzytkownik ma wplyw na 
 * tryb pomiaru okreslonym urzadzeniem, jak rowniez na moc pracy wiatraczkow oraz wartosc
 * temperatury granicznej.<br>
 * Dzialanie programu zaimpementowane zostalo przy uzyciu mikrokontrolera STM32, aplikacja
 * zas napisana w jezyku C.
 *
 * \section Peryferia
 *	Mikrokontroler STM32 F401RB<br>
 *	Termometr cyfrowy DS18B20<br>
 *	Termometr analogowy LM35<br>
 *	Wyswietlacz LCD RG1602A<br>
 *	2 Wiatraczki 5V firmy Sunon<br>
 *  Modul karty SD<br>
 *	Zestaw 4 przyciskow typu "tact-switch"<br>
 *	Mostek H do zasilania wiatrakow<br>
 *	Kasetka baterii do zasilania ukladu
 *
 *  \section Plik Main:
 *  Szczegolowy opis programu mozna znalezc w czesci opisujacej plik main.c <br>
 *
 * \section Projekt Wykonali:
 *  Piotr Drzymkowski<br>
 *  Borys Khachapuridze
 *
 
 */
  
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/** \brief Plik zawierajacy funkcje opozniajaca delay() uzywana w peryferiach ukladu */
#include "Delay.h"
/** \brief Plik zawierajacy obsluge termometru cyfrowego DS18B20 poprzez interfejs 1-wire*/
#include "DS18B20.h"
/** \brief Plik zawierajacy obsluge wyswietlacza LCD*/
#include "LCD_RG1602A.h"
#include "lcd.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** \brief Minimalny tryb pracy wiatraka okreslany jako wypelnienie sygnalu PWM */
#define FANSPEEDLOW 700
/** \brief Sredni tryb pracy wiatraka okreslany jako wypelnienie sygnalu PWM */
#define FANSPEEDMEDIUM 850 
/** \brief Maksymalny tryb pracy wiatraka okreslany jako wypelnienie sygnalu PWM */
#define FANSPEEDHIGH 999
/** \brief Minimalna wartosc temepratury krytycznej jaka moze przyjac uklad */
#define TEMPERATURE_MIN 0
/** \brief Maksymalna wartosc temepratury krytycznej jaka moze przyjac uklad */
#define TEMPERATURE_MAX 70

/** \brief Tryb pracy ukladu jako pomiar rdzenia wewnetrznym czunikiem mikrokontrolera */
#define CORE 0
/** \brief Tryb pracy ukladu jako pomiar analogowym termometrem LM35*/
#define LM35 1
/** \brief Tryb pracy ukladu jako pomiar cyfrowym termometrem DS18B20 */
#define DS18B20 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/** \brief Flaga okreslajaca czy zakonczono proces "debouncingu" dla przycisku Switch0 */
volatile uint8_t FLAG_SWITCH0_DEBOUNCING = 0; 
/** \brief Flaga okreslajaca czy zakonczono proces "debouncingu" dla przycisku Switch1 */
volatile uint8_t FLAG_SWITCH1_DEBOUNCING = 0;
/** \brief Flaga okreslajaca czy zakonczono proces "debouncingu" dla przycisku Switch2 */
volatile uint8_t FLAG_SWITCH2_DEBOUNCING = 0; 
/** \brief Flaga okreslajaca czy zakonczono proces "debouncingu" dla przycisku Switch3 */
volatile uint8_t FLAG_SWITCH3_DEBOUNCING = 0;
/** \brief Flaga okreslajaca czy nalezy dokonac pomiaru temperatury */
uint8_t FLAG_START_TEMPERATURE_MEASUREMENT = 0;
/** \brief Flaga okreslajaca czy przekroczono temeprature krytyczna i nalezy uruchomic wiatrak */
uint8_t FLAG_OVER_CRITICAL_TEMPERATURE = 0;
/** \brief Flaga okreslajaca czy nalezy zapisac w danym momencie dane na karcie SD*/
uint8_t FLAG_WRITE_SD = 0;
/** \brief Flaga okreslajaca czy nalezy zapisac w danym momencie konfiguracje urzadzenia*/
uint8_t FLAG_WRITE_CONFIG = 0;
/** \brief Flaga okreslajaca czy karta SD zostala poprawnie wykryta i znajduje sie na niej wystarczajaco miejsca*/
uint8_t FLAG_SD_SPACE_VALID = 0;
/** \brief Flaga okreslajaca stan pracy wiatraczka 1*/
uint8_t FLAG_FAN_1_WORK =0;
/** \brief Flaga okreslajaca stan pracy wiatraczka 2*/
uint8_t FLAG_FAN_2_WORK=0;
/** \brief Flaga wymuszajaca zapis informacji o wlaczeniu wiatraka*/
uint8_t FLAG_EVENT_ON=0;
/** \brief Flaga wymuszajaca zapis informacji o wylaczeniu wiatraka*/
uint8_t FLAG_EVENT_OFF=1;
/** \brief Tablica przechowujaca pomiary z przetwornika ADC */
uint32_t MeasurementADC[2] = {0};
/** \brief Wartosc zmierzonej i przeliczonej temperatury przez termometr LM35*/
volatile double Measurement_LM35 = 0;
/** \brief Wartosc zmierzonej i przeliczonej temperatury przez termometr DS18B20*/
volatile double Measurement_DS18B20 = 0;
/** \brief Wartosc zmierzonej i przeliczonej temperatury przez wewnetrzny czujnik STM*/
volatile double Measurement_CORE = 0;
/** \brief Zmienna okreslajaca ilosc sekund ktore minely od ostatniego pomiaru */
uint32_t seconds_counter = 0;
/** \brief Zmienna okreslajaca ilosc sekund ktore minely od ostatniego zapisu konfiguracji */
uint32_t sd_card_write_counter = 0;
/** \brief Okres, co ktory ma nastepowac pomiar */
uint32_t measurement_period = 0;
/** \brief Okres, co ktory konfiguracja zapisywana jest na karcie SD w celu odzyskania po utracie zasilania */
uint32_t sd_card_write_period = 0;//BORYS
/** \brief Tryb pomiaru urzadzenia */
uint8_t measurement_state = CORE;
/** \brief Tryb pracy wiatraka*/
uint16_t fan_speed = FANSPEEDLOW;
volatile uint32_t argument;
/** \brief Tablica znakow reprezentujaca bufor danych wysylanych poprzez UART */
char UART_BUFFER[100];
/** \brief Rozmiar bufora danych wysylanych poprzez UART */
volatile uint16_t UART_BUFFER_SIZE = 0; 
/** \brief Tablica przechowujaca otrzymana wiadomosc z UART'a */
char UART_RECEIVED[20];
/** \brief Tablica przechowujaca kopie otrzymanej wiadomosc z UART'a */
char UART_RECEIVED_COPY[20];
/** \brief Tablica znakow reprezentujaca bufor danych wysylanych do wyswietlacza LCD */
char LCD_BUFFER[50];
/** \brief Tablica znakow reprezentujaca bufor danych wysylanych do karty SD */
char SD_BUFFER[250];
/** \brief Rozmiar bufora danych wysylanych do wyswietlacza LCD */
volatile uint16_t LCD_BUFFER_SIZE = 0;

/** \brief Wartosc temperaturey krytycznej ukladu */
uint32_t critical_temp = 25;
/** \brief Tablica znakow przechowujaca obecna wartosc gornej linii tekstu wyswietlanego na LCD */
char LCD_upper_line [16];
/** \brief Tablica znakow przechowujaca obecna wartosc dolnej linii tekstu wyswietlanego na LCD */
char LCD_lower_line [16];

//Fatfs object
FATFS FatFs;
//File object
FIL fil;
//SD space variables
uint32_t total_space, free_space;

int i_index = 0;
int presence = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SetFanState(void);
void UpdateFan(void);
void StartTemperatureMeasurement(void);
void SetDate(uint32_t unformated_date);
void SetTime(uint32_t unformated_time);
FRESULT FATFS_DriveSize(uint32_t* total, uint32_t* free);
/**
****************************************************************************************
* @brief 			Zmiana trybu pomiaru: CORE->LM35->DS18B20 <br><br>
*							Wywolywana w momencie wcisniecia przycisku Switch 0.
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void CycleMeasurementState(){
	
	if(measurement_state == CORE)
  {
		measurement_state = LM35;
	}else if(measurement_state == LM35)
	{
		measurement_state=DS18B20;
	}else
	{
		measurement_state = CORE;
	}
}

/**
****************************************************************************************
* @brief 			Zmiana trybu mocy wiatraka: niska->srednia->duza. <br><br>
*							Wywolywana w momencie wcisniecia przycisku Switch 1.
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void CycleFanState(){
	
	if(fan_speed == FANSPEEDLOW)
  {
		fan_speed = FANSPEEDMEDIUM;
	}else if(fan_speed  == FANSPEEDMEDIUM)
	{
		fan_speed=FANSPEEDHIGH;
	}else
	{
		fan_speed = FANSPEEDLOW;
	}
}

/**
****************************************************************************************
* @brief 			Matematyczne przeliczenie wartosci zmierzonej przez wewnetrzny czujnik
*							temperatury rdzenia mikrokontrolera na konkretna wartosc temperatury na 
*							podstawie zadanego wzoru.
*	
* @param 			uint32_t temperature: wartosc zmierzona przez czunik rdzenia
*	@return			(double) : wartosc temperatury 
****************************************************************************************
*/
double TemperatureConversion_CORE(uint32_t temperature)
{
	double temp=temperature;
	temp=((temp*3.3/4095-0.76)/0.0025+25);
	if ((uint32_t)temp >=255) return 0;
	else return temp;
}

/**
****************************************************************************************
* @brief 			Matematyczne przeliczenie wartosci zmierzonej przez temrometr LM35 na
*							konkretna wartosc temperatury na podstawie zadanego wzoru.
*	
* @param 			uint32_t temperature: wartosc zmierzona przez termometr LM35
*	@return			(double) : wartosc temperatury 
****************************************************************************************
*/
double TemperatureConversion_LM35(uint32_t temperature)
{
	double temp=temperature;	
	return ((temp*3.3/4095)/0.01);//10 mv/degC
}


/**
****************************************************************************************
* @brief 			Funkcja odbierajaca dane wyslane z komputera przez UART
*							poprzez obsluge callback'u zwiazanego z zakonczeniem odbioru.<br>
*							Funkcja na odebranie danych odpowiada przesylajac kopie wiadomosci z powrotem do nadawcy.<br>
*							Po odebraniu komend wymuszany jest nowy pomiar temperatury w celu aktualizacji stanu urzadzenia.<br>
*							Komendy wysylane z komputera musza byc 10 znakowe. Oto ich lista:
*							-> CMDTCRITxx - ustawienie temperatury krytycznej na zadana wartosc (np. CMDTCRIT10, ustawia na 10 stopni C)
*							-> CMDMODDS18 - tryb pomiaru za pomoca DS18B20
*							-> CMDMODLM35 - tryb pomiaru za pomoca LM35
*							-> CMDMODCORE - tryb pomiaru wewnetrznego Core mikrokontrolera
*							-> CMDFANHIGH - tryb mocy wiatraka: silny
*							-> CMDFANMEDI - tryb mocy wiatraka: sredni
*							-> CMDFANLOWE - tryb mocy wiatraka: niski
*							-> CDAxddmmyy - ustawienie i zapis daty (np.CDA1200119 ustawia date na poniedzialek 20 stycznia 2019 roku),x-numer dnia tygodnia
*							-> CTIMhhmmss - ustawienie i zapis godziny (np. CTIM120530 ustawia godzine na 12:05:30)
*							-> CMDDUMPCON - wymusza natychmiastowe zapisanie pliku konfiguracyjnego
*							-> CMDFACTORY - wymusza wstawienie do pliku konfiguracyjnego domyslnych wartosci
*							
*	
* @param 			UART_HandleTypeDef *huart: wskaznik na strukture zawierajaca informace konfiguracyjna
*							konkretnego USART'a
*	@return			Brak
****************************************************************************************
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint32_t temp_time_value;
	strlcpy(UART_RECEIVED_COPY,UART_RECEIVED,11);	
	UART_BUFFER_SIZE = sprintf(UART_BUFFER, "%s\n\r" , UART_RECEIVED);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)UART_BUFFER, UART_BUFFER_SIZE); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
		
	
	if(strstr(UART_RECEIVED_COPY, "CMDTCRIT") != NULL){
		
		//Szukanie intow na koncu stringa
		if(atoi(UART_RECEIVED_COPY+8) <= TEMPERATURE_MAX && atoi(UART_RECEIVED_COPY+8) >= TEMPERATURE_MIN)
			critical_temp = atoi(UART_RECEIVED_COPY+8);
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
	
	}
	if(strstr(UART_RECEIVED_COPY, "CDA") != NULL){
		
		temp_time_value=strtol(UART_RECEIVED_COPY+3,NULL,16);
		SetDate(temp_time_value);
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
	
	}
	if(strstr(UART_RECEIVED_COPY, "CTIM") != NULL){
		
		temp_time_value=strtol(UART_RECEIVED_COPY+4,NULL,16);
		SetTime(temp_time_value);
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
	
	}
	else if(strcmp(UART_RECEIVED_COPY,"CMDMODDS18")==0){
		
		measurement_state = DS18B20;
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
		
	}else if(strcmp(UART_RECEIVED_COPY,"CMDMODLM35")==0){
		
		measurement_state = LM35;
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
		
	}else if(strcmp(UART_RECEIVED_COPY,"CMDMODCORE")==0){
		
		measurement_state = CORE;
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
		
	}else if(strcmp(UART_RECEIVED_COPY,"CMDFANHIGH")==0){
		
		fan_speed = FANSPEEDHIGH;
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
		
	}else if(strcmp(UART_RECEIVED_COPY,"CMDFANLOWE")==0){
		
		fan_speed = FANSPEEDLOW;
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
		
	}else if(strcmp(UART_RECEIVED_COPY,"CMDFANMEDI")==0){
		
		fan_speed = FANSPEEDMEDIUM;
		FLAG_START_TEMPERATURE_MEASUREMENT = 1;
		
	}
	else if(strcmp(UART_RECEIVED_COPY,"CMDDUMPCON")==0){
		
		FLAG_WRITE_CONFIG=1;		
	}
	else if(strcmp(UART_RECEIVED_COPY,"CMDFACTORY")==0){
		
		FLAG_WRITE_CONFIG=2;
	}
	
	HAL_UART_Receive_IT(&huart2, (uint8_t*)UART_RECEIVED, 10); // Ponowne wlaczenie nasluchiwania
}

/**
****************************************************************************************
* @brief 			Ustawienie daty zegara czasu rzeczywistego RTC<br><br> 
*							Dane do ustawiania daty dostarczane sa w formacie xddmmyy<br>
*							x: dzien tygodnia 1-7<br>
*							dd: dzien 01-31<br>
*							mm: miesiac 01-12<br>
*							yy: rok 00-99<br>
*							Otrzymanie niepoprawnych danych powoduje ustawienie pola danych z wartosci domyslnych: poniedzialek 01.01.2000<br>
*							Funkcja wypelnia strukture typu RTC_DateTypeDef zgodnie z otrzymanymi danymi po czym
*							wywoluje funkcje konfiguracyjna HAL_RTC_SetDate(&hrtc,&data_structure,RTC_FORMAT_BCD);<br>
*	
* @param 			uint32_t unformated_date: dane okreslajace date w formacie xddmmyy (x-numer dnia tygodnia)
*	@return			Brak
****************************************************************************************
*/
void SetDate(uint32_t unformated_date)
{
	//xddmmyy
	uint32_t temp;
	RTC_DateTypeDef data_structure;
	temp=(unformated_date & 0xff);
	if(temp >= 0x01 && temp <= 0x99)
	{
		data_structure.Year=temp;
	}
	else
	{
		data_structure.Year=0x00;
	}
	temp=((unformated_date >> 8) & 0xff);
	if(temp >= 0x01 && temp <= 0x12)
	{
		data_structure.Month=temp;
	}
	else
	{
		data_structure.Month=RTC_MONTH_JANUARY;
	}
	temp=((unformated_date >> 16) & 0xff);
	if(temp>=0x01 && temp<=0x31)
	{
		data_structure.Date=temp;
	}
	else
	{
		data_structure.Date=0x01;
	}
	temp=((unformated_date >> 24) & 0xf);
	if(temp >= 0x1 && temp <= 0x7)
	{
		data_structure.WeekDay=temp;
	}
	else
	{
		data_structure.WeekDay=RTC_WEEKDAY_MONDAY;
	}
	HAL_RTC_SetDate(&hrtc,&data_structure,RTC_FORMAT_BCD);
}

/**
****************************************************************************************
* @brief 			Ustawienie czasu zegara czasu rzeczywistego RTC<br><br> 
*							Dane do ustawiania czasu dostarczane sa w formacie hhmmss<br>
*							hh: godziny 00-23<br>
*							mm: minuty 00-59<br>
*							ss: sekundy 00-59<br>
*							Otrzymanie niepoprawnych danych powoduje ustawienie pola danych z wartosci domyslnych: 12:00:00<br>
*							Funkcja wypelnia strukture typu RTC_TimeTypeDef zgodnie z otrzymanymi danymi po czym
*							wywoluje funkcje konfiguracyjna HAL_RTC_SetTime(&hrtc,&time_structure,RTC_FORMAT_BCD)<br>
*	
* @param 			uint32_t unformated_time: dane okreslajace czas w formacie hhmmss
*	@return			Brak
****************************************************************************************
*/
void SetTime(uint32_t unformated_time)
{
	//hhmmss
	uint32_t temp;
	RTC_TimeTypeDef time_structure;
	temp=(unformated_time & 0xff);
	if(temp >= 0x01 && temp <= 0x59)
	{
		time_structure.Seconds=temp;
	}
	else
	{
		time_structure.Seconds=0x00;
	}
	temp=((unformated_time >> 8) & 0xff);
	if(temp >= 0x01 && temp <= 0x59)
	{
		time_structure.Minutes=temp;
	}
	else
	{
		time_structure.Minutes=0x00;
	}
	temp=((unformated_time >> 16) & 0xff);
	if(temp >=0x01 && temp<= 0x23)
	{
		time_structure.Hours=temp;
	}
	else
	{
		time_structure.Hours=0x12;
	}
	HAL_RTC_SetTime(&hrtc,&time_structure,RTC_FORMAT_BCD);
}

/**
****************************************************************************************
* @brief 			Zapisanie konfiguracji na karcie SD<br><br> 
*							Dane konfiguracyjne zapisywane sa w postaci:<br>
*							godzina, minuty, sekundy, dzien_tygodnia, dzien, miesiac, rok, czujnik, temperatura_krytyczna, tryb_wiatraka<br>
*							W zaleznosci od stanu flagi FLAG_WRITE_CONFIG zapisywane sa biezace wartosci, lub domyslne: <br>
*							1: zapis biezacych wartosci
*							2: zapis domyslnych wartosci
*							Funkcja pobiera dane o czasie przy pomocy wywolania HAL_RTC_GetTime i HAL_RTC_GetDate. Wazna jest kolejnosc wywolan,
*							uzycie tych funkcji w innej kolejnosci spowoduje utrate spojnosci otrzymywanych danych (HAL_RTC_GetDate zwraca
*							date pozyskana w momencie wywolania funkcji HAL_RTC_GetTime)<br>
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void WriteConfig(void)
{
	RTC_TimeTypeDef time_structure;
	RTC_DateTypeDef data_structure;
	HAL_RTC_GetTime(&hrtc,&time_structure,RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc,&data_structure,RTC_FORMAT_BCD);
	if(FLAG_SD_SPACE_VALID ==1)
	{
		if(FLAG_WRITE_CONFIG ==1)
			{
				if (f_mount(&FatFs, "0:", 1) == FR_OK)
					{
						if (f_open(&fil, "0:config.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
						{
							snprintf(SD_BUFFER,250,"HH, MM, SS, dTygodnia, DD, MM, YY, czujnik, tcrit, trybWiatraka\n\r");
							if (f_puts(SD_BUFFER, &fil) < 0)
							{
								FLAG_SD_SPACE_VALID = 0;
							}
							snprintf(SD_BUFFER,250,"%.2x,%.2x,%.2x,%.1x,%.2x,%.2x,%.2x,%u,%u,%u",
							time_structure.Hours,
							time_structure.Minutes,
							time_structure.Seconds,
							data_structure.WeekDay,
							data_structure.Date,
							data_structure.Month,
							data_structure.Year,
							measurement_state,
							critical_temp,
							fan_speed);
							if (f_puts(SD_BUFFER, &fil) < 0)
							{
								FLAG_SD_SPACE_VALID = 0;
							}
						}
						f_close(&fil);
					}
					else
					{
						FLAG_SD_SPACE_VALID = 0;
					}
					f_mount(0, "0:", 1);
				}
			else if(FLAG_WRITE_CONFIG ==2)
			{
				if (f_mount(&FatFs, "0:", 1) == FR_OK)
					{
						if (f_open(&fil, "0:config.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
						{
							snprintf(SD_BUFFER,250,"HH, MM, SS, dTygodnia, DD, MM, YY, czujnik, tcrit, trybWiatraka\n\r");
							if (f_puts(SD_BUFFER, &fil) < 0)
							{
								FLAG_SD_SPACE_VALID = 0;
							}
							snprintf(SD_BUFFER,250,"%.2x,%.2x,%.2x,%.1x,%.2x,%.2x,%.2x,%u,%u,%u",
							0x12,
							0x00,
							0x00,
							0x01,
							0x01,
							0x01,
							0x00,
							CORE,
							25,
							FANSPEEDLOW);
							if (f_puts(SD_BUFFER, &fil) < 0)
							{
								FLAG_SD_SPACE_VALID = 0;
							}
						}
						f_close(&fil);
					}
					else
					{
						FLAG_SD_SPACE_VALID = 0;
					}
					f_mount(0, "0:", 1);
			}
			FLAG_WRITE_CONFIG = 0;
		}
}

/**
****************************************************************************************
* @brief 			Ladowanie konfiguracji z karty SD<br><br> 
*							Dane konfiguracyjne zapisywane sa w postaci:<br>
*							godzina, minuty, sekundy, dzien_tygodnia, dzien, miesiac, rok, czujnik, temperatura_krytyczna, tryb_wiatraka<br>
*							Konfiguracja zapisywana jest automatycznie co minute (domyslnie), wiec utrata zasilania na niewielka ilosc czasu 
*							po czym wczytanie ostatnich zapisanych danych z karty sd pozwala na dalsza prace z niewielkim bledem czasu. <br>
*							Funkcja pobiera dane o czasie przy pomocy wywolania HAL_RTC_GetTime i HAL_RTC_GetDate. Wazna jest kolejnosc wywolan,
*							uzycie tych funkcji w innej kolejnosci spowoduje utrate spojnosci otrzymywanych danych (HAL_RTC_GetDate zwraca
*							date pozyskana w momencie wywolania funkcji HAL_RTC_GetTime).<br>
*							Funkcja jest wywolywana przy starcie programu.<br>
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void LoadConfig(void)
{
	RTC_TimeTypeDef time_structure;
	RTC_DateTypeDef data_structure;
	char TEMP_BUFFER[250];
	char *data;
	if(FLAG_SD_SPACE_VALID ==1)
	{
				if (f_mount(&FatFs, "0:", 1) == FR_OK)
					{
						if (f_open(&fil, "0:config.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
						{
							
							if(f_eof(&fil))
							{
								FLAG_WRITE_CONFIG = 2;
								WriteConfig();
							}
							else
						{
								//	snprintf(SD_BUFFER,250,"HH, MM, SS, dTygodnia, DD, MM, YY, czujnik, tcrit, trybWiatraka\n\r");
								f_gets(TEMP_BUFFER,sizeof(TEMP_BUFFER),&fil);
								f_gets(TEMP_BUFFER,sizeof(TEMP_BUFFER),&fil);
								strcat(TEMP_BUFFER,"/0");
								
								data=strtok(TEMP_BUFFER,",");
								time_structure.Hours=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								time_structure.Minutes=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								time_structure.Seconds=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								data_structure.WeekDay=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								data_structure.Date=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								data_structure.Month=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								data_structure.Year=(int)strtol(data,NULL,16);
								data=strtok(NULL,",");
								measurement_state=(int)strtol(data,NULL,10);
								data=strtok(NULL,",");
								critical_temp=(int)strtol(data,NULL,10);
								data=strtok(NULL,",");
								fan_speed=(int)strtol(data,NULL,10);
								HAL_RTC_SetDate(&hrtc,&data_structure,RTC_FORMAT_BCD);
								HAL_RTC_SetTime(&hrtc,&time_structure,RTC_FORMAT_BCD);
							}
						}
						f_close(&fil);
					}
					else
					{
						FLAG_SD_SPACE_VALID = 0;
					}
					f_mount(0, "0:", 1);
	}
	
}

/**
****************************************************************************************
* @brief 			Wlaczenie timera htim1 celem opoznienia odczytu wartosci przycisku, 
*							ktory przez pierwsza, krotka chwile jest niepewny w wyniku zjawiska
*							"bouncingu". <br><br>
*							Wywolywana dla odpowiedniego pinu przypisanego do switcha zglaszajacego
*							przerwanie.<br>
*							Wylacza reakcje na przerwania zewnetrze pochodzace z odpowiedniej linii
*							(przez wywolanie odpowiedniej funkcji kontrolera przerwan:<br>
*							NVIC HAL_NVIC_DisableIRQ(IRQn_Type IRQn).<br>
*	
* @param 			uint16_t GPIO_Pin: okresla pin, do ktorego przypisany jest switch
*	@return			Brak
****************************************************************************************
*/

void SwitchDebouncing(uint16_t GPIO_Pin){
		
	switch(GPIO_Pin){
		
		case SWITCH_0_Pin:	
		if(FLAG_SWITCH0_DEBOUNCING == 0)
			{
				HAL_TIM_Base_Start_IT(&htim1);
				HAL_NVIC_DisableIRQ(EXTI0_IRQn);
				FLAG_SWITCH0_DEBOUNCING = 1;
			}
			break;
		case SWITCH_1_Pin:
		if(FLAG_SWITCH1_DEBOUNCING == 0)
			{
				HAL_TIM_Base_Start_IT(&htim1);
				HAL_NVIC_DisableIRQ(EXTI1_IRQn);
				FLAG_SWITCH1_DEBOUNCING = 1;
			}
			break;
				case SWITCH_2_Pin:	
		if(FLAG_SWITCH2_DEBOUNCING == 0)
			{
				HAL_TIM_Base_Start_IT(&htim1);
				HAL_NVIC_DisableIRQ(EXTI2_IRQn);
				FLAG_SWITCH2_DEBOUNCING = 1;
			}
			break;
		case SWITCH_3_Pin:
		if(FLAG_SWITCH3_DEBOUNCING == 0)
			{
				HAL_TIM_Base_Start_IT(&htim1);
				HAL_NVIC_DisableIRQ(EXTI3_IRQn);
				FLAG_SWITCH3_DEBOUNCING = 1;
			}
			break;
			
		}
}

/**
********************************************************************************************
* @brief 			Obsluga wyswietlania na ekranie LCD 16x2. <br>
*							Funkcja ta dokonuje wyczyszczenia zawartosci wyswietlacza i wpisania linii
*							tekstu gornej i dolnej czesci ekranu. W przypadku zmiany parametru gornej linii
*							(np. trybu pomiaru) nastepuje odpowiednia aktualizacja zmiennej LCD_upper_line 
*							i jej wyswietlenie poprzez funkcje LCD_String. Dolna linijka jest wtedy 
*							zapamietana w zmiennej LCD_lower_line i rowniez wyswietlona. W przypadku zmiany
* 						parametrow dolnej linii dzieje sie analogicznie.
*							
*	
* @param 			uint8_t mode: dla 0- nadpisanie gornej linii ekranu, dla 1- dolnej linii
*	@return			Brak
********************************************************************************************
*/
void UpdateLCD(uint8_t mode)
{
		LCD_Cls();
	if(mode==0){			
		LCD_Locate(0,0);
		if(measurement_state == CORE)
				sprintf(LCD_BUFFER,"CORE: %d", (uint32_t)Measurement_CORE);
		if(measurement_state==LM35)
				sprintf(LCD_BUFFER,"LM35: %d", (uint32_t)Measurement_LM35);
		if(measurement_state==DS18B20)
				sprintf(LCD_BUFFER,"DS18: %d", (uint32_t)Measurement_DS18B20);
		
		if(FLAG_OVER_CRITICAL_TEMPERATURE == 1)
				strcat(LCD_BUFFER, "|ON");
		
		LCD_String(LCD_BUFFER);
			//sprintf(LCD_BUFFER,"%s", LCD_upper_line);
			sprintf(LCD_upper_line,"%s", LCD_BUFFER);
		LCD_Locate(0,1);
			sprintf(LCD_BUFFER,"%s",LCD_lower_line);
		LCD_String(LCD_BUFFER);
		
	}else if(mode==1){
		LCD_Locate(0,0);
		sprintf(LCD_BUFFER,"%s",LCD_upper_line);
		LCD_String(LCD_BUFFER);
		LCD_Locate(0,1);
			if(fan_speed==FANSPEEDLOW)
					sprintf(LCD_BUFFER,"Kryt:%d|Tryb:0",critical_temp);
			else if(fan_speed==FANSPEEDMEDIUM)
					sprintf(LCD_BUFFER,"Kryt:%d|Tryb:1",critical_temp);
			else if(fan_speed==FANSPEEDHIGH)
					sprintf(LCD_BUFFER,"Kryt:%d|Tryb:2",critical_temp);
		LCD_String(LCD_BUFFER);
			sprintf(LCD_lower_line,"%s", LCD_BUFFER);
	}
}


/**
********************************************************************************************
* @brief 			Wywolanie funkcji SwitchDebouncing w momencie zgloszenia przerwania przez
*							ktorys switch
*	
* @param 			uint16_t GPIO_Pin: okresla pin przycisku, ktory zglasza przerwanie
*	@return			Brak
********************************************************************************************
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
		//if(HAL_GPIO_ReadPin(SWITCH_0_GPIO_Port, GPIO_Pin)==GPIO_PIN_SET)
		//HAL_GPIO_TogglePin(DIODA_GPIO_Port,DIODA_Pin);
		//handler_zmienna=handler_zmienna+1;
		SwitchDebouncing(GPIO_Pin);
}


/**
********************************************************************************************
* @brief 			Zmiana wartosci temperatury krytycznej o 1 w gore lub w dol przy obsludze
*							przyciskami w okreslonym dla programu zakresie.<br><br>
*							Switch 2: wzrost o 1
*							Switch 3: spadek o 1
*	
* @param 			uint8_t mode: dla 0- zwiekszenie temp. krytycznej o 1; dla 1- zmniejszenie o 1
*	@return			Brak
********************************************************************************************
*/

void ChangeCriticalTemp(uint8_t mode){
	
	
	if(mode == 0)
		if(critical_temp <= TEMPERATURE_MAX+1)
		{
			critical_temp += 1; 
		}
	
	if(mode == 1)
		if(critical_temp >= TEMPERATURE_MIN+1)
		{
			critical_temp -= 1;
		}
}

/**
****************************************************************************************
* @brief 			Obsluga akcji wcisniecia klawiszy "switch" (0,1,2,3). <br>
*							Wywolywana za posrednictwem funkcji przerwania HAL_TIM_PeriodElapsedCallback
*	
* @param 			uint16_t GPIO_PIN: okresla konretny pin, ktoremu odpowiada switch
*	@return			Brak
****************************************************************************************
*/

void SwitchAction(uint16_t GPIO_PIN){
	
	switch(GPIO_PIN)
	{
	
		//Switch zmieniajacy tryb pomiaru CORE->LM35->DS18B20

		case SWITCH_0_Pin:
		//	HAL_GPIO_TogglePin(DIODA_GPIO_Port,DIODA_Pin);
		//	handler_zmienna=handler_zmienna+1;	
			CycleMeasurementState();
			UpdateLCD(0);
			break;
	
		//	Switch zmieniajacy tryb pracy wiatraczka LOW->MEDIUM->HIGH

		case SWITCH_1_Pin:
		//handler_zmienna=handler_zmienna+1;
			CycleFanState();
			UpdateLCD(1);
			break;
		

		//	Switch zwiekszajacy temp. krytyczna o 1 stopien

		case SWITCH_2_Pin:
			ChangeCriticalTemp(0);
			UpdateLCD(1);
			break;
	
		//	Switch zmniejszajacy temp. krytyczna o 1 stopien

		case SWITCH_3_Pin:
			ChangeCriticalTemp(1);
			UpdateLCD(1);
			break;
	}
	
}

/**
****************************************************************************************
* @brief 			Funkcja obslugi przerwania licznikow. Wywolywana w przypadku wykonania
*							funkcji SwitchDebouncing. <br>W odpowiednich instrukcjach warunkowych sprawdzany
*							jest licznik zglaszajacy przerwanie i wykonywana obsluga: switchy
*							(funkcja SwitchAction) wraz ze zdjeciem flagi "DEBOUNCING"; karty SD;
*							pomiaru temperatury.<br> Do kazdego z tych zdarzen przypisany jest odpowiedni
*							timer
*	
* @param 			TIM_HandleTypeDef *htim: wskaznik na okreslony licznik zglaszajacy 
*							przerwanie, czyli doliczenie do konca okresu
*	@return			Brak
****************************************************************************************
*/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
		
		if(htim->Instance == TIM1)
		{
			HAL_NVIC_EnableIRQ(EXTI0_IRQn);
			HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			if(HAL_GPIO_ReadPin(SWITCH_0_GPIO_Port, SWITCH_0_Pin) == GPIO_PIN_RESET && FLAG_SWITCH0_DEBOUNCING==1) 
			{
				SwitchAction(SWITCH_0_Pin);
				FLAG_SWITCH0_DEBOUNCING = 0;
			}
			if(HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port, SWITCH_1_Pin) == GPIO_PIN_RESET && FLAG_SWITCH1_DEBOUNCING==1) 
			{
				SwitchAction(SWITCH_1_Pin);
				FLAG_SWITCH1_DEBOUNCING = 0;
			}
			if(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port, SWITCH_2_Pin) == GPIO_PIN_RESET && FLAG_SWITCH2_DEBOUNCING==1) 
			{
				SwitchAction(SWITCH_2_Pin);
				FLAG_SWITCH2_DEBOUNCING = 0;
			}
			if(HAL_GPIO_ReadPin(SWITCH_3_GPIO_Port, SWITCH_3_Pin) == GPIO_PIN_RESET && FLAG_SWITCH3_DEBOUNCING==1) 
			{
				SwitchAction(SWITCH_3_Pin);
				FLAG_SWITCH3_DEBOUNCING = 0;
			}
			if(FLAG_SWITCH0_DEBOUNCING==0 && FLAG_SWITCH1_DEBOUNCING==0 && FLAG_SWITCH2_DEBOUNCING==0 && FLAG_SWITCH3_DEBOUNCING==0)
			{
			HAL_TIM_Base_Stop_IT(&htim1);
			}
		}
		if(htim->Instance == TIM5)
		{
			seconds_counter++;
			sd_card_write_counter++;
			SetFanState();
			UpdateFan();
			
			if (seconds_counter >= measurement_period)
			{
				seconds_counter = 0;
				FLAG_START_TEMPERATURE_MEASUREMENT = 1;
			}
			if (sd_card_write_counter >= sd_card_write_period)
			{
				sd_card_write_counter = 0;
				FLAG_WRITE_CONFIG = 1;
			}
			
	
}
}

/**
****************************************************************************************
* @brief 			Sprawdzenie, czy dla obecnego trybu pomiaru temepratura nie przekracza
*							krytycznej, ustalonej wartosci. <br> Wartosc zwracana przez funkcje
*							przez funkcje jest przypisywana do flagi FLAG_OVER_CRITICAL_TEMPERATURE, 
*							ktora w funkcj main zglasza odpowiednie przerwanie.
*	
* @param 			Brak
*	@return			(uint8_t) : 0- temperatura ponizej krytycznej; 1- ponad krytyczna
****************************************************************************************
*/
uint8_t IsTempOverCritical()
{
		
		switch(measurement_state)
		{
			case CORE:
				if(Measurement_CORE >= critical_temp)
						return 1;
				break;
			case LM35:
					if(Measurement_LM35 >= critical_temp)
						return 1;
				break;
			case DS18B20:
					if(Measurement_DS18B20 >= critical_temp)
						return 1;
				break;
			
		}
		return 0;
}


/**
****************************************************************************************
* @brief 			Pomiar temperatury odpowiednim sposobem.<br> <br>
*							Funkcja wywolywana jest w przerwaniu, w przypadku, gdy odpowiedni timer
*							je zglosi oraz w okreslonych przypadkach zmian parametrow pomiaru.
*							W przypadku trybu CORE lub LM35 stosujemy pomiar ADC z uzyciem DMA, jego
*							zapis do tablicy MeasurementADC i odpowiednie przypisanie do zmiennych
*							pomiarowych. <br>
*							Pomiar w przerwaniu jest realizowany poprzez sprawdzenie stanu flagi
*							FLAG_START_TEMPERATURE_MEASUREMENT w petl whie funkcji main. Flaga 	
*							ta jest ustalana na wartosc 1, gdy minie okres czasu okreslony zmienna 
*							measurement_period i zerowana na koncu wywolania funkcji.
*							W przypadku pomiaru termometrem cyfrowym DS18B20 wywolywana jest funkcja
*							DS18B20_ReadTemperature, a jej wartosc zwracana przypisywana jest do
*							zmiennej pomiarowej termometru.<br>
*							Po kazdorazowym pomiarze sprawdzane jest, czy temepratura obecna nie
*							przekracza krytycznej jak rowniez nastepuje aktualizacja LCD.<br>
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/

void StartTemperatureMeasurement()
{
			switch(measurement_state)
			{
				case CORE:
				HAL_ADC_Start_DMA(&hadc1, MeasurementADC, sizeof(MeasurementADC)/sizeof(MeasurementADC[0]));
	
				//nie wiem czy to na pewno ten element z tablicy ADC
				Measurement_CORE = (uint32_t)TemperatureConversion_CORE(MeasurementADC[1]);
				
				FLAG_OVER_CRITICAL_TEMPERATURE = IsTempOverCritical();
				break;
				
				case LM35:
				HAL_ADC_Start_DMA(&hadc1, MeasurementADC, sizeof(MeasurementADC)/sizeof(MeasurementADC[0]));
			
				//nie wiem czy to na pewno ten element z tablicy ADC
				Measurement_LM35 = (uint32_t)TemperatureConversion_LM35(MeasurementADC[0]);
				
				FLAG_OVER_CRITICAL_TEMPERATURE = IsTempOverCritical();
					break;
				
				case DS18B20:
						Measurement_DS18B20 = DS18B20_ReadTemperature();
						FLAG_OVER_CRITICAL_TEMPERATURE = IsTempOverCritical();
						break;
			}
			UpdateLCD(0);
			UpdateLCD(1);
			UpdateFan();
			
		FLAG_START_TEMPERATURE_MEASUREMENT = 0;
}


/**
****************************************************************************************
* @brief 			Ustawienie parametrow pracy wiatrakow<br><br>
*							W zaleznosci od zmiennych globalnych measurement_state oraz fan_speed
*							ustawiane sa rejestry CCR1 odpowiednich timerow odpowiedzialnych za sterowanie
*							wiatrakami przy pomocy generacji sygnalu modulowanego PWM o wypelnieniu zaleznym od
*							wartosci rejestrow CCR1.
*							<br>
*							Zmiana wartosci rejestrow nie aktualizuje od razu pracy timerow, wymagane jest wywolanie funkcji
*							UpdateFan()
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void SetFanState()
	//TODO W zaleznosci od measurementstate wybieramy wiatrak, oraz jego predkosc zalezna od fanSpeed
{
	
		if(measurement_state == CORE)
		{
			
			switch(fan_speed){
				
				case FANSPEEDLOW:
					htim2.Instance->CCR1 = FANSPEEDLOW;
					break;
				case FANSPEEDMEDIUM:
					htim2.Instance->CCR1 = FANSPEEDMEDIUM;
					break;
				case FANSPEEDHIGH: 
					htim2.Instance->CCR1 = FANSPEEDHIGH;
					break;
				default:
					htim2.Instance->CCR1 = FANSPEEDLOW;
					break;
				
			}
//			if (FLAG_OVER_CRITICAL_TEMPERATURE==1) HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}
		else if(measurement_state == DS18B20 || measurement_state == LM35)
		{
			switch(fan_speed){
				
				case FANSPEEDLOW:
					htim3.Instance->CCR1 = FANSPEEDLOW;
					break;
				case FANSPEEDMEDIUM:
					htim3.Instance->CCR1 = FANSPEEDMEDIUM;
					break;
				case FANSPEEDHIGH: 
					htim3.Instance->CCR1 = FANSPEEDHIGH;
					break;
				default:
					htim3.Instance->CCR1 = FANSPEEDLOW;
					break;
				
			}
//			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		}
}

/**
****************************************************************************************
* @brief 			Aktualizacja stanu pracy wiatrakow<br> <br>
*							W zaleznosci od measurement_state wlaczany jest odpowiedni wiatrak, a niewybrany aktualnie wiatrak
*							jest wylaczany<br>
*							Sterowanie wiatrakami odbywa sie przez wlaczenie generacji sygnalu PWM na
*							kanalach pierwszych timerow htim2 lub htim3, w zaleznosci od aktywnego urzadzenia pomiarowego<br>
*							Wlaczenie PWM nastepuje tylko wtedy, gdy pomiar temperatury przekracza wartosc graniczna
*							(flaga FLAG_OVER_CRITICAL_TEMPERATURE rowna 1)<br>
*							Sygnal PWM nie wysterowuje bezposrednio wiatrakow, lecz jest podawany na odpowiednie wejscia
*							modulu sterujacego L293D<br>
*							Funkcja powoduje rowniez zapis informacji o wlaczeniu wiatraka w pliku log.txt poprzez ustawienie
*							flagi FLAG_WRITE_SD.<br>
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void UpdateFan()
{
		if (FLAG_OVER_CRITICAL_TEMPERATURE==1)
		{
			if(FLAG_EVENT_ON == 0)
			{
				FLAG_EVENT_ON = 1;
				FLAG_EVENT_OFF = 0;
				FLAG_WRITE_SD = 1;
			}
			if(measurement_state == CORE)
			{
				if(FLAG_FAN_1_WORK == 0) 
				{
					HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
					FLAG_FAN_1_WORK = 1;
				}
				if(FLAG_FAN_2_WORK == 1)
				{
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					FLAG_FAN_2_WORK = 0;
				}
			}
			else if(measurement_state == DS18B20 || measurement_state == LM35)
			{
				if(FLAG_FAN_2_WORK == 0) 
				{
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
					FLAG_FAN_2_WORK = 1;
				}
				if(FLAG_FAN_1_WORK == 1)
				{
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
					FLAG_FAN_1_WORK = 0;
				}
			}
		}
		else if (FLAG_OVER_CRITICAL_TEMPERATURE ==0)
		{
			if(FLAG_EVENT_OFF == 0)
			{
				FLAG_EVENT_OFF = 1;
				FLAG_EVENT_ON = 0;
				FLAG_WRITE_SD = 1;
			}
			if(FLAG_FAN_1_WORK == 1)
				{
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
					FLAG_FAN_1_WORK = 0;
				}
				if(FLAG_FAN_2_WORK == 1)
				{
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					FLAG_FAN_2_WORK = 0;
				}
		}
}
/**
****************************************************************************************
* @brief 			Zapisanie informacji o przekroczeniu temperatury krytycznej na karcie SD<br><br> 
*							Dane zapisywane sa w postaci:<br>
*							Data, godzina, temperatura_krytyczna, temperatura_aktualna, aktywny czujnik, wlaczenie/wylaczenie wiatraka<br>
*							Funkcja pobiera dane o czasie przy pomocy wywolania HAL_RTC_GetTime i HAL_RTC_GetDate. Wazna jest kolejnosc wywolan,
*							uzycie tych funkcji w innej kolejnosci spowoduje utrate spojnosci otrzymywanych danych (HAL_RTC_GetDate zwraca
*							date pozyskana w momencie wywolania funkcji HAL_RTC_GetTime)<br>
*							O wymuszeniu zapisu wydarzenia informuje flaga FLAG_WRITE_SD.<br>
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void WriteEvent(void)
{
	RTC_TimeTypeDef time_structure;
	RTC_DateTypeDef data_structure;
	HAL_RTC_GetTime(&hrtc,&time_structure,RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc,&data_structure,RTC_FORMAT_BCD);
	char TEMP_BUFFER[100];
	if(FLAG_SD_SPACE_VALID ==1)
	{
				if (f_mount(&FatFs, "0:", 1) == FR_OK)
					{
						if (f_open(&fil, "0:log.txt", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
						{
							snprintf(SD_BUFFER,250,"%x.%x.%x %x:%x:%x TCRIT=%u ",
							data_structure.Date,
							data_structure.Month,
							data_structure.Year,
							time_structure.Hours,
							time_structure.Minutes,
							time_structure.Seconds,
							critical_temp);
							
							switch(measurement_state)
							{
								case CORE:
								snprintf(TEMP_BUFFER,100,"T=%u CORE",(uint32_t)Measurement_CORE);							
								break;
								
								case LM35:
								snprintf(TEMP_BUFFER,100,"T=%u LM35",(uint32_t)Measurement_LM35);
								break;
								
								case DS18B20:
								snprintf(TEMP_BUFFER,100,"T=%u DS18B20",(uint32_t)Measurement_DS18B20);
								break;
							}
							strncat(SD_BUFFER,TEMP_BUFFER,250-strlen(SD_BUFFER));
							if(FLAG_EVENT_ON == 1)
							{
								snprintf(TEMP_BUFFER,100," ON\n\r");
							}
							else if(FLAG_EVENT_OFF == 1)
							{
								snprintf(TEMP_BUFFER,100," OFF\n\r");
							}
							strncat(SD_BUFFER,TEMP_BUFFER,250-strlen(SD_BUFFER));
							if (f_puts(SD_BUFFER, &fil) < 0)
							{
								FLAG_SD_SPACE_VALID = 0;
							}
						}
						f_close(&fil);
					}
					else
					{
						FLAG_SD_SPACE_VALID = 0;
					}
					f_mount(0, "0:", 1);
	}
	FLAG_WRITE_SD = 0;
}

//BORYS

/**
****************************************************************************************
* @brief 			Sprawdzenie stanu zamontowanego nosnika (karta sd)<br><br>
* 						Wykorzystuje funkcje modulu FATFS do okreslenia liczby wolnych sektorow karty,
*							oraz calkowitej liczby sektorow<br>
*							
*	
* @param 			uint32_t* total : wskaznik na zmienna do ktorej zostanie wpisana liczba wszystkich sektorow
*	@param			uint32_t* free : wskaznik na zmienna do ktorej zostanie wpisana liczba wolnych sektorow
*	@return			(FRESULT) : zmienna typu FRESULT modulu FATFS, w przypadku zwracania wartosci innej niz FR_OK oznacza blad
****************************************************************************************
*/
FRESULT FATFS_DriveSize(uint32_t* total, uint32_t* free) {
	//*	@author		Tilen Majerle
	FATFS *fs;
  DWORD fre_clust;
	FRESULT res;

    /* Get volume information and free clusters of drive */
    res = f_getfree("0:", &fre_clust, &fs);
    if (res != FR_OK) {
		return res;
	}

    /* Get total sectors and free sectors */
    *total = (fs->n_fatent - 2) * fs->csize / 2;
    *free = fre_clust * fs->csize / 2;
	
	/* Return OK */
	return FR_OK;
}

/**
****************************************************************************************
* @brief 			Funkcja konfiguracyjna zwiazana z karta SD<br><br>
*							Funkcja montuje karte SD oraz otwiera (lub tworzy) plik konfiguracyjny, w ktorym przechowywane
*							sa dane o stanie urzadzenia. Sluzy do wczytywania zachowanej konfiguracji po utracie zasilania<br>
*							W przypadku bledu, braku wolnego miejsca lub braku karty SD
*							ustawiana jest flaga FLAG_SD_SPACE_VALID, blokujaca proby interakcji z karta do 
*							momentu ponownego wlaczenia urzadzenia.
*	
* @param 			Brak
*	@return			(FRESULT) : zmienna typu FRESULT modulu FATFS, w przypadku zwracania wartosci innej niz FR_OK oznacza blad
****************************************************************************************
*/
FRESULT FATFS_USER_INIT(void)
{
	FRESULT res;
	//Sproboj zamontowac
//	res=f_mount(&FatFs, "0:", 1);
//	if (res == FR_OK)
//	while (res != FR_OK || tries_index<=10)
//	res=f_mount(&FatFs, "0:", 1);
		res=f_mount(&FatFs, "0:", 1);
	/*
	if(res == FR_NOT_READY)
	{
		HAL_Delay(100);
		f_mount(0, "0:", 1);
		HAL_Delay(100);
		res=f_mount(&FatFs, "0:", 1);
	}
	*/
	if(res == FR_OK)
		{
			//Sproboj otworzyc plik
			if (f_open(&fil, "0:config.txt", FA_OPEN_ALWAYS | FA_READ |FA_WRITE) == FR_OK)
				{
					//Sprawdz dane o klastrach
					if (FATFS_DriveSize(&total_space, &free_space) == FR_OK)
						{
							/* Data for drive size are valid */
							FLAG_SD_SPACE_VALID = 1;
						}
				}
				f_close(&fil);
		}
		f_mount(0, "0:", 1);
		return res;
}

/**
****************************************************************************************
* @brief 			Funkcja inicjalizujaca peryferia zwiazane z warstwa HAL<br> <br>
*							Wywolywana jest na poczatku dzialania programu<br>
*							Inicjalizuje liczniki, porty GPIO, zegar systemowy, SPI i USART, przetworniki ADC, zegar RTC, modul FATFS<br>
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void UserHalInit(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();



  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM9_Init();

	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);	
}

/**
****************************************************************************************
* @brief 			Funkcja inicjalizujaca obsluge zewnetrznych urzadzen<br> <br>
*							Wywolywana jest na poczatku dzialania programu<br>
*							Inicjalizowany jest wyswietlacz LCD, testowana komunikacja z modulem karty SD, nasluchiwanie UART, 
*							oraz wykonywany jest pojedynczy pomiar ADC temperatury procesora STM32, w celu unikniecia nieprawidlowych pomiarow.<br>
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void UserCustomInit(void)
{
  LCD_Init();
	measurement_period = 5; //seconds
	sd_card_write_period = 60; //konfiguracja co minute
	StartTemperatureMeasurement();
	HAL_Delay(5);
	sprintf(LCD_upper_line, "CORE: TRWA...");
	sprintf(LCD_BUFFER,"T_kryt: %d|Tryb: %d",critical_temp, fan_speed);
	UpdateLCD(0);
	UpdateLCD(1);
	FATFS_USER_INIT();
	HAL_UART_Receive_IT(&huart2, (uint8_t*)UART_RECEIVED, 10);
}
/**
****************************************************************************************
* @brief 			Funkcja sprawdzajaca potrzebe wykonanie pomiaru temperatury<br> <br>
*							Sprawdza flage FLAG_START_TEMPERATURE_MEASUREMENT<br>
*							Wywoluje StartTemperatureMeasurement()<br>
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void CheckMeasurement(void)
{
 if(FLAG_START_TEMPERATURE_MEASUREMENT == 1)
		{
			StartTemperatureMeasurement();
		}
}
/**
****************************************************************************************
* @brief 			Funkcja sprawdzajaca mozliwosc komunikacji z modulem karty SD<br> <br>
*							Sprawdza flage FLAG_SD_SPACE_VALID<br>
*							W przypadku braku mozliwosci komunikacji zapala czerwona diode.<br>
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void CheckSD(void)
{
if(FLAG_SD_SPACE_VALID == 0)
		{
			HAL_GPIO_WritePin(DIODA_GPIO_Port,DIODA_Pin,GPIO_PIN_SET);
		}
else
		{
			HAL_GPIO_WritePin(DIODA_GPIO_Port,DIODA_Pin,GPIO_PIN_RESET);
		}
}

/**
****************************************************************************************
* @brief 			Funkcja sprawdzajaca potrzebe zapisu informacji o przekroczeniu temperatury granicznej.<br> <br>
*							Sprawdza flage FLAG_WRITE_SD, komunikacja z SD musi dzialac (FLAG_SD_SPACE_VALID==1).<br>
*							Funkcja transmituje wiadomosc przez UART oraz wywoluje WriteEvent()<br>
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void CheckTemperatureEvent(void)
{
if(FLAG_WRITE_SD ==1 && FLAG_SD_SPACE_VALID ==1)
		{
			UART_BUFFER_SIZE=sprintf(UART_BUFFER, "EVENT SD\n\r");
			while(HAL_UART_Transmit_IT(&huart2, (uint8_t*)UART_BUFFER, UART_BUFFER_SIZE) == HAL_BUSY)
			{
			}
			WriteEvent();
			FLAG_WRITE_SD = 0;
		}
}

/**
****************************************************************************************
* @brief 			Funkcja sprawdzajaca potrzebe zapisu informacji konfiguracyjnych<br> <br>
*							Stan pracy oraz data i godzina zblizona do aktualnej jest utrzymywana przez cykliczny zapis na karte SD.<br>
*							Sprawdza wlage FLAG_WRITE_CONFIG, komunikacja z SD musi dzialac (FLAG_SD_SPACE_VALID==1).<br>
*							Wywoluje WriteConfig()<br>
*							
*							
*	
* @param 			Brak
*	@return			Brak
****************************************************************************************
*/
void CheckConfig(void)
{
if(FLAG_WRITE_CONFIG >0 && FLAG_SD_SPACE_VALID ==1)
		{
			WriteConfig();
			FLAG_WRITE_CONFIG = 0;
		}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	UserHalInit();
	UserCustomInit();	
	LoadConfig();

  /* Infinite loop */
  while (1)
  {
		CheckMeasurement();
		CheckSD();
		CheckTemperatureEvent();
		CheckConfig();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
