
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include 								"string.h"
#include 								"stdarg.h"
#include 								"ws2812b.h"
#include 								"stdlib.h"
#include 								"ds18b20uart.h"
#define BUFF_SIZE 						256
#define MAX_PARAMETERS					5
#define COMMAND_SIZE					128
#define END             				0x2F			//Znak poczatku/konca ramki
#define ESC             				0x3C
#define ESC_END							0x3D
#define ESC_ESC							0x3E
#define WAIT_START						0
#define ESCAPE							1
#define AFTER_ESCAPE					3
#define MESSAGE							4
#define FRAME_RECEIVED					5
#define FLASH_TEMPERATURE_START			0x08018000
#define FLASH_START						0x08000000
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
char Buff_Rx[BUFF_SIZE];
char Buff_Tx[BUFF_SIZE];

volatile uint8_t Busy_Tx = 0;
volatile uint8_t Empty_Tx = 0;
volatile uint8_t Busy_Rx = 0;
volatile uint8_t Empty_Rx = 0;

uint32_t parameters[MAX_PARAMETERS];
uint8_t all_parameters = 1;
char command[COMMAND_SIZE];
uint8_t bajty_idle = 0;
uint8_t frame_state = WAIT_START;
int idx;

uint8_t crc_frame;
uint8_t frame_length;

extern uint32_t ds18b20_fix;
extern uint32_t ds18b20_fix_enable;
extern uint32_t ds18b20_fix_count;
extern uint8_t ds18b20_state;
extern rgb led_color[16];
extern uint8_t blink[16];
extern uint32_t lighton[16];
extern uint32_t lightoff[16];
extern uint32_t blink_count[16];





uint32_t flash_temperature_write;
uint32_t flash_temperature_read;
uint32_t flash_temperature_lastwrited;
uint32_t flash_temperature_idx = 1;

uint8_t temperature_mode = 0;
uint8_t temperature_mode_before = 0;
uint8_t temperature_mode_now = 0;
uint16_t temperature_mode_counter = 0;
char get_temp_sign;

struct Colors off = { .red = 0, .green = 0, .blue = 0 };
struct Colors hot = { .red = 0, .green = 255, .blue = 0};
struct Colors cold = { .red = 0, .green = 0, .blue = 255};
struct Colors owncolor = { .red = 0, .green = 0, .blue = 0};



FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t ErrorTemperatureFlash;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void USART_fsend(char* format, ...) {

	char tmp_rs[128];//zmiena tymczasowa tmp_rs
	int i;
	__IO int idx;// __IO = volatile, indeks naszej tablicy
	va_list valist; //tworzymy liste ze zmienna iloscia argumentu
	va_start(valist, format);//inicjalizuje valist, aby pobrać dodatkowe argumenty po parametrze
	vsprintf(tmp_rs, format, valist);//do zmiennej tmp_rs jest zapisywany sformatowany już ciąg znaków
	va_end(valist); // zakonczenie listy ze zmienna iloscia argumentow
	idx = Empty_Tx; //naszemu indeksowi przypisujemy wskaznik na puste miejsce w buforze nadawczym
	for (i = 0; i < strlen(tmp_rs); i++) {//wypelniamy bufor nadawczy znakami
		Buff_Tx[idx] = tmp_rs[i];
		idx++;
		if (idx >= BUFF_SIZE)//jezeli przekroczymy rozmiar to zerujemy index
			idx = 0;
	}
	__disable_irq(); //wylaczamy przerwania
	if ((Empty_Tx == Busy_Tx)//jezeli wskazniki na puste i zajete miejsce w buforze sa takie same
			&& (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE) == SET)) {//oraz UART nic nie nadaje
		//to rozpoczynamy wysylanie
		Empty_Tx = idx;//przypisujemy wskazniki pustego miejsca nasz indeks
		uint8_t tmp = Buff_Tx[Busy_Tx];//zmiennej tymczasowej przypisujemy element z bufora nadawczego
		Busy_Tx++;//zwiekszamy wskaznik miejsca zajetego w buforze
		if (Busy_Tx >= BUFF_SIZE)//jezeli przekroczymy zakres bufora to zerujemy wskaznik
			Busy_Tx = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);//transmitujemy pierwszy bajt danych
		//po przeslaniu tego bajtu zostanie wywołane przerwanie z
		//callbacku

	} else {// w przeciwnym wypadku
		Empty_Tx = idx; //wskaznikowi pustego miejsca przypisujemy nasz indeks
	}
	__enable_irq(); // wlaczamy przerwania
}
void analyze_frame(char *jmd)//analiza odebranej ramki
{
	frame_length = strlen(jmd);
	if(frame_length > 0)
	{
		crc_frame = CRC8(jmd,frame_length-1);
		crc_frame=jmd[frame_length-1];
		if(jmd[1] == 'B')
		{
			if(jmd[frame_length-1] == crc_frame)
			{
				if(strncmp(jmd+2,"setled",6) == 0)
				{
					all_parameters = sscanf(jmd+2,"setled:%d:%d:%d:%d",&parameters[0],&parameters[1],&parameters[2],&parameters[3]);
					if(all_parameters == 4)
					{
						if((parameters[0] <= 15 && parameters[0] >= 0) || parameters[0] == -1)
						{
							if(parameters[1] >= 0 && parameters[1] <= 255)
							{
								if(parameters[2] >= 0 && parameters[2] <= 255)
								{
									if(parameters[3] >= 0 && parameters[3] <= 255)
									{
										DisableWS2812BThermometerMode();
										owncolor.red = parameters[1];
										owncolor.green = parameters[2];
										owncolor.blue = parameters[3];
										if(parameters[0] == -1) SetAllLEDColor_RGB(owncolor,1);
										else SetLEDColor_RGB(parameters[0],owncolor,1);
										WS2812B_Update();
										SendFrame("SETLED OK");
									}
									else SendFrame("SETLED PARAM 4 FAIL");
								}
								else SendFrame("SETLED PARAM 3 FAIL");
							}
							else SendFrame("SETLED PARAM 2 FAIL");
						}
						else SendFrame("SETLED PARAM 1 FAIL");
					}
					else SendFrame("SETLED PARAMS FAIL");
				}
				else if(strncmp(jmd+2,"readtempflash",13) == 0)
				{
					ReadTemperatureFromFlash();
				}
				else if(strncmp(jmd+2,"readtemp",8) == 0)
				{
					if(ds18b20_fix_enable != 0) ds18b20_fix_enable = 0;
					DisableWS2812BThermometerMode();
					SendFrame("READTEMP OK");
					ReadTemp();
				}
				else if(strncmp(jmd+2,"fixtempoff",10) == 0)
				{
					DisableWS2812BThermometerMode();
					SendFrame("FIXTEMPOFF OK");
					ds18b20_fix_enable = 0;
					ds18b20_state = 0;
				}
				else if(strncmp(jmd+2,"fixtemp",7) == 0)
				{
					all_parameters = sscanf(jmd+2,"fixtemp:%d",&parameters[0]);
					if(all_parameters == 1)
					{
						if(parameters[0] >= 1000)
						{
							DisableWS2812BThermometerMode();
							ds18b20_fix_enable = 1;
							ds18b20_fix_count = 0;
							ds18b20_fix = parameters[0];
							SendFrame("FIXTEMP OK");
						}
						else SendFrame("FIXTEMP PARAM 1 FAIL");
					}
					else SendFrame("FIXTEMP PARAMS FAIL");
				}
				else if(strncmp(jmd+2,"blinkoff",8) == 0)
				{
					all_parameters = sscanf(jmd+2,"blinkoff:%d",&parameters[0]);
					if(all_parameters == 1)
					{
						if((parameters[0] >= 0 && parameters[0] <= 15) || parameters[0] == -1)
						{
							DisableWS2812BThermometerMode();
							if(parameters[0] == -1)
							{
								for(uint16_t i = 0; i < NUMBER_OF_LEDS; i++)
								{
									blink[i] = 0;
									blink_count[i] = 0;
									lighton[i] = 0;
									lightoff[i] = 0;
									SetLEDColor_RGB(i,off,1);
								}
							}
							else
							{
								blink[parameters[0]] = 0;
								blink_count[parameters[0]] = 0;
								lighton[parameters[0]] = 0;
								lightoff[parameters[0]] = 0;
								SetLEDColor_RGB(parameters[0],off,1);
							}
							WS2812B_Update();
							SendFrame("BLINKOFF OK");
						}
						else SendFrame("BLINKOFF PARAM 1 FAIL");
					}
					else SendFrame("BLINKOFF PARAMS FAIL");
				}
				else if(strncmp(jmd+2,"blink",5) == 0)
				{
					all_parameters = sscanf(jmd+2,"blink:%d:%d:%d",&parameters[0],&parameters[1],&parameters[2]);
					if(all_parameters == 3)
					{
						if(parameters[0] <= 15 && parameters[0] >= 0)
						{
							if(parameters[1] >= 10)
							{
								if(parameters[2] >= 10)
								{
									DisableWS2812BThermometerMode();
									SendFrame("BLINK OK");
									blink[parameters[0]] = 1;
									blink_count[parameters[0]] = 0;
									lighton[parameters[0]] = parameters[1];
									lightoff[parameters[0]] = parameters[2];
								}
							}
							else SendFrame("BLINK PARAM 2 FAIL");
						}
						else SendFrame("BLINK PARAM 1 FAIL");
					}
					else SendFrame("BLINK PARAMS FAIL");
				}
				else if(strncmp(jmd+2,"tmodeoff",8) == 0)
				{
					DisableWS2812BThermometerMode();
					SendFrame("TMODEOFF OK");
				}
				else if(strncmp(jmd+2,"tmode",5) == 0)
				{
					temperature_mode = 1;
					temperature_mode_counter = 0;
					SendFrame("TMODE OK");
				}
				else SendFrame("WRONG COMMAND");
			}
			else
			{
				SendFrame("WRONG CHECKSUM");
			}
		}
		else
		{
			SendFrame("WRONG ADDRESS");
		}
	}
	else SendFrame("EMPTY FRAME");
}
char getchar_BuffRx()//pobieranie kolejnego znaku z bufora RX
{
	char tmp;
	if(Busy_Rx != Empty_Rx)
	{
		tmp = Buff_Rx[Busy_Rx];
		Busy_Rx++;
		if(Busy_Rx > BUFF_SIZE) Busy_Rx = 0;
		return tmp;
	}
	return -1;
}
void clear_data()
{
	for(int i = 0; i < COMMAND_SIZE; i++) command[i] = 0;
	idx = 0;
}
void search_frame()
{
	if(Busy_Rx != Empty_Rx)
	{
		static char c;
		c = getchar_BuffRx();
		__HAL_TIM_SET_COUNTER(&htim4,0);
		switch(frame_state)
		{
			case WAIT_START:
			{
				if(c == END)
				{
					clear_data();
					frame_state = MESSAGE;
				}
				break;
			}
			case MESSAGE:
			{
				if(c == END)
				{
					analyze_frame(command);
					clear_data();
					frame_state = MESSAGE;
				}
				else if(c == ESC) frame_state = AFTER_ESCAPE;
				else
				{
					command[idx] = c;
					idx++;
				}
				break;
			}
			case AFTER_ESCAPE:
			{
				if(c == ESC_END)
				{
					command[idx] = END;
					idx++;
				}
				else if(c == ESC_ESC)
				{
					command[idx] = ESC;
					idx++;
				}
				frame_state = MESSAGE;
				break;
			}
		}
	}
}
void AddCharToFrame(char *frame,uint8_t pos, char x)
{
	if(pos < COMMAND_SIZE)
	{
		frame[pos] = x;
	}
}
void SendFrame(char *format, ...)
{
	char data[COMMAND_SIZE];
	va_list valist;
	va_start(valist, format);
	vsprintf(data, format, valist);
	va_end(valist);
	char frame_to_send[COMMAND_SIZE] = {0};
	AddCharToFrame(frame_to_send,0,'B');
	AddCharToFrame(frame_to_send,1,'A');
	for(int i = 0; data[i] != '\0';i++)
	{
		switch(data[i])
		{
			case ESC:
			{
				AddCharToFrame(frame_to_send,i+2,ESC);
				AddCharToFrame(frame_to_send,i+3,ESC_ESC);
				break;
			}
			case END:
			{
				AddCharToFrame(frame_to_send,i+2,ESC);
				AddCharToFrame(frame_to_send,i+3,ESC_END);
				break;
			}
			default:
			{
				AddCharToFrame(frame_to_send,i+2,data[i]);
				break;
			}
		}
	}
	uint8_t crc_for_frame = CRC8(frame_to_send,strlen(frame_to_send));
	AddCharToFrame(frame_to_send,strlen(frame_to_send),crc_for_frame);
	USART_fsend("%s",frame_to_send);
}
uint8_t NumberPage(uint32_t adres)
{
	uint32_t numer = adres - 0x08000000;
	numer /= 1024;
	return numer;
}

void SearchHighestFlashIdx(uint32_t *max_idx, uint32_t *write_address)
{
	uint32_t current_address_idx;
	*max_idx = 0;
	*write_address = FLASH_TEMPERATURE_START-8;
	for(uint32_t address = FLASH_TEMPERATURE_START;address < 0x08020000; address += 8)
	{
		current_address_idx = *(__IO uint32_t *)address;

		if(current_address_idx != 0xFFFFFFFF && current_address_idx > *max_idx)
		{
				*max_idx = current_address_idx;
				*write_address = address;
		}
	}
	*max_idx += 1;
	*write_address += 8;
}
void WriteTemperatureToFlash(uint16_t calkowita, uint16_t ulamkowa)
{
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.NbPages = 1;
	HAL_FLASH_Unlock();
	if(flash_temperature_write == 0x08020000) flash_temperature_write = FLASH_TEMPERATURE_START;
	if((flash_temperature_write - FLASH_START) % 1024 == 0)
	{
		uint8_t nbpage_write = NumberPage(flash_temperature_write);
		uint8_t nbpage_read = NumberPage(flash_temperature_read);
		if(nbpage_write == nbpage_read && flash_temperature_lastwrited < flash_temperature_read) flash_temperature_read = 0x08000000 + ((nbpage_read+1) * 1024);
		EraseInitStruct.PageAddress = flash_temperature_write;
		HAL_FLASHEx_Erase(&EraseInitStruct, &ErrorTemperatureFlash);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,flash_temperature_write,flash_temperature_idx);
	flash_temperature_write += 4;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,flash_temperature_write,calkowita);
	flash_temperature_write += 2;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,flash_temperature_write,ulamkowa);
	flash_temperature_lastwrited = flash_temperature_write;
	flash_temperature_write += 2;
	HAL_FLASH_Lock();
	flash_temperature_idx++;
}
void ReadTemperatureFromFlash()
{
	if(flash_temperature_read != flash_temperature_write)
	{
		if(flash_temperature_read == 0x08020000) flash_temperature_read = FLASH_TEMPERATURE_START;
		flash_temperature_read += 4;
		uint16_t calkowita = *(__IO uint16_t *)flash_temperature_read;
		flash_temperature_read += 2;
		uint16_t ulamkowa = *(__IO uint16_t *)flash_temperature_read;
		flash_temperature_read += 2;
		get_temp_sign = calkowita & 0x80?'-':'+';
		calkowita = calkowita & 0x7F;
		SendFrame("TEMP %c%d.%d",get_temp_sign,calkowita,ulamkowa);
	}
	else
	{
		SendFrame("NO DATA");
	}
}

void WS2812BThermometerMode()
{
	if(temperature_mode == 2)
	{
		SetAllLEDColor_RGB(off,1);
		get_temp_sign = temperature_mode_now & 0x80?'-':'+';
		temperature_mode_now = temperature_mode_now & 0x7F;
		uint8_t x = 0;
		if(temperature_mode_now % 5 == 0) x = ((temperature_mode_now/5) - 1);
		else (x = temperature_mode_now/5);
		if(x > 7) x = 7;
		if(get_temp_sign == '+') for(int i = 0; i <= x;i++) SetLEDColor_RGB(i,hot,1);
		if(get_temp_sign == '-') for(int i = 15; i >= 15-x;i--) SetLEDColor_RGB(i,cold,1);
		WS2812B_Update();
		temperature_mode = 1;
	}
}
void DisableWS2812BThermometerMode()
{
	if(temperature_mode != 0)
	{
		temperature_mode = 0;
		temperature_mode_counter = 0;
		temperature_mode_before = 0;
		temperature_mode_now = 0;
		SetAllLEDColor_RGB(off,1);
		ds18b20_state = 0;
		WS2812B_Update();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  WS2812B_Init();
  HAL_UART_Receive_IT(&huart2,(uint8_t *)&Buff_Rx[Empty_Rx],1);
  HAL_TIM_Base_Start_IT(&htim4);
  flash_temperature_read = FLASH_TEMPERATURE_START;
  SearchHighestFlashIdx(&flash_temperature_idx, &flash_temperature_write);
  flash_temperature_lastwrited = flash_temperature_write - 2;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  search_frame();
	  ds18b20();
	  WS2812B_Blink();
	  WS2812BThermometerMode();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 31;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 103;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2)
	{
		Empty_Rx++;
		if (Empty_Rx >= BUFF_SIZE)
					Empty_Rx = 0;
		HAL_UART_Receive_IT(&huart2,(uint8_t *)&Buff_Rx[Empty_Rx],1);
	}
	if(huart == &huart1)
	{
		ReadTempStateRx();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2)
	{
		if(Empty_Tx != Busy_Tx){
			uint8_t tmp = Buff_Tx[Busy_Tx];
			Busy_Tx++;
			if (Busy_Tx >= BUFF_SIZE)
						Busy_Tx = 0;
			HAL_UART_Transmit_IT(&huart2,&tmp,1);
		}
	}
}
uint8_t xx;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) == SET && frame_state == MESSAGE)
		{
			bajty_idle++;
			if(bajty_idle >= 3)
			{
				bajty_idle = 0;
				frame_state = WAIT_START;
			}
		}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
