#include			"stm32f1xx_hal.h"
#include			"ds18b20uart.h"

#define				OW_1				0xFF
#define				OW_0				0x00
#define 			OW_R				0xFF

extern UART_HandleTypeDef huart1;

uint8_t convert_T[] =
{
	OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
    OW_0, OW_0, OW_1, OW_0, OW_0, OW_0, OW_1, OW_0  // 0x44 CONVERT T
};
uint8_t read_scratch[] =
{
	OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
	OW_0, OW_1, OW_1, OW_1, OW_1, OW_1, OW_0, OW_1, // 0xbe READ SCRATCH
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
	OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R
};

uint8_t OW_receive;
volatile uint8_t ds18b20_state = 0;
uint16_t ds18b20_delay = 0;
uint8_t scratchpad_uart[88];
uint8_t scratchpad[9];
uint8_t calkowita_temp, ulamkowa_temp, minus = 0;
uint8_t usart1_tx_rx_timeout = 0;
uint32_t ds18b20_fix;
uint32_t ds18b20_fix_count;
uint8_t ds18b20_fix_enable;

char temp_sign = '+';
extern uint8_t temperature_mode_now;
extern uint8_t temperature_mode_before;
extern uint8_t temperature_mode;
uint8_t CRC8(uint8_t *inData, uint8_t DataLength)
{
	uint8_t bitCtr;
	uint8_t byteCtr;
	uint8_t CurrentByte;
	uint8_t crc;
	uint8_t temp;
	crc = 0;
	for ( byteCtr = 0; byteCtr<DataLength;byteCtr++)
	{
		CurrentByte = *inData++;
		for (bitCtr = 0; bitCtr <8; bitCtr++)
		{
			temp = (CurrentByte ^ crc) & 0x01 ;
			crc >>=1;
			if ( temp )
			{
				crc ^=0x8C;
			}
			CurrentByte >>= 1;
		}
	}
	return crc;
}

void DS18B20_UART_Init(uint32_t baudrate)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = baudrate;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);
}
void RESET_PULSE()
{
	uint8_t reset_pulse = 0xF0;
	DS18B20_UART_Init(9600);
	HAL_UART_Transmit_DMA(&huart1,&reset_pulse,1);
	HAL_UART_Receive_DMA(&huart1,&OW_receive,1);
}
void ReadTemp()
{
	ds18b20_state = 1;
}
uint8_t ReadBit(uint8_t bit)
{
	if(bit == 0xFF) return 1;
	else return 0;
}
uint8_t ReadByte(uint8_t number_of_byte)
{
	uint8_t value = 0;
	for(int i = 0; i < 8; i++)
	{
		if(ReadBit(scratchpad_uart[8*number_of_byte+i])) value |= 0x01 << i;
	}
	return value;
}
void ReadScratchPad()
{
	for(int i = 2;i < 12; i++) scratchpad[i-2] = ReadByte(i);
}
void ReadTempStateRx()
{
	switch(ds18b20_state)
	{
		case 2:
		{
			if(OW_receive != 0xF0) ds18b20_state = 3;
			else ds18b20_state = 254; //blad, gdy nie wykryto termometru na lini
			break;
		}
		case 4:
		{
			ds18b20_state = 0xFF;
			break;
		}
		case 6:
		{
			if(OW_receive != 0xF0) ds18b20_state = 7;
			else ds18b20_state = 254; //blad, gdy nie wykryto termometru na lini
			break;
		}
		case 8:
		{
			ds18b20_state = 9;
			break;
		}

	}
}
void ds18b20()
{
	switch(ds18b20_state)
	{
		case 1://wyslanie RESET
		{
			RESET_PULSE();
			OW_receive = 0;
			ds18b20_state = 2;
			ds18b20_delay = 0;
			usart1_tx_rx_timeout = 0;
			break;
		}
		case 3://wyslanie convert T
		{
			DS18B20_UART_Init(115200);
			HAL_UART_Transmit_DMA(&huart1,&convert_T[0],sizeof(convert_T));
			HAL_UART_Receive_DMA(&huart1,&scratchpad_uart[0],16);
			ds18b20_state = 4;
			break;
		}
		case 5://Po odczekaniu 750 ms, RESET a potem Read Scratchpad
		{
			RESET_PULSE();
			ds18b20_state = 6;
			break;
		}
		case 7:
		{
			DS18B20_UART_Init(115200);
			HAL_UART_Transmit_DMA(&huart1,&read_scratch[0],sizeof(read_scratch));
			HAL_UART_Receive_DMA(&huart1,&scratchpad_uart[0],88);
			ds18b20_state = 8;
			break;
		}
		case 9://Odczyt i analiza
		{
			ReadScratchPad();
			uint8_t crc_calc = CRC8(&scratchpad[0],8);
			if(scratchpad[8] == crc_calc) //scratchpad[8] - CRC
			{
				//scratchpad[0] - LSB
				//scratchpad[1] - MSB
				if (scratchpad[1] & 0x80)    //dla liczb ujemnych negacja i +1
				{
					scratchpad[1]=~scratchpad[1];
					scratchpad[0]=~scratchpad[0]+1;
					minus = 1;
				}
				else minus = 0;

				calkowita_temp = (uint8_t) ((uint8_t) (scratchpad[1]&0x7)<<4 ) | ((uint8_t) (scratchpad[0]&0xf0)>>4 );    //wyodrebnia calkowita wartosc temperatury
				if(minus) calkowita_temp = calkowita_temp | 0x80; //jezeli ujemna ustawiamy 1 bit na 1
				ulamkowa_temp = ((scratchpad[0] & 0x0F)*625)/1000;            //wyodrebnia ulamkowa czesc temperatury
				if(temperature_mode)
				{
					if(temperature_mode_now != temperature_mode_before)
					{
							temperature_mode = 2;
							temperature_mode_before = temperature_mode_now;
					}
					temperature_mode_now = calkowita_temp;
				}
				else if(ds18b20_fix_enable) WriteTemperatureToFlash(calkowita_temp,ulamkowa_temp);
				else
				{
					temp_sign = calkowita_temp & 0x80?'-':'+';
					calkowita_temp = calkowita_temp & 0x7F;
					SendFrame("TEMP %c%d.%d",temp_sign,calkowita_temp,ulamkowa_temp);
				}
			}
			else
			{
				SendFrame("DS18B20 WRONG CHECKSUM");
			}
			ds18b20_state = 0;
			break;
		}
		case 253:
		{
			ds18b20_state = 0;
			SendFrame("DS18B20 TX-RX ERROR");
			break;
		}
		case 254:
		{
			ds18b20_state = 0;
			SendFrame("DS18B20 NOT DETECTED");
			break;
		}
	}
}
