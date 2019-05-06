/*
 * ws2812b.c
 *
 *  Created on: 28.11.2018
 *      Author: Karol
 */


#include "stm32f1xx_hal.h"
#include "ws2812b.h"


extern SPI_HandleTypeDef hspi2;
extern struct Colors off;
uint8_t leds_buffer[LEDS_BUFFER_SIZE];
const uint8_t zero = 0b1000;
const uint8_t one = 0b1110;
const uint8_t gamma8[] =
{
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
};

rgb led_color[NUMBER_OF_LEDS];
uint8_t blink[NUMBER_OF_LEDS];
uint32_t lighton[NUMBER_OF_LEDS];
uint32_t lightoff[NUMBER_OF_LEDS];
volatile uint32_t blink_count[NUMBER_OF_LEDS];
void WS2812B_Init()
{
	rgb off;
	off.red = 0;
	off.green = 0;
	off.blue = 0;
	SetAllLEDColor_RGB(off,1);
	WS2812B_Update();
}
void SetLEDColor_RGB(uint16_t led_number,rgb color,uint8_t save)
{
	uint8_t green = gamma8[color.green];
	uint8_t blue = gamma8[color.blue];
	uint8_t red = gamma8[color.red];
	if(save) led_color[led_number] = color;
	if(led_number >= 0 && led_number < NUMBER_OF_LEDS)
	{
		  for(int j = 7; j >= 0; j--)//Green
		  {
			  if(green % 2 == 1) leds_buffer[24*led_number+j] = one;
			  else leds_buffer[24*led_number+j] = zero;
			  green /= 2;
		  }
		  for(int j = 15; j >= 8; j--)//Red
		  {
			  if(red % 2 == 1) leds_buffer[24*led_number+j] = one;
			  else leds_buffer[24*led_number+j] = zero;
			  red /= 2;
		  }
		  for(int j = 23; j >= 16;j--)//Blue
		  {
			  if(blue % 2 == 1) leds_buffer[24*led_number+j] = one;
			  else leds_buffer[24*led_number+j] = zero;
			  blue /= 2;
		  }
	}
}
void SetAllLEDColor_RGB(rgb color,uint8_t save)
{
	for(int i = 0;i < NUMBER_OF_LEDS;i++) SetLEDColor_RGB(i,color,save);
}
void WS2812B_Update()
{
	HAL_SPI_Transmit(&hspi2,leds_buffer,192,1000);
}
void WS2812B_Blink()
{
	for(int i = 0; i < NUMBER_OF_LEDS; i++)
	{
		if(blink[i])
		{
			if(blink_count[i] == (lightoff[i]+lighton[i])-1) blink_count[i] = 0;
			if(blink_count[i] == 0)
			{
				SetLEDColor_RGB(i,led_color[i],1);
				WS2812B_Update();
			}
			if(blink_count[i] == lighton[i]-1)
			{
				SetLEDColor_RGB(i,off,0);
				WS2812B_Update();
			}
		}
	}
}
