/*
 * ws2812b.h
 *
 *  Created on: 28.11.2018
 *      Author: Karol
 */

#ifndef WS2812B_H_
#define WS2812B_H_



#endif /* WS2812B_H_ */

#define NUMBER_OF_LEDS 		16
#define BITS_PER_LED		24
#define LEDS_BUFFER_SIZE 	NUMBER_OF_LEDS*BITS_PER_LED

typedef struct Colors
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgb;

void WS2812B_Init();
void SetLEDColor_RGB(uint16_t led_number,rgb color,uint8_t save);
void SetAllLEDColor_RGB(rgb color,uint8_t save);
void WS2812B_Update();
