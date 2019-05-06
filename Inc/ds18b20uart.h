/*
 * ds18b20uart.h
 *
 *  Created on: 30.12.2018
 *      Author: Karol
 */

#ifndef DS18B20UART_H_
#define DS18B20UART_H_



#endif /* DS18B20UART_H_ */

#define OW_1				0xFF
#define OW_0				0x00
#define OW_RESET			0xF0
void DS18B20_UART_Init(uint32_t baudrate);
void RESET_PULSE();
void ReadTemp();
uint8_t ReadBit(uint8_t bit);
uint8_t ReadByte(uint8_t number_of_byte);
void ReadTempStateRx();
void ds18b20();
