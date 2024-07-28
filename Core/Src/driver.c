/*
 * TMC2209.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Chris Hauser
 */

#include "main.h"

//TODO Befehle definieren

uint8_t uart_calcCRC(uint8_t* datagram, uint8_t datagramLength) //Code aus Datenblatt von TMC2209 übernommen
{
	int i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;
	*crc = 0;
	for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
		currentByte = datagram[i]; // Retrieve a byte to be sent from Array
		for (j=0; j<8; j++) {
			if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
			{
				*crc = (*crc << 1) ^ 0x07;
			}
			else
			{
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
	return *crc;
}

void send_command_to_driver_x(uint8_t command, uint8_t value)
{
	uint8_t data[3];
	data[0] = command;
	data[1] = value;
	data[2] = uart_calcCRC(data, 3); //Prüfsumme
	HAL_UART_Transmit(&huart2, data, 3, 100);
}

void send_command_to_driver_z(uint8_t command, uint8_t value)
{
	uint8_t data[3];
	data[0] = command;
	data[1] = value;
	data[2] = uart_calcCRC(data, 3); //Prüfsumme
	HAL_UART_Transmit(&huart8, data, 3, 100);
}
