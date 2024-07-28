/*
 * TMC2209.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Chris Hauser
 */

//TODO Befehle definieren

void send_command_to_driver_x(uint8_t command, uint8_t value)
{
    uint8_t data[3];
    data[0] = command;
    data[1] = value;
    data[2] = command ^ value; //TODO Prüfsumme
    HAL_UART_Transmit(&huart2, data, 3, 100);
}

void send_command_to_driver_z(uint8_t command, uint8_t value)
{
    uint8_t data[3];
    data[0] = command;
    data[1] = value;
    data[2] = command ^ value; //TODO Prüfsumme
    HAL_UART_Transmit(&huart8, data, 3, 100);
}
