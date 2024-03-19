/*
 * utils.c
 *
 *  Created on: Dec 31, 2023
 *      Author: tomma
 */

#include "utils.h"

#ifdef DEBUG_EN

extern UART_HandleTypeDef huart2;

#include <string.h>

HAL_StatusTypeDef COM_port_serial_print(const uint8_t* data) {
	return HAL_UART_Transmit(&huart2, data, strlen((const char *)data) + 1, 100);
}
#endif
