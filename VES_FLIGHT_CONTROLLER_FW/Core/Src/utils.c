/*
 * utils.c
 *
 *  Created on: Dec 31, 2023
 *      Author: tomma
 */

#include "utils.h"
#include <math.h>

#ifdef DEBUG_EN

extern UART_HandleTypeDef huart2;

#include <string.h>

HAL_StatusTypeDef COM_port_serial_print(const uint8_t* data) {
	return HAL_UART_Transmit(&huart2, data, strlen((const char *)data) + 1, 100);
}
#endif

float computeAltitude(double pressurePa, double temperature) {

	//FIXME temperature will become a constant defined in flight parameters

	double pressurehPa = pressurePa / 100;	// pressure from Pascal to Hectopascal

	double temp1 = (273.15 + temperature) / 0.0065;

	double elevation = temp1 * (pow(/*SEA_LEVEL_PRESSURE_HPA*/ 1013.25 / pressurehPa, 0.19022256039) - 1);

	return (float) elevation;
}
