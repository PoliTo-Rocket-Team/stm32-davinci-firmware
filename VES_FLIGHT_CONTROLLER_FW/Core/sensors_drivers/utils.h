/*
 * utils.h
 *
 *  Created on: Dec 31, 2023
 *      Author: tomma
 */

#ifndef SENSORS_DRIVERS_UTILS_H_
#define SENSORS_DRIVERS_UTILS_H_

#define DEBUG_EN

#ifdef DEBUG_EN

#include "stm32f4xx.h"

HAL_StatusTypeDef COM_port_serial_print(const uint8_t* data);
#endif

#endif /* SENSORS_DRIVERS_UTILS_H_ */
