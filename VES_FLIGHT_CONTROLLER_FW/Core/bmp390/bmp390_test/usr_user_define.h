#ifndef USER_DEFINE_H__
#define USER_DEFINE_H__

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "bmp3.h"
#include "stm32f4xx_hal.h"
#include "bmp3_defs.h"
#include "main.h"

#define DEBUG_EN

#define USE_BOSCH_SENSOR_API

#define USE_SPI_INTERFACE

#define USE_BMP390

#define SEA_LEVEL_PRESSURE_HPA (1013.25)

#define CS_BMP390_HIGH			(HAL_GPIO_WritePin(BARO_1_nCS_GPIO_Port, BARO_1_nCS_Pin, GPIO_PIN_SET))
#define CS_BMP390_LOW			(HAL_GPIO_WritePin(BARO_1_nCS_GPIO_Port, BARO_1_nCS_Pin, GPIO_PIN_RESET))


#if defined(USE_BMP390)

//#define READ_SENSOR_DATA


#endif

#endif
