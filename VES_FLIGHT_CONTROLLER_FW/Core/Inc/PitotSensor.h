/*
 * pitotSensor.h
 *
 *  Created on: Apr 6, 2024
 *      Author: tommaso
 */

#ifndef PITOTSENSOR_H
#define PITOTSENSOR_H

#include <stdint.h>
#include <math.h>
#include <stm32f4xx.h>

#define PITOT_SENSOR_TIMEOUT_MS 1

#define MAX_DIFF_PRESSURE 7e5
#define UNIT_DIFF_PRESSURE	MAX_DIFF_PRESSURE / 0xFFF // 0XFFF because the resolution of the ADC is 12 bits

//FIXME insert a function to get the estimated air density from the altitude
#define AIR_DENSITY_KG_M3 1.293f

typedef struct {
	uint16_t adc_value;
	uint16_t diff_pressure;
	ADC_HandleTypeDef *hadc;
} pitot_sensor_t;

uint8_t init_pitot_sensor(pitot_sensor_t *dev, ADC_HandleTypeDef *ADC_handler);

uint8_t read_diff_pressure(pitot_sensor_t *dev);

uint16_t compute_velocity(uint16_t diff_pressure);

#endif /* INC_PITOTSENSOR_H_ */
