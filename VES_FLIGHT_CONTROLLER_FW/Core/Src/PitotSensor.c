/*
 * PitotSensor.c
 *
 *  Created on: Apr 6, 2024
 *      Author: tommaso
 */

#ifndef PITOTSENSOR_H
#include "PitotSensor.h"
#endif

uint8_t init_pitot_sensor(pitot_sensor_t *dev, ADC_HandleTypeDef *ADC_handler) {

	dev -> hadc = ADC_handler;
}

uint8_t read_diff_pressure(pitot_sensor_t *dev) {

	uint8_t result = HAL_ERROR;

	HAL_ADC_Start(dev -> hadc);

	if (HAL_ADC_PollForConversion(dev -> hadc, PITOT_SENSOR_TIMEOUT_MS) == HAL_OK) {

		result = HAL_OK;

		dev -> adc_value = HAL_ADC_GetValue(dev -> hadc);

		dev -> diff_pressure = (MAX_DIFF_PRESSURE / 0xFFF) * dev -> adc_value;

	}

	return result;
}

//FIXME put the function in the correct file and place
uint16_t compute_velocity(uint16_t diff_pressure) {

	uint16_t velocity;
	//FIXME uint16_t air_density = get_air_density();

	//FIXME maybe insert a function to estimate air density wrt the altitude

	velocity = sqrt(2 * diff_pressure / AIR_DENSITY_KG_M3);

	return velocity;
}

static float get_air_density();
