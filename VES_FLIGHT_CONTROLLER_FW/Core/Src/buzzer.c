/*
 * buzzer.c
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef BUZZER_H
#include "buzzer.h"
#endif

static void setFrequency(uint16_t frequency) {

	uint16_t prescaler = 0;

	prescaler = ((TIMER4_CLK / htim4.Init.Period) / frequency) - 1;

	htim4.Init.Prescaler = prescaler;
}

void buzzerInit(buzzer_t *dev) {

	dev -> TIM_CCR = &(TIM4 -> CCR4);
	dev -> TIM_Instance = TIM4;
	htim4.Init.Period = 200;
}

void setVolume(buzzer_t *dev, uint8_t volume) {

	*(dev -> TIM_CCR) = volume < 100 ? volume : 100;
}

void setNote(char* note) {

	uint16_t freq = 0;

	//lookup to the correct frequency from the note parameter

	setFrequency(freq);
}

void beepBuzzer(uint32_t duration) {

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL);
	osDelay(duration);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL);
}
