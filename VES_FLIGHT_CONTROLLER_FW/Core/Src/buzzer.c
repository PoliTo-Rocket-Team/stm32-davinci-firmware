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

void setNote(note_t note) {

	uint16_t freq = 0;

	switch(note) {
	case A4:
		freq = 400;
		break;
	case Bb4:
		freq = 466;
		break;
	case B4:
		freq = 494;
		break;
	case C5:
		freq = 523;
		break;
	case Db5:
		freq = 554;
		break;
	case D5:
		freq = 587;
		break;
	case Eb5:
		freq = 622;
		break;
	case E5:
		freq = 659;
		break;
	case F5:
		freq = 699;
		break;
	case Gb5:
		freq = 740;
		break;
	case G5:
		freq = 784;
		break;
	case Ab5:
		freq = 831;
		break;
	case A5:
		freq = 880;
		break;
	case Bb5:
		freq = 932;
		break;
	case B5:
		freq = 988;
		break;
	case C6:
		freq = 1047;
		break;
	default:
		freq = 523;
		break;
	}

	setFrequency(freq);
}

void beepBuzzer(uint32_t duration) {

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL);
	delay(duration);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL);
}
