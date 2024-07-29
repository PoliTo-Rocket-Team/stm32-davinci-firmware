/*
 * buzzer.c
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef BUZZER_H
#include "buzzer.h"
#endif

static void setFrequency(buzzer_t *dev, uint16_t frequency) {

	*(dev -> TIM_ARR) = (1000000UL / frequency) - 1;
	/* the value 10^6 depends on TIMER4_CLK / 96, it is done on purpose to have a value of 1 usecond */

}

static void setVolume(buzzer_t *dev, uint8_t volume) {

	/* setting TIM_CCR to ARR / 2 we achieve a 50% duty cycle which is the max volume */
	uint8_t vol = volume < 100 ? volume : 100;

	/* if volume is 100 then we divide by 2 and therefore we achieve 50% duty cycle i.e. max volume
	 * if volume is 50 then we divide by 4 and therefore we achieve 25% duty cycle i.e. half volume
	 */
	*(dev -> TIM_CCR) = *(dev -> TIM_ARR) * vol / 200;
}

static void setNote(buzzer_t *dev, note_t note) {

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

	setFrequency(dev, freq);
}

void buzzerInit(buzzer_t *dev) {

	dev -> TIM_CCR = &(TIM4 -> CCR4);
	dev -> TIM_ARR = &(TIM4 -> ARR);
	dev -> TIM_Instance = TIM4;

	uint16_t prescaler = TIMER4_CLK / 1000000UL;

	htim4.Init.Prescaler = prescaler - 1;
}

void beepBuzzer(buzzer_t *dev, uint32_t duration, uint8_t volume, note_t note) {

	setNote(dev, note);
	/* setVolume is based on the value set by setNote so must be called always after setNote */
	setVolume(dev, volume);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL);
	delay(duration);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL);
}
