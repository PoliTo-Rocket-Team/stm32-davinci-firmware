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

	case C0:
		freq = 16;
		break;
	case Db0:
		freq = 17;
		break;
	case D0:
		freq =  18;
		break;
	case Eb0:
		freq =  19;
		break;
	case E0:
		freq =  20;
		break;
	case F0:
		freq =  21;
		break;
	case Gb0:
		freq =  23;
		break;
	case G0:
		freq = 24;
		break;
	case Ab0:
		freq = 25;
		break;
	case A0:
		freq = 27;
		break;
	case Bb0:
		freq = 29;
		break;
	case B0:
		freq = 30;
		break;
	case C1:
		freq = 32;
		break;
	case Db1:
		freq = 34;
		break;
	case D1:
		freq = 36;
		break;
	case Eb1:
		freq = 38;
		break;
	case E1:
		freq = 41;
		break;
	case F1:
		freq = 43;
		break;
	case Gb1:
		freq = 46;
		break;
	case G1:
		freq = 49;
		break;
	case Ab1:
		freq = 51;
		break;
	case A1:
		freq = 55;
		break;
	case Bb1:
		freq = 58;
		break;
	case B1:
		freq = 61;
		break;
	case C2:
		freq = 65;
		break;
	case Db2:
		freq = 69;
		break;
	case D2:
		freq = 73;
		break;
	case Eb2:
		freq = 77;
		break;
	case E2:
		freq = 82;
		break;
	case F2:
		freq = 87;
		break;
	case Gb2:
		freq = 92;
		break;
	case G2:
		freq = 98;
		break;
	case Ab2:
		freq = 103;
		break;
	case A2:
		freq = 110;
		break;
	case Bb2:
		freq = 116;
		break;
	case B2:
		freq = 123;
		break;
	case C3:
		freq = 130;
		break;
	case Db3:
		freq = 138;
		break;
	case D3:
		freq = 146;
		break;
	case Eb3:
		freq = 155;
		break;
	case E3:
		freq = 164;
		break;
	case F3:
		freq = 174;
		break;
	case Gb3:
		freq = 185;
		break;
	case G3:
		freq = 196;
		break;
	case Ab3:
		freq = 207;
		break;
	case A3:
		freq = 220;
		break;
	case Bb3:
		freq = 233;
		break;
	case B3:
		freq = 246;
		break;
	case C4:
		freq = 261;
		break;
	case Db4:
		freq = 277;
		break;
	case D4:
		freq = 293;
		break;
	case Eb4:
		freq = 311;
		break;
	case E4:
		freq = 329;
		break;
	case F4:
		freq = 349;
		break;
	case Gb4:
		freq = 369;
		break;
	case G4:
		freq = 392;
		break;
	case Ab4:
		freq = 415;
		break;
	case A4:
		freq = 440;
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
