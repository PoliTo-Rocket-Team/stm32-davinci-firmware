/*
 * buzzer.h
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

typedef enum {
	A4 = 400,
	Bb4 = 466,
	B4 = 494,
	C5 = 523,
	Db5 = 554,
	D5 = 587,
	Eb5 = 622,
	E5 = 659,
	F5 = 699,
	Gb5 = 740,
	G5 = 784,
	Ab5 = 831,
	A5 = 880,
	Bb5 = 932,
	B5 = 988,
	C6 = 1047
} note_t;

extern TIM_HandleTypeDef htim4;
#define TIMER4_CLK	96000000
#define TIM_CHANNEL TIM_CHANNEL_4

typedef struct {

	volatile uint32_t *TIM_CCR;
	TIM_TypeDef *TIM_Instance;

} buzzer_t;

void buzzerInit(buzzer_t *dev);

void setVolume(buzzer_t *dev, uint8_t volume);

void setNote(note_t note);

void beepBuzzer(uint32_t duration);

#endif /* INC_BUZZER_H_ */
