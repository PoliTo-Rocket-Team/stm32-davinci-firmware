/*
 * buzzer.h
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

#define A4 	400
#define Bb4	466
#define B4	494
#define C5	523
#define Db5	554
#define D5	587
#define Eb5	622
#define E5	659
#define F5	699
#define Gb5	740
#define G5	784
#define Ab5	831
#define A5	880
#define Bb5	932
#define B5	988
#define C6	1047

extern TIM_HandleTypeDef htim4;
#define TIMER4_CLK	96000000
#define TIM_CHANNEL TIM_CHANNEL_4

typedef struct {

	volatile uint32_t *TIM_CCR;
	TIM_TypeDef *TIM_Instance;

} buzzer_t;

void buzzerInit(buzzer_t *dev);

void setVolume(buzzer_t *dev, uint8_t volume);

void setNote(char* note);

void beepBuzzer(uint32_t duration);

#endif /* INC_BUZZER_H_ */
