/*
 * buzzer.h
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

#define BUZZER_CMSIS_RTOS BUZZER_CMSIS_RTOS_DISABLE

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

#if BUZZER_CMSIS_RTOS == BUZZER_CMSIS_RTOS_DISABLE
#define delay(x) HAL_Delay(x)
#else
#include "cmsis_os2.h"
#define delay(x) osDelay(x)
#endif

extern TIM_HandleTypeDef htim4;
#define TIMER4_CLK	96000000
#define TIM_CHANNEL TIM_CHANNEL_4

typedef struct {

	volatile uint32_t *TIM_CCR;
	volatile uint32_t *TIM_ARR;
	TIM_TypeDef *TIM_Instance;

} buzzer_t;

void buzzerInit(buzzer_t *dev);

void beepBuzzer(buzzer_t *dev, uint32_t duration, uint8_t volume, note_t note);

#endif /* INC_BUZZER_H_ */
