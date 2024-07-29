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

	C0 = 16,
	Db0 = 17,
	D0 = 18,
	Eb0 = 19,
	E0 = 20,
	F0 = 21,
	Gb0 = 23,
	G0 = 24,
	Ab0 = 25,
	A0 = 27,
	Bb0 = 29,
	B0 = 30,
	C1 = 32,
	Db1 = 34,
	D1 = 36,
	Eb1 = 38,
	E1 = 41,
	F1 = 43,
	Gb1 = 46,
	G1 = 49,
	Ab1 = 51,
	A1 = 55,
	Bb1 = 58,
	B1 = 61,
	C2 = 65,
	Db2 = 69,
	D2 = 73,
	Eb2 = 77,
	E2 = 82,
	F2 = 87,
	Gb2 = 92,
	G2 = 98,
	Ab2 = 103,
	A2 = 110,
	Bb2 = 116,
	B2 = 123,
	C3 = 130,
	Db3 = 138,
	D3 = 146,
	Eb3 = 155,
	E3 = 164,
	F3 = 174,
	Gb3 = 185,
	G3 = 196,
	Ab3 = 207,
	A3 = 220,
	Bb3 = 233,
	B3 = 246,
	C4 = 261,
	Db4 = 277,
	D4 = 293,
	Eb4 = 311,
	E4 = 329,
	F4 = 349,
	Gb4 = 369,
	G4 = 392,
	Ab4 = 415,
	A4 = 440,
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
