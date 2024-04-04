/*
 * servo.h
 *
 *  Created on: Mar 9, 2024
 *      Author: tomma
 */

#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;

#define DEBUG_MODE 1

#define SERVO_GPIO_PORT GPIOB
#define SERVO_GPIO_PIN 	4
#define TIM_CLK			48000000
#define PWM_TIM_CH		TIM_CHANNEL_1
#define TIM_INSTANCE 	TIM3
#define PWM_FREQUENCY	50
#define PWM_PERIOD		20.0f
#if (DEBUG_MODE == 1)
#define SERVO_MIN_MS	0.8f
#define SERVO_MAX_MS	2.2f
#define SERVO_DEGREES	180.0f
#else
#define SERVO_MIN_MS	0.5f
#define SERVO_MAX_MS	2.5f
#define SERVO_DEGREES	180.0f
#endif

typedef struct {

	float minpos_period_ms;
	float maxpos_period_ms;
	volatile uint32_t *TIM_CCR;
	TIM_TypeDef *TIM_Instance;
	uint16_t maxpos_pulse;
	uint16_t minpos_pulse;
} servo_t;

void servo_init(servo_t *servo);
void servo_moveto_deg(servo_t *servo, float angle);
void servo_rawmove(servo_t *servo, uint16_t pulse);
uint16_t servo_get_maxpulse(servo_t *servo);
uint16_t servo_get_minpulse(servo_t *servo);

#endif /* SERVO_H_ */
