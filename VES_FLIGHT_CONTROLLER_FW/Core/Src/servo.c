/*
 * servo.c
 *
 *  Created on: Mar 9, 2024
 *      Author: tomma
 */

#ifndef SERVO_H
#include "servo.h"
#endif

void servo_init(servo_t *servo) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_HandleTypeDef htim;

	uint32_t PSC = 0;
	uint32_t ARR = 0;

	servo -> minpos_period_ms = SERVO_MIN_MS;
	servo -> maxpos_period_ms = SERVO_MAX_MS;

	servo -> TIM_Instance = TIM3;
	servo -> TIM_CCR = &(TIM3 -> CCR1);

	PSC = (uint32_t) (TIM_CLK / (PWM_FREQUENCY * 65536));
	ARR = (uint32_t) ((TIM_CLK / (PWM_FREQUENCY * (PSC + 1))) - 1);

	__HAL_RCC_TIM3_CLK_ENABLE();
	htim.Instance = TIM_INSTANCE;
	htim.Init.Prescaler = PSC;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = ARR;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//	HAL_TIM_Base_Init(&htim);
//	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, PWM_TIM_CH);

	servo -> minpos_pulse = (uint16_t) (ARR * (servo -> minpos_period_ms / PWM_PERIOD));
	servo -> maxpos_pulse = (uint16_t) (ARR * (servo -> maxpos_period_ms / PWM_PERIOD));

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = SERVO_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStruct);

	HAL_TIM_PWM_Start(&htim, PWM_TIM_CH);

}

void servo_moveto_deg(servo_t *servo, float angle) {

	uint16_t pulse = 0;

	pulse = ((angle * (servo -> maxpos_pulse - servo -> minpos_pulse)) / SERVO_DEGREES) + servo -> minpos_pulse;

	*(servo -> TIM_CCR) = pulse;
}

void servo_rawmove(servo_t *servo, uint16_t pulse) {

	if (pulse <= servo -> maxpos_pulse && pulse >= servo -> minpos_pulse) {
		*(servo -> TIM_CCR) = pulse;
	}
}

uint16_t servo_get_maxpulse(servo_t *servo) {

	return servo -> maxpos_pulse;
}

uint16_t servo_get_minpulse(servo_t *servo) {

	return servo -> minpos_pulse;
}
