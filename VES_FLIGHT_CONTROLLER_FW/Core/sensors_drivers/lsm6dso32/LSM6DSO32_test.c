/*
 * LSM6DSO32_test.c
 *
 *  Created on: Dec 24, 2023
 *      Author: tomma
 */

#ifndef LSM6DSO32_REGS_H

#include "lsm6dso32_reg.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#define BOOT_TIME 10

#define IMU_1_nCS_Pin GPIO_PIN_4
#define IMU_1_nCS_GPIO_Port GPIOC

#define IMU_CS_HIGH HAL_GPIO_WritePin(IMU_1_nCS_GPIO_Port, IMU_1_nCS_Pin, GPIO_PIN_SET);
#define IMU_CS_LOW HAL_GPIO_WritePin(IMU_1_nCS_GPIO_Port, IMU_1_nCS_Pin, GPIO_PIN_RESET);

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);

void lsm6dso32_read_data_polling_mode() {

	stmdev_ctx_t dev_ctx;

	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hspi1;

	platform_delay(BOOT_TIME);

	lsm6dso32_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LSM6DSO32_ID) {
		while (1);
	}

	/* Restore default configuration */
	lsm6dso32_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lsm6dso32_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface */
	lsm6dso32_i3c_disable_set(&dev_ctx, LSM6DSO32_I3C_DISABLE);
	/* Enable Block Data Update */
	lsm6dso32_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set full scale */
	lsm6dso32_xl_full_scale_set(&dev_ctx, LSM6DSO32_4g);
	lsm6dso32_gy_full_scale_set(&dev_ctx, LSM6DSO32_2000dps);
	/* Set ODR (Output Data Rate) and power mode*/
	lsm6dso32_xl_data_rate_set(&dev_ctx, LSM6DSO32_XL_ODR_12Hz5_LOW_PW);
	lsm6dso32_gy_data_rate_set(&dev_ctx, LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF);

	/* Read samples in polling mode (no int) */
	while (1) {
		lsm6dso32_reg_t reg;
		/* Read output only if new data is available */
		lsm6dso32_status_reg_get(&dev_ctx, &reg.status_reg);

		if (reg.status_reg.xlda) {
			/* Read acceleration data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm6dso32_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
			acceleration_mg[0] = lsm6dso32_from_fs4_to_mg(data_raw_acceleration[0]);
			acceleration_mg[1] = lsm6dso32_from_fs4_to_mg(data_raw_acceleration[1]);
			acceleration_mg[2] = lsm6dso32_from_fs4_to_mg(data_raw_acceleration[2]);
			sprintf((char *)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
			tx_com(tx_buffer, strlen((char const *)tx_buffer));
		}

		if (reg.status_reg.gda) {
			/* Read angular rate field data */
			memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
			lsm6dso32_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
			angular_rate_mdps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[0]);
			angular_rate_mdps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[1]);
			angular_rate_mdps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[2]);
			sprintf((char *)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
			tx_com(tx_buffer, strlen((char const *)tx_buffer));
		}

		if (reg.status_reg.tda) {
			/* Read temperature data */
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lsm6dso32_temperature_raw_get(&dev_ctx, &data_raw_temperature);
			temperature_degC = lsm6dso32_from_lsb_to_celsius(data_raw_temperature);
			sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC);
			tx_com(tx_buffer, strlen((char const *)tx_buffer));
		}
	}
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {

	IMU_CS_LOW;
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) bufp, len, 1000);
	IMU_CS_HIGH;

	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {

	reg |= 0x80;
	IMU_CS_LOW;
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Receive(&hspi1, bufp, len, 1000);
	IMU_CS_HIGH;

  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);
}

#ifdef __cplusplus
}
#endif

#endif
