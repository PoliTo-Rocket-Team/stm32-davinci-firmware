/*
 * utils.c
 *
 *  Created on: Dec 31, 2023
 *      Author: tomma
 */

#ifndef UTILITIES_H_
#include "utilities.h"
#endif

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

#ifdef DEBUG_EN

extern UART_HandleTypeDef huart2;

#include <string.h>

HAL_StatusTypeDef COM_port_serial_print(const uint8_t* data) {
	return HAL_UART_Transmit(&huart2, data, strlen((const char *)data) + 1, 100);
}
#endif

static uint8_t bmp390_1_addr = 0;
static uint8_t bmp390_2_addr = 0;
osEventFlagsId_t fsm_flag_id;
osMessageQueueId_t event_queue;

float computeAltitude(double pressurePa, double temperature) {

	//FIXME temperature will become a constant defined in flight parameters

	double pressurehPa = pressurePa / 100;	// pressure from Pascal to Hectopascal

	double temp1 = (273.15 + temperature) / 0.0065;

	double elevation = temp1 * (pow(/*SEA_LEVEL_PRESSURE_HPA*/ 1013.25 / pressurehPa, 0.19022256039) - 1);

	return (float) elevation;
}

int32_t imu1_write(void *handle, uint8_t reg_addr, const uint8_t *buf, uint16_t len) {

	int32_t result = HAL_ERROR;

    IMU1_CS_LOW;

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	IMU1_CS_HIGH;

    return result;
}

int32_t imu1_read(void *handle, uint8_t reg_addr, uint8_t *buf, uint16_t len) {

	int32_t result = HAL_ERROR;

	reg_addr |= 0x80;

	IMU1_CS_LOW;

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Receive(&hspi1, buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	IMU1_CS_HIGH;

    return result;
}

int8_t bmp390_1_write(uint8_t reg_addr, const uint8_t *buf, uint32_t len, void *intf_ptr) {

	int8_t result = HAL_ERROR;

	BMP390_1_CS_LOW;

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	BMP390_1_CS_HIGH;

    return result;
}

int8_t bmp390_1_read(uint8_t reg_addr, uint8_t *buf, uint32_t len, void *intf_ptr) {

	int8_t result = HAL_ERROR;

	BMP390_1_CS_LOW;

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Receive(&hspi1, buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	BMP390_1_CS_HIGH;

    return result;
}

int32_t imu2_write(void *handle, uint8_t reg_addr, const uint8_t *buf, uint16_t len) {

	int32_t result = HAL_ERROR;

    IMU2_CS_LOW;

	if (HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	IMU2_CS_HIGH;

    return result;
}

int32_t imu2_read(void *handle, uint8_t reg_addr, uint8_t *buf, uint16_t len) {

	int32_t result = HAL_ERROR;

	reg_addr |= 0x80;

	IMU2_CS_LOW;

	if (HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Receive(&hspi2, buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	IMU2_CS_HIGH;

    return result;
}

int8_t bmp390_2_write(uint8_t reg_addr, const uint8_t *buf, uint32_t len, void *intf_ptr) {

	int8_t result = HAL_ERROR;

	BMP390_2_CS_LOW;

	if (HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	BMP390_2_CS_HIGH;

    return result;
}

int8_t bmp390_2_read(uint8_t reg_addr, uint8_t *buf, uint32_t len, void *intf_ptr) {

	int8_t result = HAL_ERROR;

	BMP390_2_CS_LOW;

	if (HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 100) == HAL_OK) {
		if (HAL_SPI_Receive(&hspi2, buf, len, 100) == HAL_OK) {
			result = HAL_OK;
		}
	}

	BMP390_2_CS_HIGH;

    return result;
}

static void bmp390_delay_us(uint32_t period, void *intf_ptr) {
	uint32_t i;

	while (period--) {
		for (i = 0; i < 96; i++) {
			;
		}
	}
}

static int8_t bmp3_1_spi_init(struct bmp3_dev *dev) {
	int8_t result = BMP3_OK;

	if (dev != NULL) {
		bmp390_1_addr = 0;
		dev -> read = bmp390_1_read;
		dev -> write = bmp390_1_write;
		dev -> intf = BMP3_SPI_INTF;

		dev -> delay_us = bmp390_delay_us;
		dev -> intf_ptr = &bmp390_1_addr;
	} else {
		result = -1;
	}

	return result;
}

static int8_t bmp3_2_spi_init(struct bmp3_dev *dev) {
	int8_t result = BMP3_OK;

	if (dev != NULL) {
		bmp390_2_addr = 0;
		dev -> read = bmp390_2_read;
		dev -> write = bmp390_2_write;
		dev -> intf = BMP3_SPI_INTF;

		dev -> delay_us = bmp390_delay_us;
		dev -> intf_ptr = &bmp390_2_addr;
	} else {
		result = -1;
	}

	return result;
}

int8_t init_imu1(stmdev_ctx_t *imu, lsm6dso32_fs_xl_t acc_full_scale, lsm6dso32_fs_g_t gyro_full_scale, lsm6dso32_odr_xl_t acc_output_data_rate, lsm6dso32_odr_g_t gyro_output_data_rate) {

//	int8_t result = HAL_ERROR;

	uint8_t who_am_i = 0;
	uint8_t rst = 1;
	uint8_t result = 1;

	BMP390_1_CS_HIGH;

	imu -> write_reg = imu1_write;
	imu -> read_reg = imu1_read;
	imu -> handle = &hspi1;

	HAL_Delay(10);	//FIXME here HAL_Delay used because this init is done during startup, so before kernel takes control of execution

	lsm6dso32_device_id_get(imu, &who_am_i);

	if (who_am_i != LSM6DSO32_ID) {
		return HAL_ERROR;
	}

	/* Restore default configuration */
	result = lsm6dso32_reset_set(imu, PROPERTY_ENABLE);

	/* Wait until the imu does not exit reset status */
	do {
		lsm6dso32_reset_get(imu, &rst);
	} while (rst);

	if (result != 0) {
		return HAL_ERROR;
	}

	/* Disable I3C interface */
	result = lsm6dso32_i3c_disable_set(imu, LSM6DSO32_I3C_DISABLE);

	if (result != 0) {
		return HAL_ERROR;
	}

	/* Enable Block Data Update, this means that registers
	 * are not update until all the data have been read */
	result = lsm6dso32_block_data_update_set(imu, PROPERTY_ENABLE);

	if (result != 0) {
		return HAL_ERROR;
	}

	/* Set accelerometer and gyroscope full scale */
	lsm6dso32_xl_full_scale_set(imu, acc_full_scale);
	lsm6dso32_gy_full_scale_set(imu, gyro_full_scale);

	/* Set accelerometer and gyroscope output data rate ODR */
	lsm6dso32_xl_data_rate_set(imu, acc_output_data_rate);
	lsm6dso32_gy_data_rate_set(imu, gyro_output_data_rate);

	return HAL_OK;
}

int8_t init_bmp390_1(struct bmp3_dev *bmp390) {

	int8_t result = HAL_ERROR;
	uint8_t settings_sel = 0;
	struct bmp3_settings settings = { 0 };

	IMU1_CS_HIGH;

	result = bmp3_1_spi_init(bmp390);

	if (result != BMP3_OK)
		return HAL_ERROR;

	result = bmp3_init(bmp390);

	BMP390_1_CS_HIGH;

	if (result != BMP3_OK)
		return HAL_ERROR;

	/* enabling both pressure and temperature reading */
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	/* interrupt disabling */
	settings.int_settings.drdy_en = BMP3_DISABLE;

	/* pressure and temperature oversampling */
	settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
	settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;

	/* output data rate */
	settings.odr_filter.odr = BMP3_ODR_200_HZ;

	/* IIR filter disabling */
	settings.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;

	/* specifying all the settings we want to modify using the bmp3_settings struct */
	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
				   BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
				   BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;

	/* setting the previously specified settings */
	result = bmp3_set_sensor_settings(settings_sel, &settings, bmp390);

	if (result != BMP3_OK)
		return HAL_ERROR;

	/* setting the operating mode of the sensor */
	settings.op_mode = BMP3_MODE_NORMAL;

	result = bmp3_set_op_mode(&settings, bmp390);

	if (result != BMP3_OK)
		return HAL_ERROR;

	return HAL_OK;
}

int8_t init_imu2(stmdev_ctx_t *imu, lsm6dso32_fs_xl_t acc_full_scale, lsm6dso32_fs_g_t gyro_full_scale, lsm6dso32_odr_xl_t acc_output_data_rate, lsm6dso32_odr_g_t gyro_output_data_rate) {

//	int8_t result = HAL_ERROR;

	uint8_t who_am_i = 0;
	uint8_t rst = 1;
	uint8_t result = 1;

	BMP390_2_CS_HIGH;

	imu -> write_reg = imu2_write;
	imu -> read_reg = imu2_read;
	imu -> handle = &hspi2;

	HAL_Delay(10);	//FIXME here HAL_Delay used because this init is done during startup, so before kernel takes control of execution

	lsm6dso32_device_id_get(imu, &who_am_i);

	if (who_am_i != LSM6DSO32_ID) {
		return HAL_ERROR;
	}

	/* Restore default configuration */
	result = lsm6dso32_reset_set(imu, PROPERTY_ENABLE);

	/* Wait until the imu does not exit reset status */
	do {
		lsm6dso32_reset_get(imu, &rst);
	} while (rst);

	if (result != 0) {
		return HAL_ERROR;
	}

	/* Disable I3C interface */
	result = lsm6dso32_i3c_disable_set(imu, LSM6DSO32_I3C_DISABLE);

	if (result != 0) {
		return HAL_ERROR;
	}

	/* Enable Block Data Update, this means that registers
	 * are not update until all the data have been read */
	result = lsm6dso32_block_data_update_set(imu, PROPERTY_ENABLE);

	if (result != 0) {
		return HAL_ERROR;
	}

	/* Set accelerometer and gyroscope full scale */
	lsm6dso32_xl_full_scale_set(imu, acc_full_scale);
	lsm6dso32_gy_full_scale_set(imu, gyro_full_scale);

	/* Set accelerometer and gyroscope output data rate ODR */
	lsm6dso32_xl_data_rate_set(imu, acc_output_data_rate);
	lsm6dso32_gy_data_rate_set(imu, gyro_output_data_rate);

	return HAL_OK;
}

int8_t init_bmp390_2(struct bmp3_dev *bmp390) {

	int8_t result = HAL_ERROR;
	uint8_t settings_sel = 0;
	struct bmp3_settings settings = { 0 };

	IMU2_CS_HIGH;

	result = bmp3_2_spi_init(bmp390);

	if (result != BMP3_OK)
		return HAL_ERROR;

	result = bmp3_init(bmp390);

	BMP390_2_CS_HIGH;

	if (result != BMP3_OK)
		return HAL_ERROR;

	/* enabling both pressure and temperature reading */
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	/* interrupt disabling */
	settings.int_settings.drdy_en = BMP3_DISABLE;

	/* pressure and temperature oversampling */
	settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
	settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;

	/* output data rate */
	settings.odr_filter.odr = BMP3_ODR_200_HZ;

	/* IIR filter disabling */
	settings.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;

	/* specifying all the settings we want to modify using the bmp3_settings struct */
	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
				   BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
				   BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;

	/* setting the previously specified settings */
	result = bmp3_set_sensor_settings(settings_sel, &settings, bmp390);

	if (result != BMP3_OK)
		return HAL_ERROR;

	/* setting the operating mode of the sensor */
	settings.op_mode = BMP3_MODE_NORMAL;

	result = bmp3_set_op_mode(&settings, bmp390);

	if (result != BMP3_OK)
		return HAL_ERROR;

	return HAL_OK;
}



int8_t init_flash(W25Q128_t *flash, chip_erasing erase) {

	int8_t result = HAL_ERROR;

	result = W25Q128_init(flash, FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, &hspi3);

	if (result != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_Delay(10);

	result = W25Q128_read_manufacturer_dev_id(flash);

	if (result != HAL_OK) {
		return HAL_ERROR;
	}

	result = W25Q128_read_JEDEC_id(flash);

	if (result != HAL_OK) {
		return HAL_ERROR;
	}

	/* completely erase the nor flash chip, otherwise it will never work */
	if (erase == ERASE) {
		W25Q128_chip_erase(flash);

		uint8_t reg1 = 0x01;

		/* waiting until the flash chip is not fully erased */
		do {
			result = W25Q128_read_statusreg1(flash, &reg1);
		} while ((reg1 & 1) && result == HAL_OK);
	}

	return HAL_OK;
}

uint16_t compute_air_density(float_t temperature,float_t pressure){
	uint16_t air_d;

	air_d = (uint16_t) pressure/(temperature*SPECIFIC_GAS_CONSTANT);
	return air_d;
}

//FIXME put the function in the correct file and place
//int8_t calibrateBMP390(struct bmp3_dev *bmp390, uint16_t iterationNum) {
//
//	struct bmp3_data temp = { -1, -1 };
//	int8_t result = BMP3_E_NULL_PTR;
//
//	result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &temp, bmp390);
//
//	if (result != BMP3_OK) {
//		return result;
//	}
//
//	double mean_pressure = temp.pressure;
//	double mean_temperature = temp.temperature;
//
//	for (int i = 0; i < iterationNum; i++) {
//			struct bmp3_data data = { -1, -1 };
//			struct bmp3_status status = { { 0 } };
//
//			result = bmp3_get_status(&status, bmp390);
//
//			if (result != BMP3_OK)	return result;
//
//			result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, bmp390);
//
//			mean_pressure += data.pressure;
//			mean_pressure /= 2;
//
//			mean_temperature += data.temperature;
//			mean_temperature /= 2;
//		}
//
//	// add the code to add a possible software offset, if needed
//
//	return 0;
//}


//FIXME remember that when on the rocket the gravity acceleration will be along the X axis

int8_t calibrateIMU(stmdev_ctx_t *imu, uint16_t iterationNum, OFFSET_TYPE type) {

	float_t mean_acceleration_mg[3] = {0};
	float_t mean_angular_rate_mdps[3] = {0};
	float_t mean_temperature_degC = 0.0;
	float_t acceleration_mg[3] = {0};
	float_t angular_rate_mdps[3] = {0};
	float_t temperature_degC = 0.0;
	int16_t data_raw_acceleration[3] = {0};
	int16_t data_raw_angular_rate[3] = {0};
	int16_t data_raw_temperature = 0.0;

	lsm6dso32_acceleration_raw_get(imu, data_raw_acceleration);
	lsm6dso32_angular_rate_raw_get(imu, data_raw_angular_rate);
	lsm6dso32_temperature_raw_get(imu, &data_raw_temperature);

	acceleration_mg[0] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[0]);
	acceleration_mg[1] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[1]);
	acceleration_mg[2] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[2]);

	angular_rate_mdps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[0]);
	angular_rate_mdps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[1]);
	angular_rate_mdps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[2]);

	temperature_degC = lsm6dso32_from_lsb_to_celsius(data_raw_temperature);

	mean_acceleration_mg[0] += acceleration_mg[0];
	mean_acceleration_mg[1] += acceleration_mg[1];
	mean_acceleration_mg[2] += acceleration_mg[2];

	mean_angular_rate_mdps[0] += angular_rate_mdps[0];
	mean_angular_rate_mdps[1] += angular_rate_mdps[1];
	mean_angular_rate_mdps[2] += angular_rate_mdps[2];

	mean_temperature_degC += temperature_degC;

	for (int i = 0; i < iterationNum; i++) {
		lsm6dso32_reg_t reg;
		/* Read output only if new data is available */
		lsm6dso32_status_reg_get(imu, &reg.status_reg);

		if (reg.status_reg.xlda) {
			/* Read acceleration data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm6dso32_acceleration_raw_get(imu, data_raw_acceleration);
			acceleration_mg[0] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[0]);
			acceleration_mg[1] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[1]);
			acceleration_mg[2] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[2]);
			mean_acceleration_mg[0] += acceleration_mg[0];
			mean_acceleration_mg[0] /= 2;
			mean_acceleration_mg[1] += acceleration_mg[1];
			mean_acceleration_mg[1] /= 2;
			mean_acceleration_mg[2] += acceleration_mg[2];
			mean_acceleration_mg[2] /= 2;
		}

		if (reg.status_reg.gda) {
			/* Read angular rate field data */
			memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
			lsm6dso32_angular_rate_raw_get(imu, data_raw_angular_rate);
			angular_rate_mdps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[0]);
			angular_rate_mdps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[1]);
			angular_rate_mdps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[2]);
			mean_angular_rate_mdps[0] += angular_rate_mdps[0];
			mean_angular_rate_mdps[0] /= 2;
			mean_angular_rate_mdps[1] += angular_rate_mdps[1];
			mean_angular_rate_mdps[1] /= 2;
			mean_angular_rate_mdps[2] += angular_rate_mdps[2];
			mean_angular_rate_mdps[2] /= 2;
		}

		if (reg.status_reg.tda) {
			/* Read temperature data */
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lsm6dso32_temperature_raw_get(imu, &data_raw_temperature);
			temperature_degC = lsm6dso32_from_lsb_to_celsius(data_raw_temperature);
			mean_temperature_degC += temperature_degC;
			mean_temperature_degC /= 2;
		}
	}

	switch(type) {
//			case SWOFFSET:
//				/* FIXME handle a possible sw offset type, store offset values somewhere and use
//				 * them to calibrate measurements. */
//
//				acceleration_mg_swoffset[0] = mean_acceleration_mg[0];
//				acceleration_mg_swoffset[1] = mean_acceleration_mg[1];
//				acceleration_mg_swoffset[2] = mean_acceleration_mg[2];
//
//				angular_rate_mdps_swoffset[0] = mean_angular_rate_mdps[0];
//				angular_rate_mdps_swoffset[1] = mean_angular_rate_mdps[1];
//				angular_rate_mdps_swoffset[2] = mean_angular_rate_mdps[2];
//
//				break;
			case HWOFFSET:

				// add code to set a possible hardware offset in the IMU, if needed

				/*
					LSM6DSO32_X_OFS_USR
					LSM6DSO32_Y_OFS_USR
					LSM6DSO32_Z_OFS_USR

					are the registers used to set an hardware offset to accelerometer measurements

					To enable offset USR_OFF_ON_OUT bit of the CTRL7_G register must be set.

					The value of the offset is expressed on 8 bits 2's complement.

					The weight [g/LSB] to be applied to the offset register values is independent
					 of the accelerometer selected full scale
					 and can be configured using the USR_OFF_W bit of the CTRL6_C register:
					• 2^-10 g/LSB if the USR_OFF_W bit is set to 0
					• 2^-6  g/LSB if the USR_OFF_W bit is set to 1
				*/

				/*
				 * 1. offset is computed using the finest weight
				 * 2. we check whether the offset can be represented on 8 bits 2's complement
				 * 3.1. If yes, the weight is set
				 * 3.2. Otherwise, the coarser weight is used and the offsets are recomputed
				 * 4. the value of the offset is stored in imu's registers
				 */
				int16_t offsetValX = (int16_t)mean_acceleration_mg[0] >> 1;
				int16_t offsetValY = (int16_t)mean_acceleration_mg[1] >> 1;
				int16_t offsetValZ = -((1000 - (int16_t)mean_acceleration_mg[2]) >> 1);

				if (offsetValX > -128 && offsetValX < 128 && offsetValY > -128 && offsetValY < 128
						&& offsetValZ > -128 && offsetValZ < 128) {

					lsm6dso32_xl_offset_weight_set(imu, LSM6DSO32_LSb_1mg);
				} else {
					/* the right shift by 4 operation is done to perform the division by 16 */
					lsm6dso32_xl_offset_weight_set(imu, LSM6DSO32_LSb_16mg);
					offsetValX >>= 4;
					offsetValY >>= 4;
					offsetValZ >>= 4;
				}

				//		int32_t lsm6dso32_xl_offset_weight_get(stmdev_ctx_t *ctx, lsm6dso32_usr_off_w_t *val);
				//		int32_t lsm6dso32_xl_usr_offset_x_get(stmdev_ctx_t *ctx, uint8_t *buff);
				//		int32_t lsm6dso32_xl_usr_offset_y_get(stmdev_ctx_t *ctx, uint8_t *buff);
				//		int32_t lsm6dso32_xl_usr_offset_z_get(stmdev_ctx_t *ctx, uint8_t *buff);
				//		int32_t lsm6dso32_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val);

				lsm6dso32_xl_usr_offset_x_set(imu, (uint8_t *) &offsetValX);
				lsm6dso32_xl_usr_offset_y_set(imu, (uint8_t *) &offsetValY);
				lsm6dso32_xl_usr_offset_z_set(imu, (uint8_t *) &offsetValZ);

				/* this function call enables the offset previously set */
				lsm6dso32_xl_usr_offset_set(imu, 1);

				break;

			case RESET_HWOFFSET:

				lsm6dso32_xl_usr_offset_x_set(imu, 0);
				lsm6dso32_xl_usr_offset_y_set(imu, 0);
				lsm6dso32_xl_usr_offset_z_set(imu, 0);

				lsm6dso32_xl_usr_offset_set(imu, 0);

				break;

			default:

				break;
		}

	//TODO STILL TO TEST

	return 0;
}

