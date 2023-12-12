/*
 * bmp390.c
 *
 *  Created on: Dec 12, 2023
 *      Author: Tommaso Gualtierotti
 */

#include "bmp390.h"

static INT8 null_ptr_check(const struct bmp390_handler *handler);

static INT8 get_calibration_data(struct bmp390_handler *handler);

static void parse_calibration_data(const UINT8 *reg_data, struct bmp390_handler *handler);

static void interleave_register_address(const UINT8 *reg_address, UINT8 *temp_buff, const UINT8 *reg_data, UINT32 len);

static void parse_settings_data(const uint8_t *reg_data, struct bmp390_settings *settings);

static INT8 get_sensor_status(struct bmp390_status *status, struct bmp390_handler *handler);

static INT8 get_int_status(struct bmp390_status *status, struct bmp390_handler *handler);

static INT8 get_err_status(struct bmp390_status *status, struct bmp390_handler *handler);

static void parse_sensor_data(const UINT8 *reg_data, struct bmp390_uncompensated_sensor_data *uncompensated_data);

static INT8 compensate_data(UINT8 sensor_comp, const struct bmp390_uncompensated_sensor_data *uncompensated_data, struct bmp390_compensated_sensor_data *compensated_data, struct bmp390_calibration_data *calib_data);

INT8 bmp390_INIT(struct bmp390_handler *handler) {

	INT8 result;

	result = null_ptr_check(handler);

	if (result == BMP390_OK) {
		handler -> dummy_byte = 1;

		bmp390_soft_reset(handler);

		if (result == BMP390_OK) {

			result = get_calibration_data(handler);
		}
	}

	return result;
}

INT8 bmp390_get_registers(UINT8 reg_address, UINT8 *reg_data, UINT32 len, struct bmp390_handler *handler) {

	INT8 result;
	UINT32 idx;

	result = null_ptr_check(handler);

	if ((result == BMP390_OK) && (reg_data != NULL)) {

		UINT32 temp_len = len + handler -> dummy_byte;
		UINT8 temp_buff[len + handler -> dummy_byte];

		reg_address = reg_address | 0x80;		// setting the 8th bit to 1 because we are reading (see sensor datasheet pag. 43)

		handler -> interface_result = handler -> read(reg_address, temp_buff, temp_len, handler -> stm32_spi_handler);
		for (idx = 0; idx < len; idx++) {
			reg_data[idx] = temp_buff[idx + handler -> dummy_byte];
		}

		if (handler -> interface_result != HAL_OK) {
				result = BMP390_ERR_COMMUNICATION_FAILED;
		}
	}
	else {

		result = BMP390_ERR_NULL_PTR;
	}

	return result;
}

INT8 bmp390_set_registers(UINT8 *reg_address, const UINT8 *reg_data, UINT32 len, struct bmp390_handler *handler) {

	INT8 result;
	UINT8 temp_buff[len * 2];
	UINT32 temp_len;
	UINT8 register_address_count;

	result = null_ptr_check(handler);

	if ((result == BMP390_OK) && (reg_address != NULL) && (reg_data != NULL)) {

		if (len != 0) {

			temp_buff[0] = reg_data[0];

			for (register_address_count = 0; register_address_count < len; register_address_count++) {

				// setting the 8th bit to 0 to perform the read, according to datasheet (see datasheet pag. 42)
				reg_address[register_address_count] = reg_address[register_address_count] & 0x7F;
			}

			if (len > 1) {
				interleave_register_address(reg_address, temp_buff, reg_data, len);
				temp_len = len * 2;
			}
			else {
				temp_len = len;
			}

			handler -> interface_result = handler -> write(reg_address[0], temp_buff, temp_len, handler -> stm32_spi_handler);

			if (handler -> interface_result != HAL_OK) {

				result = BMP390_ERR_COMMUNICATION_FAILED;
			}
		}
		else {

			result = BMP390_INVALID_LENGTH;
		}
	}
	else {

		result = BMP390_ERR_NULL_PTR;
	}

	return result;
}

INT8 bmp390_get_sensor_settings(struct bmp390_settings *settings, struct bmp390_handler *handler) {

	INT8 result;
	UINT8 settings_data[BMP390_LEN_GEN_SETTINGS];

	if (settings != NULL) {

		result = bmp390_get_registers(BMP390_INT_CTRL_REG, settings_data, BMP390_LEN_GEN_SETTINGS, handler);

		if (result == BMP390_OK) {
			parse_settings_data(settings_data, settings);
		}
	}
	else {

		result = BMP390_ERR_NULL_PTR;
	}

	return result;
}

INT8 bmp390_get_status(struct bmp390_status *status, struct bmp390_handler *handler) {

	INT8 result;

	if (status != NULL) {

		result = get_sensor_status(status, handler);

		if (result == BMP390_OK) {

			result = get_int_status(status, handler);

			if (result == BMP390_OK) {
				result = get_err_status(status, handler);
			}
		}
	}
	else {
		result = BMP390_ERR_NULL_PTR;
	}

	return result;
}

INT8 bmp390_get_sensor_data(UINT8 sensor_comp, struct bmp390_compensated_sensor_data *compensated_data, struct bmp390_handler *handler) {

	UINT8 result;

	UINT8 reg_data[BMP390_LEN_PRESSURE_TEMP_DATA] = { 0 };
	struct bmp390_uncompensated_sensor_data uncompensated_data = { 0 };

	if (compensated_data != NULL) {

		result = bmp390_get_registers(BMP390_DATA_REG, reg_data, BMP390_LEN_PRESSURE_TEMP_DATA, handler);

		if (result == BMP390_OK) {

			parse_sensor_data(reg_data, &uncompensated_data);

			result = compensate_data(sensor_comp, &uncompensated_data, compensated_data, &handler -> calib_data);
		}
	}
	else {

		result = BMP390_ERR_NULL_PTR;
	}

	return result;
}



/* STATIC FUNCTIONS DEFINITIONS */

static INT8 null_ptr_check(const struct bmp390_handler *handler) {

	INT8 result;

	if ((handler == NULL) || (handler -> read == NULL) ||
		(handler -> write == NULL) || (handler -> stm32_spi_handler == NULL) ||
		(handler -> delay_us == NULL)) {

			result = BMP390_ERR_NULL_PTR;
	} else {
			result = BMP390_OK;
	}

	return result;
}

static INT8 get_calibration_data(struct bmp390_handler *handler) {

	INT8 result;
	UINT8 reg_addr = BMP390_CALIB_DATA_REG;

	UINT8 calibration_data[BMP390_LEN_CALIBRATION_DATA] = {0};

	result = bmp390_get_registers(reg_addr, calibration_data, BMP390_LEN_CALIBRATION_DATA, handler);

	parse_calibration_data(calibration_data, handler);

	return result;
}

static void parse_calibration_data(const UINT8 *reg_data, struct bmp390_handler *handler) {

	struct bmp390_reg_calib_data *calibration_data = &(handler -> calib_data.reg_calib_data);

	calibration_data -> par_t1 = BMP390_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calibration_data -> par_t2 = BMP390_CONCAT_BYTES(reg_data[3], reg_data[2]);
	calibration_data -> par_t3 = (INT8)reg_data[4];
	calibration_data -> par_p1 = (INT16)BMP390_CONCAT_BYTES(reg_data[6], reg_data[5]);
	calibration_data -> par_p2 = (INT16)BMP390_CONCAT_BYTES(reg_data[8], reg_data[7]);
	calibration_data -> par_p3 = (INT8)reg_data[9];
	calibration_data -> par_p4 = (INT8)reg_data[10];
	calibration_data -> par_p5 = BMP390_CONCAT_BYTES(reg_data[12], reg_data[11]);
	calibration_data -> par_p6 = BMP390_CONCAT_BYTES(reg_data[14], reg_data[13]);
	calibration_data -> par_p7 = (INT8)reg_data[15];
	calibration_data -> par_p8 = (INT8)reg_data[16];
	calibration_data -> par_p9 = (INT16)BMP390_CONCAT_BYTES(reg_data[18], reg_data[17]);
	calibration_data -> par_p10 = (INT8)reg_data[19];
	calibration_data -> par_p11 = (INT8)reg_data[20];
}

static void interleave_register_address(const UINT8 *reg_address, UINT8 *temp_buff, const UINT8 *reg_data, UINT32 len) {

	UINT32 index;

	for (index = 1; index < len; index++) {
		temp_buff[(index * 2) - 1] = reg_address[index];
		temp_buff[index * 2] = reg_data[index];
	}
}



static void parse_sensor_data(const UINT8 *reg_data, struct bmp390_uncompensated_sensor_data *uncompensated_data) {

	UINT32 data_xlsb;
	UINT32 data_lsb;
	UINT32 data_msb;

	data_xlsb = (UINT32)reg_data[0];
	data_lsb = (UINT32)reg_data[1] << 8;
	data_msb = (UINT32)reg_data[2] << 16;

	uncompensated_data -> pressure = data_msb | data_lsb | data_xlsb;

	data_xlsb = (UINT32)reg_data[3];
	data_lsb = (UINT32)reg_data[4] << 8;
	data_msb = (UINT32)reg_data[5] << 16;

	uncompensated_data -> temperature = data_msb | data_lsb | data_xlsb;
}

























