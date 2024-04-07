/*
 * SensorCalibration.c
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef SENSOR_CALIBRATION_H
#include "SensorCalibration.h"
#endif

int8_t calibrateBMP390(struct bmp3_dev *dev, uint16_t iterationNum) {

	struct bmp3_data temp = { -1, -1 };
	int8_t result = BMP3_E_NULL_PTR;

	result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &temp, dev);

	if (result != BMP3_OK) {
		return result;
	}

	double mean_pressure = temp.pressure;
	double mean_temperature = temp.temperature;

	for (int i = 0; i < iterationNum; i++) {
			struct bmp3_data data = { -1, -1 };
			struct bmp3_status status = { { 0 } };

			result = bmp3_get_status(&status, dev);

			if (result != BMP3_OK)	return result;

			result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, dev);

			mean_pressure += data.pressure;
			mean_pressure /= 2;

			mean_temperature += data.temperature;
			mean_temperature /= 2;
		}

	// add the code to add a possible software offset, if needed

	return 0;
}

//FIXME remember that when on the rocket the gravity acceleration will be along the X axis

int8_t calibrateIMU(stmdev_ctx_t *dev, uint16_t iterationNum, uint8_t offsetType) {

	float mean_acceleration_mg[3] = {0};
	int16_t data_raw_acceleration[3] = {0};
	float acceleration_mg[3] = {0};
	float mean_angular_rate_mdps[3] = {0};
	int16_t data_raw_angular_rate[3] = {0};
	float angular_rate_mdps[3] = {0};
	float mean_temperature_degC = 0.0;
	int16_t data_raw_temperature = 0.0;
	float temperature_degC = 0.0;

	lsm6dso32_acceleration_raw_get(dev, data_raw_acceleration);
	acceleration_mg[0] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[0]);
	acceleration_mg[1] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[1]);
	acceleration_mg[2] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[2]);
	mean_acceleration_mg[0] += acceleration_mg[0];
	mean_acceleration_mg[1] += acceleration_mg[1];
	mean_acceleration_mg[2] += acceleration_mg[2];

	lsm6dso32_angular_rate_raw_get(dev, data_raw_angular_rate);
	angular_rate_mdps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[0]);
	angular_rate_mdps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[1]);
	angular_rate_mdps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[2]);
	mean_angular_rate_mdps[0] += angular_rate_mdps[0];
	mean_angular_rate_mdps[1] += angular_rate_mdps[1];
	mean_angular_rate_mdps[2] += angular_rate_mdps[2];

	lsm6dso32_temperature_raw_get(dev, &data_raw_temperature);
	temperature_degC = lsm6dso32_from_lsb_to_celsius(data_raw_temperature);
	mean_temperature_degC += temperature_degC;

	for (int i = 0; i < iterationNum; i++) {
		lsm6dso32_reg_t reg;
		/* Read output only if new data is available */
		lsm6dso32_status_reg_get(dev, &reg.status_reg);

		if (reg.status_reg.xlda) {
			/* Read acceleration data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			lsm6dso32_acceleration_raw_get(dev, data_raw_acceleration);
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
			lsm6dso32_angular_rate_raw_get(dev, data_raw_angular_rate);
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
			lsm6dso32_temperature_raw_get(dev, &data_raw_temperature);
			temperature_degC = lsm6dso32_from_lsb_to_celsius(data_raw_temperature);
			mean_temperature_degC += temperature_degC;
			mean_temperature_degC /= 2;
		}
	}

	switch(offsetType) {
			case SWOFFSET:

				acceleration_mg_swoffset[0] = mean_acceleration_mg[0];
				acceleration_mg_swoffset[1] = mean_acceleration_mg[1];
				acceleration_mg_swoffset[2] = mean_acceleration_mg[2];

				angular_rate_mdps_swoffset[0] = mean_angular_rate_mdps[0];
				angular_rate_mdps_swoffset[1] = mean_angular_rate_mdps[1];
				angular_rate_mdps_swoffset[2] = mean_angular_rate_mdps[2];

				break;
			case HWOFFSET:
					/* FIXME handle a possible sw offset type, store offset values somewhere and use
					 * them to calibrate measurements. */

					// add code to set a possible hardware offset in the IMU, if needed

					/*
						LSM6DSO32_X_OFS_USR
						LSM6DSO32_Y_OFS_USR
						LSM6DSO32_Z_OFS_USR

						are the registers used to set an hardware offset to accelerometer measurements

						To enable offset USR_OFF_ON_OUT bit of the CTRL7_G register must be set.

						The value of the offset is expressed on 8 bits 2's complement.

						The weight [g/LSB] to be applied to the offset register values is independent of the accelerometer selected full
						scale and can be configured using the USR_OFF_W bit of the CTRL6_C register:
						• 2^-10 g/LSB if the USR_OFF_W bit is set to 0
						• 2^-6  g/LSB if the USR_OFF_W bit is set to 1
					 */

					if (offsetType == HWOFFSET) {

						int16_t offsetValX = mean_acceleration_mg[0] / OFFSET_2_10;
						int16_t offsetValY = mean_acceleration_mg[1] / OFFSET_2_10;
						int16_t offsetValZ = (1 - mean_acceleration_mg[2]) / OFFSET_2_10;

						if (offsetValX > -128 && offsetValX < 128 && offsetValY > -128 && offsetValY < 128
								&& offsetValZ > -128 && offsetValZ < 128) {
							lsm6dso32_xl_offset_weight_set(dev, LSM6DSO32_LSb_1mg);
						} else {
							lsm6dso32_xl_offset_weight_set(dev, LSM6DSO32_LSb_16mg);
							offsetValX = mean_acceleration_mg[0] / OFFSET_2_6;
							offsetValY = mean_acceleration_mg[1] / OFFSET_2_6;
							offsetValZ = (1 - mean_acceleration_mg[2]) / OFFSET_2_6;
						}

				//		int32_t lsm6dso32_xl_offset_weight_get(stmdev_ctx_t *ctx,lsm6dso32_usr_off_w_t *val);

						lsm6dso32_xl_usr_offset_x_set(dev, (uint8_t *) &offsetValX);
				//		int32_t lsm6dso32_xl_usr_offset_x_get(stmdev_ctx_t *ctx,uint8_t *buff);

						lsm6dso32_xl_usr_offset_y_set(dev, (uint8_t *) &offsetValY);
				//		int32_t lsm6dso32_xl_usr_offset_y_get(stmdev_ctx_t *ctx,uint8_t *buff);

						lsm6dso32_xl_usr_offset_z_set(dev, (uint8_t *) &offsetValZ);
				//		int32_t lsm6dso32_xl_usr_offset_z_get(stmdev_ctx_t *ctx,uint8_t *buff);

						lsm6dso32_xl_usr_offset_set(dev, 1);
				//		int32_t lsm6dso32_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val);
					}
				break;
			default:

				break;
		}

	//TODO STILL TO TEST

	return 0;
}
