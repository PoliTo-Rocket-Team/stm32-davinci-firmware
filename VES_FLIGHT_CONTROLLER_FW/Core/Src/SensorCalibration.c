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

int8_t calibrateIMU(stmdev_ctx_t *dev, uint16_t iterationNum) {

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

	for (int i = 0; i < 100; i++) {
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

	// add code to set a possible hardware offset in the IMU, if needed

	return 0;
}
