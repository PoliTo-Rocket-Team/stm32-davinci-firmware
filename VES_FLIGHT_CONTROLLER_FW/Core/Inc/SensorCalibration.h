/*
 * SensorCalibration.h
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include "main.h"

#define OFFSET_2_6	pow(2, -6)
#define OFFSET_2_10	pow(2, -10)

#define HWOFFSET 0
#define SWOFFSET 1

float acceleration_mg_swoffset[3] = {0};
float angular_rate_mdps_swoffset[3] = {0};

int8_t calibrateBMP390(struct bmp3_dev *dev, uint16_t iterationNum);

int8_t calibrateIMU(stmdev_ctx_t *dev, uint16_t iterationNum, uint8_t offsetType);

#endif /* INC_SENSORCALIBRATION_H_ */
