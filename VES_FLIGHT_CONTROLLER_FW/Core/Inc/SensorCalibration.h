/*
 * SensorCalibration.h
 *
 *  Created on: Apr 3, 2024
 *      Author: tommaso
 */

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include "main.h"

int8_t calibrateBMP390(struct bmp3_dev *dev, uint16_t iterationNum);

int8_t calibrateIMU(stmdev_ctx_t *dev, uint16_t iterationNum);

#endif /* INC_SENSORCALIBRATION_H_ */
