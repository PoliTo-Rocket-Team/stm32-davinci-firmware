/*
 * bmp390.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Tommaso Gualtierotti
 */

#ifndef BMP390_H
#define BMP390_H

#include "bmp390_definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @details This API is the entry point.
 *
 * @param[in, out] handler : structure instance of bmp390_handler
 * @retval Result of API execution status
 * @retval 0	-> Success
 * @retval > 0	-> Warning
 * @retval < 0	-> Error
 */
INT8 bmp390_INIT(struct bmp390_handler *handler);

INT8 bmp390_soft_reset(struct bmp390_handler *handler);

INT8 bmp390_set_sensor_settings(INT32 desired_settings, struct bmp390_settings *settings, struct bmp390_handler *handler);

INT8 bmp390_get_sensor_settings(struct bmp390_settings *settings, struct bmp390_handler *handler);

INT8 bmp390_set_operating_mode(struct bmp390_settings *settings, struct bmp390_handler *handler);

INT8 bmp390_get_operating_mode(UINT8 *op_mode, struct bmp390_handler *handler);

INT8 bmp390_get_sensor_data(UINT8 sensor_comp, struct bmp390_compensated_sensor_data *data, struct bmp390_handler *handler);

INT8 bmp390_set_registers(UINT8 reg_address, const UINT8 *reg_data, UINT32 len, struct bmp390_handler *handler);

INT8 bmp390_get_registers(UINT8 reg_address, UINT8 *reg_data, UINT32 len, struct bmp390_handler *handler);

INT8 bmp390_get_status(struct bmp390_status *status, struct bmp390_handler *handler);

#ifdef __cplusplus
}
#endif

#endif /* BMP390_H */
