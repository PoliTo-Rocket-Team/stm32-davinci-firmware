/*
 * utils.h
 *
 *  Created on: Dec 31, 2023
 *      Author: tomma
 */



#ifndef UTILITIES_H_
#define UTILITIES_H_

#ifndef _BMP3_H
#include "bmp3.h"
#endif

#ifndef LSM6DSO32_REGS_H
#include "lsm6dso32_reg.h"
#endif

#ifndef W25Q128_H
#include "W25Q128.h"
#endif

//#define DEBUG_EN

#ifdef DEBUG_EN

#include "stm32f4xx.h"

HAL_StatusTypeDef COM_port_serial_print(const uint8_t* data);
#endif

#include "main.h"
#include <stdbool.h>



extern osEventFlagsId_t fsm_flag_id;
extern osMessageQueueId_t event_queue;


#define AIR_DENSITY_KG_M3	1.293f
#define SPECIFIC_GAS_CONSTANT 287.0f // Joules/Kg*Kelvin

typedef enum {
	HWOFFSET = 0,
	SWOFFSET = 1,
	RESET_HWOFFSET = 2
} OFFSET_TYPE;



//float sw_offset_acceleration_mg[3] = {0};
//float swoffset_angular_rate_mdps[3] = {0};

float computeAltitude(double pressurePa, double temperature);

int32_t imu1_write(void *handle, uint8_t reg_addr, const uint8_t *buf, uint16_t len);

int32_t imu1_read(void *handle, uint8_t reg_addr, uint8_t *buf, uint16_t len);

int8_t bmp390_1_write(uint8_t reg_addr, const uint8_t *buf, uint32_t len, void *intf_ptr);

int8_t bmp390_1_read(uint8_t reg_addr, uint8_t *buf, uint32_t len, void *intf_ptr);

int32_t imu2_write(void *handle, uint8_t reg_addr, const uint8_t *buf, uint16_t len);

int32_t imu2_read(void *handle, uint8_t reg_addr, uint8_t *buf, uint16_t len);

int8_t bmp390_2_write(uint8_t reg_addr, const uint8_t *buf, uint32_t len, void *intf_ptr);

int8_t bmp390_2_read(uint8_t reg_addr, uint8_t *buf, uint32_t len, void *intf_ptr);

int8_t init_imu1(stmdev_ctx_t *imu, lsm6dso32_fs_xl_t acc_full_scale, lsm6dso32_fs_g_t gyro_full_scale, lsm6dso32_odr_xl_t acc_output_data_rate, lsm6dso32_odr_g_t gyro_output_data_rate);

int8_t init_bmp390_1(struct bmp3_dev *bmp390);

int8_t init_imu2(stmdev_ctx_t *imu, lsm6dso32_fs_xl_t acc_full_scale, lsm6dso32_fs_g_t gyro_full_scale, lsm6dso32_odr_xl_t acc_output_data_rate, lsm6dso32_odr_g_t gyro_output_data_rate);

int8_t init_bmp390_2(struct bmp3_dev *bmp390);

int8_t init_flash(W25Q128_t *flash, chip_erasing erase);

uint16_t compute_velocity(float_t temperature,float_t blessings,uint16_t diff_pressure);

//int8_t calibrateBMP390(struct bmp3_dev *bmp390, uint16_t iterationNum);

int8_t calibrateIMU(stmdev_ctx_t *imu, uint16_t iterationNum, OFFSET_TYPE type);

//XXX
//FIXME
// function to be implemented
float get_air_density();

#endif /* UTILITIES_H_ */
