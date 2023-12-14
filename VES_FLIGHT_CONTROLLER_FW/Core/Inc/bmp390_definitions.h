/*
 * bmp390_definitions.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Tommaso Gualtierotti
 */

#ifndef BMP390_DEFINITIONS_H
#define BMP390_DEFINITIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "main.h"

typedef uint8_t 			UINT8;
typedef int8_t 				INT8;
typedef uint16_t 			UINT16;
typedef int16_t 			INT16;
typedef uint32_t 			UINT32;
typedef int32_t 			INT32;
typedef uint64_t 			UINT64;
typedef int64_t 			INT64;

/* REGISTER ADDRESS */
#define BMP390_CHIP_ID								UINT8_C(0x00)
#define BMP390_ERR_REG								UINT8_C(0x02)
#define BMP390_SENSOR_STATUS_REG					UINT8_C(0x03)
#define BMP390_DATA_REG								UINT8_C(0x04)
#define BMP390_EVENT_REG							UINT8_C(0x10)
#define BMP390_INT_STATUS_REG						UINT8_C(0x11)
#define BMP390_INT_CTRL_REG							UINT8_C(0x19)
#define BMP390_INTERFACE_CONF_REG					UINT8_C(0x1A)
#define BMP390_PWR_CTRL_REG							UINT8_C(0x1B)
#define BMP390_OSR_REG								UINT8_C(0x1C)
#define BMP390_ODR_REG								UINT8_C(0x1D)
#define BMP390_CONFIG_REG							UINT8_C(0x1F)
#define BMP390_CALIB_DATA_REG						UINT8_C(0x31)
#define BMP390_COMAND_REG							UINT8_C(0x7E)

#define BMP390_OK									INT8_C(0)
#define BMP390_ERR_NULL_PTR							INT8_C(-1)
#define BMP390_ERR_COMMUNICATION_FAILED				INT8_C(-2)
#define BMP390_INVALID_LENGTH						INT8_C(-3)

/* POWER MODE */
#define BMP390_MODE_SLEEP							UINT8_C(0x00)
#define BMP390_MODE_FORCED							UINT8_C(0x01)
#define BMP390_MODE_NORMAL							UINT8_C(0x03)

/* FIFO ENABLE */
#define BMP390_FIFO_DISABLE							UINT8_C(0x00)
#define BMP390_FIFO_ENABLE							UINT8_C(0x01)

/* ERROR REGISTER STATUS */
#define BMP390_FATAL_ERROR							UINT8_C(0x01)
#define BMP390_CMD_ERROR							UINT8_C(0x02)
#define BMP390_CONFIG_ERROR							UINT8_C(0x04)

/* STATUS REGISTER */
#define BMP390_CMD_RDY								UINT8_C(0x10)
#define BMP390_DATARDY_PRESSURE						UINT8_C(0x20)
#define BMP390_DATARDY_TEMP							UINT8_C(0x40)

/* OVERSAMPLING */
#define BMP390_NO_OVERSAMPLING						UINT8_C(0x00)
#define BMP390_OVERSAMPLING_2X						UINT8_C(0x01)
#define BMP390_OVERSAMPLING_4X						UINT8_C(0x02)
#define BMP390_OVERSAMPLING_8X						UINT8_C(0x03)
#define BMP390_OVERSAMPLING_16X						UINT8_C(0x04)
#define BMP390_OVERSAMPLING_32X						UINT8_C(0x05)

/* INTERRUPT PIN CONFIGURATION */
/* OUTPUT MODE */
#define BMP390_INT_PIN_OPEN_DRAIN                 	UINT8_C(0x01)
#define BMP390_INT_PIN_PUSH_PULL                  	UINT8_C(0x00)

/* ACTIVE LEVEL */
#define BMP390_INT_PIN_ACTIVE_HIGH                	UINT8_C(0x01)
#define BMP390_INT_PIN_ACTIVE_LOW                 	UINT8_C(0x00)

/* FILTER SETTING */
#define BMP390_IIR_FILTER_DISABLE                 	UINT8_C(0x00)
#define BMP390_IIR_FILTER_COEFF_1                 	UINT8_C(0x01)
#define BMP390_IIR_FILTER_COEFF_3                 	UINT8_C(0x02)
#define BMP390_IIR_FILTER_COEFF_7                 	UINT8_C(0x03)
#define BMP390_IIR_FILTER_COEFF_15                	UINT8_C(0x04)
#define BMP390_IIR_FILTER_COEFF_31                	UINT8_C(0x05)
#define BMP390_IIR_FILTER_COEFF_63                	UINT8_C(0x06)
#define BMP390_IIR_FILTER_COEFF_127               	UINT8_C(0x07)

/* ODR SETTING */
#define BMP390_ODR_200_HZ                         	UINT8_C(0x00)
#define BMP390_ODR_100_HZ                         	UINT8_C(0x01)
#define BMP390_ODR_50_HZ                          	UINT8_C(0x02)
#define BMP390_ODR_25_HZ                          	UINT8_C(0x03)
#define BMP390_ODR_12_5_HZ                        	UINT8_C(0x04)
#define BMP390_ODR_6_25_HZ                        	UINT8_C(0x05)
#define BMP390_ODR_3_1_HZ                         	UINT8_C(0x06)
#define BMP390_ODR_1_5_HZ                         	UINT8_C(0x07)
#define BMP390_ODR_0_78_HZ                        	UINT8_C(0x08)
#define BMP390_ODR_0_39_HZ                        	UINT8_C(0x09)
#define BMP390_ODR_0_2_HZ                         	UINT8_C(0x0A)
#define BMP390_ODR_0_1_HZ                         	UINT8_C(0x0B)
#define BMP390_ODR_0_05_HZ                        	UINT8_C(0x0C)
#define BMP390_ODR_0_02_HZ                        	UINT8_C(0x0D)
#define BMP390_ODR_0_01_HZ                        	UINT8_C(0x0E)
#define BMP390_ODR_0_006_HZ                       	UINT8_C(0x0F)
#define BMP390_ODR_0_003_HZ                       	UINT8_C(0x10)
#define BMP390_ODR_0_001_HZ                       	UINT8_C(0x11)

#define BMP390_LEN_CALIBRATION_DATA                	UINT8_C(21)
#define BMP390_LEN_GEN_SETTINGS	                    UINT8_C(7)
#define BMP390_LEN_PRESSURE_TEMP_DATA              	UINT8_C(6)

/* MACROS FOR BIT MASKING */
#define BMP390_FATAL_ERR_MASK						UINT8_C(0x01)

#define BMP390_CMD_ERROR_MASK                      	UINT8_C(0x02)

#define BMP390_CONFIG_ERROR_MASK                   	UINT8_C(0x04)

#define BMP390_CMD_RDY_MASK                 		UINT8_C(0x10)

#define BMP390_DATARDY_PRESSURE_MASK             	UINT8_C(0x20)

#define BMP390_DATARDY_TEMP_MASK              	 	UINT8_C(0x40)

#define BMP390_OPERATING_MODE_MASK                	UINT8_C(0x30)
#define BMP390_OPERATING_MODE_POS					UINT8_C(0x04)

#define BMP390_PRESSURE_EN_MASK                    	UINT8_C(0x01)

#define BMP390_TEMP_EN_MASK                        	UINT8_C(0x02)

#define BMP390_IIR_FILTER_MASK                     	UINT8_C(0x0E)

#define BMP390_ODR_MASK                            	UINT8_C(0x1F)

#define BMP390_PRESSURE_OSR_MASK					UINT8_C(0x07)

#define BMP390_TEMP_OS_MASK                     	UINT8_C(0x38)

#define BMP390_INT_OUTPUT_MODE_MASK             	UINT8_C(0x01)

#define BMP390_INT_LEVEL_MASK                      	UINT8_C(0x02)

#define BMP390_INT_LATCH_MASK                      	UINT8_C(0x04)

#define BMP390_INT_DRDY_EN_MASK                    	UINT8_C(0x40)

#define BMP390_INT_STATUS_DRDY_MASK                	UINT8_C(0x08)

/* UTILITY MACROS */
#define BMP390_SET_LOW_BYTE                       	UINT16_C(0x00FF)
#define BMP390_SET_HIGH_BYTE                      	UINT16_C(0xFF00)

#define BMP390_CONCAT_BYTES(MSB, LSB)             	(((UINT16)MSB << 8) | (UINT16)LSB)

#define BMP390_COMPENSATE_PRESSURE                 	UINT8_C(1)
#define BMP390_COMPENSATE_TEMP                     	UINT8_C(2)
#define BMP390_COMPENSATE_BOTH_PRESSURE_TEMP       	UINT8_C(3)

#define BMP390_GET_BITS(reg_data, bitname)			((reg_data & (bitname##_MASK) >> (bitname##_POS)))

#define BMP390_SENSOR1_CS_LOW						HAL_GPIO_WritePin(BARO_1_nCS_GPIO_Port, BARO_1_nCS_Pin, GPIO_PIN_RESET);
#define BMP390_SENSOR1_CS_HIGH						HAL_GPIO_WritePin(BARO_1_nCS_GPIO_Port, BARO_1_nCS_Pin, GPIO_PIN_SET);



struct bmp390_reg_calib_data {
    /*! Trim Variables */

    UINT8 par_t1;
    UINT16 par_t2;
    INT8 par_t3;
    INT16 par_p1;
    INT16 par_p2;
    INT8 par_p3;
    INT8 par_p4;
    UINT16 par_p5;
    UINT16 par_p6;
    INT8 par_p7;
    INT8 par_p8;
    INT16 par_p9;
    INT8 par_p10;
    INT8 par_p11;
    INT64 t_lin;
};


struct bmp390_odr_osr_filter_settings {

	UINT8 pressure_os;

	UINT8 temperature_os;

	UINT8 iir_filter_param;

	UINT8 odr;
};

struct bmp390_sensor_status {

	UINT8 command_rdy;

	UINT8 pressure_datardy;

	UINT8 tempe_datardy;
};

struct bmp390_error_status {

	UINT8 fatal;

	UINT8 command;

	UINT8 config;
};

struct bmp390_status {

	struct bmp390_sensor_status sensor;

	struct bmp390_error_status error;

	UINT8 power_on_reset;
};

struct bmp390_settings {

	UINT8 operating_mode;

	UINT8 pressure_enable;

	UINT8 temperature_enable;

	struct bmp390_odr_osr_filter_settings odr_and_filter;
};

#ifdef BMP3_FLOAT_COMPENSATION

struct bmp390_quantized_calib_data
{
    /*! Quantized Trim Variables */

    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
};

struct bmp390_calib_data
{
    /* Quantized data */
    struct bmp390_quantized_calib_data quantized_calib_data;

    /* Register data */
    struct bmp390_reg_calib_data reg_calib_data;
};

struct bmp390_compensated_sensor_data
{
    /*! Compensated temperature */
    double temperature;

    /*! Compensated pressure */
    double pressure;
};

#else

struct bmp390_compensated_sensor_data
{
    /*! Compensated temperature */
    INT64 temperature;

    /*! Compensated pressure */
    UINT64 pressure;
};

/*!
 * @brief Calibration data
 */
struct bmp390_calibration_data
{
    /*! Register data */
    struct bmp390_reg_calib_data reg_calib_data;
};

#endif /* BMP3_FLOAT_COMPENSATION */


struct bmp390_uncompensated_sensor_data
{
    /*! un-compensated pressure */
    UINT64 pressure;

    /*! un-compensated temperature */
    INT64 temperature;
};

/********************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef INT8 (*bmp390_read_func_ptr_t)(UINT8 reg_addr, UINT8 *read_data, UINT32 len, void *intf_ptr);


/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef INT8 (*bmp390_write_func_ptr_t)(UINT8 reg_addr, const UINT8 *read_data, UINT32 len, void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bmp390_delay_us_func_ptr_t)(UINT32 period, void *intf_ptr);

/********************************************************/

struct bmp390_handler
{
    /*! Chip Id */
    uint8_t chip_id;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    SPI_HandleTypeDef *stm32_spi_handler;

    /*! Decide SPI or I2C read mechanism */
    UINT8 dummy_byte;

    /*! Variable to store interface error or result */
    HAL_StatusTypeDef interface_result;

    /*! Read function pointer */
    bmp390_read_func_ptr_t read;

    /*! Write function pointer */
    bmp390_write_func_ptr_t write;

    /*! Delay function pointer */
    bmp390_delay_us_func_ptr_t delay_us;

    /*! Trim data */
    struct bmp390_calibration_data calib_data;
};

#ifdef __cplusplus
}
#endif

#endif /* INC_BMP390_DEFINITIONS_H_ */























