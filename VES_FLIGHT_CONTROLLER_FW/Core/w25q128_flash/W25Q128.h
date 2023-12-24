/*
 * W25Q128.h
 *
 *  Created on: Dec 23, 2023
 *      Author: tomma
 */

#ifndef W25Q128_H
#define W25Q128_H

#include <stdint.h>
#include <stm32f4xx.h>

#define SPIF_CMSIS_RTOS_DISABLE               0
#define SPIF_CMSIS_RTOS_V1                    1
#define SPIF_CMSIS_RTOS_V2                    2

#define SPIF_CMSIS_RTOS      SPIF_CMSIS_RTOS_DISABLE

#if SPIF_CMSIS_RTOS == SPIF_CMSIS_RTOS_DISABLE
#define Delay(x) HAL_Delay(x)
#elif SPIF_CMSIS_RTOS == SPIF_CMSIS_RTOS_V1
#include "cmsis_os.h"
#define Delay(x) osDelay(x)
#else
#include "cmsis_os2.h"
#define Delay(x) osDelay(x)
#endif

#define W25Q128_CS_HIGH(memory)				HAL_GPIO_WritePin(memory -> CS_Port, memory -> CS_Pin, GPIO_PIN_SET)
#define W25Q128_CS_LOW(memory)				HAL_GPIO_WritePin(memory -> CS_Port, memory -> CS_Pin, GPIO_PIN_RESET)

#define W25Q128_TIMEOUT						100

#define SPI_MODE	0x4018

#define W25Q128_WRITE_ENABLE				0x06
#define W25Q128_VOLATILE_SR_WRITE_ENABLE	0x50
#define W25Q128_WRITE_DISABLE				0x04
#define W25Q128_CHIP_ERASE					0xC7	/*or 0x60*/
#define W25Q128_ERASE_PROGRAM_SUSPEND		0x75
#define W25Q128_ERASE_PROGRAM_RESUME		0x7A
#define W25Q128_POWER_DOWN					0xB9
#define W25Q128_GLOBAL_BLOCK_LOCK			0x7E
#define W25Q128_GLOBAL_BLOCK_UNLOCK			0x98
#define W25Q128_ENABLE_RESET				0x66
#define W25Q128_RESET_DEVICE				0x99
#define W25Q128_READ_STATUS_REGISTER_1		0x05
#define W25Q128_WRITE_STATUS_REGISTER_1		0x01
#define W25Q128_READ_STATUS_REGISTER_2		0x35
#define W25Q128_WRITE_STATUS_REGISTER_2		0x31
#define W25Q128_READ_STATUS_REGISTER_3		0x15
#define W25Q128_WRITE_STATUS_REGISTER_3		0x11
#define W25Q128_RELEASE_POWER_DOWN_ID		0xAB
#define W25Q128_MANUFACTURER_DEVICE_ID		0x90
#define W25Q128_JEDEC_ID					0x9F

#define W25Q128_READ_UNIQUE_ID				0x4B
#define W25Q128_PAGE_PROGRAM				0x02
#define W25Q128_QUAD_PAGE_PROGRAM			0x32
#define W25Q128_SECTOR_ERASE_4KB			0x20	/* 4 KB */
#define W25Q128_BLOCK_ERASE_32KB			0x52
#define W25Q128_BLOCK_ERASE_64KB			0xD8
#define W25Q128_READ_DATA					0x03
#define W25Q128_FAST_READ					0x0B
#define W25Q128_FAST_READ_DUAL_OUTPUT		0x3B
#define W25Q128_FAST_READ_QUAD_OUTPUT		0x6B
#define W25Q128_READ_SFDP_REGISTER			0x5A
#define W25Q128_ERASE_SECURITY_REGISTER		0x44
#define W25Q128_PROGRAM_SECURITY_REGISTER	0x42
#define W25Q128_READ_SECURITY_REGISTER		0x48
#define W25Q128_INDIVIDUAL_BLOCK_LOCK		0x36
#define W25Q128_INDIVIDUAL_BLOCK_UNLOCK		0x39
#define W25Q128_READ_BLOCK_LOCK				0x3D

typedef struct {
	uint8_t isBusy;
	GPIO_TypeDef *CS_Port;
	uint16_t CS_Pin;
	uint8_t ID[8];
	SPI_HandleTypeDef *spi_handle;
} W25Q128;

uint8_t W25Q128_init(W25Q128*, GPIO_TypeDef*, uint16_t, SPI_HandleTypeDef*);
uint8_t W25Q128_read_id(W25Q128*);
uint8_t W25Q128_power_up(W25Q128*);
uint8_t W25Q128_power_down(W25Q128*);

uint8_t W25Q128_read_statusreg1(W25Q128*, uint8_t*);
uint8_t W25Q128_write_statusreg1(W25Q128*, uint8_t);
uint8_t W25Q128_read_statusreg2(W25Q128*, uint8_t*);
uint8_t W25Q128_write_statusreg2(W25Q128*, uint8_t);
uint8_t W25Q128_read_statusreg3(W25Q128*, uint8_t*);
uint8_t W25Q128_write_statusreg3(W25Q128*, uint8_t);
uint8_t W25Q128_write_enable(W25Q128*);
uint8_t W25Q128_write_disable(W25Q128*);
uint8_t W25Q128_write_volatile_enable(W25Q128*);

uint8_t W25Q128_read_data(W25Q128*, uint8_t*, uint8_t*, uint16_t);
uint8_t W25Q128_write_data(W25Q128*, uint8_t*, uint8_t*, uint16_t);
uint8_t W25Q128_erase_sector(W25Q128*, uint8_t*);
uint8_t W25Q128_global_block_lock(W25Q128*);
uint8_t W25Q128_global_block_unlock(W25Q128*);
uint8_t W25Q128_individual_block_lock(W25Q128*, uint8_t*);
uint8_t W25Q128_individual_block_unlock(W25Q128*, uint8_t*);
uint8_t W25Q128_chip_erase(W25Q128*);

#endif /* INC_W25Q128_H_ */
