/*
 * W25Q128.c
 *
 *  Created on: Dec 23, 2023
 *      Author: tomma & Francesco
 */

#ifndef W25Q128_H
#include "W25Q128.h"
#endif

static uint8_t transmit(W25Q128_t *memory, uint8_t *buf, uint32_t len) {

    uint8_t result = HAL_ERROR;

    if (HAL_SPI_Transmit(memory -> spi_handle, buf, len, W25Q128_TIMEOUT) == HAL_OK) {
    	result = HAL_OK;
    }

    return result;
}

static uint8_t receive(W25Q128_t *memory, uint8_t *buf, uint32_t len) {

	uint8_t result = HAL_ERROR;

    if (HAL_SPI_Receive(memory -> spi_handle, buf, len, W25Q128_TIMEOUT) == HAL_OK) {
    	result = HAL_OK;
    }

    return result;
}

uint8_t W25Q128_init(W25Q128_t* memory, GPIO_TypeDef* CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef* hspi) {
	memory -> CS_Port = CS_Port;
	memory -> CS_Pin = CS_Pin;
	memory -> spi_handle = hspi;

	W25Q128_CS_HIGH(memory);

	return HAL_OK;
}

uint8_t W25Q128_read_id(W25Q128_t* memory) {
	uint8_t result = HAL_ERROR;
	uint8_t txdata[5] = {W25Q128_READ_UNIQUE_ID, 0x00, 0x00, 0x00, 0x00};
	uint8_t rxbuffer[8] = {0};

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 5) == HAL_OK) {
		if (receive(memory, rxbuffer, 8) == HAL_OK) {
			result = HAL_OK;
			//id = 0xE4620C90D7124B42
			//id = 0xE4620C90D7124B2A
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_read_manufacturer_dev_id(W25Q128_t* memory) {
	uint8_t txdata[4] = {0};
	uint8_t result = HAL_ERROR;
	uint8_t rxbuffer[2] = {0};
	uint16_t ret = 0;

	txdata[0] = W25Q128_MANUFACTURER_DEVICE_ID;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 4) == HAL_OK) {
		if (receive(memory, rxbuffer, 2) == HAL_OK) {
			ret = (rxbuffer[0] << 8) | (rxbuffer[1]);
			if (ret == 0xEF17) {
				result = HAL_OK;
			}
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_read_JEDEC_id(W25Q128_t* memory) {
	uint8_t txdata= W25Q128_JEDEC_ID;
	uint8_t result = HAL_ERROR;
	uint8_t rxbuffer[3] = {0};
	uint32_t ret = 0;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, &txdata, 1) == HAL_OK) {
		if (receive(memory, rxbuffer, 3) == HAL_OK) {
			ret = (uint32_t) ((rxbuffer[0] << 16) | (rxbuffer[1] << 8) | (rxbuffer[2]));
			if (ret == 0xEF4018) {
				result = HAL_OK;
			}
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_power_up(W25Q128_t* memory) {
	uint8_t txdata = W25Q128_RELEASE_POWER_DOWN_ID;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, &txdata, 1) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

//	Delay(1);

	return result;
}

uint8_t W25Q128_power_down(W25Q128_t* memory) {
	uint8_t txdata = W25Q128_POWER_DOWN;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, &txdata, 1) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_read_statusreg1(W25Q128_t* memory, uint8_t* data) {
	uint8_t txdata[1] = {W25Q128_READ_STATUS_REGISTER_1};
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 1) == HAL_OK) {
		if (receive(memory, data, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_write_statusreg1(W25Q128_t* memory, uint8_t settings) {
	uint8_t txdata[2] = {W25Q128_WRITE_STATUS_REGISTER_1, settings & 0x7C};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 2) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_read_statusreg2(W25Q128_t* memory, uint8_t* data) {
	uint8_t txdata[1] = {W25Q128_READ_STATUS_REGISTER_2};
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 1) == HAL_OK) {
		if (receive(memory, data, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_write_statusreg2(W25Q128_t* memory, uint8_t settings) {
	uint8_t txdata[2] = {W25Q128_WRITE_STATUS_REGISTER_2, settings & 0x7C};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 2) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_read_statusreg3(W25Q128_t* memory, uint8_t* data) {
	uint8_t txdata = W25Q128_READ_STATUS_REGISTER_3;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, &txdata, 1) == HAL_OK) {
		if (receive(memory, data, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_write_statusreg3(W25Q128_t* memory, uint8_t settings) {
	uint8_t txdata[2] = {W25Q128_WRITE_STATUS_REGISTER_3, settings & 0x7C};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 2) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_write_enable(W25Q128_t* memory) {
	uint8_t txdata[1] = {W25Q128_WRITE_ENABLE};
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 1) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_write_disable(W25Q128_t* memory) {
	uint8_t txdata[1] = {W25Q128_WRITE_DISABLE};
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 1) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}
uint8_t W25Q128_write_volatile_enable(W25Q128_t* memory) {\
	uint8_t txdata = W25Q128_VOLATILE_SR_WRITE_ENABLE;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, &txdata, 1) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_read_data(W25Q128_t* memory, uint8_t* address, uint8_t* data, uint16_t length) {
	uint8_t txdata[4] = {W25Q128_READ_DATA, address[0], address[1], address[2]};
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (transmit(memory, txdata, 4) == HAL_OK) {
		if (receive(memory, data, length) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_write_data(W25Q128_t* memory, uint8_t* address, uint8_t* data, uint16_t length) {
	uint8_t txdata[4] = {W25Q128_PAGE_PROGRAM, address[0], address[1], address[2]};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory, txdata, 4) == HAL_OK) {
			if (transmit(memory, data, length) == HAL_OK) {
				result = HAL_OK;
			}
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory); //T:done automatically by the memory (?) (It seems to be) F:Yes

	return result;
}

uint8_t W25Q128_erase_sector(W25Q128_t* memory, uint8_t* address) {
	uint8_t txdata[4] = {W25Q128_SECTOR_ERASE_4KB, address[0], address[1], address[2]};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory, txdata, 4) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_write_flown_flag(W25Q128_t* memory, uint8_t* address, uint8_t* data, uint16_t length,uint8_t flag) {
    uint8_t txdata[4] = {W25Q128_PAGE_PROGRAM, address[0], address[1], address[2]};
    uint8_t result = HAL_ERROR;

    result = W25Q128_write_enable(memory);

    W25Q128_CS_LOW(memory);

    if (result == HAL_OK) {
        result = HAL_ERROR;

        if (transmit(memory, txdata, 4) == HAL_OK) {
            data[0] = flag; // Set the first byte to true/1 or false/0
            if (transmit(memory, data, length) == HAL_OK) {
                result = HAL_OK;
            }
        }
    }

    W25Q128_CS_HIGH(memory);

    W25Q128_write_disable(memory);

    return result;
}

uint8_t W25Q128_read_flown_flag(W25Q128_t* memory, uint8_t* address, uint8_t* data, uint16_t length) {
    uint8_t txdata[4] = {W25Q128_READ_DATA, address[0], address[1], address[2]};
    uint8_t result = HAL_ERROR;

    W25Q128_CS_LOW(memory);

    if (transmit(memory, txdata, 4) == HAL_OK) {
        if (receive(memory, data, 1) == HAL_OK) { // Read only 1 byte
            result = HAL_OK;
        }
    }

    W25Q128_CS_HIGH(memory);

    return result;
}

uint8_t W25Q128_chip_erase(W25Q128_t* memory) {
	uint8_t txdata[1] = {W25Q128_CHIP_ERASE};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory, txdata, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	result = W25Q128_write_disable(memory);

	return result;
}










/* BLOCK LOCK AND UNLOCK (BOTH GLOBAL AND INDIVIDUAL) */

uint8_t W25Q128_global_block_lock(W25Q128_t* memory) {
	uint8_t txdata = W25Q128_GLOBAL_BLOCK_LOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory , &txdata, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_global_block_unlock(W25Q128_t* memory) {
	uint8_t txdata = W25Q128_GLOBAL_BLOCK_UNLOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory, &txdata, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_individual_block_lock(W25Q128_t* memory, uint8_t* address) {
	uint8_t txdata = W25Q128_INDIVIDUAL_BLOCK_LOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory , &txdata, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_individual_block_unlock(W25Q128_t* memory, uint8_t* address) {
	uint8_t txdata = W25Q128_INDIVIDUAL_BLOCK_UNLOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (transmit(memory, &txdata, 1) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}
