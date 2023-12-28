/*
 * W25Q128.c
 *
 *  Created on: Dec 23, 2023
 *      Author: tomma
 */

#ifndef W25Q128_H
#include "W25Q128.h"
#endif

uint8_t W25Q128_init(W25Q128* memory, GPIO_TypeDef* CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef* hspi) {
	memory -> CS_Port = CS_Port;
	memory -> CS_Pin = CS_Pin;
	memory -> spi_handle = hspi;

	W25Q128_CS_HIGH(memory);

	return HAL_OK;
}

uint8_t W25Q128_read_id(W25Q128* memory) {
	uint8_t txdata[5] = {W25Q128_READ_UNIQUE_ID, 0x00, 0x00, 0x00, 0x00};
	uint8_t rxbuffer[13];

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_TransmitReceive(memory -> spi_handle, txdata, rxbuffer, 13, W25Q128_TIMEOUT) == HAL_OK) {
		W25Q128_CS_HIGH(memory);

		for (int i = 0; i < 8; i++) {
			memory -> ID[i] = rxbuffer[i + 5];
		}

		return HAL_OK;
	}
	else {
		W25Q128_CS_HIGH(memory);

		return HAL_ERROR;
	}
}

uint8_t W25Q128_power_up(W25Q128* memory) {
	uint8_t txdata = W25Q128_RELEASE_POWER_DOWN_ID;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

//	Delay(1);

	return result;
}

uint8_t W25Q128_power_down(W25Q128* memory) {
	uint8_t txdata = W25Q128_POWER_DOWN;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_read_statusreg1(W25Q128* memory, uint8_t* data) {
	uint8_t txdata = W25Q128_READ_STATUS_REGISTER_1;
	uint8_t result = HAL_ERROR;
	uint8_t rxbuffer[2];

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_TransmitReceive(memory -> spi_handle, &txdata, rxbuffer, 2, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	data[0] = rxbuffer[1];

	return result;
}

uint8_t W25Q128_write_statusreg1(W25Q128* memory, uint8_t settings) {
	uint8_t txdata[2] = {W25Q128_WRITE_STATUS_REGISTER_1, settings & 0x7C};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, txdata, 2, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_read_statusreg2(W25Q128* memory, uint8_t* data) {
	uint8_t txdata = W25Q128_READ_STATUS_REGISTER_2;
	uint8_t result = HAL_ERROR;
	uint8_t rxbuffer[2];

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_TransmitReceive(memory -> spi_handle, &txdata, rxbuffer, 2, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	data[0] = rxbuffer[1];

	return result;
}

uint8_t W25Q128_write_statusreg2(W25Q128* memory, uint8_t settings) {
	uint8_t txdata[2] = {W25Q128_WRITE_STATUS_REGISTER_2, settings & 0x7C};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, txdata, 2, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_read_statusreg3(W25Q128* memory, uint8_t* data) {
	uint8_t txdata = W25Q128_READ_STATUS_REGISTER_3;
	uint8_t result = HAL_ERROR;
	uint8_t rxbuffer[2];

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_TransmitReceive(memory -> spi_handle, &txdata, rxbuffer, 2, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	data[0] = rxbuffer[1];

	return result;
}

uint8_t W25Q128_write_statusreg3(W25Q128* memory, uint8_t settings) {
	uint8_t txdata[2] = {W25Q128_WRITE_STATUS_REGISTER_3, settings & 0x7C};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, txdata, 2, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_write_enable(W25Q128* memory) {
	uint8_t txdata = W25Q128_WRITE_ENABLE;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_write_disable(W25Q128* memory) {
	uint8_t txdata = W25Q128_WRITE_DISABLE;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}
uint8_t W25Q128_write_volatile_enable(W25Q128* memory) {\
	uint8_t txdata = W25Q128_VOLATILE_SR_WRITE_ENABLE;
	uint8_t result = HAL_ERROR;

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_read_data(W25Q128* memory, uint8_t* address, uint8_t* data, uint16_t length) {
	uint8_t txdata[4] = {W25Q128_READ_DATA, address[0], address[1], address[2]};
	uint8_t result = HAL_ERROR;
	uint8_t rxbuffer[length + 4];

	W25Q128_CS_LOW(memory);

	if (HAL_SPI_TransmitReceive(memory -> spi_handle, txdata, rxbuffer, (length + 4), W25Q128_TIMEOUT) == HAL_OK) {
		result = HAL_OK;
	}

	W25Q128_CS_HIGH(memory);

	for (int i = 0; i < length; i++) {
		data [i] = rxbuffer[i + 4];
	}

	return result;
}

uint8_t W25Q128_write_data(W25Q128* memory, uint8_t* address, uint8_t* data, uint16_t length) {
	uint8_t txdata[4 + length];
	uint8_t result = HAL_ERROR;

	txdata[0] = W25Q128_PAGE_PROGRAM;
	txdata[1] = address[0];
	txdata[2] = address[1];
	txdata[3] = address[2];

	for (int i = 0; i < length; i++) {
		txdata[i + 4] = data[i];
	}

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (HAL_SPI_Transmit(memory -> spi_handle, txdata, length + 4, W25Q128_TIMEOUT) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_erase_sector(W25Q128* memory, uint8_t* address) {
	uint8_t txdata[4] = {W25Q128_SECTOR_ERASE_4KB, address[0], address[1], address[2]};
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (HAL_SPI_Transmit(memory -> spi_handle, txdata, 4, W25Q128_TIMEOUT) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_chip_erase(W25Q128* memory) {
	uint8_t txdata = W25Q128_CHIP_ERASE;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	return result;
}

uint8_t W25Q128_global_block_lock(W25Q128* memory) {
	uint8_t txdata = W25Q128_GLOBAL_BLOCK_LOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_global_block_unlock(W25Q128* memory) {
	uint8_t txdata = W25Q128_GLOBAL_BLOCK_UNLOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_individual_block_lock(W25Q128* memory, uint8_t* address) {
	uint8_t txdata = W25Q128_INDIVIDUAL_BLOCK_LOCK;
	uint8_t result = HAL_ERROR;

	result = W25Q128_write_enable(memory);

	W25Q128_CS_LOW(memory);

	if (result == HAL_OK) {
		result = HAL_ERROR;

		if (HAL_SPI_Transmit(memory -> spi_handle, &txdata, 1, W25Q128_TIMEOUT) == HAL_OK) {
			result = HAL_OK;
		}
	}

	W25Q128_CS_HIGH(memory);

	W25Q128_write_disable(memory);

	return result;
}

uint8_t W25Q128_individual_block_unlock(W25Q128* memory, uint8_t* address) {
	return HAL_ERROR;
}
