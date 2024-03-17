/*
 * test.c
 *
 *  Created on: Mar 17, 2024
 *      Author: tommaso
 */

#include "main.h"

extern SPI_HandleTypeDef hspi3;

void test_w25q128(W25Q128* flash) {
	int result = HAL_ERROR;
	uint8_t read_data[4] = {0};
	uint8_t read_page[4096] = {0};
	uint8_t addr[3] = {0};
	uint8_t data[4] = {0x88, 0x44, 0x22, 0x11};

	/* FLASH INITIALIZATION */
	result = W25Q128_init(flash, FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, &hspi3);

	if (result != HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(100);

	HAL_GPIO_WritePin(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin, GPIO_PIN_SET);

	W25Q128_chip_erase(flash);

	uint8_t reg1 = 1;

	do {
	  W25Q128_read_statusreg1(flash, &reg1);
	} while (reg1 & 1);

	HAL_GPIO_WritePin(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin, GPIO_PIN_RESET);

	W25Q128_read_data(flash, addr, read_page, 4096);

	for (int i = 0; i < 4096; i++) {
	  if (read_page[i] != 255) {
		  HAL_GPIO_WritePin(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin, GPIO_PIN_SET);
	  }
	}

	W25Q128_read_id(flash);
	HAL_Delay(100);
	W25Q128_read_manufacturer_dev_id(flash);
	HAL_Delay(100);
	W25Q128_read_JEDEC_id(flash);
	HAL_Delay(100);

	/* XXX it seems that to program the flash
	* we need to write to previously erased cells (0xFF) content
	*/
	/* ERASE THE CONTENT OF THE BLOCK OF 4KB STARTING AT 0x0 */
	W25Q128_erase_sector(flash, addr);
	do {
	  W25Q128_read_statusreg1(flash, &reg1);
	} while (reg1 & 1);
	W25Q128_read_data(flash, addr, read_data, 4);
	HAL_Delay(100);
	/* WRITE 4 BYTES OF DATA STARTING FROM 0x0 */
	W25Q128_write_data(flash, addr, data, 4);
	do {
	  W25Q128_read_statusreg1(flash, &reg1);
	} while (reg1 & 1);
	/* READ 4 BYTES OF DATA STARTING FROM 0x0 */
	W25Q128_read_data(flash, addr, read_data, 4);
	HAL_Delay(100);

	W25Q128_erase_sector(flash, addr);
	do {
	  W25Q128_read_statusreg1(flash, &reg1);
	} while ((reg1 & 0x11) == 0x11);
	W25Q128_read_data(flash, addr, read_data, 4);
	HAL_Delay(100);

	W25Q128_chip_erase(flash);

	do {
		W25Q128_read_statusreg1(flash, &reg1);
	} while (reg1 & 1);

	W25Q128_read_data(flash, addr, read_data, 4);

	for (int i = 0; i < 64; i++) {
	  data[0] = data[1] = data[2] = data[3] = (uint8_t) (i % 254);
	  W25Q128_write_data(flash, addr, data, 4);
	  HAL_Delay(50);
	  W25Q128_read_data(flash, addr, read_data, 4);
	  HAL_Delay(50);
	  for (int j = 0; j < 4; j++) {
		  if (data[j] != read_data[j]) {
			  HAL_GPIO_WritePin(DEBUG_LED_FLASH_GPIO_Port, DEBUG_LED_FLASH_Pin, GPIO_PIN_SET);
			  Error_Handler();
		  }
	  }

	  addr[0] += 4;
	}

}
