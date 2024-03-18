/*
 * test.c
 *
 *  Created on: Mar 17, 2024
 *      Author: tommaso
 */

#include "test.h"

//FIXME DECLARE TRANSMIT AND RECEIVE FUNCTIONS TO BE USED IN BMP3_SPI_INIT

static int8_t bmp3_spi_init(struct bmp3_dev *dev) {
	int8_t result = BMP3_OK;

//	if (dev != NULL) {
//		dev_addr = 0;
//		dev->read = transmit();
//		dev->write = receive();
//		dev->intf = BMP3_SPI_INTF;
//
//		//dev->delay_us = bmp3_delay_us;
//		dev->intf_ptr = &dev_addr;
//	} else {
//		rslt = -1;
//	}

	return result;
}

void test_bmp390(struct bmp3_dev *dev) {

	int8_t result = 0;

	uint8_t settings_sel;

	struct bmp3_settings settings = { 0 };

	result = bmp3_spi_init(dev);

	if (result != BMP3_OK)	return;

	result = bmp3_init(dev);

	if (result != BMP3_OK)	return;
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	settings.int_settings.drdy_en = BMP3_DISABLE;

	settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
	settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;

	settings.odr_filter.odr = BMP3_ODR_200_HZ;

	settings.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
				   BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
				   BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;

	result = bmp3_set_sensor_settings(settings_sel, &settings, dev);

	if (result != BMP3_OK)	return;

	settings.op_mode = BMP3_MODE_NORMAL;
	result = bmp3_set_op_mode(&settings, dev);

	if (result != BMP3_OK)	return;

	//XXX read data for 100 times

	for (int i = 0; i < 100; i++) {
		struct bmp3_data data = { -1, -1 };
		struct bmp3_status status = { { 0 } };

		result = bmp3_get_status(&status, dev);

		if (result != BMP3_OK)	return;

		result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, dev);

		//XXX print data, or do something like that
	}

}

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
