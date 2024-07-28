/*
 * test.h
 *
 *  Created on: Mar 17, 2024
 *      Author: tommaso
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include "W25Q128.h"
#include "main.h"

//struct bmp3_dev;

extern SPI_HandleTypeDef hspi3;

//extern SPI_HandleTypeDef hspi1;

void test_w25q128(W25Q128*);

//void test_bmp390(struct bmp3_dev *);

#endif /* INC_TEST_H_ */
