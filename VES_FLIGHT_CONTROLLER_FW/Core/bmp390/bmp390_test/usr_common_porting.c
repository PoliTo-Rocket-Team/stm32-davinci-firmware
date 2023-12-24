#include "usr_common_porting.h"

extern SPI_HandleTypeDef hspi1;

uint8_t GTXBuffer[512], GRXBuffer[2048];

volatile uint8_t int1_flag = 0;
volatile uint8_t int2_flag = 0;

//void Enable_MCU_INT1_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT1_Pin */
//	GPIO_InitStruct.Pin = INT1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//}
//
//void Disable_MCU_INT1_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT1_Pin */
//	GPIO_InitStruct.Pin = INT1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//}
//
//void Enable_MCU_INT2_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin */
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
//  	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
//}
//
//void Disable_MCU_INT2_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin */
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
//  	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
//
//}

#if defined(FIFO_WM_INT)
extern volatile uint16_t bma456_fifo_ready;
#endif

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == GPIO_PIN_10) //INT1
//	{
//		printf("INT1 Triggered\r\n");
//		int1_flag = 1;
//#if defined(FIFO_WM_INT)
//		bma456_fifo_ready = 1;
//		#endif
//	} else if (GPIO_Pin == GPIO_PIN_3) //INT2
//	{
//		//printf("INT2 Triggered\r\n");
//		int2_flag = 1;
//#if defined(FIFO_WM_INT)
//		#if defined(USE_BMA456)
//		//bma456_fifo_ready = 1;
//		#endif
//	#endif
//	} else if (GPIO_Pin == GPIO_PIN_5) //DRDY_BMM150
//	{
//		printf("DRDY_BMM150 Triggered\r\n");
//	}
//}

void DelayUs(uint32_t Delay) {
	uint32_t i;

	while (Delay--) {
		for (i = 0; i < 84; i++) {
			;
		}
	}
}

void bmp3_delay_us(uint32_t period, void *intf_ptr) {
	uint32_t i;

	while (period--) {
		for (i = 0; i < 84; i++) {
			;
		}
	}
}

//void UART_Printf(uint8_t *buff, uint16_t size) {
//	//HAL_UART_Transmit_DMA(&huart2, buff, size);
//	HAL_UART_Transmit(&UART_HANDLE, buff, size, BUS_TIMEOUT);
//}

char chBuffer[256];
//#if 1
//void printf(char *format, ...) {
//#if defined(DEBUG_EN)
//	va_list ap;
//	//char timestamp[16];
//	va_start(ap, format);
//	vsnprintf(chBuffer, sizeof(chBuffer), format, ap);
//	//sprintf(timestamp, "[%d]", xTaskGetTickCount()); //xTaskGetTickCountFromISR()
//	//Printf((uint8_t *)timestamp, strlen(timestamp));
//	UART_Printf((uint8_t*) chBuffer, strlen(chBuffer));
//	va_end(ap);
//#endif
//}

//#else
//void printf(char *format, ...)
//{
//#if defined(DEBUG_EN)
//    va_list ap;
//    //char timestamp[16];
//    va_start(ap, format);
//    vsnprintf(GTXBuffer, sizeof(GTXBuffer), format, ap);
//    //sprintf(timestamp, "[%d]", xTaskGetTickCount()); //xTaskGetTickCountFromISR()
//    //Printf((uint8_t *)timestamp, strlen(timestamp));
//    UART_Printf((uint8_t *)GTXBuffer,strlen(GTXBuffer));
//    va_end(ap);
//#endif
//}
//#endif

#if defined(USE_BOSCH_SENSOR_API)

/*******************************************************************************
 * Function Name  : SPI_Read
 * Description    : Read data from slave device.
 * Input          : register address, destination buffer, num of char read, interface ptr
 * Output         : None
 * Return         : number of bytes transmitted
 *******************************************************************************/
int8_t SensorAPI_SPIx_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr) {
	GTXBuffer[0] = subaddress | 0x80;

	CS_BMP390_LOW;

	HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, ReadNumbr + 1, BUS_TIMEOUT); // timeout 1000msec;
	while (SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

	CS_BMP390_HIGH;

	memcpy(pBuffer, GRXBuffer + 1, ReadNumbr);

	return 0;
}

int8_t SensorAPI_SPIx_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr) {
	GTXBuffer[0] = subaddress & 0x7F;
	memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);

	CS_BMP390_LOW;

	HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, WriteNumbr + 1, BUS_TIMEOUT); // send register address + write data
	while (SPI_HANDLE.State == HAL_SPI_STATE_BUSY); // wait for xmission complete

	CS_BMP390_HIGH;

	return 0;
}

#endif

