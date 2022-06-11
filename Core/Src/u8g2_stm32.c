/*
 * u8g2_stm32f4.c
 *
 */

#include "main.h"
#include "u8g2/u8g2.h"

#  if __has_include("stm32f0xx_hal.h")
#    include "stm32f0xx_hal.h"
#  elif  __has_include("stm32f1xx_hal.h")
#    include "stm32f1xx_hal.h"
#  elif  __has_include("stm32f3xx_hal.h")
#    include "stm32f3xx_hal.h"
#  elif  __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  endif

#ifdef __cplusplus
 extern "C" {
#endif

#define DEVICE_ADDRESS 	0x3C
#define TX_TIMEOUT		100

//#define spiMode
#define i2cMode

#ifdef spiMode
 	 extern SPI_HandleTypeDef hspi2;
#endif

#ifdef i2cMode
extern I2C_HandleTypeDef hi2c2;
#endif

uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	/* STM32 supports HW SPI, Remove unused cases like U8X8_MSG_DELAY_XXX & U8X8_MSG_GPIO_XXX */
	switch(msg)
	{
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		/* Insert codes for initialization */
		break;
	case U8X8_MSG_DELAY_MILLI:
		/* ms Delay */
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_CS:
		/* Insert codes for SS pin control */
#ifdef spiMode
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, arg_int);
		break;
#endif
	case U8X8_MSG_GPIO_DC:
		/* Insert codes for DC pin control */
#ifdef spiMode
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
#endif
		break;
	case U8X8_MSG_GPIO_RESET:
		/* Insert codes for RST pin control */
#ifdef spiMode
		HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, arg_int);
#endif
		break;
	}
	return 1;
}

uint8_t u8x8_byte_stm32_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch(msg) {
	case U8X8_MSG_BYTE_SEND:
		/* Insert codes to transmit data */
#ifdef spiMode
		if(HAL_SPI_Transmit(&hspi1, arg_ptr, arg_int, TX_TIMEOUT) != HAL_OK) return 0;
		break;
#endif
	case U8X8_MSG_BYTE_INIT:
		/* Insert codes to begin SPI transmission */
		break;
	case U8X8_MSG_BYTE_SET_DC:
		/* Control DC pin, U8X8_MSG_GPIO_DC will be called */
		u8x8_gpio_SetDC(u8x8, arg_int);
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		/* Select slave, U8X8_MSG_GPIO_CS will be called */
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
		HAL_Delay(1);
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_Delay(1);
		/* Insert codes to end SPI transmission */
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
		break;
	default:
		return 0;
	}
	return 1;
}

uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buffer[32];
	static uint8_t buf_idx;
	uint8_t *data;

	switch(msg)
	{
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while( arg_int > 0 )
		{
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		/* add your custom code to init i2c subsystem */
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
#ifdef i2cMode
		if(HAL_I2C_Master_Transmit(&hi2c2, (DEVICE_ADDRESS << 1), buffer, buf_idx, TX_TIMEOUT) != HAL_OK) return 0;
#endif
		break;
	default:
		return 0;
	}
	return 1;
}




