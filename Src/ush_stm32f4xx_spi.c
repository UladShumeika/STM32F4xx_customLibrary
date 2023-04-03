/**
  ******************************************************************************
  * @file    ush_stm32f4xx_spi.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    16-January-2023
  * @brief   This file contains the implementation of functions for working with SPI.
  *
  * NOTE: This file is not a full-fledged SPI driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_spi.h"

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the SPIx peripheral according to the specified parameters in the USH_SPI_initDefaultTypeDef.
 * @note 	This function has default settings:
 *			- Master mode selected;
 *			- Full duplex selected;
 *			- 16bits data size selected;
 *			- Chip select pin software control selected;
 *			- MSB transmitted first;
 *			- CRC calculation disabled;
 *			- TI mode disable.
 * @param 	initStructure - A pointer to a USH_SPI_initDefaultTypeDef structure that contains the configuration
 * 							information for the specified SPI peripheral.
 * @retval	None.
 */
void SPI_init(USH_SPI_initDefaultTypeDef *initStructure)
{
	USH_GPIO_initTypeDef initGpioStructure = {0,};

	uint16_t temp;

	// Check parameters
	assert_param(IS_SPI_ALL_INSTANCE(initStructure->SPIx));
	assert_param(IS_SPI_PINSPACK(initStructure->PinsPack));
	assert_param(IS_SPI_BAUDRATE_PRESCALER(initStructure->BaudRatePrescaler));
	assert_param(IS_SPI_MODE(initStructure->Mode));
	assert_param(IS_SPI_FRAME_FORMAT(initStructure->FrameFormat));

	/* ----------------------- GPIO configuration -------------------------- */

	if(initStructure->SPIx == SPI1)
	{
		// SPI1 clock enable
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

		if(initStructure->PinsPack == SPI_PINSPACK_1) 	// Pins configuration according to pinsPack_1
		{
			// GPIOA clock enable
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

			// SPI1 GPIO pins pack 1 configuration
			// PA4 	   ------> SPI1_CS
			// PA5     ------> SPI1_SCK
			// PA6     ------> SPI1_MISO
			// PA7     ------> SPI1_MOSI
			initGpioStructure.GPIOx 		= GPIOA;
			initGpioStructure.Pin			= (GPIO_PIN_6 | GPIO_PIN_7);
			initGpioStructure.Mode			= GPIO_MODE_ALTERNATE_PP;
			initGpioStructure.Pull			= GPIO_NOPULL;
			initGpioStructure.Speed			= GPIO_SPEED_VERY_HIGH;
			initGpioStructure.Alternate		= GPIO_AF5_SPI1;
			GPIO_init(&initGpioStructure);

			initGpioStructure.Pin			= GPIO_PIN_5;

			if(initStructure->Mode == SPI_MODE_1 || initStructure->Mode == SPI_MODE_3)
			{
				initGpioStructure.Pull			= GPIO_PULLDOWN;
			} else
			{
				initGpioStructure.Pull			= GPIO_PULLUP;
			}

			GPIO_init(&initGpioStructure);

			initGpioStructure.Pin			= GPIO_PIN_4;
			initGpioStructure.Mode			= GPIO_MODE_OUTPUT_PP;
			initGpioStructure.Pull			= GPIO_PULLUP;
			GPIO_writeBits(GPIOA, GPIO_PIN_4, SET);
			GPIO_init(&initGpioStructure);
		} else											// Pins configuration according to pinsPack_2
		{
			// GPIOA and GPIOB clock enable
			RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);

			// SPI1 GPIO pins pack 2 configuration
			// PA15    ------> SPI1_CS
			// PB3     ------> SPI1_SCK
			// PB4     ------> SPI1_MISO
			// PB5     ------> SPI1_MOSI
			initGpioStructure.GPIOx 		= GPIOB;
			initGpioStructure.Pin			= (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
			initGpioStructure.Mode			= GPIO_MODE_ALTERNATE_PP;
			initGpioStructure.Pull			= GPIO_NOPULL;
			initGpioStructure.Speed			= GPIO_SPEED_VERY_HIGH;
			initGpioStructure.Alternate		= GPIO_AF5_SPI1;
			GPIO_init(&initGpioStructure);

			initGpioStructure.GPIOx 		= GPIOA;
			initGpioStructure.Pin			= GPIO_PIN_15;
			GPIO_init(&initGpioStructure);
		}
	}
	/* ----------------------- DMA configuration --------------------------- */


	/* ----------------------- SPI configuration --------------------------- */

	// Setting the default settings for CR1 register
	temp = initStructure->SPIx->CR1;
	temp |= (SPI_MASTER | SPI_DATASIZE | SPI_NSS);
	temp &= ~(SPI_DIRECTION_2LINES | SPI_CRC_CALCULATION);

	// Setting the settings from structure
	temp &= ~(SPI_BAUDRATE_MASK | SPI_MODE_MASK | SPI_FRAME_FORMAT_MASK);
	temp |= (initStructure->BaudRatePrescaler | initStructure->Mode | initStructure->FrameFormat);
	initStructure->SPIx->CR1 = temp;

	// Setting the default settings for CR2 register
	temp = initStructure->SPIx->CR2;
	temp &= ~(SPI_TI_MODE);
	initStructure->SPIx->CR2 = temp;
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function writes data to a register.
 * @param 	SPIx - A pointer to SPIx peripheral to be used where x is between 1 to 6.
 * @param 	reg - A register to write data to.
 * @param 	data - Data to be recorded.
 * @retval	None.
 */
void SPI_writeData(SPI_TypeDef *SPIx, uint8_t reg, uint8_t data)
{
	uint16_t temp;

	// Check parameters
	assert_param(IS_SPI_ALL_INSTANCE(SPIx));

	// Check if the SPI is already enabled
	if((SPIx->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		// Enable SPI peripheral
		SPIx->CR1 |= SPI_CR1_SPE;
	}

	while(!(SPIx->SR & SPI_SR_TXE));

	// Write in the DR register the data to be sent
	temp = (reg << 8) | data;
	SPIx->DR = temp;

	while(!(SPIx->SR & SPI_SR_RXNE));
	(void) SPI1->DR;
}

/**
  * @brief  Chip select (CS) pin switching.
  * @param	GPIOx - A pointer to GPIOx peripheral to be used where x is between A to F.
  * @param	csPin - The GPIO pin which is used as CS pin.
  * 				This parameter can be any value of @ref USH_GPIO_pins.
  * @param  state - The value to be written to the CS pin.
  * 				This parameter can be any value of @ref USH_SPI_csState.
  * @retval None.
  */
void SPI_csPin(GPIO_TypeDef *GPIOx, uint16_t csPin, USH_SPI_csState state)
{
	// Check parameters
	assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
	assert_param(IS_SPI_PIN_STATE(state));

	if(state == HIGH)
	{
		GPIO_writeBits(GPIOx, csPin, GPIO_PIN_SET);
	} else
	{
		GPIO_writeBits(GPIOx, csPin, GPIO_PIN_RESET);
	}
}
