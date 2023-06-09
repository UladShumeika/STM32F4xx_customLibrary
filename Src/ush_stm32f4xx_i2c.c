/**
  ******************************************************************************
  * @file    ush_stm32f4xx_i2c.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    27 May 2023
  * @brief	 This file contains the implementation of functions for working with
  * 		 I2C bus.
  *
  * NOTE: This driver is not a full-fledged I2C driver, but contains only some of
  * 	  the functions were necessary at the time of development of this driver.
  * 	  However, if necessary, the function of this driver will be expanded.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_i2c.h"
#include "stddef.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

/*!
 * @name i2c_checking_pclk_frequency
 * @{
 */
#define PRJ_I2C_MIN_PCLK_FREQ_STANDARD					(2000000U)	/*!< 2 MHz */
#define PRJ_I2C_MIN_PCLK_FREQ_FAST						(4000000U)	/*!< 4 MHz */
#define PRJ_I2C_FREQ_STANDARD							(100000U)	/*!< 100 kHz */
#define PRJ_I2C_FREQ_FAST								(400000U)	/*!< 400 kHz */
#define PRJ_I2C_1_MHZ									(1000000U)  /*!< 1 MHz */

/*! @} */

/*!
 * @name i2c_calculation_clock_control_register
 * @{
 */
#define PRJ_I2C_DUTYCYCLE_2_COEF						(3U)	/*!< The coefficient for duty cycle 2 */
#define PRJ_I2C_DUTYCYCLE_16_9_COEF						(25U)	/*!< The coefficient for duty cycle 16/9 */
#define PRJ_I2C_MIN_CCR_VALUE_FOR_SM					(4U)	/*!< The minimum CCR value for standard mode */
#define PRJ_I2C_MIN_CCR_VALUE_FOR_FM					(1U)	/*!< The minimum CCR value for fast mode */

/*! @} */

/*!
 * @name i2c_timeouts
 * @{
 */
#define PRJ_I2C_TIMEOUT_FLAG							(35U)	/*!< 35 ms */
#define PRJ_I2C_TIMEOUT_BUSY_FLAG						(25U)	/*!< 25 ms */
#define PRJ_I2C_TIMEOUT_STOP_FLAG						(5U)	/*!< 5 ms */

/*! @} */

/*!
 * @name i2c_flags
 * @{
 */
#define PRJ_I2C_FLAG_OVR                    			(0x00010800U)	/*!< OVR flag */
#define PRJ_I2C_FLAG_AF                     			(0x00010400U)	/*!< AF flag */
#define PRJ_I2C_FLAG_ARLO                   			(0x00010200U)	/*!< ARLO flag */
#define PRJ_I2C_FLAG_BERR                   			(0x00010100U)	/*!< BERR flag */
#define PRJ_I2C_FLAG_TXE                    			(0x00010080U)	/*!< TXE flag */
#define PRJ_I2C_FLAG_RXNE                   			(0x00010040U)   /*!< TXE flag */
#define PRJ_I2C_FLAG_STOPF                  			(0x00010010U)	/*!< STOPF flag */
#define PRJ_I2C_FLAG_ADD10                  			(0x00010008U)	/*!< ADD10 flag */
#define PRJ_I2C_FLAG_BTF                    			(0x00010004U)	/*!< BTF flag */
#define PRJ_I2C_FLAG_ADDR                   			(0x00010002U)	/*!< ADDR flag */
#define PRJ_I2C_FLAG_SB                     			(0x00010001U)	/*!< SB flag */
#define PRJ_I2C_FLAG_DUALF                  			(0x00100080U)	/*!< DUALF flag */
#define PRJ_I2C_FLAG_GENCALL                			(0x00100010U)	/*!< GENCALL flag */
#define PRJ_I2C_FLAG_TRA                    			(0x00100004U)	/*!< TRA flag */
#define PRJ_I2C_FLAG_BUSY                   			(0x00100002U)	/*!< BUSY flag */
#define PRJ_I2C_FLAG_MSL                    			(0x00100001U)	/*!< MSL flag */
#define PRJ_I2C_FLAG_MASK								(0x0000FFFFU)	/*!< Mask for flags */

/*! @} */

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static uint32_t i2c_checking_pclk_frequency(uint32_t pclk1, uint32_t i2c_clock_speed);
static uint32_t i2c_calc_rise_time(uint32_t freq_range, uint32_t i2c_clock_speed);
static uint32_t i2c_ccr_calc(uint32_t pclk1, uint32_t i2c_clock_speed, uint32_t duty_cycle);

static void i2c_clear_addr_flag(I2C_TypeDef* p_i2c);
static uint32_t i2c_flag_get(I2C_TypeDef* p_i2c, uint32_t flag);
static uint32_t i2c_wait_on_reset_flags(I2C_TypeDef* p_i2c, uint32_t flag, uint32_t timeout);
static uint32_t i2c_wait_on_set_flags(I2C_TypeDef* p_i2c, uint32_t flag, uint32_t timeout);

static uint32_t i2c_request_memory_write(prj_i2c_data_request_t* request_write);
static uint32_t i2c_request_memory_read(prj_i2c_data_request_t* request_read);

static void i2c_dma_complete(void* p_controls_peripherals);
static void i2c_dma_error(void* p_controls_peripherals);

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize i2c peripherals.
 *
 * This function is used to initialize i2c peripherals.
 *
 * @param[in] p_i2c_init	A pointer to i2c initialization structure.
 *
 * @return @ref PRJ_STATUS_OK if I2C initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
uint32_t prj_i2c_init(prj_i2c_init_t* p_i2c_init)
{
	uint32_t status = PRJ_STATUS_OK;
	uint32_t pclk1 = 0;
	uint32_t freq_range = 0;
	uint32_t rise_time = 0;
	uint32_t ccr_calc = 0;

	/* Check the pointer */
	if(p_i2c_init == NULL)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		; /* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Disable the selected I2C peripheral */
		p_i2c_init->p_i2c->CR1 &= ~I2C_CR1_PE;

		/* Reset I2C peripheral */
		p_i2c_init->p_i2c->CR1 |= I2C_CR1_SWRST;
		p_i2c_init->p_i2c->CR1 &= ~I2C_CR1_SWRST;

		/* Check the minimum allowed PCLK1 frequency */
		pclk1 = RCC_getPCLK1freq();
		status = i2c_checking_pclk_frequency(pclk1, p_i2c_init->clock_speed);
	}
	else
	{
		; /* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Calculate frequency range */
		freq_range = pclk1 / PRJ_I2C_1_MHZ;

		/* Configure p_i2c: Frequency range */
		p_i2c_init->p_i2c->CR2 = freq_range;

		/* Configure p_i2c: Rise Time */
		rise_time = i2c_calc_rise_time(freq_range, p_i2c_init->clock_speed);
		p_i2c_init->p_i2c->TRISE |= rise_time;

		/* Configure p_i2c: Speed */
		ccr_calc = i2c_ccr_calc(pclk1, p_i2c_init->clock_speed, p_i2c_init->duty_cycle);
		p_i2c_init->p_i2c->CCR |= ccr_calc;

		/* Configure p_i2c: Generalcall and NoStretch mode */
		p_i2c_init->p_i2c->CR1 |= (p_i2c_init->general_call_mode | p_i2c_init->nostretch_mode);

		/* Configure p_i2c: Own Address1 and addressing mode */
		p_i2c_init->p_i2c->OAR1 |= (p_i2c_init->addressing_mode | p_i2c_init->own_address_1);

		/* Configure p_i2c: Dual mode and Own Address2 */
		p_i2c_init->p_i2c->OAR2 |= (p_i2c_init->dual_address_mode |  p_i2c_init->own_address_2);

		/* Enable the selected I2C peripheral */
		p_i2c_init->p_i2c->CR1 |= I2C_CR1_PE;
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Write an amount of data with dma to a specific memory address.
 *
 * This function is used to write an amount of data with dma to a specific memory address.
 *
 * @param p_i2c_tx		A pointer to an i2c transmit structure that contains all
 * 						the necessary information to transfer data.
 *
 * @return @ref PRJ_STATUS_OK if the data is written successfully.
 * @return @ref PRJ_STATUS_ERROR if a pointer is not passed either to the structure itself
 * 		   or to the DMA peripheral or the size of the transmitted data is 0.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_i2c_write_dma(prj_i2c_transmission_t* p_i2c_tx)
{
	uint32_t status = PRJ_STATUS_OK;
	prj_dma_config_t dma_config = {0};
	prj_i2c_data_request_t request_write = {0};

	/* Check the pointer */
	if(p_i2c_tx == NULL)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		; /* DO NOTHING */
	}

	if((p_i2c_tx->data_size != 0U) || p_i2c_tx->p_dma != NULL)
	{
		/* Wait until BUSY flag is reset */
		status = i2c_wait_on_reset_flags(p_i2c_tx->p_i2c, PRJ_I2C_FLAG_BUSY, PRJ_I2C_TIMEOUT_BUSY_FLAG);

		if(status == PRJ_STATUS_OK)
		{
			/* Disable Pos */
			p_i2c_tx->p_i2c->CR1 &= ~I2C_CR1_POS;

			/* Set DMA callbacks */
			p_i2c_tx->p_dma->p_complete_callback 	= i2c_dma_complete;
			p_i2c_tx->p_dma->p_error_callback 		= i2c_dma_error;

			/* Config DMA peripheral */
			dma_config.p_dma_stream 			= p_i2c_tx->p_dma->p_dma_stream;
			dma_config.data_size				= p_i2c_tx->data_size;
			dma_config.destination_address 		= (uint32_t)&p_i2c_tx->p_i2c->DR;
			dma_config.sourse_address			= (uint32_t)p_i2c_tx->p_data;
			dma_config.direction				= PRJ_DMA_MEMORY_TO_PERIPH;
			status = prj_dma_config(&dma_config);
		}
		else
		{
			; /* DO NOTHING */
		}

		if(status == PRJ_STATUS_OK)
		{
			/* Send request memory write */
			request_write.p_i2c				= p_i2c_tx->p_i2c;
			request_write.dev_address 		= p_i2c_tx->dev_address;
			request_write.mem_address 		= p_i2c_tx->mem_address;
			request_write.mem_address_size 	= p_i2c_tx->mem_address_size;
			status = i2c_request_memory_write(&request_write);
		}
		else
		{
			/* Clear DMA callbacks */
			p_i2c_tx->p_dma->p_complete_callback 	= NULL;
			p_i2c_tx->p_dma->p_error_callback 		= NULL;
		}

		if(status == PRJ_STATUS_OK)
		{
			/* Clear ADDR flag */
			i2c_clear_addr_flag(p_i2c_tx->p_i2c);

			/* Enable ERR interrupt */
			p_i2c_tx->p_i2c->CR2 |= I2C_CR2_ITERREN;

			/* Enable DMA request */
			p_i2c_tx->p_i2c->CR2 |= I2C_CR2_DMAEN;
		}
		else
		{
			/* Disable I2C peripheral */
			p_i2c_tx->p_i2c->CR1 &= ~I2C_CR1_PE;
		}
	}
	else
	{
		status = PRJ_STATUS_ERROR;
	}

	return status;
}

/*!
 * @brief Read an amount of data with dma to a specific memory address.
 *
 * This function is used to read an amount of data with dma to a specific memory address.
 *
 * @param p_i2c_rx	A pointer to an i2c reception structure that contains all
 * 							the necessary information to receive data.
 *
 * @return @ref PRJ_STATUS_OK if the data is read successfully.
 * @return @ref PRJ_STATUS_ERROR if a pointer is not passed either to the structure itself
 * 		   or to the DMA peripheral or the size of the received data is 0.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_i2c_read_dma(prj_i2c_transmission_t* p_i2c_rx)
{
	uint32_t status = PRJ_STATUS_OK;
	prj_dma_config_t dma_config = {0};
	prj_i2c_data_request_t request_read = {0};

	/* Check pointers and the data size */
	if((p_i2c_rx == NULL) || (p_i2c_rx->p_dma->p_dma_stream == NULL) || (p_i2c_rx->data_size == 0U))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		; /* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Wait until BUSY flag is reset */
		status = i2c_wait_on_reset_flags(p_i2c_rx->p_i2c, PRJ_I2C_FLAG_BUSY, PRJ_I2C_TIMEOUT_BUSY_FLAG);

		/* Enable I2C peripherals if they are disabled */
		if((p_i2c_rx->p_i2c->CR1 & I2C_CR1_PE) == 0U)
		{
			p_i2c_rx->p_i2c->CR1 |= I2C_CR1_PE;
		}
		else
		{
			; /* DO NOTHING */
		}

		/* Disable Pos */
		p_i2c_rx->p_i2c->CR1 &= ~I2C_CR1_POS;

		/* Set DMA callbacks */
		p_i2c_rx->p_dma->p_complete_callback 	= i2c_dma_complete;
		p_i2c_rx->p_dma->p_error_callback 		= i2c_dma_error;

		/* Config DMA peripheral */
		dma_config.p_dma_stream 			= p_i2c_rx->p_dma->p_dma_stream;
		dma_config.data_size				= p_i2c_rx->data_size;
		dma_config.destination_address 		= (uint32_t)p_i2c_rx->p_data;
		dma_config.sourse_address			= (uint32_t)&p_i2c_rx->p_i2c->DR;
		dma_config.direction				= PRJ_DMA_PERIPH_TO_MEMORY;
		status = prj_dma_config(&dma_config);
	}
	else
	{
		; /* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Send request memory read */
		request_read.p_i2c				= p_i2c_rx->p_i2c;
		request_read.dev_address 		= p_i2c_rx->dev_address;
		request_read.mem_address 		= p_i2c_rx->mem_address;
		request_read.mem_address_size 	= p_i2c_rx->mem_address_size;
		status = i2c_request_memory_read(&request_read);
	}
	else
	{
		; /* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		if(p_i2c_rx->data_size == 1U)
		{
			/* Disable Acknowledge */
			p_i2c_rx->p_i2c->CR1 &= ~I2C_CR1_ACK;
		}
		else
		{
			p_i2c_rx->p_i2c->CR2 |= I2C_CR2_LAST;
		}

		/* Clear ADDR flag */
		i2c_clear_addr_flag(p_i2c_rx->p_i2c);

		/* Enable ERR interrupt */
		p_i2c_rx->p_i2c->CR2 |= I2C_CR2_ITERREN;

		/* Enable DMA request */
		p_i2c_rx->p_i2c->CR2 |= I2C_CR2_DMAEN;
	}
	else
	{
		/* Disable Acknowledge */
		p_i2c_rx->p_i2c->CR1 &= ~I2C_CR1_ACK;

		/* Disable I2C peripherals */
		p_i2c_rx->p_i2c->CR1 &= ~I2C_CR1_PE;
	}

	return status;
}

/*!
 * @brief I2C TX/RX completed callbacks.
 *
 * @note This function should not be modified, when the callback is needed,
 * 		 the prj_i2c_complete_callback could be implemented in the user file.
 *
 * @param p_i2c		A pointer to I2Cx peripheral to be used where x is from 1 to 3.
 *
 * @return None.
 */
__WEAK void prj_i2c_complete_callback(I2C_TypeDef* p_i2c)
{
	macro_prj_common_unused(p_i2c);
}

/*!
 * @brief i2c tx/rx error callbacks.
 *
 * @note This function should not be modified, when the callback is needed,
 * 		 the prj_i2c_error_callback could be implemented in the user file.
 *
 * @param p_i2c		A pointer to i2c peripheral to be used.
 *
 * @return None.
 */
__WEAK void prj_i2c_error_callback(I2C_TypeDef* p_i2c)
{
	macro_prj_common_unused(p_i2c);
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Check pclk frequency for i2c peripherals.
 *
 * This function is used to check pclk frequency for i2c peripherals.
 *
 * @param[in] pclk1			  	pclk1 clock frequency.
 * @param[in] i2c_clock_speed	i2c peripheral clock speed.
 *
 * @return @ref PRJ_STATUS_OK if PCLK1 clock frequency is within the allowable value.
 * @return @ref PRJ_STATUS_ERROR if PCLK1 clock frequency isn't within the allowable value.
 */
static uint32_t i2c_checking_pclk_frequency(uint32_t pclk1, uint32_t i2c_clock_speed)
{
	uint32_t status = PRJ_STATUS_OK;

	if(i2c_clock_speed <= PRJ_I2C_FREQ_STANDARD)
	{
		if(pclk1 < PRJ_I2C_MIN_PCLK_FREQ_STANDARD)
		{
			status = PRJ_STATUS_ERROR;
		}
		else
		{
			; /* DO NOTHING */
		}
	}
	else /* for PRJ_I2C_FREQ_FAST */
	{
		if(pclk1 < PRJ_I2C_MIN_PCLK_FREQ_FAST)
		{
			status = PRJ_STATUS_ERROR;
		}
		else
		{
			; /* DO NOTHING */
		}
	}

	return status;
}

/*!
 * @brief Calculate rise time for i2c peripherals.
 *
 * This function is used to calculate rise time.
 *
 * @param[in] freq_range		frequency range.
 * @param[in] i2c_clock_speed 	i2c peripheral clock speed.
 *
 * @return rise_time.
 */
static uint32_t i2c_calc_rise_time(uint32_t freq_range, uint32_t i2c_clock_speed)
{
	uint32_t rise_time = 0;

	if(i2c_clock_speed <= PRJ_I2C_FREQ_STANDARD)
	{
		rise_time = freq_range + 1U;
	}
	else /* for PRJ_I2C_FREQ_FAST */
	{
		rise_time = (((freq_range * 300U) / 1000U) + 1U);
	}

	return rise_time;
}

/*!
 * @brief Calculate the ccr (clock control register) value.
 *
 * This function is used to calculate the CCR value taking into account the selected i2c speed.
 *
 * @param[in] pclk1				pclk1 clock frequency.
 * @param[in] i2c_clock_speed	i2c peripheral clock speed.
 * @param[in] duty_cycle		i2c fast mode duty cycle.
 *
 * @return The ccr value.
 */
static uint32_t i2c_ccr_calc(uint32_t pclk1, uint32_t i2c_clock_speed, uint32_t duty_cycle)
{
	uint32_t i2c_ccr_calc = 0;

	if(i2c_clock_speed <= PRJ_I2C_FREQ_STANDARD)
	{
		i2c_ccr_calc = ((pclk1 - 1U) / (((i2c_clock_speed) * 2U) + 1U) & I2C_CCR_CCR);

		if(i2c_ccr_calc < PRJ_I2C_MIN_CCR_VALUE_FOR_SM)
		{
			i2c_ccr_calc = PRJ_I2C_MIN_CCR_VALUE_FOR_SM;
		}
		else
		{
			; /* DO NOTHING */
		}
	}
	else /* for PRJ_I2C_FREQ_FAST */
	{
		if(duty_cycle == PRJ_I2C_DUTYCYCLE_2)
		{
			i2c_ccr_calc = (((pclk1 - 1U) / (i2c_clock_speed * PRJ_I2C_DUTYCYCLE_2_COEF)) + 1U) & I2C_CCR_CCR;
		}
		else
		{
			i2c_ccr_calc = ((((pclk1 - 1U) / (i2c_clock_speed * PRJ_I2C_DUTYCYCLE_16_9_COEF)) + 1U) & I2C_CCR_CCR) | PRJ_I2C_DUTYCYCLE_16_9;
		}

		if((i2c_ccr_calc & I2C_CCR_CCR) < PRJ_I2C_MIN_CCR_VALUE_FOR_FM)
		{
			i2c_ccr_calc = PRJ_I2C_MIN_CCR_VALUE_FOR_FM;
		}
		else
		{
			i2c_ccr_calc = i2c_ccr_calc | I2C_CCR_FS;
		}
	}

	return i2c_ccr_calc;
}

/*!
 * @brief Clear addr flag.
 *
 * This function is used to clear addr flag.
 *
 * @param p_i2c		A pointer to i2c peripheral.
 *
 * @return None.
 */
static void i2c_clear_addr_flag(I2C_TypeDef* p_i2c)
{
	uint32_t temp = 0U;

	temp = p_i2c->SR1;
	temp = p_i2c->SR2;
	macro_prj_common_unused(temp);
}

/*!
 * @brief Get i2c flag.
 *
 * This function is used to get i2c flag.
 *
 * @param[in] p_i2c		A pointer to i2c peripheral.
 * @param[in] flag		i2c flag. This parameter can be a value of @ref i2c_flags.
 *
 * @return @ref PRJ_FLAG_SET if I2C flag set.
 * @return @ref PRJ_FLAG_RESET if I2C flag reset.
 */
static uint32_t i2c_flag_get(I2C_TypeDef* p_i2c, uint32_t flag)
{
	uint32_t flag_status = PRJ_FLAG_RESET;

	if((flag >> 16U) == 0x01U)
	{
		if((p_i2c->SR1) & ((flag) & (PRJ_I2C_FLAG_MASK)))
		{
			flag_status = PRJ_FLAG_SET;
		}
		else
		{
			flag_status = PRJ_FLAG_RESET;
		}
	}
	else
	{
		if((p_i2c->SR2) & ((flag) & (PRJ_I2C_FLAG_MASK)))
		{
			flag_status = PRJ_FLAG_SET;
		}
		else
		{
			flag_status = PRJ_FLAG_RESET;
		}
	}

	return flag_status;
}

/*!
 * @brief Wait on reset i2c flag.
 *
 * This function is used to wait on reset the specified i2c flag.
 *
 * @param[in] p_i2c		A pointer to i2c peripheral.
 * @param[in] flag		i2c flag. This parameter can be a value of @ref i2c_flags.
 * @param[in] timeout 	Timeout for i2c flags.
 *
 * @return @ref PRJ_STATUS_OK if the specified flag is reset.
 * @return @ref PRJ_STATUS_TIMEOUT if the specified flag is not reset and the timeout has passed.
 */
static uint32_t i2c_wait_on_reset_flags(I2C_TypeDef* p_i2c, uint32_t flag, uint32_t timeout)
{
	uint32_t status 		= PRJ_STATUS_OK;
	uint32_t start_ticks	= MISC_timeoutGetTick();;
	uint32_t past_ticks 	= 0U;
	uint32_t flag_status 	= PRJ_FLAG_SET;

	do
	{
		flag_status = i2c_flag_get(p_i2c, flag);

		past_ticks = MISC_timeoutGetTick() - start_ticks;
		if(past_ticks > timeout)
		{
			status = PRJ_STATUS_TIMEOUT;
		}
		else
		{
			; /* DO NOTHING */
		}

	} while((status == PRJ_STATUS_OK) && (flag_status == PRJ_FLAG_SET));

	return status;
}

/*!
 * @brief Wait on set i2c flag.
 *
 * This function is used to wait on set the specified i2c flag.
 *
 * @param[in] p_i2c		A pointer to i2c peripheral.
 * @param[in] flag		I2C flag. This parameter can be a value of @ref i2c_flags.
 * @param[in] timeout 	Timeout for i2c flags.
 *
 * @return @ref PRJ_STATUS_OK if the specified flag is set.
 * @return @ref PRJ_STATUS_TIMEOUT if the specified flag is not set and the timeout has passed.
 * @return @ref PRJ_STATUS_ERROR if the acknowledge failure is set while waiting for the specified flag.
 */
static uint32_t i2c_wait_on_set_flags(I2C_TypeDef* p_i2c, uint32_t flag, uint32_t timeout)
{
	uint32_t status 		= PRJ_STATUS_OK;
	uint32_t start_ticks	= MISC_timeoutGetTick();;
	uint32_t past_ticks 	= 0U;
	uint32_t flag_status 	= PRJ_FLAG_RESET;
	uint32_t flag_af		= PRJ_FLAG_RESET;

	do
	{
		flag_status = i2c_flag_get(p_i2c, flag);
		flag_af	= i2c_flag_get(p_i2c, PRJ_I2C_FLAG_AF);

		past_ticks = MISC_timeoutGetTick() - start_ticks;
		if(past_ticks > timeout)
		{
			status = PRJ_STATUS_TIMEOUT;
		}
		else
		{
			; /* DO NOTHING */
		}

	} while((status == PRJ_STATUS_OK) && ((flag_status == PRJ_FLAG_RESET) && (flag_af) == PRJ_FLAG_RESET));

	if(flag_af != PRJ_FLAG_RESET)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Send a write request to the specified address.
 *
 * This function is used to send a write request to the specified address of the target device.
 *
 * @param[in] request_write		A pointer to a structure that contains information about the device
 * 								and the destination address.
 *
 * @return @ref PRJ_STATUS_OK if the request was sent successfully.
 * @return @ref PRJ_STATUS_ERROR if there are any problems with the input parameters.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
static uint32_t i2c_request_memory_write(prj_i2c_data_request_t* request_write)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Check the pointer */
	if((request_write == NULL) || (request_write->p_i2c == NULL))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Generate start */
		request_write->p_i2c->CR1 |= I2C_CR1_START;

		/* Wait until SB flag is set */
		status = i2c_wait_on_set_flags(request_write->p_i2c, PRJ_I2C_FLAG_SB, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Send slave address with write request */
		request_write->p_i2c->DR = (request_write->dev_address << 1U) & (uint8_t)(~I2C_OAR1_ADD0);

		/* Wait until ADDR flag is set */
		status = i2c_wait_on_set_flags(request_write->p_i2c, PRJ_I2C_FLAG_ADDR, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Clear ADDR flag */
		i2c_clear_addr_flag(request_write->p_i2c);

		/* Wait until TXE flag is set */
		status = i2c_wait_on_set_flags(request_write->p_i2c, PRJ_I2C_FLAG_TXE, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		if(request_write->mem_address_size == PRJ_I2C_MEM_ADDRESS_SIZE_8BIT)
		{
			/* Send LSB of memory address */
			request_write->p_i2c->DR = (uint8_t)(request_write->mem_address & 0x00FFU);
		}
		else
		{
			/* Send MSB of memory address */
			request_write->p_i2c->DR = (uint8_t)(request_write->mem_address & 0xFF00U);

			/* Wait until TXE flag is set */
			status = i2c_wait_on_set_flags(request_write->p_i2c, PRJ_I2C_FLAG_TXE, PRJ_I2C_TIMEOUT_FLAG);

			if(status == PRJ_STATUS_OK)
			{
				/* Send LSB of memory address */
				request_write->p_i2c->DR = (uint8_t)(request_write->mem_address & 0x00FFU);
			}
			else
			{
				/* Generate stop */
				request_write->p_i2c->CR1 |= I2C_CR1_STOP;
			}
		}

		/* Wait until TXE flag is set */
		status = i2c_wait_on_set_flags(request_write->p_i2c, PRJ_I2C_FLAG_TXE, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* Generate stop */
		request_write->p_i2c->CR1 |= I2C_CR1_STOP;
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Clear ADDR flag */
		i2c_clear_addr_flag(request_write->p_i2c);
	}
	else
	{
		/* Generate stop */
		request_write->p_i2c->CR1 |= I2C_CR1_STOP;
	}

	return status;
}

/*!
 * @brief Send a read request to the specified address.
 *
 * This function is used to send a read request to the specified address of the target device.
 *
 * @param[in] request_read		A pointer to a structure that contains information about the device
 * 								and the destination address.
 *
 * @return @ref PRJ_STATUS_OK if the request was sent successfully.
 * @return @ref PRJ_STATUS_ERROR if there are any problems with the input parameters.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
static uint32_t i2c_request_memory_read(prj_i2c_data_request_t* request_read)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Check the pointers */
	if((request_read == NULL) && (request_read->p_i2c == NULL))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Enable Acknowledge */
		request_read->p_i2c->CR1 |= I2C_CR1_ACK;

		/* Generate start */
		request_read->p_i2c->CR1 |= I2C_CR1_START;

		/* Wait until SB flag is set */
		status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_SB, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Send slave address with write request */
		request_read->p_i2c->DR = (request_read->dev_address << 1U) & (uint8_t)(~I2C_OAR1_ADD0);

		/* Wait until ADDR flag is set */
		status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_ADDR, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Clear ADDR flag */
		i2c_clear_addr_flag(request_read->p_i2c);

		/* Wait until TXE flag is set */
		status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_TXE, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		if(request_read->mem_address_size == PRJ_I2C_MEM_ADDRESS_SIZE_8BIT)
		{
			/* Send LSB of memory address */
			request_read->p_i2c->DR = (uint8_t)(request_read->mem_address & 0x00FFU);
		}
		else
		{
			/* Send MSB of memory address */
			request_read->p_i2c->DR = (uint8_t)(request_read->mem_address & 0xFF00U);

			/* Wait until TXE flag is set */
			status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_TXE, PRJ_I2C_TIMEOUT_FLAG);

			if(status == PRJ_STATUS_OK)
			{
				/* Send LSB of memory address */
				request_read->p_i2c->DR = (uint8_t)(request_read->mem_address & 0x00FFU);
			}
			else
			{
				/* Generate stop */
				request_read->p_i2c->CR1 |= I2C_CR1_STOP;
			}
		}

		/* Wait until TXE flag is set */
		status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_TXE, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* Generate stop */
		request_read->p_i2c->CR1 |= I2C_CR1_STOP;
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Generate restart */
		request_read->p_i2c->CR1 |= I2C_CR1_START;

		/* Wait until SB flag is set */
		status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_SB, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* Generate stop */
		request_read->p_i2c->CR1 |= I2C_CR1_STOP;
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Send slave address with read request */
		request_read->p_i2c->DR = (request_read->dev_address << 1U) | (uint8_t)(I2C_OAR1_ADD0);

		/* Wait until ADDR flag is set */
		status = i2c_wait_on_set_flags(request_read->p_i2c, PRJ_I2C_FLAG_ADDR, PRJ_I2C_TIMEOUT_FLAG);
	}
	else
	{
		/* DO NOTHING */
	}

	return status;
}

/*!
 * @brief dma i2c process complete callback.
 *
 * This function is used to handle the completion of receiving or transmitting data via i2c using dma.
 *
 * @param[in] p_controls_peripherals	A pointer to an i2c structure instance.
 *
 * @return None.
 */
static void i2c_dma_complete(void* p_controls_peripherals)
{
	/* Convert pointer to pointer to I2C instance structure */
	prj_i2c_transmission_t *i2c_tx_rx = (prj_i2c_transmission_t*)p_controls_peripherals;

	/* Disable EVT and ERR interrupts */
	i2c_tx_rx->p_i2c->CR1 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

	if(i2c_tx_rx->data_size == 1U)
	{
		/* Disable Acknowledge */
		i2c_tx_rx->p_i2c->CR1 &= ~I2C_CR1_ACK;
	}
	else
	{
		; /* DO NOTHING */
	}

	/* Generate stop */
	i2c_tx_rx->p_i2c->CR1 |= I2C_CR1_STOP;

	/* Disable last DMA */
	i2c_tx_rx->p_i2c->CR2 &= ~I2C_CR2_LAST;

	/* Disable DMA request */
	i2c_tx_rx->p_i2c->CR2 &= ~I2C_CR2_DMAEN;

	/* Call I2C complete callback */
	prj_i2c_complete_callback(i2c_tx_rx->p_i2c);
}

/*!
 * @brief dma i2c process error callback.
 *
 * This function is used to handle the errors of receiving or transmitting data via i2c using dma.
 *
 * @param[in] p_controls_peripherals	A pointer to an I2C structure instance.
 *
 * @return None.
 */
static void i2c_dma_error(void* p_controls_peripherals)
{
	/* Convert pointer to pointer to I2C instance structure */
	prj_i2c_transmission_t *i2c_tx_rx = (prj_i2c_transmission_t*)p_controls_peripherals;

	/* Disable Acknowledge */
	i2c_tx_rx->p_i2c->CR1 &= ~I2C_CR1_ACK;

	/* Call I2C error callback */
	prj_i2c_error_callback(i2c_tx_rx->p_i2c);
}
