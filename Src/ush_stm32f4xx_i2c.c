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
#include "ush_stm32f4xx_conf.h"
#include "stddef.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

/*!
 * @name I2C_checking_pclk_frequency
 * @{
 */
#define PRJ_I2C_MIN_PCLK_FREQ_STANDARD					(2000000U)	/*!< 2 MHz */
#define PRJ_I2C_MIN_PCLK_FREQ_FAST						(4000000U)	/*!< 4 MHz */
#define PRJ_I2C_FREQ_STANDARD							(100000U)	/*!< 100 kHz */
#define PRJ_I2C_FREQ_FAST								(400000U)	/*!< 400 kHz */
#define PRJ_I2C_1_MHZ									(1000000U)  /*!< 1 MHz */

/*! @} */

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
/*!
 * @name I2C_calculation_clock_control_register
 * @{
 */
#define PRJ_I2C_DUTYCYCLE_2_COEF						(3U)	/*!< The coefficient for duty cycle 2 */
#define PRJ_I2C_DUTYCYCLE_16_9_COEF						(25U)	/*!< The coefficient for duty cycle 16/9 */
#define PRJ_I2C_MIN_CCR_VALUE_FOR_SM					(4U)	/*!< The minimum CCR value for standard mode */
#define PRJ_I2C_MIN_CCR_VALUE_FOR_FM					(1U)	/*!< The minimum CCR value for fast mode */

/*! @} */

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static uint32_t i2c_checking_pclk_frequency(uint32_t pclk1, uint32_t i2c_clock_speed);
static uint32_t i2c_calc_rise_time(uint32_t freq_range, uint32_t i2c_clock_speed);
static uint32_t i2c_ccr_calc(uint32_t pclk1, uint32_t i2c_clock_speed, uint32_t duty_cycle);

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize I2C peripherals.
 *
 * This function is used to initialize I2C peripherals.
 *
 * @param[in] p_i2c_init_structure		A pointer to I2C initialization structure.
 *
 * @return @ref PRJ_STATUS_OK if I2C initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
uint32_t prj_i2c_init(prj_i2c_init_t* p_i2c_init_structure)
{
	uint32_t status = PRJ_STATUS_OK;
	uint32_t pclk1 = 0;
	uint32_t freq_range = 0;
	uint32_t rise_time = 0;
	uint32_t ccr_calc = 0;

	/* Check the pointer */
	if(p_i2c_init_structure == NULL)
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
		p_i2c_init_structure->p_i2c->CR1 &= ~I2C_CR1_PE;

		/* Reset I2C peripheral */
		p_i2c_init_structure->p_i2c->CR1 |= I2C_CR1_SWRST;
		p_i2c_init_structure->p_i2c->CR1 &= ~I2C_CR1_SWRST;

		/* Check the minimum allowed PCLK1 frequency */
		pclk1 = RCC_getPCLK1freq();
		status = i2c_checking_pclk_frequency(pclk1, p_i2c_init_structure->clock_speed);
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
		p_i2c_init_structure->p_i2c->CR2 = freq_range;

		/* Configure p_i2c: Rise Time */
		rise_time = i2c_calc_rise_time(freq_range, p_i2c_init_structure->clock_speed);
		p_i2c_init_structure->p_i2c->TRISE |= rise_time;

		/* Configure p_i2c: Speed */
		ccr_calc = i2c_ccr_calc(pclk1, p_i2c_init_structure->clock_speed, p_i2c_init_structure->duty_cycle);
		p_i2c_init_structure->p_i2c->CCR |= ccr_calc;

		/* Configure p_i2c: Generalcall and NoStretch mode */
		p_i2c_init_structure->p_i2c->CR1 |= (p_i2c_init_structure->general_call_mode | \
										    p_i2c_init_structure->nostretch_mode);

		/* Configure p_i2c: Own Address1 and addressing mode */
		p_i2c_init_structure->p_i2c->OAR1 |= (p_i2c_init_structure->addressing_mode | \
											 p_i2c_init_structure->own_address_1);

		/* Configure p_i2c: Dual mode and Own Address2 */
		p_i2c_init_structure->p_i2c->OAR2 |= (p_i2c_init_structure->dual_address_mode | \
											 p_i2c_init_structure->own_address_2);

		/* Enable the selected I2C peripheral */
		p_i2c_init_structure->p_i2c->CR1 |= I2C_CR1_PE;
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Check pclk frequency for I2C peripherals.
 *
 * This function is used to check pclk frequency for I2C peripherals.
 *
 * @param[in] pclk1			  	PCLK1 clock frequency.
 * @param[in] i2c_clock_speed	I2C peripheral clock speed.
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
 * @brief Calculate rise time for I2C peripherals.
 *
 * This function is used to calculate rise time.
 *
 * @param[in] freq_range		frequency range.
 * @param[in] i2c_clock_speed 	I2C peripheral clock speed.
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
 * @brief Calculate the CCR (clock control register) value.
 *
 * This function is used to calculate the CCR value taking into account the selected i2c speed.
 *
 * @param[in] pclk1				PCLK1 clock frequency.
 * @param[in] i2c_clock_speed	I2C peripheral clock speed.
 * @param[in] duty_cycle		I2C fast mode duty cycle.
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
