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
