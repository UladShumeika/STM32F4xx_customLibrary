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
