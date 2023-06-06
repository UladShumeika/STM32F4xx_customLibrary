/**
  ******************************************************************************
  * @file    ush_stm32f4xx_i2c.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    27 May 2023
  * @brief   Header file of I2C module.
  *
  * NOTE: This driver is not a full-fledged I2C driver, but contains only some of
  * 	  the functions were necessary at the time of development of this driver.
  * 	  However, if necessary, the function of this driver will be expanded.
  ******************************************************************************
  */

#ifndef ush_stm32f4xx_i2c_h
#define ush_stm32f4xx_i2c_h

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"
#include "ush_stm32f4xx_conf.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

/*!
 * @name I2C_duty_cycle_in_fast_mode
 * @{
 */
#define PRJ_I2C_DUTYCYCLE_2                 (0x00000000U)	/*!< I2C duty cycle 2 in fast mode */
#define PRJ_I2C_DUTYCYCLE_16_9              (I2C_CCR_DUTY)	/*!< I2C duty cycle 16/9 in fast mode */

/*! @}*/

/*!
 * @name I2C_addressing_mode
 * @{
 */
#define PRJ_I2C_ADDRESSING_MODE_7BIT        (0x00004000U)						/*!< I2C addressing mode 7-bit */
#define PRJ_I2C_ADDRESSING_MODE_10BIT		(I2C_OAR1_ADDMODE | 0x00004000U)	/*!< I2C addressing mode 10-bit */

/*! @} */

/*!
 * @name I2C_dual_addressing_mode
 * @{
 */
#define PRJ_I2C_DUAL_ADDRESS_DISABLE        (0x00000000U)		/*!< I2C dual address disable */
#define PRJ_I2C_DUAL_ADDRESS_ENABLE         (I2C_OAR2_ENDUAL)	/*!< I2C dual address enable */

/*! @} */

/*!
 * @name I2C_general_call_addressing_mode
 * @{
 */
#define PRJ_I2C_GENERAL_CALL_DISABLE        (0x00000000U)	/*!< I2C general call disable */
#define PRJ_I2C_GENERAL_CALL_ENABLE         (I2C_CR1_ENGC)	/*!< I2C general call enable */

/*! @} */

/*!
 * @name I2C_nostretch_mode
 * @{
 */
#define PRJ_I2C_NOSTRETCH_DISABLE          (0x00000000U)		/*!< I2C nostretch mode disable */
#define PRJ_I2C_NOSTRETCH_ENABLE           (I2C_CR1_NOSTRETCH)	/*!< I2C nostretch mode enable */

/*! @} */

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

/*!
 * @brief I2C configuration structure definition.
 */
typedef struct
{
	I2C_TypeDef* p_i2c;			  /*!< A pointer to I2Cx peripheral to be used where x is from 1 to 3. */

	uint32_t clock_speed;         /*!< The clock frequency.
	                                   This parameter must be set to a value lower than 400kHz */

	uint32_t duty_cycle;          /*!< The I2C fast mode duty cycle.
									   This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

	uint32_t own_address_1;       /*!< The first device own address.
		                               This parameter can be a 7-bit or 10-bit address */

	uint32_t addressing_mode;     /*!< The mode selection (7-bit or 10-bit).
									   This parameter can be a value of @ref I2C_addressing_mode */

	uint32_t dual_address_mode;   /*!< The dual addressing mode.
		                               This parameter can be a value of @ref I2C_dual_addressing_mode */

	uint32_t own_address_2;       /*!< The second device own address if dual addressing mode is selected.
	 	 	 	 	 	 	 	 	   This parameter can be a 7-bit address. */

	uint32_t general_call_mode;   /*!< The general call mode.
									   This parameter can be a value of @ref I2C_general_call_addressing_mode */

	uint32_t nostretch_mode;      /*!< The nostretch mode.
									   This parameter can be a value of @ref I2C_nostretch_mode */
} prj_i2c_init_t;

/*!
 * @brief I2C transmission structure definition.
 * @note  This structure is used for as sending as reception data.
 */
typedef struct
{
	I2C_TypeDef* p_i2c;			 		/*!< A pointer to I2Cx peripheral to be used where x is from 1 to 3. */

	uint32_t dev_address;		  		/*!< A target device address. */

	uint32_t mem_address;		  		/*!< An internal memory address. */

	uint8_t* p_data;			  		/*!< A pointer to data buffer. */

	uint8_t reserved1; 					/*!< For structure size alignment. */

	uint8_t reserved2; 					/*!< For structure size alignment. */

	uint8_t reserved3; 					/*!< For structure size alignment. */

	uint32_t data_size;		 	 		/*!< Amount of data to be sent. */

	prj_dma_handler_t* p_dma;			/*!< A pointer to dma instance which will be used for data transfer. */

} prj_i2c_transmission_t;

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
uint32_t prj_i2c_init(prj_i2c_init_t* p_i2c_init_structure);

#endif /* ush_stm32f4xx_i2c_h */
