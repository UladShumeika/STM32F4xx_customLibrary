/**
  ******************************************************************************
  * @file    ush_stm32f4xx_i2c.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    27 May 2023
  * @brief   Header file of I2C module.
  *
  *	@DOTO
  * 	- add assert param tests;
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
 * @name i2c_duty_cycle_in_fast_mode
 * @{
 */
#define PRJ_I2C_DUTYCYCLE_2                 (0x00000000U)	/*!< i2c duty cycle 2 in fast mode */
#define PRJ_I2C_DUTYCYCLE_16_9              (I2C_CCR_DUTY)	/*!< i2c duty cycle 16/9 in fast mode */

/*! @}*/

/*!
 * @name i2c_addressing_mode
 * @{
 */
#define PRJ_I2C_ADDRESSING_MODE_7BIT        (0x00004000U)						/*!< i2c addressing mode 7-bit */
#define PRJ_I2C_ADDRESSING_MODE_10BIT		(I2C_OAR1_ADDMODE | 0x00004000U)	/*!< i2c addressing mode 10-bit */

/*! @} */

/*!
 * @name i2c_dual_addressing_mode
 * @{
 */
#define PRJ_I2C_DUAL_ADDRESS_DISABLE        (0x00000000U)		/*!< i2c dual address disable */
#define PRJ_I2C_DUAL_ADDRESS_ENABLE         (I2C_OAR2_ENDUAL)	/*!< i2c dual address enable */

/*! @} */

/*!
 * @name i2c_general_call_addressing_mode
 * @{
 */
#define PRJ_I2C_GENERAL_CALL_DISABLE        (0x00000000U)	/*!< i2c general call disable */
#define PRJ_I2C_GENERAL_CALL_ENABLE         (I2C_CR1_ENGC)	/*!< i2c general call enable */

/*! @} */

/*!
 * @name i2c_nostretch_mode
 * @{
 */
#define PRJ_I2C_NOSTRETCH_DISABLE          (0x00000000U)		/*!< i2c nostretch mode disable */
#define PRJ_I2C_NOSTRETCH_ENABLE           (I2C_CR1_NOSTRETCH)	/*!< i2c nostretch mode enable */

/*! @} */

/*!
 * @name i2c_memory_address_size
 * @{
 */
#define PRJ_I2C_MEM_ADDRESS_SIZE_8BIT					(0x00000008U)	/*!< Memory address 8 bit */
#define PRJ_I2C_MEM_ADDRESS_SIZE_16BIT					(0x00000010U)	/*!< Memory address 16 bit */

/*! @} */

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

/*!
 * @brief i2c configuration structure definition.
 */
typedef struct
{
	I2C_TypeDef* p_i2c;			  /*!< A pointer to i2c peripheral. */

	uint32_t clock_speed;         /*!< The clock frequency.
	                                   This parameter must be set to a value lower than 400kHz */

	uint32_t duty_cycle;          /*!< The I2C fast mode duty cycle.
									   This parameter can be a value of @ref i2c_duty_cycle_in_fast_mode */

	uint32_t own_address_1;       /*!< The first device own address.
		                               This parameter can be a 7-bit or 10-bit address */

	uint32_t addressing_mode;     /*!< The mode selection (7-bit or 10-bit).
									   This parameter can be a value of @ref i2c_addressing_mode */

	uint32_t dual_address_mode;   /*!< The dual addressing mode.
		                               This parameter can be a value of @ref i2c_dual_addressing_mode */

	uint32_t own_address_2;       /*!< The second device own address if dual addressing mode is selected.
	 	 	 	 	 	 	 	 	   This parameter can be a 7-bit address. */

	uint32_t general_call_mode;   /*!< The general call mode.
									   This parameter can be a value of @ref i2c_general_call_addressing_mode */

	uint32_t nostretch_mode;      /*!< The nostretch mode.
									   This parameter can be a value of @ref i2c_nostretch_mode */
} prj_i2c_init_t;

/*!
 * @brief i2c transmission structure definition.
 * @note  This structure is used for as sending as reception data.
 */
typedef struct
{
	I2C_TypeDef* p_i2c;			 		/*!< A pointer to i2c peripheral. */

	uint8_t dev_address;		  		/*!< A target device address. */

	uint8_t reserved1; 					/*!< For structure size alignment. */

	uint16_t mem_address;		  		/*!< An internal memory address. */

	uint16_t mem_address_size;		  	/*!< A size of an internal memory address. */

	uint8_t reserved2; 					/*!< For structure size alignment. */

	uint8_t reserved3; 					/*!< For structure size alignment. */

	uint8_t* p_data;			  		/*!< A pointer to data buffer. */

	uint32_t data_size;		 	 		/*!< Amount of data to be sent. */

	prj_dma_handler_t* p_dma;			/*!< A pointer to dma instance which will be used for data transfer. */

} prj_i2c_transmission_t;

/*!
 * @brief i2c data request structure definition.
 */
typedef struct
{
	I2C_TypeDef* p_i2c;			 		/*!< A pointer to i2c peripheral. */

	uint8_t dev_address;		  		/*!< A target device address. */

	uint8_t reserved1; 					/*!< For structure size alignment. */

	uint16_t mem_address;		  		/*!< An internal memory address. */

	uint16_t mem_address_size;		  	/*!< A size of an internal memory address. */

	uint8_t reserved2; 					/*!< For structure size alignment. */

	uint8_t reserved3; 					/*!< For structure size alignment. */

} prj_i2c_data_request_t;

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
 * @return @ref PRJ_STATUS_OK if i2c initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
uint32_t prj_i2c_init(prj_i2c_init_t* p_i2c_init);

/*!
 * @brief Write an amount of data with dma to a specific memory address.
 *
 * This function is used to write an amount of data with dma to a specific memory address.
 *
 * @param p_i2c_transmit	A pointer to an i2c transmit structure that contains all
 * 							the necessary information to transfer data.
 *
 * @return @ref PRJ_STATUS_OK if the data is written successfully.
 * @return @ref PRJ_STATUS_ERROR if a pointer is not passed either to the structure itself
 * 		   or to the DMA peripheral or the size of the transmitted data is 0.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_i2c_write_dma(prj_i2c_transmission_t* p_i2c_tx);

/*!
 * @brief Read an amount of data with dma to a specific memory address.
 *
 * This function is used to read an amount of data with dma to a specific memory address.
 *
 * @param p_i2c_reception	A pointer to an i2c reception structure that contains all
 * 							the necessary information to receive data.
 *
 * @return @ref PRJ_STATUS_OK if the data is read successfully.
 * @return @ref PRJ_STATUS_ERROR if a pointer is not passed either to the structure itself
 * 		   or to the DMA peripheral or the size of the received data is 0.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_i2c_read_dma(prj_i2c_transmission_t* p_i2c_rx);

/*!
 * @brief Check device availability on the i2c bus.
 *
 * This function is used to check the specified device availability several times.
 *
 * @param[in] p_i2c			A pointer to I2Cx peripheral.
 * @param[in] dev_address	A target device address.
 * @param[in] trials		Number of trials.
 *
 * @return @ref PRJ_STATUS_OK if the device is available.
 * @return @ref PRJ_STATUS_ERROR if a null pointer to the peripheral is passed
 * 		   or the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if the timeout has passed.
 */
uint32_t prj_i2c_is_device_ready(I2C_TypeDef* p_i2c, uint16_t dev_address, uint32_t trials);

/*!
 * @brief Handle i2c ev interrupt request.
 *
 * This function is used to handle i2c ev interrupt request.
 *
 * @param[in] p_i2c		A pointer to I2Cx peripheral.
 *
 * @return None.
 */
void prj_i2c_irq_handler(I2C_TypeDef* p_i2c);

/*!
 * @brief i2c tx completed callbacks.
 *
 * @note This function should not be modified, when the callback is needed,
 * 		 the prj_i2c_tx_complete_callback could be implemented in the user file.
 *
 * @param p_i2c		A pointer to i2c peripheral.
 *
 * @return None.
 */
__WEAK void prj_i2c_tx_complete_callback(I2C_TypeDef* p_i2c);

/*!
 * @brief i2c rx completed callbacks.
 *
 * @note This function should not be modified, when the callback is needed,
 * 		 the prj_i2c_rx_complete_callback could be implemented in the user file.
 *
 * @param p_i2c		A pointer to i2c peripheral.
 *
 * @return None.
 */
__WEAK void prj_i2c_rx_complete_callback(I2C_TypeDef* p_i2c);

/*!
 * @brief i2c tx/rx error callbacks.
 *
 * @note This function should not be modified, when the callback is needed,
 * 		 the prj_i2c_error_callback could be implemented in the user file.
 *
 * @param p_i2c		A pointer to I2Cx peripheral.
 *
 * @return None.
 */
__WEAK void prj_i2c_error_callback(I2C_TypeDef* p_i2c);

#endif /* ush_stm32f4xx_i2c_h */
