/**
  ******************************************************************************
  * @file    ush_stm32f4xx_conf.h
  * @author  Ulad Shumeika
  * @version v1.2
  * @date    03-March-2023
  * @brief   Library configuration file.
  *
  *
  *	@Major changes v1.1
  *		- added STATUS_ERROR in USH_peripheryStatus enumeration.
  *
  *	@Major changes v1.2
  *		- changed code style;
  *		- periphery status enumeration replaced with definitions;
  *		- moved parameter check macro to misc module;
  *
  ******************************************************************************
  */

#ifndef ush_stm32f4xx_conf_h
#define ush_stm32f4xx_conf_h

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

/*!
 * @name custom_drivers
 * @{
 */
#define PRJ_MISC_DRIVER				/*!< Miscellaneous module */
#define PRJ_RCC_DRIVER				/*!< RCC module */
#define PRJ_GPIO_DRIVER				/*!< GPIO module */
/* #define PRJ_DMA_DRIVER  */		/*!< DMA module */
/* #define PRJ_SPI_DRIVER  */		/*!< SPI module */
/* #define PRJ_UART_DRIVER */		/*!< UART module */
#define PRJ_CAN_DRIVER				/*!< CAN module */

/*! @}*/

/*!
 * @name status_definitions
 * @{
 */
#define PRJ_STATUS_OK						(0x00000000U)	/*!< Periphery status OK */
#define PRJ_STATUS_ERROR					(0x00000001U)	/*!< Periphery status error */
#define PRJ_STATUS_TIMEOUT					(0x00000002U)	/*!< Periphery status timeout */

/*! @}*/

/*!
 * @name flag_definitions
 * @{
 */
#define PRJ_FLAG_SET						(0x00000001U)	/*!< Flag set */
#define PRJ_FLAG_RESET						(0x00000000U)	/*!< Flag reset */

/*! @}*/

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define macro_prj_common_unused(x) 					(void)(x)

//---------------------------------------------------------------------------
// Includes' modules
//---------------------------------------------------------------------------

#ifdef PRJ_MISC_DRIVER
	#include "ush_stm32f4xx_misc.h"
#endif

#ifdef PRJ_RCC_DRIVER
	#include "ush_stm32f4xx_rcc.h"
#endif

#ifdef PRJ_GPIO_DRIVER
	#include "ush_stm32f4xx_gpio.h"
#endif

#ifdef PRJ_DMA_DRIVER
	#include "ush_stm32f4xx_dma.h"
#endif

#ifdef PRJ_SPI_DRIVER
	#include "ush_stm32f4xx_spi.h"
#endif

#ifdef PRJ_UART_DRIVER
	#include "ush_stm32f4xx_uart.h"
#endif

#ifdef PRJ_CAN_DRIVER
	#include "ush_stm32f4xx_can.h"
#endif

#endif	/* ush_stm32f4xx_conf_h */
