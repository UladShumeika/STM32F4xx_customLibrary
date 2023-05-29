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
  *		- added i2c driver.
  *
  ******************************************************************************
  */

#ifndef ush_stm32f4xx_conf_h
#define ush_stm32f4xx_conf_h

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_STATUS_OK						(0x00000000U)	/*!< Periphery status OK */
#define PRJ_STATUS_ERROR					(0x00000001U)	/*!< Periphery status error */
#define PRJ_STATUS_TIMEOUT					(0x00000002U)	/*!< Periphery status timeout */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_misc.h"
#include "ush_stm32f4xx_gpio.h"
//#include "ush_stm32f4xx_dma.h"
//#include "ush_stm32f4xx_spi.h"
#include "ush_stm32f4xx_uart.h"
#include "ush_stm32f4xx_rcc.h"
#include "ush_stm32f4xx_can.h"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define IS_FUNCTIONAL_STATE(STATE) 			(((STATE) == DISABLE) || ((STATE) == ENABLE))

//---------------------------------------------------------------------------
// Function's parameters check.
//---------------------------------------------------------------------------
#ifdef USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr - If expr is false, it calls assert_failed function
  *   			   which reports the name of the source file and the source
  *   			   line number of the call that failed.
  *   			   If expr is true, it returns no value.
  * @retval None.
  */
  	#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))

//---------------------------------------------------------------------------
// Function's parameters check.
//---------------------------------------------------------------------------
  	void assert_failed(uint8_t* file, uint32_t line);
#else
	#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif	/* ush_stm32f4xx_conf_h */
