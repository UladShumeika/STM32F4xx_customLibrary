/**
  ******************************************************************************
  * @file    ush_stm32f4xx_rcc.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    21 March 2023
  * @brief   Header file of RCC module.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4XX_RCC_H
#define __USH_STM32F4XX_RCC_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief RCC PLL source enumeration.
 */
typedef enum
{
	RCC_PLLSOURCE_HSI	= RCC_PLLCFGR_PLLSRC_HSI,	/* HSI oscillator clock selected as PLL and PLLI2S clock entry */
	RCC_PLLSOURCE_HSE	= RCC_PLLCFGR_PLLSRC_HSE	/* HSE oscillator clock selected as PLL and PLLI2S clock entry */
} USH_RCC_PLL_source;

/**
 * @brief RCC SYSCLK sources enumeration.
 */
typedef enum
{
	RCC_SYSCLKSOURCE_HSI		= RCC_CFGR_SWS_HSI,		/* HSI oscillator selected as system clock */
	RCC_SYSCLKSOURCE_HSE		= RCC_CFGR_SWS_HSE,		/* HSE oscillator selected as system clock*/
	RCC_SYSCLKSOURCE_PLL		= RCC_CFGR_SWS_PLL		/* PLL selected as system clock	*/
} USH_RCC_SYSCLK_sources;

/**
 * @brief RCC SYSCLK dividers enumeration.
 */
typedef enum
{
	RCC_SYSCLK_DIVIDER_1		= RCC_CFGR_HPRE_DIV1,		/* SYSCLK not divided */
	RCC_SYSCLK_DIVIDER_2		= RCC_CFGR_HPRE_DIV2, 		/* SYSCLK divided by 2 */
	RCC_SYSCLK_DIVIDER_4		= RCC_CFGR_HPRE_DIV4,		/* SYSCLK divided by 4 */
	RCC_SYSCLK_DIVIDER_8		= RCC_CFGR_HPRE_DIV8,		/* SYSCLK divided by 8 */
	RCC_SYSCLK_DIVIDER_16		= RCC_CFGR_HPRE_DIV16,		/* SYSCLK divided by 16 */
	RCC_SYSCLK_DIVIDER_64		= RCC_CFGR_HPRE_DIV64,		/* SYSCLK divided by 64 */
	RCC_SYSCLK_DIVIDER_128		= RCC_CFGR_HPRE_DIV128,		/* SYSCLK divided by 128 */
	RCC_SYSCLK_DIVIDER_256		= RCC_CFGR_HPRE_DIV256,		/* SYSCLK divided by 256 */
	RCC_SYSCLK_DIVIDER_512		= RCC_CFGR_HPRE_DIV512		/* SYSCLK divided by 512 */
} USH_RCC_SYSCLK_dividers;

/**
 * @brief RCC HCLK clock dividers enumeration.
 */
typedef enum
{
	RCC_HCLK_DIVIDER_1		= RCC_CFGR_PPRE1_DIV1,		/* HCLK not divided */
	RCC_HCLK_DIVIDER_2		= RCC_CFGR_PPRE1_DIV2,		/* HCLK divided by 2 */
	RCC_HCLK_DIVIDER_4		= RCC_CFGR_PPRE1_DIV4,		/* HCLK divided by 4 */
	RCC_HCLK_DIVIDER_8		= RCC_CFGR_PPRE1_DIV8,		/* HCLK divided by 8 */
	RCC_HCLK_DIVIDER_16		= RCC_CFGR_PPRE1_DIV16		/* HCLK divided by 16 */
} USH_RCC_APB_dividers;



/**
 * @brief RCC PLL settings structure definition.
 */
typedef struct
{
	USH_RCC_PLL_source PLL_source;		/* PLL source. This parameter can be a value of @ref USH_RCC_PLL_source. */

	uint32_t PLLM;						/* Division factor for the main PLL (PLL) and audio PLL (PLLI2S) input clock. */

	uint32_t PLLN;						/* Main PLL (PLL) multiplication factor for VCO. */

	uint32_t PLLP;						/* Main PLL (PLL) division factor for main system clock. */

	uint32_t PLLQ;						/* Main PLL (PLL) division factor for USB OTG FS, SDIO and random number generator clocks. */

} USH_RCC_PLL_settingsTypeDef;



/**
  * @brief RCC System, AHB and APB busses clock configuration structure definition.
  */
typedef struct
{
	USH_RCC_SYSCLK_sources SYSCLK_source;          	/* The clock source (SYSCLKS) used as system clock.
                                       	   	   	   	   This parameter can be a value of @ref USH_RCC_SYSCLK_sources. */

	USH_RCC_SYSCLK_dividers HCLK_divider;           /* The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       	   	   	   	   This parameter can be a value of @ref USH_RCC_SYSCLK_dividers. */

	USH_RCC_APB_dividers APB1_divider;				/* The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       	   	   	       This parameter can be a value of @ref USH_RCC_APB_dividers. */

	USH_RCC_APB_dividers APB2_divider;      		/* The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       	   	   	   	   This parameter can be a value of @ref USH_RCC_APB_dividers. */

} USH_RCC_clocksInitTypeDef;

/**
 * @brief RCC flags enumeration
 */
typedef enum
{
	RCC_FLAG_HSIRDY			= RCC_CR_HSIRDY,		/* HSI clock ready flag */
	RCC_FLAG_HSERDY			= RCC_CR_HSERDY,		/* HSE clock ready flag */
	RCC_FLAG_PLLRDY			= RCC_CR_PLLRDY,		/* PLL clock ready flag */
	RCC_FLAG_PLLI2SRDY		= RCC_CR_PLLI2SRDY,		/* PLLI2S clock ready flag */
	RCC_FLAG_PLLSAIRDY		= RCC_CR_PLLSAIRDY		/* PLLSAI clock ready flag */
} USH_RCC_flags;

//---------------------------------------------------------------------------
// Test macros
//---------------------------------------------------------------------------
#define IS_RCC_FLAGS(FLAG)						   (((FLAG) == RCC_FLAG_HSIRDY)	   || \
													((FLAG) == RCC_FLAG_HSERDY)    || \
													((FLAG) == RCC_FLAG_PLLRDY)    || \
													((FLAG) == RCC_FLAG_PLLI2SRDY) || \
													((FLAG) == RCC_FLAG_PLLSAIRDY))

#define IS_RCC_PLL_SOURCE(SOURCE)				   (((SOURCE) == RCC_PLLSOURCE_HSI) || \
													((SOURCE) == RCC_PLLSOURCE_HSE))

#define IS_RCC_PLLM_VALUE(VALUE) 				   ((2U <= (VALUE)) && ((VALUE) <= 63U))

#define IS_RCC_PLLN_VALUE(VALUE) 				   ((50U <= (VALUE)) && ((VALUE) <= 432U))

#define IS_RCC_PLLP_VALUE(VALUE) 				   (((VALUE) == 2U) || \
													((VALUE) == 4U) || \
													((VALUE) == 6U) || \
													((VALUE) == 8U))

#define IS_RCC_PLLQ_VALUE(VALUE) 				   ((2U <= (VALUE)) && ((VALUE) <= 15U))

#define IS_RCC_SYSCLK_TYPES(TYPE)			   	  (((TYPE) == RCC_CLOCKTYPE_SYSCLK) || \
												   ((TYPE) == RCC_CLOCKTYPE_HCLK) 	|| \
												   ((TYPE) == RCC_CLOCKTYPE_PCLK1) 	|| \
												   ((TYPE) == RCC_CLOCKTYPE_PCLK2))

#define IS_RCC_SYSCLK_SOURCE(SOURCE)			  (((SOURCE) == RCC_SYSCLKSOURCE_HSI) || \
												   ((SOURCE) == RCC_SYSCLKSOURCE_HSE) || \
												   ((SOURCE) == RCC_SYSCLKSOURCE_PLL))

#define IS_RCC_SYSCLK_DIVIDER(DIVIDER)			  (((DIVIDER) == RCC_SYSCLK_DIVIDER_1)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_2)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_4)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_8)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_16)  || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_32)  || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_64)  || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_128) || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_256) || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_512))

#define IS_RCC_PCLK_DIVIDER(DIVIDER)			 (((DIVIDER) == RCC_HCLK_DIVIDER_1)  || \
												  ((DIVIDER) == RCC_HCLK_DIVIDER_2)  || \
												  ((DIVIDER) == RCC_HCLK_DIVIDER_4)  || \
												  ((DIVIDER) == RCC_HCLK_DIVIDER_8)	 || \
												  ((DIVIDER) == RCC_HCLK_DIVIDER_16))

//---------------------------------------------------------------------------
// Other macros
//---------------------------------------------------------------------------

/**
 * @brief This macro returns the system clock source
 */
#define RCC_GET_SYSCLOCK_SOURCE()				   (RCC->CFGR & RCC_CFGR_SWS)

//---------------------------------------------------------------------------
// Enable or disable peripheral clocking
//---------------------------------------------------------------------------

/**
 * @brief Power interface clock enable
 */
#define RCC_powerInterfaceClockEnable()		RCC->APB1ENR |= RCC_APB1ENR_PWREN

/**
 * @brief TIM14 clock enable
 */
#define RCC_TIM14_ClockEnable()				RCC->APB1ENR |= RCC_APB1ENR_TIM14EN

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes HSE oscillator.
 * @retval	The periphery status.
 */
USH_peripheryStatus RCC_initHSE(void);

/**
 * @brief 	This function initializes PLL.
 * @param	initStructure - A pointer to USH_RCC_PLL_settingsTypeDef structure that contains the configuration
 * 							information for PLL.
 * @retval	The peripheral status.
 */
USH_peripheryStatus RCC_initPLL(USH_RCC_PLL_settingsTypeDef *initStructure);

/**
 * @brief 	This function returns flags status.
 * @param	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	Flags status.
 */
FlagStatus RCC_getFlagStatus(USH_RCC_flags flags);

#endif /* __USH_STM32F4XX_MISC_H */
