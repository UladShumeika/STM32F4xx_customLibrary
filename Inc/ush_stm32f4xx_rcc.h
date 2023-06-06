/**
  ******************************************************************************
  * @file    ush_stm32f4xx_rcc.h
  * @author  Ulad Shumeika
  * @version v1.1
  * @date    21 March 2023
  * @brief   Header file of RCC module.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  *
  *
  *
  * @Major changes v1.1
  *		- periphery status enumeration replaced with definitions;
  *		- added RCC_getHCLKfreq, RCC_getPCLK1freq and RCC_getPCLK2freq functions;
  *		- added I2C clock enable/disable;
  *
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
	RCC_SYSCLKSOURCE_HSI		= RCC_CFGR_SW_HSI,		/* HSI oscillator selected as system clock */
	RCC_SYSCLKSOURCE_HSE		= RCC_CFGR_SW_HSE,		/* HSE oscillator selected as system clock*/
	RCC_SYSCLKSOURCE_PLL		= RCC_CFGR_SW_PLL		/* PLL selected as system clock	*/
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

#if defined(STM32F429xx)
	RCC_FLAG_PLLSAIRDY		= RCC_CR_PLLSAIRDY,		/* PLLSAI clock ready flag */
#endif

	RCC_FLAG_PLLI2SRDY		= RCC_CR_PLLI2SRDY		/* PLLI2S clock ready flag */
} USH_RCC_flags;

//---------------------------------------------------------------------------
// Test macros
//---------------------------------------------------------------------------

#if defined(STM32F429xx)

#define IS_RCC_FLAGS(FLAG)						   (((FLAG) == RCC_FLAG_HSIRDY)	   || \
													((FLAG) == RCC_FLAG_HSERDY)    || \
													((FLAG) == RCC_FLAG_PLLRDY)    || \
													((FLAG) == RCC_FLAG_PLLI2SRDY) || \
													((FLAG) == RCC_FLAG_PLLSAIRDY))
#elif defined(STM32F407xx)

#define IS_RCC_FLAGS(FLAG)						   (((FLAG) == RCC_FLAG_HSIRDY)	   || \
													((FLAG) == RCC_FLAG_HSERDY)    || \
													((FLAG) == RCC_FLAG_PLLRDY)    || \
													((FLAG) == RCC_FLAG_PLLI2SRDY))
#endif

#define IS_RCC_PLL_SOURCE(SOURCE)				   (((SOURCE) == RCC_PLLSOURCE_HSI) || \
													((SOURCE) == RCC_PLLSOURCE_HSE))

#define IS_RCC_PLLM_VALUE(VALUE) 				   ((2U <= (VALUE)) && ((VALUE) <= 63U))

#define IS_RCC_PLLN_VALUE(VALUE) 				   ((50U <= (VALUE)) && ((VALUE) <= 432U))

#define IS_RCC_PLLP_VALUE(VALUE) 				   (((VALUE) == 2U) || \
													((VALUE) == 4U) || \
													((VALUE) == 6U) || \
													((VALUE) == 8U))

#define IS_RCC_PLLQ_VALUE(VALUE) 				   ((2U <= (VALUE)) && ((VALUE) <= 15U))

#define IS_RCC_SYSCLK_SOURCE(SOURCE)			  (((SOURCE) == RCC_SYSCLKSOURCE_HSI) || \
												   ((SOURCE) == RCC_SYSCLKSOURCE_HSE) || \
												   ((SOURCE) == RCC_SYSCLKSOURCE_PLL))

#define IS_RCC_HCLK_DIVIDER(DIVIDER)			  (((DIVIDER) == RCC_SYSCLK_DIVIDER_1)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_2)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_4)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_8)   || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_16)  || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_64)  || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_128) || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_256) || \
												   ((DIVIDER) == RCC_SYSCLK_DIVIDER_512))

#define IS_RCC_APB_DIVIDER(DIVIDER)			  	  (((DIVIDER) == RCC_HCLK_DIVIDER_1) || \
												   ((DIVIDER) == RCC_HCLK_DIVIDER_2) || \
												   ((DIVIDER) == RCC_HCLK_DIVIDER_4) || \
												   ((DIVIDER) == RCC_HCLK_DIVIDER_8) || \
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

/*
 * GPIOA clock enable/disable
 */
#define __RCC_GPIOA_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define __RCC_GPIOA_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOARST)

/*
 * GPIOB clock enable/disable
 */
#define __RCC_GPIOB_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define __RCC_GPIOB_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST)

/*
 * GPIOC clock enable/disable
 */
#define __RCC_GPIOC_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN)
#define __RCC_GPIOC_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOCRST)

/*
 * GPIOD clock enable/disable
 */
#define __RCC_GPIOD_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN)
#define __RCC_GPIOD_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIODRST)

/*
 * GPIOE clock enable/disable
 */
#define __RCC_GPIOE_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN)
#define __RCC_GPIOE_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOERST)

/*
 * GPIOF clock enable/disable
 */
#define __RCC_GPIOF_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN)
#define __RCC_GPIOF_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOFRST)

/*
 * GPIOG clock enable/disable
 */
#define __RCC_GPIOG_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN)
#define __RCC_GPIOG_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOGRST)

/*
 * CAN1 clock enable/disable
 */
#define __RCC_CAN1_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_CAN1EN)
#define __RCC_CAN1_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST)

/*
 * CAN2 clock enable/disable
 */
#define __RCC_CAN2_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_CAN2EN)
#define __RCC_CAN2_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_CAN2RST)

/*
 * PWR clock enable/disable
 */
#define __RCC_PWR_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_PWREN)
#define __RCC_PWR_CLOCK_DISABLE()					(RCC->APB1RSTR) |= RCC_APB1RSTR_PWRRST)

/*
 * TIM13 clock enable/disable
 */
#define __RCC_TIM13_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_TIM13EN)
#define __RCC_TIM13_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_TIM13RST)

/*
 * TIM14 clock enable/disable
 */
#define __RCC_TIM14_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_TIM14EN)
#define __RCC_TIM14_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_TIM14RST)

/*
 * I2C1 clock enable/disable
 */
#define __RCC_I2C1_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_I2C1EN)
#define __RCC_I2C1_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST)

/*
 * I2C2 clock enable/disable
 */
#define __RCC_I2C2_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_I2C2EN)
#define __RCC_I2C2_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST)

/*
 * I2C3 clock enable/disable
 */
#define __RCC_I2C3_CLOCK_ENABLE()					(RCC->APB1ENR |= RCC_APB1ENR_I2C3EN)
#define __RCC_I2C3_CLOCK_DISABLE()					(RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST)

/*
 * DMA1 clock enable/disable
 */
#define __RCC_DMA1_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN)
#define __RCC_DMA1_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST)

/*
 * DMA2 clock enable/disable
 */
#define __RCC_DMA2_CLOCK_ENABLE()					(RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN)
#define __RCC_DMA2_CLOCK_DISABLE()					(RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST)

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes HSE oscillator.
 * @retval	The periphery status.
 */
uint32_t RCC_initHSE(void);

/**
 * @brief 	This function initializes PLL.
 * @param	initStructure - A pointer to USH_RCC_PLL_settingsTypeDef structure that contains the configuration
 * 							information for PLL.
 * @retval	The peripheral status.
 */
uint32_t RCC_initPLL(USH_RCC_PLL_settingsTypeDef *initStructure);

/**
 * @brief 	This function configures SYSCLK, HCLK and PCLKs.
 * @param	initStructure - A pointer to USH_RCC_clocksInitTypeDef structure that contains the configuration
 * 							information for SYSCLK, HCLK and PCLKs.
 * @retval	The peripheral status.
 */
uint32_t RCC_initClocks(USH_RCC_clocksInitTypeDef *initStructure);

/**
 * @brief 	This function returns flags status.
 * @param	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	Flags status.
 */
FlagStatus RCC_getFlagStatus(USH_RCC_flags flags);

/**
  * @brief  This function is used to return the HCLK frequency.
  * @retval HCLK frequency.
  */
uint32_t RCC_getHCLKfreq(void);

/**
  * @brief  This function is used to return the PCLK1 frequency.
  * @retval PCLK1 frequency.
  */
uint32_t RCC_getPCLK1freq(void);

/**
  * @brief  This function is used to return the PCLK2 frequency.
  * @retval PCLK2 frequency.
  */
uint32_t RCC_getPCLK2freq(void);

#endif /* __USH_STM32F4XX_MISC_H */
