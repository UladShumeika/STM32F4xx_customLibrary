/**
  ******************************************************************************
  * @file    ush_stm32f4xx_rcc.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    21 March 2023
  * @brief	 This file contains the implementation of functions for working with RCC.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_rcc.h"
#include "ush_stm32f4xx_conf.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define HSE_TIMEOUT_VALUE						(100U)	// 100 ms
#define PLL_TIMEOUT_VALUE						(2U)	// 2 ms
#define CLOCKSWITCH_TIMEOUT_VALUE				(5000U) // 5 s

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static uint32_t RCC_waitFlag(USH_RCC_flags flag, uint8_t timeout, FlagStatus expectedState);

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes HSE oscillator.
 * @retval	The periphery status.
 */
uint32_t RCC_initHSE(void)
{
	uint32_t status = PRJ_STATUS_OK;

	// Enable HSE oscillator
	RCC->CR |= RCC_CR_HSEON;

	// Wait till HSE is enabled
	status = RCC_waitFlag(RCC_FLAG_HSERDY, HSE_TIMEOUT_VALUE, SET);

	return status;
}

/**
 * @brief 	This function initializes PLL.
 * @param	initStructure - A pointer to USH_RCC_PLL_settingsTypeDef structure that contains the configuration
 * 							information for PLL.
 * @retval	The peripheral status.
 */
uint32_t RCC_initPLL(USH_RCC_PLL_settingsTypeDef *initStructure)
{
	uint32_t status = PRJ_STATUS_OK;

	// Check parameters
	if(initStructure == 0) return PRJ_STATUS_ERROR;
	macro_prj_assert_param(IS_RCC_PLL_SOURCE(initStructure->PLL_source));
	macro_prj_assert_param(IS_RCC_PLLM_VALUE(initStructure->PLLM));
	macro_prj_assert_param(IS_RCC_PLLN_VALUE(initStructure->PLLN));
	macro_prj_assert_param(IS_RCC_PLLP_VALUE(initStructure->PLLP));
	macro_prj_assert_param(IS_RCC_PLLQ_VALUE(initStructure->PLLQ));

	// Configure PLL if it's disabled
	if(RCC_getFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
		// Configure PLL
		RCC->PLLCFGR = initStructure->PLL_source								 	| \
				       initStructure->PLLM 										 	| \
				       (initStructure->PLLN << RCC_PLLCFGR_PLLN_Pos) 				| \
				       (((initStructure->PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) | \
				       initStructure->PLLQ << RCC_PLLCFGR_PLLQ_Pos;

		// Enable Main PLL
		RCC->CR |= RCC_CR_PLLON;

		// Wait till PLL is enabled
		status = RCC_waitFlag(RCC_FLAG_PLLRDY, PLL_TIMEOUT_VALUE, SET);
	}

	return status;
}

/**
 * @brief 	This function configures SYSCLK, HCLK and PCLKs.
 * @param	initStructure - A pointer to USH_RCC_clocksInitTypeDef structure that contains the configuration
 * 							information for SYSCLK, HCLK and PCLKs.
 * @retval	The peripheral status.
 */
uint32_t RCC_initClocks(USH_RCC_clocksInitTypeDef *initStructure)
{
	uint32_t startTicks = 0;
	uint32_t status = PRJ_STATUS_OK;

	// Check parameters
	if(initStructure == 0)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		; /* DO NOTHING */
	}

	macro_prj_assert_param(IS_RCC_SYSCLK_SOURCE(initStructure->SYSCLK_source));
	macro_prj_assert_param(IS_RCC_HCLK_DIVIDER(initStructure->HCLK_divider));
	macro_prj_assert_param(IS_RCC_APB_DIVIDER(initStructure->APB1_divider));
	macro_prj_assert_param(IS_RCC_APB_DIVIDER(initStructure->APB2_divider));

	if(status == PRJ_STATUS_OK)
	{
		// Set HCLK, PCLKs dividers
		RCC->CFGR |= initStructure->HCLK_divider | initStructure->APB1_divider | (initStructure->APB2_divider << 3U);

		// Set SYSCLK source
		RCC->CFGR |= initStructure->SYSCLK_source;

		// Wait till PLL is enabled
		startTicks = MISC_timeoutGetTick();
		while((uint32_t)(RCC->CFGR & RCC_CFGR_SWS) != (initStructure->SYSCLK_source << RCC_CFGR_SWS_Pos))
		{
			if((MISC_timeoutGetTick() - startTicks) > CLOCKSWITCH_TIMEOUT_VALUE)
			{
				status = PRJ_STATUS_TIMEOUT;
				break;
			}
			else
			{
				; /* DO NOTHING */
			}
		}
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function returns flags status.
 * @param	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	Flags status.
 */
FlagStatus RCC_getFlagStatus(USH_RCC_flags flags)
{
	uint32_t statusReg = 0;
	FlagStatus status = RESET;

	// Check parameters
	macro_prj_assert_param(IS_RCC_FLAGS(flags));

	// Read RCC->CR register
	statusReg = RCC->CR;

	// Set flags status
	if(statusReg & flags)
	{
		status = SET;
	} else
	{
		status = RESET;
	}

	return status;
}

//---------------------------------------------------------------------------
// Static Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function waits for a flag to be reset or set.
 * @param 	flag - A flag to watch.
 * @param 	timeout - timeout time for the specified flag.
 * @retval	Periphery status.
 */
static uint32_t RCC_waitFlag(USH_RCC_flags flag, uint8_t timeout, FlagStatus expectedState)
{
	uint8_t startTicks = MISC_timeoutGetTick();
	uint32_t status = PRJ_STATUS_OK;

	// Check parameters
	macro_prj_assert_param(IS_RCC_FLAGS(flag));

	if(expectedState == RESET)
	{
		// Wait till the flag is disabled
		while(RCC_getFlagStatus(flag) != RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > timeout)
			{
				status = PRJ_STATUS_TIMEOUT;
				break;
			}
		}
	} else
	{
		// Wait till the flag is enabled
		while(RCC_getFlagStatus(flag) == RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > timeout)
			{
				status = PRJ_STATUS_TIMEOUT;
				break;
			}
		}
	}

	return status;
}
