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

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define HSE_TIMEOUT_VALUE						(100U)	// 100 ms
#define PLL_TIMEOUT_VALUE						(2U)	// 2 ms
#define CLOCKSWITCH_TIMEOUT_VALUE				(5000U) // 5 s

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static USH_peripheryStatus RCC_waitFlag(USH_RCC_flags flag, uint8_t timeout, FlagStatus expectedState);

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes HSE oscillator.
 * @retval	The periphery status.
 */
USH_peripheryStatus RCC_initHSE(void)
{
	USH_peripheryStatus status = STATUS_OK;

	// Enable HSE oscillator
	RCC->CR |= RCC_CR_HSEON;

	// Wait till HSE is enabled
	if(!RCC_waitFlag(RCC_FLAG_HSERDY, HSE_TIMEOUT_VALUE, SET)) status =  STATUS_TIMEOUT;

	return status;
}

/**
 * @brief 	This function initializes PLL.
 * @param	initStructure - A pointer to USH_RCC_PLL_settingsTypeDef structure that contains the configuration
 * 							information for PLL.
 * @retval	The peripheral status.
 */
USH_peripheryStatus RCC_initPLL(USH_RCC_PLL_settingsTypeDef *initStructure)
{
	USH_peripheryStatus status = STATUS_OK;

	// Check parameters
	if(initStructure == 0) return STATUS_ERROR;
	assert_param(IS_RCC_PLL_SOURCE(initStructure->PLL_source));
	assert_param(IS_RCC_PLLM_VALUE(initStructure->PLLM));
	assert_param(IS_RCC_PLLN_VALUE(initStructure->PLLN));
	assert_param(IS_RCC_PLLP_VALUE(initStructure->PLLP));
	assert_param(IS_RCC_PLLQ_VALUE(initStructure->PLLQ));

	// Configure PLL if it's disabled
	if(RCC_getFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
		// Configure PLL
		RCC->CFGR = initStructure->PLL_source 									 | \
				    initStructure->PLLM 										 | \
				    (initStructure->PLLN << RCC_PLLCFGR_PLLN_Pos) 				 | \
				    (((initStructure->PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) | \
				    initStructure->PLLQ << RCC_PLLCFGR_PLLQ_Pos;

		// Enable Main PLL
		RCC->CR |= RCC_CR_PLLON;

		// Wait till PLL is enabled
		if(!RCC_waitFlag(RCC_FLAG_PLLRDY, PLL_TIMEOUT_VALUE, SET)) status = STATUS_TIMEOUT;
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
	assert_param(IS_RCC_FLAGS(flags));

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
static USH_peripheryStatus RCC_waitFlag(USH_RCC_flags flag, uint8_t timeout, FlagStatus expectedState)
{
	uint8_t startTicks = MISC_timeoutGetTick();

	// Check parameters
	assert_param(IS_RCC_FLAGS(flag));

	if(expectedState == RESET)
	{
		// Wait till the flag is disabled
		while(RCC_getFlagStatus(flag) != RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > timeout)
			{
				return STATUS_TIMEOUT;
			}
		}
	} else
	{
		// Wait till the flag is enabled
		while(RCC_getFlagStatus(flag) == RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > timeout)
			{
				return STATUS_TIMEOUT;
			}
		}
	}

	return STATUS_OK;
}
