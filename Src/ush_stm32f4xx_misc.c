/**
  ******************************************************************************
  * @file    ush_stm32f4xx_misc.c
  * @author  Ulad Shumeika
  * @version v1.1
  * @date    27-February-2023
  * @brief	 This file contains the implementation of functions for the miscellaneous
  * 		 firmware library functions.
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_misc.h"
#include "ush_stm32f4xx_conf.h"
#include <stdio.h>

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static volatile uint32_t timeoutTicks = 0;

//---------------------------------------------------------------------------
// Redefine printf function
//---------------------------------------------------------------------------
int _write(int file, char *ptr, int len)
{
	for(int i=0 ; i < len ; i++)
	{
		ITM_SendChar((*ptr++));
	}

  return len;
}

//---------------------------------------------------------------------------
// The section of power controller
//---------------------------------------------------------------------------

/**
 * @brief 	This function configures the main internal regulator output voltage.
 * @param 	voltageScaling - specifies the regulator output voltage to achieve
 * 							 a tradeoff between performance and power consumption
 * 							 when the device does not operate at the maximum frequency
 * 							 (refer to the datasheets for more details).
 * @retval 	None.
 */
void MISC_PWR_mainRegulatorModeConfig(USH_PWR_voltageScaling voltageScaling)
{
	uint32_t tempReg = 0;

	// Check parameters
	macro_prj_assert_param(IS_MISC_PWR_VOLTAGE_SCALING(voltageScaling));

	// Enable power interface clock
	__RCC_PWR_CLOCK_ENABLE();

	// Read PWR_CR register and clear PWR_CR_VOS bits
	tempReg = PWR->CR;
	tempReg &= ~PWR_CR_VOS;

	// Set voltageScaling and write to PWR_CR register
	tempReg |= voltageScaling;
	PWR->CR = tempReg;
}

#if defined(STM32F429xx)

/**
 * @brief 	This function returns flag status.
 * @param	flags - PWR flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	Flags status.
 */
FlagStatus MISC_PWR_getFlagStatus(USH_PWR_flags flags)
{
	uint32_t statusReg = 0;
	FlagStatus status = RESET;

	// check parameters
	macro_prj_assert_param(IS_MISC_PWR_FLAGS(flags));

	// Read PWR->CR register
	statusReg = PWR->CR;

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

#endif

//---------------------------------------------------------------------------
// The section of timeout timer
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes TIM14 timer to check for timeout.
 * @retval	None.
 */
void MISC_timeoutTimerInit(void)
{
	// Enable TIM14 clock
	__RCC_TIM14_CLOCK_ENABLE();

	// Disable TIM14
	TIM14->CR1 &= ~TIM_CR1_CEN;

	// Set the clock division
	TIM14->CR1 &= ~TIM_CR1_CKD;

	// Set the auto-reload preload
	TIM14->CR1 &= ~TIM_CR1_ARPE;

	// Set the Autoreload value
	TIM14->ARR = (uint32_t)(5625U);		// 5625

	// Set the Prescaler value
	if(RCC_getFlagStatus(RCC_FLAG_HSERDY))
	{
		TIM14->PSC = 16U - 1U;		// HSI enabled. 16 MHz
	} else
	{
		TIM14->PSC = 90U - 1U;		// HSE enabled and system clock is 180 MHz
	}

	TIM14->EGR = TIM_EGR_UG;

	// Enable the TIM update interrupt
	TIM14->DIER |= TIM_DIER_UIE;

	// Enable TIM14
	TIM14->CR1 |= TIM_CR1_CEN;

	// Enable the TIM14 global interrupt
	MISC_NVIC_enableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	MISC_NVIC_setPriority(TIM8_TRG_COM_TIM14_IRQn, MIN_PREEMPPRIORITY, MIN_SUBPRIORITY);
}

/**
 * @brief 	This function increments a variable "timeoutTicks".
 * @retval	None.
 */
void MISC_timeoutTimerIncTick(void)
{
	timeoutTicks++;
}

/**
 * @brief 	This function returns "timeoutTicks" variable value.
 * @retval	None.
 */
uint32_t MISC_timeoutGetTick(void)
{
	return timeoutTicks;
}

/**
 * @brief 	This function provides minimum delay (in milliseconds) based
 *          on variable incremented.
 * @param	delay - The delay time length, in milliseconds.
 * @retval	None.
 */
void MISC_timeoutDelay(uint32_t delay)
{
  uint32_t start_ticks = MISC_timeoutGetTick();
  uint32_t past_ticks = 0;

  do
  {
	  past_ticks = MISC_timeoutGetTick() - start_ticks;

  } while(past_ticks < delay);
}

//---------------------------------------------------------------------------
// The section of NVIC
//---------------------------------------------------------------------------

/**
  * @brief  This function sets the priority grouping field (preemption priority and subpriority)
  *         using the required unlock sequence.
  * @param  priorityGroup - The priority grouping bits length.
  * @note   When the NVIC_PriorityGroup_0 is selected, IRQ preemption is no more possible.
  *         The pending IRQ priority will be managed only by the subpriority.
  * @retval None.
  */
void MISC_NVIC_setPriorityGrouping(USH_NVIC_priorityGroup priorityGroup)
{
	// Check the parameters
	macro_prj_assert_param(IS_MISC_NVIC_PRIORITY_GROUP(priorityGroup));

	// Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value
	NVIC_SetPriorityGrouping(priorityGroup);
}

/**
  * @brief  This function sets the priority of an interrupt.
  * @param  IRQn External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @param  PreemptPriority - The preemption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 15.
  *         A lower priority value indicates a higher priority.
  * @param  SubPriority - The subpriority level for the IRQ channel.
  *         This parameter can be a value between 0 and 15.
  *         A lower priority value indicates a higher priority.
  * @retval	None.
  */
void MISC_NVIC_setPriority(IRQn_Type IRQn, uint32_t preemptPriority, uint32_t subPriority)
{
  uint32_t prioritygroup = 0x00U;

  // Check the parameters
  macro_prj_assert_param(IS_MISC_NVIC_DEVICE_IRQ(IRQn));
  macro_prj_assert_param(IS_MISC_NVIC_SUB_PRIORITY(subPriority));
  macro_prj_assert_param(IS_MISC_NVIC_PREEMPTION_PRIORITY(preemptPriority));

  prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, preemptPriority, subPriority));
}

/**
  * @brief  This function enables a device specific interrupt in the NVIC interrupt controller.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before.
  * @param  IRQn - The external interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @retval None.
  */
void MISC_NVIC_enableIRQ(IRQn_Type IRQn)
{
	// Check the parameters
	macro_prj_assert_param(IS_MISC_NVIC_DEVICE_IRQ(IRQn));

	// Enable interrupt
	NVIC_EnableIRQ(IRQn);
}

/**
  * @brief  This function disables a device specific interrupt in the NVIC interrupt controller.
  * @param  IRQn - The external interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @retval None.
  */
void MISC_NVIC_disableIRQ(IRQn_Type IRQn)
{
  // Check the parameters
  macro_prj_assert_param(IS_MISC_NVIC_DEVICE_IRQ(IRQn));

  // Disable interrupt
  NVIC_DisableIRQ(IRQn);
}

//---------------------------------------------------------------------------
// The section of FLASH memory
//---------------------------------------------------------------------------

/**
  * @brief  This function enables or disables the prefetch buffer.
  * @param  newState - A new state of the prefetch buffer.
  *          		   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MISC_FLASH_prefetchBufferCmd(FunctionalState newState)
{
  // Check the parameters
  macro_prj_assert_param(IS_FUNCTIONAL_STATE(newState));

  // Enable or disable the prefetch buffer
  if(newState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_PRFTEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_PRFTEN);
  }
}

/**
  * @brief  This function enables or disables the instruction cache feature.
  * @param  newState - A new state of the instruction cache.
  *          		   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MISC_FLASH_instructionCacheCmd(FunctionalState newState)
{
  // Check the parameters
  macro_prj_assert_param(IS_FUNCTIONAL_STATE(newState));

  if(newState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_ICEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_ICEN);
  }
}

/**
  * @brief  This function enables or disables the data cache feature.
  * @param  newState - A new state of the data cache.
  *          		   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MISC_FLASH_dataCacheCmd(FunctionalState newState)
{
  // Check the parameters
  macro_prj_assert_param(IS_FUNCTIONAL_STATE(newState));

  if(newState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_DCEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_DCEN);
  }
}

/**
  * @brief  This function sets the code latency value.
  * @param  flashLatency - specifies the FLASH Latency value.
  *          			   This parameter can be a value of @ref USH_FLASH_latency.
  *
  * @note For STM32F405xx/407xx, STM32F415xx/417xx, STM32F401xx/411xE/STM32F412xG and STM32F413_423xx devices
  *       this parameter can be a value between FLASH_LATENCY_0 and FLASH_LATENCY_7.
  *
  * @note For STM32F42xxx/43xxx devices this parameter can be a value between
  *       FLASH_LATENCY_0 and FLASH_LATENCY_15.
  *
  * @retval None
  */
void MISC_FLASH_setLatency(USH_FLASH_latency flashLatency)
{
  // Check the parameters
  macro_prj_assert_param(IS_MISC_FLASH_LATENCY(flashLatency));

  // Perform Byte access to FLASH_ACR[8:0] to set the Latency value
  *(__IO uint8_t *)FLASH_ACR_BYTE0_ADDRESS = (uint8_t)flashLatency;
}

//---------------------------------------------------------------------------
// Function's parameters check.
//---------------------------------------------------------------------------

#ifdef USE_FULL_ASSERT

/*!
 * @brief  Reports the name of the source file and the source line number
 *         where the macro_prj_assert_param error has occurred.
 *
 * @param[in] file 		pointer to the source file name
 * @param[in] line 		macro_prj_assert_param error line source number
 * @return None.
 */
void prj_misc_assert_failed(uint8_t* file, uint32_t line)
{
	printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
}

#endif
