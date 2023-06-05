/**
  ******************************************************************************
  * @file    ush_stm32f4xx_dma.c
  * @author  Ulad Shumeika
  * @version v1.1
  * @date    04 February 2023
  * @brief   This file contains the implementation of functions for working with DMA.
  *
  * NOTE: This driver is not a full-fledged DMA driver, but contains only some of
  * 	  the functions were necessary at the time of development of this driver.
  * 	  However, if necessary, the function of this driver will be expanded.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_dma.h"
#include "ush_stm32f4xx_conf.h"
#include "stddef.h"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

#define macro_prj_dma_check_instance(instance)    						(((instance) == DMA1_Stream0) || \
																		 ((instance) == DMA1_Stream1) || \
																		 ((instance) == DMA1_Stream2) || \
																		 ((instance) == DMA1_Stream3) || \
																		 ((instance) == DMA1_Stream4) || \
																		 ((instance) == DMA1_Stream5) || \
																		 ((instance) == DMA1_Stream6) || \
																		 ((instance) == DMA1_Stream7) || \
																		 ((instance) == DMA2_Stream0) || \
																		 ((instance) == DMA2_Stream1) || \
																		 ((instance) == DMA2_Stream2) || \
																		 ((instance) == DMA2_Stream3) || \
																		 ((instance) == DMA2_Stream4) || \
																		 ((instance) == DMA2_Stream5) || \
																		 ((instance) == DMA2_Stream6) || \
																		 ((instance) == DMA2_Stream7))

#define macro_prj_dma_check_channel(channel)							(((channel) == PRJ_DMA_CHANNEL_0) || \
																		 ((channel) == PRJ_DMA_CHANNEL_1) || \
																		 ((channel) == PRJ_DMA_CHANNEL_2) || \
																		 ((channel) == PRJ_DMA_CHANNEL_3) || \
																		 ((channel) == PRJ_DMA_CHANNEL_4) || \
																		 ((channel) == PRJ_DMA_CHANNEL_5) || \
																		 ((channel) == PRJ_DMA_CHANNEL_6) || \
																		 ((channel) == PRJ_DMA_CHANNEL_7))

#define macro_prj_dma_check_direction(direction)					    (((direction) == PRJ_DMA_PERIPH_TO_MEMORY) || \
																		 ((direction) == PRJ_DMA_MEMORY_TO_PERIPH) || \
																		 ((direction) == PRJ_DMA_MEMORY_TO_MEMORY))

#define macro_prj_dma_check_periph_inc(command)							(((command) == PRJ_DMA_PINC_ENABLE) || \
													 	 	 	 	 	 ((command) == PRJ_DMA_PINC_DISABLE))

#define macro_prj_dma_check_mem_inc(command)						    (((command) == PRJ_DMA_MINC_ENABLE) || \
													 	 	 	 	 	 ((command) == PRJ_DMA_MINC_DISABLE))

#define macro_prj_dma_check_periph_size(periph_size)					(((periph_size) == PRJ_DMA_PERIPH_SIZE_BYTE) 	 || \
													 	 	 	 	 	 ((periph_size) == PRJ_DMA_PERIPH_SIZE_HALFWORD) || \
																		 ((periph_size) == PRJ_DMA_PERIPH_SIZE_WORD))

#define macro_prj_dma_check_mem_size(mem_size)							(((mem_size) == PRJ_DMA_MEMORY_SIZE_BYTE) 	  || \
													 	 	 	 	 	 ((mem_size) == PRJ_DMA_MEMORY_SIZE_HALFWORD) || \
																		 ((mem_size) == PRJ_DMA_MEMORY_SIZE_WORD))

#define macro_prj_dma_check_mode(mode)									(((mode) == PRJ_DMA_NORMAL_MODE)	  || \
													 	 	 	 	 	 ((mode) == PRJ_DMA_CIRCULAR_MODE)	  || \
																		 ((mode) == PRJ_DMA_DOUBLE_BUFFERING) || \
																		 ((mode) == PRJ_DMA_PERIPH_CTRL_MODE))

#define macro_prj_dma_check_priority(priority)							(((priority) == PRJ_DMA_PRIORITY_LOW) 	 || \
																		 ((priority) == PRJ_DMA_PRIORITY_MEDIUM) || \
																		 ((priority) == PRJ_DMA_PRIORITY_HIGH)	 || \
																		 ((priority) == PRJ_DMA_PRIORITY_VERY_HIGH))

#define macro_prj_dma_check_mburst(mburst)								(((mburst) == PRJ_DMA_MBURST_SINGLE) || \
													 	 	 	 	 	 ((mburst) == PRJ_DMA_MBURST_INCR4)  || \
																		 ((mburst) == PRJ_DMA_MBURST_INCR8)  || \
																		 ((mburst) == PRJ_DMA_MBURST_INCR16))

#define macro_prj_dma_check_pburst(pburst)								(((pburst) == PRJ_DMA_PBURST_SINGLE) || \
													 	 	 	 	 	 ((pburst) == PRJ_DMA_PBURST_INCR4)  || \
																		 ((pburst) == PRJ_DMA_PBURST_INCR8)  || \
																		 ((pburst) == PRJ_DMA_PBURST_INCR16))

#define macro_prj_dma_check_fifo_mode(fifo_mode)						(((fifo_mode) == PRJ_DMA_FIFO_MODE_ENABLE) || \
													 	 	 	 	 	 ((fifo_mode) == PRJ_DMA_FIFO_MODE_DISABLE))

#define macro_prj_dma_check_fifo_threshold(threshold)				 	(((threshold) == PRJ_DMA_FIFO_THRESHOLD_1QUARTER) || \
																		 ((threshold) == PRJ_DMA_FIFO_THRESHOLD_HALF)     || \
																		 ((threshold) == PRJ_DMA_FIFO_THRESHOLD_3QUARTER) || \
																		 ((threshold) == PRJ_DMA_FIFO_THRESHOLD_FULL))

#define macro_prj_dma_check_interrupt_flags(flags)   					((flags) >= PRJ_DMA_FLAG_DMEIF) && \
																		 (flags) <= PRJ_DMA_FLAG_ALL))

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static const uint8_t m_flag_bit_shift_offset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
















//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the DMAx peripheral according to the specified parameters in the USH_DMA_initTypeDef.
 * @param 	initStructure - A pointer to a USH_DMA_initTypeDef structure that contains the configuration
 * 							information for the specified DMA peripheral.
 * @retval	None.
 */
void DMA_init(prj_dma_handler_t *p_dma)
{
	uint32_t tmpReg = 0;

	// check parameters
//	macro_prj_assert_param(IS_DMA_STREAM_ALL_INSTANCE(p_dma->p_dma_init->p_dma_stream));
//	macro_prj_assert_param(IS_DMA_CHANNEL(p_dma->p_dma_init->channel));
//	macro_prj_assert_param(IS_DMA_DIRECTION(initStructure->Direction));
//	macro_prj_assert_param(IS_DMA_PERIPH_INC(initStructure->PeriphInc));
//	macro_prj_assert_param(IS_DMA_MEM_INC(initStructure->MemInc));
//	macro_prj_assert_param(IS_DMA_PERIPH_SIZE(initStructure->PeriphDataAlignment));
//	macro_prj_assert_param(IS_DMA_MEM_SIZE(initStructure->MemDataAlignment));
//	macro_prj_assert_param(IS_DMA_MODE(initStructure->Mode));
//	macro_prj_assert_param(IS_DMA_PRIORITY(initStructure->Priority));
//	macro_prj_assert_param(IS_DMA_MBURST(initStructure->MemBurst));
//	macro_prj_assert_param(IS_DMA_PBURST(initStructure->PeriphBurst));
//	macro_prj_assert_param(IS_DMA_FIFO_MODE(initStructure->FIFOMode));
//	macro_prj_assert_param(IS_DMA_FIFO_THRESHOLD(initStructure->FIFOThreshold));

	DMA_state(p_dma->p_dma_stream, DISABLE);

	// Get the CR register value
	tmpReg = p_dma->p_dma_stream->CR;

	// Clear all bits except PFCTRL, TCIE, HTIE, TEIE, DMEIE, EN
	tmpReg &= 0x3FU;

	// Prepare the DMA Stream configuration
	tmpReg |= p_dma->dma_init.channel 	| p_dma->dma_init.mem_data_alignment 	| p_dma->dma_init.mem_inc |
			  p_dma->dma_init.direction | p_dma->dma_init.periph_data_alignment	| p_dma->dma_init.periph_inc |
			  p_dma->dma_init.mode		| p_dma->dma_init.priority;

	// The memory burst and peripheral burst are not used when the FIFO is disabled
	if(p_dma->dma_init.fifo_mode == PRJ_DMA_FIFO_MODE_ENABLE)
	{
		tmpReg |= p_dma->dma_init.mem_burst | p_dma->dma_init.periph_burst;
	}

	// Write to DMA Stream CR register
	p_dma->p_dma_stream->CR = tmpReg;

	// Get the FCR register value
	tmpReg = p_dma->p_dma_stream->FCR;

	// Clear direct mode and FIFO threshold bits
	tmpReg &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

	// Prepare the DMA stream FIFO configuration
	tmpReg |= p_dma->dma_init.fifo_mode;

	if(p_dma->dma_init.fifo_mode == PRJ_DMA_FIFO_MODE_ENABLE)
	{
		tmpReg |= p_dma->dma_init.fifo_threshold;
	}

	// Write to DMA stream FCR
	p_dma->p_dma_stream->FCR = tmpReg;

	// Clear stream interrupt flags
	DMA_clearFlags(p_dma->p_dma_stream, PRJ_DMA_FLAG_ALL);
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function enables and disables the selected DMA stream.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @param 	state - The state of the selected stream. This parameter can be a value of @ref FunctionalState.
 * @retval	None.
 */
void DMA_state(DMA_Stream_TypeDef *DMAy_Streamx, FunctionalState state)
{
	// Check parameters
//	macro_prj_assert_param(IS_DMA_STREAM_ALL_INSTANCE(DMAy_Streamx));
//	macro_prj_assert_param(IS_FUNCTIONAL_STATE(state));

	if(state == ENABLE)
	{
		DMAy_Streamx->CR |= DMA_SxCR_EN;
	} else
	{
		DMAy_Streamx->CR &= ~DMA_SxCR_EN;
	}
}

/**
 * @brief 	This function clears DMA flags.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @param 	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	None.
 */
void DMA_clearFlags(DMA_Stream_TypeDef *DMAy_Streamx, uint32_t dma_flags)
{
	// Check parameters
//	macro_prj_assert_param(IS_DMA_STREAM_ALL_INSTANCE(DMAy_Streamx));
//	macro_prj_assert_param(IS_DMA_INTERRUPT_FLAGS(dma_flags));

	DMA_TypeDef* DMAy;

	uint32_t streamNumber = ((uint32_t)DMAy_Streamx & 0xFFU) / 0x18U;	// 0xFF is a mask. 0x18 is a step between stream registers.
																		// For a better understanding of magic numbers. See the reference manual.
	DMAy = (DMAy_Streamx < DMA2_Stream0) ? DMA1 : DMA2;

	if(streamNumber < 4U)	// Stream 0-3 is LIFCR and stream 4-6 is HIFCR
	{
		DMAy->LIFCR = dma_flags << m_flag_bit_shift_offset[streamNumber];
	} else
	{
		DMAy->HIFCR = dma_flags << m_flag_bit_shift_offset[streamNumber];
	}
}

/**
 * @brief 	This function gets DMA flags.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	DMA flags.
 */
uint32_t DMA_getFlags(DMA_Stream_TypeDef *p_dma_stream)
{
	// Check parameters
//	macro_prj_assert_param(IS_DMA_STREAM_ALL_INSTANCE(p_dma_stream));

	uint32_t flags = 0;

	DMA_TypeDef* DMAy;

	uint32_t streamNumber = ((uint32_t)p_dma_stream & 0xFFU) / 0x18U;	// 0xFF is a mask. 0x18 is a step between stream registers.
																		// For a better understanding of magic numbers. See the reference manual.
	DMAy = (p_dma_stream < DMA2_Stream0) ? DMA1 : DMA2;

	if(streamNumber < 4U)	// Stream 0-3 is LIFCR and stream 4-6 is HIFCR
	{
		flags = DMAy->LISR >> m_flag_bit_shift_offset[streamNumber];
	} else
	{
		flags = DMAy->HISR >> m_flag_bit_shift_offset[streamNumber];
	}

	return flags;
}

/**
 * @brief 	This function returns number of data items to transfer.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	Number of data items to transfer.
 */
uint16_t DMA_getNumberOfData(DMA_Stream_TypeDef *DMAy_Streamx)
{
	return (DMAy_Streamx->NDTR);
}

/**
 * @brief 	This function handles DMA interrupt request.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	None.
 */
void DMA_IRQHandler(prj_dma_handler_t *p_dma)
{
	// Get interrupt flags
	uint32_t flags = DMA_getFlags(p_dma->p_dma_stream);

	// Clear interrupt flags
	DMA_clearFlags(p_dma->p_dma_stream, PRJ_DMA_FLAG_ALL);

	// Check transfer complete flag
	if((flags & PRJ_DMA_FLAG_TCIF) && (p_dma->p_dma_stream->CR & DMA_SxCR_TCIE))
	{
		DMA_transferCompleteCallback(p_dma->p_dma_stream);
	}

	// Check half transfer complete flag
	if((flags & PRJ_DMA_FLAG_HTIF) && (p_dma->p_dma_stream->CR & DMA_SxCR_HTIE))
	{
		DMA_halfTransferCompleteCallback(p_dma->p_dma_stream);
	}

	// Check transfer error flag
	if((flags & PRJ_DMA_FLAG_TEIF) && (p_dma->p_dma_stream->CR & DMA_SxCR_TEIE))
	{
		DMA_transferErrorCallback(p_dma->p_dma_stream);
	}

	// Check direct mode error flag
	if((flags & PRJ_DMA_FLAG_DMEIF) && (p_dma->p_dma_stream->CR & DMA_SxCR_DMEIE))
	{
		DMA_directModeErrorCallback(p_dma->p_dma_stream);
	}

	// Check FIFO error flag
	if((flags & PRJ_DMA_FLAG_FEIF) && (p_dma->p_dma_stream->FCR & DMA_SxFCR_FEIE))
	{
		DMA_fifoErrorCallback(p_dma->p_dma_stream);
	}
}

//---------------------------------------------------------------------------
// DMA interrupt user callbacks
//---------------------------------------------------------------------------

/**
  * @brief  Tx Transfer completed callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_transferCompleteCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_transferCompleteCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  Tx Half transfer completed callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_halfTransferCompleteCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_halfTransferCompleteCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  Tx transfer error callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_transferErrorCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_transferErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  Direct mode error callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_directModeErrorCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_directModeErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  FIFO error callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_fifoErrorCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_fifoErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}
