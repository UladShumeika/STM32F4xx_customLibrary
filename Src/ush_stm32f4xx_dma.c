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

#define macro_prj_dma_check_interrupt_flags(flags)   					(((flags) >= PRJ_DMA_FLAG_FEIF) && \
																		 ((flags) <= PRJ_DMA_FLAG_ALL))

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static const uint8_t m_flag_bit_shift_offset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static uint32_t dma_clear_flags(DMA_Stream_TypeDef *p_dma_stream, uint32_t dma_flags);
static uint32_t dma_get_flags(DMA_Stream_TypeDef *p_dma_stream);

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize DMA peripherals.
 *
 * This function is used to initialize DMA peripherals.
 *
 * @param[in] p_dma		A pointer to DMA handler structure.
 *
 * @return @ref PRJ_STATUS_OK if DMA initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
uint32_t prj_dma_init(prj_dma_handler_t *p_dma)
{
	uint32_t status = PRJ_STATUS_OK;
	uint32_t tmp_reg = 0U;

	/* Check the pointer */
	if(p_dma == NULL)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

	/* Check parameters */
	macro_prj_assert_param(macro_prj_dma_check_instance(p_dma->p_dma_stream));
	macro_prj_assert_param(macro_prj_dma_check_channel(p_dma->dma_init.channel));
	macro_prj_assert_param(macro_prj_dma_check_direction(p_dma->dma_init.direction));
	macro_prj_assert_param(macro_prj_dma_check_periph_inc(p_dma->dma_init.periph_inc));
	macro_prj_assert_param(macro_prj_dma_check_mem_inc(p_dma->dma_init.mem_inc));
	macro_prj_assert_param(macro_prj_dma_check_periph_size(p_dma->dma_init.periph_data_alignment));
	macro_prj_assert_param(macro_prj_dma_check_mem_size(p_dma->dma_init.mem_data_alignment));
	macro_prj_assert_param(macro_prj_dma_check_mode(p_dma->dma_init.mode));
	macro_prj_assert_param(macro_prj_dma_check_priority(p_dma->dma_init.priority));
	macro_prj_assert_param(macro_prj_dma_check_mburst(p_dma->dma_init.mem_burst));
	macro_prj_assert_param(macro_prj_dma_check_pburst(p_dma->dma_init.periph_burst));
	macro_prj_assert_param(macro_prj_dma_check_fifo_mode(p_dma->dma_init.fifo_mode));
	macro_prj_assert_param(macro_prj_dma_check_fifo_threshold(p_dma->dma_init.fifo_threshold));

	/* Disable DMA peripherals */
	p_dma->p_dma_stream->CR &= ~DMA_SxCR_EN;

  	/* Get the CR register value */
	tmp_reg = p_dma->p_dma_stream->CR;

  	/* Clear all bits except PFCTRL, TCIE, HTIE, TEIE, DMEIE, EN */
	tmp_reg &= 0x3FU;

  	/* Prepare the DMA Stream configuration */
	tmp_reg |= p_dma->dma_init.channel 	 | p_dma->dma_init.mem_data_alignment 	 | p_dma->dma_init.mem_inc |
  			   p_dma->dma_init.direction | p_dma->dma_init.periph_data_alignment | p_dma->dma_init.periph_inc |
  			   p_dma->dma_init.mode		 | p_dma->dma_init.priority;

  	/* The memory burst and peripheral burst are not used when the FIFO is disabled */
  	if(p_dma->dma_init.fifo_mode == PRJ_DMA_FIFO_MODE_ENABLE)
  	{
  		tmp_reg |= p_dma->dma_init.mem_burst | p_dma->dma_init.periph_burst;
  	}
  	else
  	{
  		/* DO NOTHING */
  	}

  	/* Write to DMA Stream CR register */
  	p_dma->p_dma_stream->CR = tmp_reg;

	/* Get the FCR register value */
  	tmp_reg = p_dma->p_dma_stream->FCR;

  	/* Clear direct mode and FIFO threshold bits */
  	tmp_reg &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  	/* Prepare the DMA stream FIFO configuration */
  	tmp_reg |= p_dma->dma_init.fifo_mode;

  	if(p_dma->dma_init.fifo_mode == PRJ_DMA_FIFO_MODE_ENABLE)
  	{
  		tmp_reg |= p_dma->dma_init.fifo_threshold;
  	}
  	else
  	{
  		/* DO NOTHING */
  	}

  	/* Write to DMA stream FCR */
  	p_dma->p_dma_stream->FCR = tmp_reg;

  	/* Clear stream interrupt flags */
  	dma_clear_flags(p_dma->p_dma_stream, PRJ_DMA_FLAG_ALL);

	return status;
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Clear DMA interrupt flags
 *
 * This function is used to clear DMA interrupt flags.
 *
 * @param[in] p_dma_stream		A pointer to Stream peripheral.
 * @param[in] dma_flags			DMA interrupt flags.
 *
 * @return @ref PRJ_STATUS_OK if DMA flags reset was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t dma_clear_flags(DMA_Stream_TypeDef *p_dma_stream, uint32_t dma_flags)
{
	uint32_t status = PRJ_STATUS_OK;
	DMA_TypeDef* p_dma;
	uint32_t stream_number = 0U;

	/* Check the pointer */
	if(p_dma_stream == NULL)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Check parameters */
		macro_prj_assert_param(macro_prj_dma_check_instance(p_dma_stream));
		macro_prj_assert_param(macro_prj_dma_check_interrupt_flags(dma_flags));

		stream_number = ((uint32_t)p_dma_stream & 0xFFU) / 0x18U;	/* 0xFF is a mask. 0x18 is a step between stream registers.
																	   For a better understanding of magic numbers.
																	   See the reference manual. */
		p_dma = (p_dma_stream < DMA2_Stream0) ? DMA1 : DMA2;

		if(stream_number < 4U)	// Stream 0-3 is LIFCR and stream 4-6 is HIFCR
		{
			p_dma->LIFCR = dma_flags << m_flag_bit_shift_offset[stream_number];
		}
		else
		{
			p_dma->HIFCR = dma_flags << m_flag_bit_shift_offset[stream_number];
		}
	}
	else
	{
		/* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Get DMA interrupt flags
 *
 * This function is used to get DMA interrupt flags.
 *
 * @param[in] p_dma_stream		A pointer to Stream peripheral.
 *
 * @return DMA interrupt flags.
 */
static uint32_t dma_get_flags(DMA_Stream_TypeDef *p_dma_stream)
{
	uint32_t status = PRJ_STATUS_OK;
	DMA_TypeDef* p_dma;
	uint32_t stream_number = 0U;
	uint32_t dma_flags = 0U;

	/* Check the pointer */
	if(p_dma_stream == NULL)
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

	if(status == PRJ_STATUS_OK)
	{
		/* Check parameters */
		macro_prj_assert_param(macro_prj_dma_check_instance(p_dma_stream));

		stream_number = ((uint32_t)p_dma_stream & 0xFFU) / 0x18U;	/* 0xFF is a mask. 0x18 is a step between stream registers.
																	   For a better understanding of magic numbers.
																	   See the reference manual. */
		p_dma = (p_dma_stream < DMA2_Stream0) ? DMA1 : DMA2;

		if(stream_number < 4U)	// Stream 0-3 is LIFCR and stream 4-6 is HIFCR
		{
			dma_flags = p_dma->LISR >> m_flag_bit_shift_offset[stream_number];
		} else
		{
			dma_flags = p_dma->HISR >> m_flag_bit_shift_offset[stream_number];
		}
	}
	else
	{
		/* DO NOTHING */
	}

	return dma_flags;
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
