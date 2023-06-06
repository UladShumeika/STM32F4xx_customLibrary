/**
  ******************************************************************************
  * @file    ush_stm32f4xx_dma.h
  * @author  Ulad Shumeika
  * @version v1.1
  * @date    04 February 2023
  * @brief   Header file of DMA module.
  *
  * NOTE: This driver is not a full-fledged DMA driver, but contains only some of
  * 	  the functions were necessary at the time of development of this driver.
  * 	  However, if necessary, the function of this driver will be expanded.
  *
  * @Major changes v1.1
  *		- changed code style;
  *		- replaced enumerations with definitions;
  *		- redesigned module;
  *
  ******************************************************************************
  */

#ifndef ush_stm32f4xx_dma_h
#define ush_stm32f4xx_dma_h

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

/*!
 * @name dma_channels
 * @{
 */
#define PRJ_DMA_CHANNEL_0					(0x00000000U)		/*!< DMA Channel 0 */
#define	PRJ_DMA_CHANNEL_1					(0x02000000U)		/*!< DMA Channel 1 */
#define	PRJ_DMA_CHANNEL_2					(0x04000000U)		/*!< DMA Channel 2 */
#define	PRJ_DMA_CHANNEL_3					(0x06000000U)		/*!< DMA Channel 3 */
#define	PRJ_DMA_CHANNEL_4 					(0x08000000U)		/*!< DMA Channel 4 */
#define	PRJ_DMA_CHANNEL_5					(0x0A000000U)		/*!< DMA Channel 5 */
#define	PRJ_DMA_CHANNEL_6					(0x0C000000U)		/*!< DMA Channel 6 */
#define	PRJ_DMA_CHANNEL_7					(0x0E000000U)		/*!< DMA Channel 7 */

/*! @}*/

/*!
 * @name dma_data_transfer_direction
 * @{
 */
#define PRJ_DMA_PERIPH_TO_MEMORY			(0x00000000U)		/*!< Peripheral to memory direction */
#define	PRJ_DMA_MEMORY_TO_PERIPH			(DMA_SxCR_DIR_0)	/*!< Memory to peripheral direction */
#define	PRJ_DMA_MEMORY_TO_MEMORY			(DMA_SxCR_DIR_1)	/*!< Memory to memory direction */

/*! @}*/

/*!
 * @name dma_peripheral_incremented_mode
 * @{
 */
#define PRJ_DMA_PINC_ENABLE					(DMA_SxCR_PINC)		/*!< Peripheral increment mode enable */
#define PRJ_DMA_PINC_DISABLE				(0x00000000U)		/*!< Peripheral increment mode disable */

/*! @}*/

/*!
 * @name dma_memory_incremented_mode
 * @{
 */
#define PRJ_DMA_MINC_ENABLE					(DMA_SxCR_MINC)		/*!< Memory increment mode enable */
#define	PRJ_DMA_MINC_DISABLE				(0x00000000U)		/*!< Memory increment mode disable */

/*! @}*/

/*!
 * @name dma_peripheral_data_size
 * @{
 */
#define PRJ_DMA_PERIPH_SIZE_BYTE			(0x00000000U)  		/*!< Peripheral data alignment: Byte */
#define PRJ_DMA_PERIPH_SIZE_HALFWORD		(DMA_SxCR_PSIZE_0)	/*!< Peripheral data alignment: Half word	*/
#define PRJ_DMA_PERIPH_SIZE_WORD			(DMA_SxCR_PSIZE_1)	/*!< Peripheral data alignment: Word */

/*! @}*/

/*!
 * @name dma_memory_data_size
 * @{
 */
#define PRJ_DMA_MEMORY_SIZE_BYTE			(0x00000000U)		/*!< Memory data alignment: Byte */
#define PRJ_DMA_MEMORY_SIZE_HALFWORD		(DMA_SxCR_MSIZE_0)	/*!< Memory data alignment: Half word */
#define PRJ_DMA_MEMORY_SIZE_WORD			(DMA_SxCR_MSIZE_1)	/*!< Memory data alignment: Word	*/

/*! @}*/

/*!
 * @name dma_mode
 * @{
 */
#define PRJ_DMA_NORMAL_MODE					(0x00000000U)		/*!< Normal mode */
#define	PRJ_DMA_CIRCULAR_MODE				(DMA_SxCR_CIRC)		/*!< Circular mode */
#define	PRJ_DMA_DOUBLE_BUFFERING			(DMA_SxCR_DBM)  	/*!< Double buffering mode */
#define	PRJ_DMA_PERIPH_CTRL_MODE			(DMA_SxCR_PFCTRL)	/*!< Peripheral control mode */

/*! @}*/

/*!
 * @name dma_priority
 * @{
 */
#define PRJ_DMA_PRIORITY_LOW				(0x00000000U)		/*!< Priority level: Low */
#define PRJ_DMA_PRIORITY_MEDIUM				(0x00004000U)		/*!< Priority level: Medium */
#define PRJ_DMA_PRIORITY_HIGH				(0x00008000U)		/*!< Priority level: High */
#define PRJ_DMA_PRIORITY_VERY_HIGH			(0x00030000U)		/*!< Priority level: Low */

/*! @}*/

/*!
 * @name dma_memory_burst
 * @{
 */
#define PRJ_DMA_MBURST_SINGLE				(0x00000000U)		/*!< Single transfer configuration */
#define	PRJ_DMA_MBURST_INCR4				(0x00800000U)		/*!< Incremental burst of 4 beats */
#define PRJ_DMA_MBURST_INCR8				(0x01000000U)		/*!< Incremental burst of 8 beats */
#define PRJ_DMA_MBURST_INCR16				(0x01800000U)		/*!< Incremental burst of 16 beats */

/*! @}*/

/*!
 * @name dma_peripheral_burst
 * @{
 */
#define	PRJ_DMA_PBURST_SINGLE				(0x00000000U)		/*!< Single transfer configuration */
#define	PRJ_DMA_PBURST_INCR4				(0x00200000U)		/*!< Incremental burst of 4 beats */
#define	PRJ_DMA_PBURST_INCR8				(0x00400000U)		/*!< Incremental burst of 8 beats */
#define	PRJ_DMA_PBURST_INCR16				(0x00600000U)		/*!< Incremental burst of 16 beats */

/*! @}*/

/*!
 * @name dma_fifo_mode
 * @{
 */
#define	PRJ_DMA_FIFO_MODE_ENABLE			(0x00000004U)		/*!< FIFO mode enable */
#define PRJ_DMA_FIFO_MODE_DISABLE			(0x00000000U)		/*!< FIFO mode disable */

/*! @}*/

/*!
 * @name dma_fifo_threshold_level
 * @{
 */
#define	PRJ_DMA_FIFO_THRESHOLD_1QUARTER		(0x00000000U)	   /*!< FIFO threshold 1 quart full configuration */
#define	PRJ_DMA_FIFO_THRESHOLD_HALF			(0x00000001U)	   /*!< FIFO threshold half full configuration */
#define PRJ_DMA_FIFO_THRESHOLD_3QUARTER		(0x00000002U)	   /*!< FIFO threshold 3 quart full configuration */
#define PRJ_DMA_FIFO_THRESHOLD_FULL			(0x00000003U)	   /*!< FIFO threshold full configuration */

/*! @}*/

/*!
 * @name dma_flags
 * @{
 */
#define PRJ_DMA_FLAG_FEIF					(0x01U)				/*!< FIFO error interrupt flag */
#define PRJ_DMA_FLAG_DMEIF					(0x04U)				/*!< Direct mode error interrupt flag */
#define PRJ_DMA_FLAG_TEIF					(0x08U)				/*!< Transfer error interrupt flag */
#define PRJ_DMA_FLAG_HTIF					(0x10U)				/*!< Half transfer interrupt flag */
#define PRJ_DMA_FLAG_TCIF					(0x20U)				/*!< Transfer complete interrupt flag */
#define PRJ_DMA_FLAG_ALL					(0x3DU)				/*!< All flags */

/*! @}*/

//---------------------------------------------------------------------------
// Typedefs and enumerations
//---------------------------------------------------------------------------

/**
  * @brief DMA initialization structure definition
  */
typedef struct
{
	uint32_t channel;							/*!< A channel to be used for the specified stream.
											   	     This parameter can be a value of @ref dma_channels */

	uint32_t direction;							/*!< A direction to be used for the specified stream.
											   	     This parameter can be a value of @ref dma_data_transfer_direction */

	uint32_t periph_inc;						/*!< This parameter specifies whether the peripheral address register should
	                                                 be incremented or not. This parameter can be a value of @ref dma_peripheral_incremented_mode */

	uint32_t mem_inc;							/*!< This parameter specifies whether the memory address register should
		 	 	 	 	 	 	 	 	 	 	   	 be incremented or not. This parameter can be a value of @ref dma_memory_incremented_mode */

	uint32_t periph_data_alignment;				/*!< This parameter specifies the peripheral data width.
		 	 	 	 	 	 	 	 	 	 	   	 This parameter can be a value of @ref dma_peripheral_data_size */

	uint32_t mem_data_alignment;				/*!< This parameter specifies the memory data width.
												     This parameter can be a value of @ref dma_memory_data_size */

	uint32_t mode;								/*!< This parameter specifies the operation mode of the selected stream.
		 	 	 	 	 	 	 	 	 	 	   	 This parameter can be a value of @ref dma_mode */

	uint32_t priority;							/*!< This parameter specifies the software priority for the selected stream.
		 	 	 	 	 	 	 	 	 	 	   	 This parameter can be a value of @ref dma_priority */

	uint32_t mem_burst;							/*!< This parameter specifies the burst transfer configuration for
												   	 the memory transfers. It specifies the amount of data to be
												   	 transferred in a single non interruptible transaction.
												   	 This parameter can be a value of @ref dma_memory_burst */

	uint32_t periph_burst;						/*!< This parameter specifies the burst transfer configuration for
	   	   	   	   	   	   	   	   	   	   	   	     the peripheral transfers. It specifies the amount of data to be
	   	   	   	   	   	   	   	   	   	   	   	     transferred in a single non interruptible transaction.
	   	   	   	   	   	   	   	   	   	   	   	     This parameter can be a value of @ref dma_peripheral_burst */

	uint32_t fifo_mode;							/*!< This parameter specifies if the FIFO mode or Direct mode will be used
												   	 for the selected stream. This parameter can be a value of @ref dma_fifo_mode */

	uint32_t fifo_threshold;					/*!< This parameter specifies the FIFO threshold level.
	 	 	 	 	 	 	 	 	 	 	   	   	 This parameter can be a value of @ref dma_fifo_threshold_level */

} prj_dma_init_t;

/*!
 * @brief DMA handler structure definition
 */
typedef struct
{
	DMA_Stream_TypeDef *p_dma_stream;						/*!< A pointer to the stream peripheral.
		 	 	 	 	 	 	 	 	 	 	 	 	 	     DMA can accept DMA1 or DMA2, and stream from 0 to 7 */

	prj_dma_init_t dma_init;								/*!< DMA initialization structure. */

	void *p_controls_peripherals;							/*!< A pointer to the peripheral structure
															     that uses this DMA stream */

	void (*p_complete_callback) (void*);					/*!< A pointer to complete callback function */

	void (*p_error_callback) (void*);						/*!< A pointer to error callback function */

} prj_dma_handler_t;

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
uint32_t prj_dma_init(prj_dma_handler_t *p_dma);

/*!
 * @brief Handle DMA interrupt request.
 *
 * This function is used to handle DMA interrupt request.
 *
 * @param[in] p_dma		A pointer to DMA handler structure.
 *
 * @return None.
 */
void prj_dma_irq_handler(prj_dma_handler_t *p_dma);

#endif /* ush_stm32f4xx_dma_h */
