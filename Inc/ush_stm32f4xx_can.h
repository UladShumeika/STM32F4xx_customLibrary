/**
  ******************************************************************************
  * @file    ush_stm32f4xx_can.h
  * @author  Ulad Shumeika
  * @version v1.1
  * @date    04 April 2023
  * @brief   Header file of CAN module.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  *
  *
  *
  * @Major changes v1.1
  * 	- periphery status enumeration replaced with definitions.
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4XX_CAN_H
#define __USH_STM32F4XX_CAN_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief CAN the number of time quanta in Time Segment 1 enumeration.
 */
typedef enum
{
	CAN_TS1_TQ1 	= 0x00000U, 	/* 1 time quantum */
	CAN_TS1_TQ2 	= 0x10000U, 	/* 2 time quantum */
	CAN_TS1_TQ3 	= 0x20000U, 	/* 3 time quantum */
	CAN_TS1_TQ4 	= 0x30000U,		/* 4 time quantum */
	CAN_TS1_TQ5 	= 0x40000U, 	/* 5 time quantum */
	CAN_TS1_TQ6 	= 0x50000U, 	/* 6 time quantum */
	CAN_TS1_TQ7 	= 0x60000U, 	/* 7 time quantum */
	CAN_TS1_TQ8 	= 0x70000U, 	/* 8 time quantum */
	CAN_TS1_TQ9 	= 0x80000U, 	/* 9 time quantum */
	CAN_TS1_TQ10 	= 0x90000U,		/* 10 time quantum */
	CAN_TS1_TQ11 	= 0xA0000U,		/* 11 time quantum */
	CAN_TS1_TQ12 	= 0xB0000U,		/* 12 time quantum */
	CAN_TS1_TQ13 	= 0xC0000U,		/* 13 time quantum */
	CAN_TS1_TQ14 	= 0xD0000U,		/* 14 time quantum */
	CAN_TS1_TQ15 	= 0xE0000U,		/* 15 time quantum */
	CAN_TS1_TQ16 	= 0xF0000U		/* 16 time quantum */
} USH_CAN_timeSegment1;

/**
 * @brief CAN the number of time quanta in Time Segment 2 enumeration.
 */
typedef enum
{
	CAN_TS2_TQ1 	= 0x000000U, 	/* 1 time quantum */
	CAN_TS2_TQ2 	= 0x100000U, 	/* 2 time quantum */
	CAN_TS2_TQ3 	= 0x200000U, 	/* 3 time quantum */
	CAN_TS2_TQ4 	= 0x300000U,	/* 4 time quantum */
	CAN_TS2_TQ5 	= 0x400000U, 	/* 5 time quantum */
	CAN_TS2_TQ6 	= 0x500000U, 	/* 6 time quantum */
	CAN_TS2_TQ7 	= 0x600000U, 	/* 7 time quantum */
	CAN_TS2_TQ8 	= 0x700000U	 	/* 8 time quantum */
} USH_CAN_timeSegment2;

/**
 * @brief CAN resynchronization jump width enumeration.
 */
typedef enum
{
	CAN_SJW_TQ1 	= 0x000000U, 	/* 1 time quantum */
	CAN_SJW_TQ2 	= 0x100000U, 	/* 2 time quantum */
	CAN_SJW_TQ3 	= 0x200000U, 	/* 3 time quantum */
	CAN_SJW_TQ4 	= 0x300000U		/* 4 time quantum */
} USH_CAN_resynchJimpWidth;

/**
 * @brief CAN operating modes enumeration.
 */
typedef enum
{
	CAN_MODE_NORMAL				= 0x00U,							/* Normal mode */
	CAN_MODE_LOOPBACK			= CAN_BTR_LBKM,						/* Loopback mode */
	CAN_MODE_SILENT				= CAN_BTR_SILM,						/* Silent mode */
	CAN_MODE_SILENT_LOOPBACK	= (CAN_BTR_LBKM | CAN_BTR_SILM)		/* Loopback combined with silent mode */
} USH_CAN_operatingModes;

/**
 * @brief CAN identifier type enumeration.
 */
typedef enum
{
	CAN_ID_STD		= 0x00UL,	/* Standard id */
	CAN_ID_EXT		= 0x04UL	/* Extended id */
} USH_CAN_identifierType;

/**
 * @brief CAN remote transmission request enumeration.
 */
typedef enum
{
	CAN_RTR_DATA		= 0x00UL,	/* Data frame */
	CAN_RTR_REMOTE		= 0x02UL	/* Remote frame */
} USH_CAN_remoteTransRequest;

/**
 * @brief CAN filter FIFO enumeration.
 */
typedef enum
{
	CAN_FILTER_FIFO_0 	= 0x00UL,	/* Filter FIFO 0 assignment for filter x */
	CAN_FILTER_FIFO_1	= 0x01UL	/* Filter FIFO 1 assignment for filter x */
} USH_CAN_filterFIFO;

/**
 * @brief CAN filter modes enumeration.
 */
typedef enum
{
	CAN_FILTER_MODE_IDMASK 		= 0x00UL,	/* Filter FIFO 0 assignment for filter x */
	CAN_FILTER_MODE_IDLIST		= 0x01UL	/* Filter FIFO 1 assignment for filter x */
} USH_CAN_filterModes;

/**
 * @brief CAN filter scales enumeration.
 */
typedef enum
{
	CAN_FILTERSCALE_16BIT 		= 0x00UL,	/* Two 16-bit filters */
	CAN_FILTERSCALE_32BIT		= 0x01UL	/* One 32-bit filter */
} USH_CAN_filterScales;

/**
 * @brief CAN filter activation enumeration.
 */
typedef enum
{
	CAN_FILTER_DISABLE 		= 0x00UL,	/* Disable filter */
	CAN_FILTER_ENABLE		= 0x01UL	/* Enable filter */
} USH_CAN_filterActivation;



/**
 * @brief CAN timings structure definition.
 */
typedef struct
{
	uint16_t BaudratePrescaler;						/* Baud rate prescaler. This parameter must not exceed 1024. */

	USH_CAN_timeSegment1 TimeSegment1;				/* Time segment 1.
	 	 	 	 	 	 	 	 	 	 	 	 	   This parameter can be a value of @ref USH_CAN_timeSegment1. */

	USH_CAN_timeSegment2 TimeSegment2;				/* Time segment 2.
													   This parameter can be a value of @ref USH_CAN_timeSegment2. */

	USH_CAN_resynchJimpWidth ResynchJumpWidth;		/* Resynchronization jump width.
													   This parameter can be a value of @ref USH_CAN_resynchJimpWidth. */

} USH_CAN_timingsTypeDef;



/**
 * @brief CAN settings structure definition.
 */
typedef struct
{
	CAN_TypeDef* CANx;							/* A pointer to CANx peripheral to be used where x is 1 or 2. */

	USH_CAN_timingsTypeDef Timings;				/* Timings for CAN. */

	USH_CAN_operatingModes Mode;				/* CAN operating mode.
	 	 	 	 	 	 	 	 	 	 	 	   This parameter can be a value of @ref USH_CAN_operatingModes */

	FunctionalState AutoBusOff;          		/* Enable or disable the automatic bus-off management.
	                                               This parameter can be set to ENABLE or DISABLE. */

	FunctionalState AutoWakeUp;          		/* Enable or disable the automatic wake-up mode.
	                                               This parameter can be set to ENABLE or DISABLE. */

	FunctionalState AutoRetransmission;  		/* Enable or disable the non-automatic retransmission mode.
	                                               This parameter can be set to ENABLE or DISABLE. */

	FunctionalState ReceiveFifoLocked;   		/* Enable or disable the Receive FIFO Locked mode.
	                                               This parameter can be set to ENABLE or DISABLE. */

	FunctionalState TransmitFifoPriority;		/* Enable or disable the transmit FIFO priority.
	                                               This parameter can be set to ENABLE or DISABLE. */

} USH_CAN_settingsTypeDef;



/**
 * @brief  CAN filter configuration structure definition.
 */
typedef struct
{
	uint32_t FilterIdHigh;          			/* The filter identification number (MSBs for a 32-bit configuration,
                                       	   	   	   first one for a 16-bit configuration). This parameter must be
                                       	   	   	   a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	uint32_t FilterIdLow;           			/* The filter identification number (LSBs for a 32-bit configuration,
                                       	   	   	   second one for a 16-bit configuration). This parameter must be
                                       	   	   	   a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	uint32_t FilterMaskIdHigh;      			/* The filter mask number or identification number, according to
                                       	   	   	   the mode (MSBs for a 32-bit configuration, first one for a 16-bit
                                       	   	   	   configuration). This parameter must be a number between
                                       	   	   	   Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	uint32_t FilterMaskIdLow;      				/* The filter mask number or identification number, according to
                                       	   	   	   the mode (LSBs for a 32-bit configuration, second one
                                       	   	   	   for a 16-bit configuration). This parameter must be a number
                                       	   	   	   between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	USH_CAN_filterFIFO FilterFIFOAssignment;  	/* The FIFO (0 or 1U) which will be assigned to the filter.
                                       	   	   	   This parameter can be a value of @ref USH_CAN_filterFIFO */

	uint32_t FilterBank;            			/* The filter bank which will be initialized. For single
												   CAN instance(14 dedicated filter banks), this parameter must
                                       	   	   	   be a number between Min_Data = 0 and  Max_Data = 13. For dual
                                       	   	   	   CAN instances(28 filter banks shared), this parameter must be
                                       	   	   	   a number between Min_Data = 0 and Max_Data = 27. */

	USH_CAN_filterModes FilterMode;            	/* The filter mode to be initialized.
                                       	   	   	   This parameter can be a value of @ref USH_CAN_filterModes. */

	USH_CAN_filterScales FilterScale;           /* The filter scale.
                                       	   	   	   This parameter can be a value of @ref USH_CAN_filterScales. */

	USH_CAN_filterActivation FilterActivation;	/* Enable or disable the filter.
                                       	   	   	   This parameter can be a value of @ref USH_CAN_filterActivation. */

	uint32_t SlaveStartFilterBank;  			/* Select the start filter bank for the slave CAN instance.
                                       	   	   	   For single CAN instances, this parameter is meaningless.
                                       	   	   	   For dual CAN instances, all filter banks with lower index are
                                       	   	   	   assigned to master CAN instance, whereas all filter banks with
                                       	   	   	   greater index are assigned to slave CAN instance.
                                       	   	   	   This parameter must be a number between Min_Data = 0 and
                                       	   	   	   Max_Data = 27. */

} USH_CAN_filterTypeDef;



/**
 * @brief CAN Tx message header structure definition.
 */
typedef struct
{
	uint32_t StdId;    					/* The standard identifier.
                          	  	  	  	   This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

	uint32_t ExtId;    					/* The extended identifier.
                          	  	  	  	   This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

	USH_CAN_identifierType IDE;      	/* The type of identifier for the message that will be transmitted.
                          	  	  	  	   This parameter can be a value of @ref USH_CAN_identifierType */

	USH_CAN_remoteTransRequest RTR;     /* The type of frame for the message that will be transmitted.
                          	  	  	  	   This parameter can be a value of @ref USH_CAN_remoteTransRequest */

	uint32_t DLC;      					/* The length of the frame that will be transmitted.
                          	  	  	  	   This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

} USH_CAN_txHeaderTypeDef;



/**
 * @brief CAN Rx message header structure definition.
 */
typedef struct
{
	uint32_t StdId;    					/* The standard identifier.
                          	  	  	  	   This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

	uint32_t ExtId;    					/* The extended identifier.
                          	  	  	  	   This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

	USH_CAN_identifierType IDE;      	/* The type of identifier for the message that will be transmitted.
                          	  	  	  	   This parameter can be a value of @ref USH_CAN_identifierType */

	USH_CAN_remoteTransRequest RTR;     /* The type of frame for the message that will be transmitted.
                          	  	  	  	   This parameter can be a value of @ref USH_CAN_remoteTransRequest */

	uint32_t DLC;      					/* The length of the frame that will be transmitted.
                          	  	  	  	   This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

	uint32_t FilterMatchIndex;			/* The index of matching acceptance filter element.
										   This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF. */

} USH_CAN_rxHeaderTypeDef;

/**
 * @brief CAN interrupts enumeration.
 */
typedef enum
{
	CAN_IT_SLEEP 					= CAN_IER_SLKIE,		// Sleep interrupt
	CAN_IT_WAKEUP					= CAN_IER_WKUIE,		// Wake-up interrupt
	CAN_IT_ERROR					= CAN_IER_ERRIE,		// Error interrupt
	CAN_IT_LAST_ERROR_CODE			= CAN_IER_LECIE,		// Last error code interrupt
	CAN_IT_BUSOFF					= CAN_IER_BOFIE,		// Bus-off interrupt
	CAN_IT_ERROR_PASSIVE			= CAN_IER_EPVIE, 		// Error passive interrupt
	CAN_IT_ERROR_WARNING			= CAN_IER_EWGIE,		// Error warning interrupt
	CAN_IT_RX_FIFO1_OVERRUN			= CAN_IER_FOVIE1,		// FIFO 1 overrun interrupt
	CAN_IT_RX_FIFO1_FULL			= CAN_IER_FFIE1, 		// FIFO 1 full interrupt
	CAN_IT_RX_FIFO1_MSG_PENDING		= CAN_IER_FMPIE1,		// FIFO 1 message pending interrupt
	CAN_IT_RX_FIFO0_OVERRUN			= CAN_IER_FOVIE0, 		// FIFO 0 overrun interrupt
	CAN_IT_RX_FIFO0_FULL			= CAN_IER_FFIE0,		// FIFO 0 message pending interrupt
	CAN_IT_RX_FIFO0_MSG_PENDING		= CAN_IER_FMPIE0,		// FIFO 0 message pending interrupt
	CAN_IT_TX_MAILBOX_EMPTY			= CAN_IER_TMEIE			// Transmit mailbox empty interrupt
} USH_CAN_interrupts;

/**
 * @brief CAN flags enumeration.
 */
typedef enum
{
	CAN_FLAG_RQCP0		= 0x00U,		// Request complete MailBox 0 Flag
	CAN_FLAG_TXOK0		= 0x01U,		// Transmission OK MailBox 0 Flag
	CAN_FLAG_ALST0		= 0x02U,		// Arbitration Lost MailBox 0 Flag
	CAN_FLAG_TERR0		= 0x03U,		// Transmission error MailBox 0 Flag

	CAN_FLAG_RQCP1		= 0x08U,		// Request complete MailBox 1 Flag
	CAN_FLAG_TXOK1		= 0x09U,		// Transmission OK MailBox 1 Flag
	CAN_FLAG_ALST1		= 0x0AU,		// Arbitration Lost MailBox 1 Flag
	CAN_FLAG_TERR1		= 0x0BU,		// Transmission error MailBox 1 Flag

	CAN_FLAG_RQCP2		= 0x10U,		// Request complete MailBox 2 Flag
	CAN_FLAG_TXOK2		= 0x11U,		// Transmission OK MailBox 2 Flag
	CAN_FLAG_ALST2		= 0x12U,		// Arbitration Lost MailBox 2 Flag
	CAN_FLAG_TERR2		= 0x13U,		// Transmission error MailBox 2 Flag

	CAN_FLAG_FF0		= 0x103U,   	// RX FIFO 0 Full Flag
	CAN_FLAG_FOV0		= 0x104U,		// RX FIFO 0 Overrun Flag
	CAN_FLAG_FF1		= 0x203U,		// RX FIFO 1 Full Flag
	CAN_FLAG_FOV1		= 0x204U,		// RX FIFO 1 Overrun Flag
} USH_CAN_flags;

//---------------------------------------------------------------------------
// Test macros
//---------------------------------------------------------------------------
#define IS_CAN_BAUDRATE_PRESCALER(PRESCALER) 	(((PRESCALER) >= 1U) && ((PRESCALER) <= 1024U))

#define IS_CAN_TS1(TQ) 							(((TQ) == CAN_TS1_1TQ)  || ((TQ) == CAN_TS1_2TQ)  || \
                         	 	 	 	 	 	 ((TQ) == CAN_TS1_3TQ)  || ((TQ) == CAN_TS1_4TQ)  || \
												 ((TQ) == CAN_TS1_5TQ)  || ((TQ) == CAN_TS1_6TQ)  || \
												 ((TQ) == CAN_TS1_7TQ)  || ((TQ) == CAN_TS1_8TQ)  || \
												 ((TQ) == CAN_TS1_9TQ)  || ((TQ) == CAN_TS1_10TQ) || \
												 ((TQ) == CAN_TS1_11TQ) || ((TQ) == CAN_TS1_12TQ) || \
												 ((TQ) == CAN_TS1_13TQ) || ((TQ) == CAN_TS1_14TQ) || \
												 ((TQ) == CAN_TS1_15TQ) || ((TQ) == CAN_TS1_16TQ))

#define IS_CAN_TS2(TQ) 							(((TQ) == CAN_TS1_1TQ)  || ((TQ) == CAN_TS1_2TQ)  || \
                         	 	 	 	 	 	 ((TQ) == CAN_TS1_3TQ)  || ((TQ) == CAN_TS1_4TQ)  || \
												 ((TQ) == CAN_TS1_5TQ)  || ((TQ) == CAN_TS1_6TQ)  || \
												 ((TQ) == CAN_TS1_7TQ)  || ((TQ) == CAN_TS1_8TQ)

#define IS_CAN_SJW(TQ) 							(((TQ) == CAN_SJW_1TQ) || ((TQ) == CAN_SJW_2TQ) || \
                         	 	 	 	 	 	 ((TQ) == CAN_SJW_3TQ) || ((TQ) == CAN_SJW_4TQ))

#define IS_CAN_MODE(MODE) 						(((MODE) == CAN_MODE_NORMAL)   || \
                           	   	   	   	   	   	 ((MODE) == CAN_MODE_LOOPBACK) || \
												 ((MODE) == CAN_MODE_SILENT)   || \
												 ((MODE) == CAN_MODE_SILENT_LOOPBACK))

#define IS_CAN_STDID(STDID)   					((STDID) <= 0x7FFU)

#define IS_CAN_EXTID(EXTID)   					((EXTID) <= 0x1FFFFFFFU)

#define IS_CAN_IDTYPE(IDTYPE)  					(((IDTYPE) == CAN_ID_STD) || \
                                				 ((IDTYPE) == CAN_ID_EXT))

#define IS_CAN_RTR(RTR) 						(((RTR) == CAN_RTR_DATA) || ((RTR) == CAN_RTR_REMOTE))

#define IS_CAN_DLC(DLC)       					((DLC) <= 8U)

#define IS_CAN_FILTER_ID_HALFWORD(HALFWORD) 	((HALFWORD) <= 0xFFFFU)

#define IS_CAN_FILTER_FIFO(FIFO) 				(((FIFO) == CAN_FILTER_FIFO0) || \
                                  	  	  	  	 ((FIFO) == CAN_FILTER_FIFO1))

#define IS_CAN_FILTER_BANK_DUAL(BANK) 			((BANK) <= 27U)

#define IS_CAN_FILTER_MODE(MODE) 				(((MODE) == CAN_FILTERMODE_IDMASK) || \
                                  	  	  	  	 ((MODE) == CAN_FILTERMODE_IDLIST))

#define IS_CAN_FILTER_SCALE(SCALE) 				(((SCALE) == CAN_FILTERSCALE_16BIT) || \
                                    			 ((SCALE) == CAN_FILTERSCALE_32BIT))

#define IS_CAN_FILTER_ACTIVATION(ACTIVATION) 	(((ACTIVATION) == CAN_FILTER_DISABLE) || \
                                              	 ((ACTIVATION) == CAN_FILTER_ENABLE))

#define IS_CAN_INTERRUPT(INTERRUPT) 			((INTERRUPT) <= (CAN_IT_SLEEP     		 	 | \
																 CAN_IT_WAKEUP               | \
													 	 	 	 CAN_IT_ERROR        	 	 | \
																 CAN_IT_LAST_ERROR_CODE      | \
																 CAN_IT_BUSOFF 			 	 | \
																 CAN_IT_ERROR_PASSIVE        | \
																 CAN_IT_ERROR_WARNING    	 | \
																 CAN_IT_RX_FIFO1_OVERRUN     | \
																 CAN_IT_RX_FIFO1_FULL    	 | \
																 CAN_IT_RX_FIFO1_MSG_PENDING | \
																 CAN_IT_RX_FIFO0_OVERRUN     | \
																 CAN_IT_RX_FIFO0_FULL        | \
																 CAN_IT_RX_FIFO0_MSG_PENDING | \
																 CAN_IT_TX_MAILBOX_EMPTY))

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the CAN peripheral according to the specified parameters
 * 			in the USH_CAN_settingsTypeDef.
 * @param 	initStructure - A pointer to a USH_CAN_settingsTypeDef structure that contains the configuration
 * 							information for the specified CAN peripheral.
 * @retval	The peripheral status.
 */
uint32_t CAN_init(USH_CAN_settingsTypeDef* initStructure);

/**
 * @brief 	This function configures the CAN reception filter according to the specified parameters
 *          in the USH_CAN_filterTypeDef.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	initFilterStructure - A pointer to a USH_CAN_filterTypeDef structure.
 * @return	The peripheral status.
 */
uint32_t CAN_filtersConfig(CAN_TypeDef* can, USH_CAN_filterTypeDef* initFilterStructure);

/**
 * @brief 	This function is used to enable the specified CAN module.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @retval	The peripheral status.
 */
uint32_t CAN_enable(CAN_TypeDef* can);

/**
 * @brief  	This function is used to initialize CAN modules global interrupts.
 * @note	This function should not be modified, when global interrupts of CAN modules are required,
 * 			this function must be implemented in the user file.
 * @retval	None.
 */
__WEAK void CAN_initGlobalInterrupts(void);

/**
 * @brief 	This function is used to add a message to the first free Tx mailbox and activate
 * 		  	the corresponding transmission request.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	pHeader - A pointer to a CAN_TxHeaderTypeDef structure.
 * @param 	pData - A pointer to an array containing the payload of the Tx frame.
 * @retval	The peripheral status.
 */
uint32_t CAN_addTxMessage(CAN_TypeDef* can, USH_CAN_txHeaderTypeDef *pHeader, uint8_t* pData);

/**
 * @brief 	This function is used to get an CAN frame from the Rx FIFO zone into the message RAM.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	rxFifo - A number of FIFO mailbox.
 * @param 	pHeader - A pointer to a CAN_RxHeaderTypeDef structure.
 * @param 	pData - A pointer to an array containing the payload of the Rx frame.
 * @retval	None.
 */
void CAN_getRxMessage(CAN_TypeDef* can, USH_CAN_filterFIFO rxFifo, USH_CAN_rxHeaderTypeDef* pHeader, uint8_t* pData);

/**
 * @brief	This function is used to enable the specified interrupt.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	interrupt - Interrupts to be enabled.
 * @retval	None.
 */
void CAN_interruptEnable(CAN_TypeDef* can, USH_CAN_interrupts interrupt);

/**
 * @brief	This function is used to disable the specified interrupt.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	interrupt - Interrupts to be enabled.
 * @retval	None.
 */
void CAN_interruptDisable(CAN_TypeDef* can, USH_CAN_interrupts interrupt);

/**
 * @brief	This function clears the specified CAN pending flag.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	flag - A flag to clear.
 * @retval	None.
 */
void CAN_clearFlag(CAN_TypeDef* can, uint32_t flag);

/**
 * @brief 	This function handles CAN interrupt request.
 * @param 	initStructure - A pointer to a USH_CAN_settingsTypeDef structure that contains the configuration information
 * 							for the specified CAN peripheral.
 * @retval	None.
 */
void CAN_IRQHandler(USH_CAN_settingsTypeDef *initStructure);

//---------------------------------------------------------------------------
// CAN interrupt user callbacks
//---------------------------------------------------------------------------
__WEAK void CAN_txMailbox0CompleteCallback(CAN_TypeDef* can);
__WEAK void CAN_txMailbox1CompleteCallback(CAN_TypeDef* can);
__WEAK void CAN_txMailbox2CompleteCallback(CAN_TypeDef* can);
__WEAK void CAN_rxFifo0MsgPendingCallback(CAN_TypeDef* can);

#endif /* __USH_STM32F4XX_CAN_H */
