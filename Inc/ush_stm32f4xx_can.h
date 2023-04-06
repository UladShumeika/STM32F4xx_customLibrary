/**
  ******************************************************************************
  * @file    ush_stm32f4xx_can.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    04 April 2023
  * @brief   Header file of CAN module.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
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
USH_peripheryStatus CAN_init(USH_CAN_settingsTypeDef* initStructure);

/**
 * @brief  	This function is used to initialize CAN modules global interrupts.
 * @note	This function should not be modified, when global interrupts of CAN modules are required,
 * 			this function must be implemented in the user file.
 * @retval	None.
 */
__WEAK void CAN_initGlobalInterrupts(void);

#endif /* __USH_STM32F4XX_CAN_H */
