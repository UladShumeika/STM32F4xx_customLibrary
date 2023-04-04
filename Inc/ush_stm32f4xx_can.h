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

//---------------------------------------------------------------------------
// Test macros
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

#endif /* __USH_STM32F4XX_CAN_H */
