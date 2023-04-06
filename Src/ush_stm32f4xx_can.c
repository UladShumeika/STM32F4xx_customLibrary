/**
  ******************************************************************************
  * @file    ush_stm32f4xx_can.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    04 April 2023
  * @brief	 This file contains the implementation of functions for working with CAN bus.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_can.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the CAN peripheral according to the specified parameters
 * 			in the USH_CAN_settingsTypeDef.
 * @param 	initStructure - A pointer to a USH_CAN_settingsTypeDef structure that contains the configuration
 * 							information for the specified CAN peripheral.
 * @retval	The peripheral status.
 */
USH_peripheryStatus CAN_init(USH_CAN_settingsTypeDef* initStructure)
{
	USH_GPIO_initTypeDef initGpioStructure = {0};

	USH_peripheryStatus status = STATUS_OK;

	// Check parameters
	if(initStructure == 0) status = STATUS_ERROR;

	if(status == STATUS_OK)
	{
		// Check parameters
		assert_param(IS_CAN_ALL_INSTANCE(initStructure->CANx));
		assert_param(IS_CAN_BAUDRATE_PRESCALER(initStructure->Timings->BaudratePrescaler));
		assert_param(IS_CAN_TS1(initStructure->Timings->TimeSegment1));
		assert_param(IS_CAN_TS2(initStructure->Timings->TimeSegment2));
		assert_param(IS_CAN_SJW(initStructure->Timings->ResynchJumpWidth));
		assert_param(IS_CAN_MODE(initStructure->Mode));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->TimeTriggeredMode));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->AutoBusOff));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->AutoWakeUp));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->AutoRetransmission));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->ReceiveFifoLocked));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->TransmitFifoPriority));

		/* ----------------------- GPIO configuration -------------------------- */

		// Fill in the initGpioStructure to initialize the GPIO pins, these parameters are used for both CAN
		initGpioStructure.Mode 			= GPIO_MODE_ALTERNATE_PP;
		initGpioStructure.Pull 			= GPIO_NOPULL;
		initGpioStructure.Speed 		= GPIO_SPEED_HIGH;

		if(initStructure->CANx == CAN1)
		{
			// Enable GPIOA clock
			__RCC_GPIOA_CLOCK_ENABLE();

			// CAN1 GPIO pins configuration
			// PA11	   ------> CAN1_RX
			// PA12    ------> CAN1_TX
			initGpioStructure.GPIOx 		= GPIOA;
			initGpioStructure.Pin 			= (GPIO_PIN_11 | GPIO_PIN_12);
			initGpioStructure.Alternate 	= GPIO_AF9_CAN1;
			status = GPIO_init(&initGpioStructure);
		} else	// for CAN2
		{
			// Enable GPIOB clock
			__RCC_GPIOB_CLOCK_ENABLE();

			// CAN1 GPIO pins configuration
			// PB12	   ------> CAN1_RX
			// PB13    ------> CAN1_TX
			initGpioStructure.GPIOx 		= GPIOB;
			initGpioStructure.Pin 			= (GPIO_PIN_12 | GPIO_PIN_13);
			initGpioStructure.Alternate 	= GPIO_AF9_CAN1;
			status = GPIO_init(&initGpioStructure);
		}
	}

	return status;
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Static Functions
//---------------------------------------------------------------------------
