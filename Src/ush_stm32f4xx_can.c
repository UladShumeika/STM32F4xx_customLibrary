/**
  ******************************************************************************
  * @file    ush_stm32f4xx_can.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    04 April 2023
  * @brief	 This file contains the implementation of functions for working with CAN bus.
  *
  * NOTE: This file is not a full-fledged CAN driver, but contains only some of
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
#define CAN_TIMEOUT_VALUE		(10U)

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
	uint32_t ticksStart = 0;

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
		assert_param(IS_FUNCTIONAL_STATE(initStructure->AutoBusOff));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->AutoWakeUp));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->AutoRetransmission));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->ReceiveFifoLocked));
		assert_param(IS_FUNCTIONAL_STATE(initStructure->TransmitFifoPriority));

		/* -------------------------- GPIO configuration -------------------------- */

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

		/* -------------------------- CAN interrupts configuration ---------------- */

		if(status == STATUS_OK)
		{
			CAN_initGlobalInterrupts();
		}

		/* -------------------------- CAN configuration --------------------------- */

		if(status == STATUS_OK)
		{
			// Request initialization
			initStructure->CANx->MCR |= CAN_MCR_INRQ;

			// Wait till the CAN initialization mode is enabled
			ticksStart = MISC_timeoutGetTick();
			while(((initStructure->CANx->MSR) & CAN_MSR_INAK) != CAN_MSR_INAK)
			{
				if((MISC_timeoutGetTick() - ticksStart) > CAN_TIMEOUT_VALUE) status = STATUS_TIMEOUT;
			}

			// Exit from sleep mode (see Figure 336 RM0090)
			initStructure->CANx->MCR &= ~CAN_MCR_SLEEP;

			// Wait till the CAN sleep mode is disabled
			ticksStart = MISC_timeoutGetTick();
			while(((initStructure->CANx->MSR) & CAN_MSR_SLAK) != CAN_MSR_SLAK)
			{
				if((MISC_timeoutGetTick() - ticksStart) > CAN_TIMEOUT_VALUE) status = STATUS_TIMEOUT;
			}

			// Set the automatic bus-off management
			if(initStructure->AutoBusOff == ENABLE)
			{
				initStructure->CANx->MCR |= CAN_MCR_ABOM;
			}
			else
			{
				initStructure->CANx->MCR &= ~CAN_MCR_ABOM;
			}

			// Set the automatic wake-up mode
			if(initStructure->AutoWakeUp == ENABLE)
			{
				initStructure->CANx->MCR |= CAN_MCR_AWUM;
			}
			else
			{
				initStructure->CANx->MCR &= ~CAN_MCR_AWUM;
			}

			// Set the automatic retransmission
			if(initStructure->AutoRetransmission == ENABLE)
			{
				initStructure->CANx->MCR |= CAN_MCR_NART;
			}
			else
			{
				initStructure->CANx->MCR &= ~CAN_MCR_NART;
			}

			// Set the receive FIFO locked mode
			if(initStructure->ReceiveFifoLocked == ENABLE)
			{
				initStructure->CANx->MCR |= CAN_MCR_RFLM;
			}
			else
			{
				initStructure->CANx->MCR &= ~CAN_MCR_RFLM;
			}

			// Set the transmit FIFO priority
			if (initStructure->TransmitFifoPriority == ENABLE)
			{
				initStructure->CANx->MCR |= CAN_MCR_TXFP;
			}
			else
			{
				initStructure->CANx->MCR &= ~CAN_MCR_TXFP;
			}

			// Set the bit timing register
			initStructure->CANx->BTR = ((initStructure->Timings.BaudratePrescaler - 1) | \
									   initStructure->Timings.TimeSegment1			    | \
									   initStructure->Timings.TimeSegment2			    | \
									   initStructure->Timings.ResynchJumpWidth);
		}
	}

	return status;
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief  	This function is used to initialize CAN modules global interrupts.
 * @note	This function should not be modified, when global interrupts of CAN modules are required,
 * 			this function must be implemented in the user file.
 * @retval	None.
 */
__WEAK void CAN_initGlobalInterrupts(void)
{

}

/**
 * @brief 	This function is used to add a message to the first free Tx mailbox and activate
 * 		  	the corresponding transmission request.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	pHeader - A pointer to a CAN_TxHeaderTypeDef structure.
 * @param 	pData - A pointer to an array containing the payload of the Tx frame.
 * @retval	The peripheral status.
 */
USH_peripheryStatus CAN_addTxMessage(CAN_TypeDef* can, CAN_TxHeaderTypeDef *pHeader, uint8_t* pData)
{
	USH_peripheryStatus status = STATUS_OK;

	uint32_t tsrReg = can->TSR;
	uint32_t transmitMailbox = 0;

	// Check parameters
	assert_param(IS_CAN_IDTYPE(pHeader->IDE));
	assert_param(IS_CAN_RTR(pHeader->RTR));
	assert_param(IS_CAN_DLC(pHeader->DLC));

	if(pHeader->IDE == CAN_ID_STD)
	{
		assert_param(IS_CAN_STDID(pHeader->StdId));
	}
	else
	{
		assert_param(IS_CAN_EXTID(pHeader->ExtId));
	}

	// Check that all the Tx mailboxes are not full
	if((tsrReg & CAN_TSR_TME) != 0)
	{
		// Select an empty transmit mailbox
		transmitMailbox = (tsrReg & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

		// Check transmit mailbox value
		if(transmitMailbox > 2) status = STATUS_ERROR;

		// Set up the ID
		if(pHeader->IDE == CAN_ID_STD)
		{
			can->sTxMailBox[transmitMailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) | pHeader->RTR);
		}else
		{
			can->sTxMailBox[transmitMailbox].TIR = (pHeader->ExtId << CAN_TI0R_EXID_Pos) | pHeader->RTR | pHeader->IDE;
		}

		// Set up the DLC
		can->sTxMailBox[transmitMailbox].TDTR = pHeader->DLC;

		// Set up the data field
		can->sTxMailBox[transmitMailbox].TDHR = pData[7] << CAN_TDH0R_DATA7_Pos |
												pData[6] << CAN_TDH0R_DATA6_Pos |
												pData[5] << CAN_TDH0R_DATA5_Pos |
												pData[4] << CAN_TDH0R_DATA4_Pos;

		can->sTxMailBox[transmitMailbox].TDLR = pData[3] << CAN_TDL0R_DATA3_Pos |
												pData[2] << CAN_TDL0R_DATA2_Pos |
												pData[1] << CAN_TDL0R_DATA1_Pos |
												pData[0] << CAN_TDL0R_DATA0_Pos;

		// Request transmission
		can->sTxMailBox[transmitMailbox].TIR |= CAN_TI0R_TXRQ;
	}

	return status;
}

//---------------------------------------------------------------------------
// Static Functions
//---------------------------------------------------------------------------
