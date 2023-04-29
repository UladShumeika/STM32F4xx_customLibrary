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
#define CAN_FLAG_MASK			(0xFFU)

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
			initGpioStructure.Alternate 	= GPIO_AF9_CAN2;
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
			// Enable CAN1 clock
			__RCC_CAN1_CLOCK_ENABLE();

			// Request initialization
			initStructure->CANx->MCR |= CAN_MCR_INRQ;

			// Wait till the CAN initialization mode is enabled
			ticksStart = MISC_timeoutGetTick();
			while(((initStructure->CANx->MSR) & CAN_MSR_INAK) != CAN_MSR_INAK)
			{
				if((MISC_timeoutGetTick() - ticksStart) > CAN_TIMEOUT_VALUE)
				{
					status = STATUS_TIMEOUT;
					break;
				}
			}

			// Exit from sleep mode (see Figure 336 RM0090)
			initStructure->CANx->MCR &= ~CAN_MCR_SLEEP;

			// Wait till the CAN sleep mode is disabled
			ticksStart = MISC_timeoutGetTick();
			while(((initStructure->CANx->MSR) & CAN_MSR_SLAK) != 0U)
			{
				if((MISC_timeoutGetTick() - ticksStart) > CAN_TIMEOUT_VALUE)
				{
					status = STATUS_TIMEOUT;
					break;
				}
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
			initStructure->CANx->BTR = (initStructure->Mode							  | \
									   (initStructure->Timings.BaudratePrescaler - 1) | \
									   initStructure->Timings.TimeSegment1			  | \
									   initStructure->Timings.TimeSegment2			  | \
									   initStructure->Timings.ResynchJumpWidth);
		}
	}

	return status;
}

/**
 * @brief 	This function configures the CAN reception filter according to the specified parameters
 *          in the USH_CAN_filterTypeDef.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	initFilterStructure - A pointer to a USH_CAN_filterTypeDef structure.
 * @retval	The peripheral status.
 */
USH_peripheryStatus CAN_filtersConfig(CAN_TypeDef* can, USH_CAN_filterTypeDef* initFilterStructure)
{
	USH_peripheryStatus status = STATUS_OK;
	CAN_TypeDef *can_ip = can;

	uint32_t filterNumberBitPos = 0;

	if(can_ip == CAN2) can_ip = CAN1;

	// Check parameters
	if(initFilterStructure == 0) status = STATUS_ERROR;

	if(status == STATUS_OK)
	{
		// Check parameters
		assert_param(IS_CAN_FILTER_ID_HALFWORD(initFilterStructure->FilterIdHigh));
		assert_param(IS_CAN_FILTER_ID_HALFWORD(initFilterStructure->FilterIdLow));
		assert_param(IS_CAN_FILTER_ID_HALFWORD(initFilterStructure->FilterMaskIdHigh));
		assert_param(IS_CAN_FILTER_ID_HALFWORD(initFilterStructure->FilterMaskIdLow));
		assert_param(IS_CAN_FILTER_FIFO(initFilterStructure->FilterFIFOAssignment));
		assert_param(IS_CAN_FILTER_BANK_DUAL(initFilterStructure->FilterBank));
		assert_param(IS_CAN_FILTER_BANK_DUAL(initFilterStructure->SlaveStartFilterBank));
		assert_param(IS_CAN_FILTER_MODE(initFilterStructure->FilterMode));
		assert_param(IS_CAN_FILTER_SCALE(initFilterStructure->FilterScale));
		assert_param(IS_CAN_FILTER_ACTIVATION(initFilterStructure->FilterActivation));

		// Enable initialization mode for the filter
		can_ip->FMR |= CAN_FMR_FINIT;

		// Select the start filter number of CAN2 slave instance
		can_ip->FMR &= ~CAN_FMR_CAN2SB;
		can_ip->FMR |= initFilterStructure->SlaveStartFilterBank << CAN_FMR_CAN2SB_Pos;

		// Convert filter number into bit position
		filterNumberBitPos = 1UL << (initFilterStructure->FilterBank & 0x1FU);

		// Filter deactivation
		can_ip->FA1R &= ~filterNumberBitPos;

		// Configuration filter scale
		if(initFilterStructure->FilterScale == CAN_FILTERSCALE_16BIT)
		{
			// Set 16-bit scale for the filter
			can_ip->FS1R &= ~filterNumberBitPos;

			// First 16-bit identifier and first 16-bit mask
			// or first 16-bit identifier and second 16-bit identifier
			can_ip->sFilterRegister[initFilterStructure->FilterBank].FR1 =
					((initFilterStructure->FilterMaskIdLow & 0x0000FFFFU) << 16U) | (initFilterStructure->FilterIdLow);

			// Second 16-bit identifier and second 16-bit mask
			// or third 16-bit identifier and fourth 16-bit identifier
			can_ip->sFilterRegister[initFilterStructure->FilterBank].FR2 =
					((initFilterStructure->FilterMaskIdHigh & 0x0000FFFFU) << 16U) | (initFilterStructure->FilterIdHigh);

		} else // for CAN_FILTERSCALE_32BIT
		{
			// Set 32-bit scale for the filter
			can_ip->FS1R |= filterNumberBitPos;

			// 32-bit identifier or First 32-bit identifier
			can_ip->sFilterRegister[initFilterStructure->FilterBank].FR1 =
					(initFilterStructure->FilterIdHigh << 16U) | initFilterStructure->FilterIdLow;

			// 32-bit mask or second 32-bit identifier
			can_ip->sFilterRegister[initFilterStructure->FilterBank].FR2 =
					(initFilterStructure->FilterMaskIdHigh << 16U) | initFilterStructure->FilterMaskIdLow;
		}

		// Set filter mode
		if(initFilterStructure->FilterMode == CAN_FILTER_MODE_IDMASK)
		{
			// Set identifier mask mode
			can_ip->FM1R &= ~filterNumberBitPos;

		} else // for CAN_FILTER_MODE_IDLIST
		{
			// Set identifier list mode
			can_ip->FM1R |= filterNumberBitPos;
		}

		// Set filter FIFO assignment
		if(initFilterStructure->FilterFIFOAssignment == CAN_FILTER_FIFO_0)
		{
			// Set filter assigned to FIFO 0
			can_ip->FFA1R &= ~filterNumberBitPos;

		} else // for CAN_FILTER_FIFO_1
		{
			// Set filter assigned to FIFO 1
			can_ip->FFA1R |= filterNumberBitPos;
		}

		// Set filter activation
		if(initFilterStructure->FilterActivation == CAN_FILTER_ENABLE)
		{
			can_ip->FA1R |= filterNumberBitPos;
		}

		// Leave the initialization mode for the filter
		can_ip->FMR &= ~CAN_FMR_FINIT;
	}

	return status;
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function is used to enable the specified CAN module.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @retval	The peripheral status.
 */
USH_peripheryStatus CAN_enable(CAN_TypeDef* can)
{
	USH_peripheryStatus status = STATUS_OK;
	uint32_t ticksStart = 0;

	// Check parameters
	assert_param(IS_CAN_ALL_INSTANCE(can));

	// Request leave initialization
	can->MCR &= ~CAN_MCR_INRQ;

	// Wait till the CAN initialization mode is enabled
	ticksStart = MISC_timeoutGetTick();
	while(((can->MSR) & CAN_MSR_INAK) != 0U)
	{
		if((MISC_timeoutGetTick() - ticksStart) > CAN_TIMEOUT_VALUE)
		{
			status = STATUS_TIMEOUT;
			break;
		}
	}

	return status;
}

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
USH_peripheryStatus CAN_addTxMessage(CAN_TypeDef* can, CAN_TxHeaderTypeDef* pHeader, uint8_t* pData)
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

/**
 * @brief 	This function is used to get an CAN frame from the Rx FIFO zone into the message RAM.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	rxFifo - A number of FIFO mailbox.
 * @param 	pHeader - A pointer to a CAN_RxHeaderTypeDef structure.
 * @param 	pData - A pointer to an array containing the payload of the Rx frame.
 * @retval	None.
 */
void CAN_getRxMessage(CAN_TypeDef* can, USH_CAN_filterFIFO rxFifo, CAN_RxHeaderTypeDef* pHeader, uint8_t* pData)
{
	// Check parameters
	assert_param(IS_CAN_FILTER_FIFO(rxFifo));

	// Get the frame information
	pHeader->IDE = can->sFIFOMailBox[rxFifo].RIR & CAN_RI0R_IDE;

	if(pHeader->IDE == CAN_ID_STD)
	{
		pHeader->StdId = (can->sFIFOMailBox[rxFifo].RIR & CAN_RI0R_STID) >> CAN_RI0R_STID_Pos;
	} else // for CAN_ID_EXT
	{
		pHeader->ExtId = (can->sFIFOMailBox[rxFifo].RIR & (CAN_RI0R_STID | CAN_RI0R_EXID)) >> CAN_RI0R_EXID_Pos;
	}

	pHeader->RTR = can->sFIFOMailBox[rxFifo].RIR & CAN_RI0R_RTR;
	pHeader->DLC = can->sFIFOMailBox[rxFifo].RDTR & CAN_RDT0R_DLC;
	pHeader->FilterMatchIndex = (can->sFIFOMailBox[rxFifo].RDTR & CAN_RDT0R_FMI) >> CAN_RDT0R_FMI_Pos;

	// Get the data
	pData[0] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDLR & CAN_RDL0R_DATA0) >> CAN_RDL0R_DATA0_Pos);
	pData[1] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDLR & CAN_RDL0R_DATA1) >> CAN_RDL0R_DATA1_Pos);
	pData[2] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDLR & CAN_RDL0R_DATA2) >> CAN_RDL0R_DATA2_Pos);
	pData[3] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDLR & CAN_RDL0R_DATA3) >> CAN_RDL0R_DATA3_Pos);
	pData[4] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDHR & CAN_RDH0R_DATA4) >> CAN_RDH0R_DATA4_Pos);
	pData[5] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDHR & CAN_RDH0R_DATA5) >> CAN_RDH0R_DATA5_Pos);
	pData[6] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDHR & CAN_RDH0R_DATA6) >> CAN_RDH0R_DATA6_Pos);
	pData[7] = (uint8_t)((can->sFIFOMailBox[rxFifo].RDHR & CAN_RDH0R_DATA7) >> CAN_RDH0R_DATA7_Pos);

	// Release FIFO mailbox
	if(rxFifo == CAN_FILTER_FIFO_0)
	{
		can->RF0R |= CAN_RF0R_RFOM0;
	} else
	{
		can->RF1R |= CAN_RF1R_RFOM1;
	}
}

/**
 * @brief	This function is used to enable or disable the specified interrupt.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	interrupt - Interrupts to be enabled.
 * @param 	state - The state of the selected stream. This parameter can be a value of @ref FunctionalState.
 * @retval	None.
 */
void CAN_interruptConfig(CAN_TypeDef* can, USH_CAN_interrupts interrupt, FunctionalState state)
{
	// Check parameters
	assert_param(IS_CAN_ALL_INSTANCE(can));
	assert_param(IS_CAN_INTERRUPT(interrupt));
	assert_param(IS_FUNCTIONAL_STATE(state));

	uint32_t ierReg = can->IER;

	if(state == ENABLE)
	{
		ierReg |= interrupt;
	} else
	{
		ierReg &= ~interrupt;
	}

	can->IER = ierReg;
}

/**
 * @brief	This function clears the specified CAN pending flag.
 * @param 	can - A pointer to CAN peripheral to be used where x is 1 or 2.
 * @param 	flag - A flag to clear.
 * @retval	None.
 */
void CAN_clearFlag(CAN_TypeDef* can, uint32_t flag)
{
	if(flag <= CAN_FLAG_TERR2)
	{
		can->TSR = (1 << flag);
	} else if((flag == CAN_FLAG_FF0) || (flag == CAN_FLAG_FOV0))
		   {
				can->RF0R = (1 << (flag & CAN_FLAG_MASK));
		   } else 					// for CAN_FLAG_FF1 and CAN_FLAG_FOV1
		   {
			   	can->RF1R = (1 << (flag & CAN_FLAG_MASK));
		   }
}

/**
 * @brief 	This function handles CAN interrupt request.
 * @param 	initStructure - A pointer to a USH_CAN_settingsTypeDef structure that contains the configuration information
 * 							for the specified CAN peripheral.
 * @retval	None.
 */
void CAN_IRQHandler(USH_CAN_settingsTypeDef *initStructure)
{
	uint32_t interrupts = initStructure->CANx->IER;
	uint32_t tsrReg		= initStructure->CANx->TSR;

/* -------------- Mailbox interrupt handling -------------- */

	// Сheck if interrupt on empty mailbox is enabled
	if((interrupts & CAN_IT_TX_MAILBOX_EMPTY) != 0U)
	{
		// Mailbox 0 is empty?
		if((tsrReg & CAN_TSR_RQCP0) != 0U)
		{
			CAN_clearFlag(initStructure->CANx, CAN_FLAG_RQCP0);

			// Transmission OK of mailbox 0
			if((tsrReg & CAN_TSR_TXOK0) != 0)
			{
				CAN_txMailbox0CompleteCallback(initStructure->CANx);
			}
		} else if((tsrReg & CAN_TSR_RQCP1) != 0U) // Mailbox 1 is empty?
			   {
					CAN_clearFlag(initStructure->CANx, CAN_FLAG_RQCP1);

					// Transmission OK of mailbox 1
					if((tsrReg & CAN_TSR_TXOK1) != 0U)
					{
						CAN_txMailbox1CompleteCallback(initStructure->CANx);
					}
			   } else if((tsrReg & CAN_TSR_RQCP2) != 0U) // Mailbox 2 is empty?
			   {
				   CAN_clearFlag(initStructure->CANx, CAN_FLAG_RQCP2);

				   // Transmission OK of mailbox 2
				   if((tsrReg & CAN_TSR_TXOK1) != 0U)
				   {
					   CAN_txMailbox2CompleteCallback(initStructure->CANx);
				   }
			   }
	}

/* -------------- FIFO 0 message pending interrupt handling -------------- */

	// Check if there is a message waiting interrupt
	if((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
	{
		// Additionally, check if there are any unaccepted messages
		if((initStructure->CANx->RF0R & CAN_RF0R_FMP0) != 0U)
		{
			CAN_rxFifo0MsgPendingCallback(initStructure->CANx);
		}
	}
}

//---------------------------------------------------------------------------
// Static Functions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// CAN interrupt user callbacks
//---------------------------------------------------------------------------

/**
  * @brief  TX mailbox 0 complete callback.
  * @note	This function should not be modified, when the callback is needed,
  * 		the CAN_txMailbox0CompleteCallback could be implemented in the user file.
  * @param  can - A pointer to CAN peripheral to be used where x is 1 or 2.
  * @retval None.
  */
__WEAK void CAN_txMailbox0CompleteCallback(CAN_TypeDef* can)
{
	(void)can;
}

/**
  * @brief  TX mailbox 1 complete callback.
  * @note	This function should not be modified, when the callback is needed,
  * 		the CAN_txMailbox1CompleteCallback could be implemented in the user file.
  * @param  can - A pointer to CAN peripheral to be used where x is 1 or 2.
  * @retval None.
  */
__WEAK void CAN_txMailbox1CompleteCallback(CAN_TypeDef* can)
{
	(void)can;
}

/**
  * @brief  TX mailbox 2 complete callback.
  * @note	This function should not be modified, when the callback is needed,
  * 		the CAN_txMailbox2CompleteCallback could be implemented in the user file.
  * @param  can - A pointer to CAN peripheral to be used where x is 1 or 2.
  * @retval None.
  */
__WEAK void CAN_txMailbox2CompleteCallback(CAN_TypeDef* can)
{
	(void)can;
}

/**
  * @brief  FIFO 0 message pending callback.
  * @note	This function should not be modified, when the callback is needed,
  * 		the CAN_rxFifo0MsgPendingCallback could be implemented in the user file.
  * @param  can - A pointer to CAN peripheral to be used where x is 1 or 2.
  * @retval None.
  */
__WEAK void CAN_rxFifo0MsgPendingCallback(CAN_TypeDef* can)
{
	(void)can;
}
