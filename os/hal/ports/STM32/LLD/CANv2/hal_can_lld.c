/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_can_lld.c
 * @brief   STM32 CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"

#if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define READ_REG_MASK_VALUE(REG, MASK) ((READ_REG(REG) & (MASK ## _Msk)) >> (MASK ## _Pos))
#define WRITE_REG_MASK_VALUE(REG, MASK, VAL) MODIFY_REG(REG, (MASK ## _Msk), (VAL) << MASK ## _Pos)

#define SRAMCAN_FLS_NBR                  (28U)         /* Max. Filter List Standard Number      */
#define SRAMCAN_FLE_NBR                  ( 8U)         /* Max. Filter List Extended Number      */
#define SRAMCAN_RF0_NBR                  ( 3U)         /* RX FIFO 0 Elements Number             */
#define SRAMCAN_RF1_NBR                  ( 3U)         /* RX FIFO 1 Elements Number             */
#define SRAMCAN_TEF_NBR                  ( 3U)         /* TX Event FIFO Elements Number         */
#define SRAMCAN_TFQ_NBR                  ( 3U)         /* TX FIFO/Queue Elements Number         */

#define SRAMCAN_FLS_SIZE            ( 1U * 4U)         /* Filter Standard Element Size in bytes */
#define SRAMCAN_FLE_SIZE            ( 2U * 4U)         /* Filter Extended Element Size in bytes */
#define SRAMCAN_RF0_SIZE            (18U * 4U)         /* RX FIFO 0 Elements Size in bytes      */
#define SRAMCAN_RF1_SIZE            (18U * 4U)         /* RX FIFO 1 Elements Size in bytes      */
#define SRAMCAN_TEF_SIZE            ( 2U * 4U)         /* TX Event FIFO Elements Size in bytes  */
#define SRAMCAN_TFQ_SIZE            (18U * 4U)         /* TX FIFO/Queue Elements Size in bytes  */


#define SRAMCAN_FLSSA ((uint32_t)0)                                                      /* Filter List Standard Start Address */
#define SRAMCAN_FLESA ((uint32_t)(SRAMCAN_FLSSA + (SRAMCAN_FLS_NBR * SRAMCAN_FLS_SIZE))) /* Filter List Extended Start Address */
#define SRAMCAN_RF0SA ((uint32_t)(SRAMCAN_FLESA + (SRAMCAN_FLE_NBR * SRAMCAN_FLE_SIZE))) /* Rx FIFO 0 Start Address            */
#define SRAMCAN_RF1SA ((uint32_t)(SRAMCAN_RF0SA + (SRAMCAN_RF0_NBR * SRAMCAN_RF0_SIZE))) /* Rx FIFO 1 Start Address            */
#define SRAMCAN_TEFSA ((uint32_t)(SRAMCAN_RF1SA + (SRAMCAN_RF1_NBR * SRAMCAN_RF1_SIZE))) /* Tx Event FIFO Start Address        */
#define SRAMCAN_TFQSA ((uint32_t)(SRAMCAN_TEFSA + (SRAMCAN_TEF_NBR * SRAMCAN_TEF_SIZE))) /* Tx FIFO/Queue Start Address        */
#define SRAMCAN_SIZE  ((uint32_t)(SRAMCAN_TFQSA + (SRAMCAN_TFQ_NBR * SRAMCAN_TFQ_SIZE))) /* Message RAM size                   */


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CAN1 driver identifier.
 */
#if (STM32_CAN_USE_CAN1 == TRUE) || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/**
 * @brief   CAN2 driver identifier.
 */
#if (STM32_CAN_USE_CAN2 == TRUE) || defined(__DOXYGEN__)
CANDriver CAND2;
#endif

/**
 * @brief   CAN3 driver identifier.
 */
#if (STM32_CAN_USE_CAN3 == TRUE) || defined(__DOXYGEN__)
CANDriver CAND3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Configures and activates a standard CAN filter.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] filter    pointer to the @p CANRxStandardFilter object or NULL
 * @param[in] num       index of filter slot to use (0-28 inclusive)
 *
 * @notapi
 */
static void can_lld_set_filters(CANDriver* canp, CANRxStandardFilter *filter,
                                uint8_t num) {

  if (filter == NULL) {
    CANRxStandardFilter default_filter;
    default_filter.SFT = 2;       // Classic Filter
    default_filter.SFEC = 1;      // Store in FIFO 0
    default_filter.SFID1 = 0x000; // ID
    default_filter.SFID2 = 0x000; // Mask
    filter = &default_filter;
    num = 0;
  }
  //WRITE_REG((uint32_t *) *(SRAMCAN_BASE + SRAMCAN_FLSSA), default_filter.word);
  uint32_t *addr = canp->ram_base + SRAMCAN_FLSSA / sizeof(canp->ram_base);
  addr += num * (SRAMCAN_FLS_SIZE /  sizeof(canp->ram_base));
  WRITE_REG(*addr, filter->data32);

  /*
  // Set filter count to SRAMCAN_FLS_NBR
  MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_LSS,
          FDCAN_RXGFC_LSS & (SRAMCAN_FLS_NBR << FDCAN_RXGFC_LSS_Pos));
  */
  // Standard filter enable 1 filter
  //MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_LSS, (num + 1) << FDCAN_RXGFC_LSS_Pos);
  WRITE_REG_MASK_VALUE(canp->can->RXGFC, FDCAN_RXGFC_LSS, num + 1);

  WRITE_REG_MASK_VALUE(canp->can->RXGFC, FDCAN_RXGFC_ANFE, canp->config->anfe);
  WRITE_REG_MASK_VALUE(canp->can->RXGFC, FDCAN_RXGFC_ANFS, canp->config->anfs);
  // MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_ANFE_Msk,
  //     canp->config->anfe << FDCAN_RXGFC_ANFE_Pos);
  // MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_ANFS_Msk,
  //     canp->config->anfs << FDCAN_RXGFC_ANFS_Pos);
}


/**
 * @brief   Common RX0 ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_rx0_handler(CANDriver *canp) {
  //if (can_lld_is_rx_nonempty(canp, 1)) {
  if (READ_BIT(canp->can->IR, FDCAN_IR_RF0F | FDCAN_IR_RF0N)) {
    /* No more receive events until the queue 0 has been emptied.*/
    CLEAR_BIT(canp->can->IE, FDCAN_IE_RF0FE | FDCAN_IE_RF0NE);
    // Reset the IR bit for full and new frame interrupts
    SET_BIT(canp->can->IR, FDCAN_IR_RF0F | FDCAN_IR_RF0N);
    // "full_isr" is actually nonempty or full
    _can_rx_full_isr(canp, CAN_MAILBOX_TO_MASK(1U));
  }
  // Overflow events handling.
  if (READ_BIT(canp->can->IR, FDCAN_IR_RF0L)) {
    // Reset the IR bit for lost frame
    SET_BIT(canp->can->IR, FDCAN_IR_RF0L);
    _can_error_isr(canp, CAN_OVERFLOW_ERROR);
  }
}


/**
 * @brief   Common TX ISR handler.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
static void can_lld_tx_handler(CANDriver *canp) {
  /* Flags to be signaled through the TX event source.*/
  eventflags_t flags = 0U;

  if (READ_BIT(canp->can->IR, FDCAN_IR_TC)) {
    /* Clearing IRQ sources.*/
    SET_BIT(canp->can->IR, FDCAN_IR_TC);
    flags |= 1U;
  }

  if (READ_BIT(canp->can->IR, FDCAN_IR_TEFL)) {
    /* Clearing IRQ sources.*/
    SET_BIT(canp->can->IR, FDCAN_IR_TEFL);
    // Upper 16 indicate errors
    flags |= 1U << 16;
  }

  /* Checking TX buffer full status.*/
  if (READ_BIT(canp->can->TXFQS, FDCAN_TXFQS_TFQF) == 0) {
    /* Signaling flags and waking up threads waiting for a transmission slot.*/
    // It seams that error flags should always be broadcast regardless of TX
    // buffer emptiness. _can_tx_empty_isr always sends MSG_OK, only OK to send
    // that if there is a slot in TX buffer.
    _can_tx_empty_isr(canp, flags);
  }



}
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   CAN1 Unified interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_CAN1_IRQ_HANDLER_0) {

  OSAL_IRQ_PROLOGUE();

  can_lld_rx0_handler(&CAND1);
  can_lld_tx_handler(&CAND1);

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void canConfigObjectInit(CANConfig * config) {
  config->anfs = 0;
  config->anfe = 0;
  config->dar = 0;
  config->monitor = 0;
  config->loopback = 0;
}

void _can_lld_init(CANDriver *cand, FDCAN_GlobalTypeDef *fdcan, uint32_t *ram_base) {

  /* Driver initialization.*/
  canObjectInit(cand);
  cand->can = fdcan;
  cand->ram_base = ram_base;

  // Zero out the SRAM
  // TODO: Use memset?
  uint32_t * addr;
  for(addr=ram_base; addr<(ram_base + SRAMCAN_SIZE); addr+=1U)
  {
      *addr = (uint32_t) 0U;
  }

  SET_BIT(cand->can->CCCR, FDCAN_CCCR_INIT);

  // Wait for INIT to begin
  systime_t start = osalOsGetSystemTimeX();
  systime_t end = osalTimeAddX(start, TIME_MS2I(500));
  while(READ_BIT(cand->can->CCCR, FDCAN_CCCR_INIT) != 1
      && osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
    osalThreadSleepS(1);
  }
  osalDbgAssert(READ_BIT(cand->can->CCCR, FDCAN_CCCR_INIT) == 1,
      "CAN1 did not exit init, check clocks and pin config");

  SET_BIT(cand->can->CCCR, FDCAN_CCCR_CCE);
  can_lld_set_filters(cand, NULL, 0);
}

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

#if STM32_CAN_USE_CAN1 == TRUE
  rccResetFDCAN1();
  rccEnableFDCAN1(true);  // Stays on in sleep
  _can_lld_init(&CAND1, FDCAN1, (uint32_t *) (SRAMCAN_BASE + 0U * SRAMCAN_SIZE));
#endif
#if STM32_CAN_USE_CAN2 == TRUE
  rccResetFDCAN2();
  rccEnableFDCAN2(true);  // Stays on in sleep
  _can_lld_init(&CAND2, FDCAN2, (uint32_t *) (SRAMCAN_BASE + 1U * SRAMCAN_SIZE));
#endif
#if STM32_CAN_USE_CAN3 == TRUE
  rccResetFDCAN3();
  rccEnableFDCAN3(true);  // Stays on in sleep
  _can_lld_init(&CAND3, FDCAN3, (uint32_t *) (SRAMCAN_BASE + 2U * SRAMCAN_SIZE));
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_start(CANDriver *canp) {

  if (canp->state == CAN_STOP) {
    /* Enables the peripheral.*/
#if STM32_CAN_USE_CAN1 == TRUE
    if (&CAND1 == canp) {
      rccEnableFDCAN1(true);  // Stays on in sleep
    }
#endif
  }
  /* Configures the peripheral.*/
  CLEAR_BIT(canp->can->CCCR, FDCAN_CCCR_CSR);
  // Wait for clock stop ack to de-assert
  while(READ_BIT(canp->can->CCCR,FDCAN_CCCR_CSA)) {
    osalThreadSleepS(1);
  }
  SET_BIT(canp->can->CCCR, FDCAN_CCCR_INIT);
  // Wait for init
  while(READ_BIT(canp->can->CCCR,FDCAN_CCCR_INIT) != 1) {
    osalThreadSleepS(1);
  }
  SET_BIT(canp->can->CCCR, FDCAN_CCCR_CCE);

  // FDCAN_CONFIG->CKDIV = 8;
  if (canp->config->dar) {
    SET_BIT(canp->can->CCCR, FDCAN_CCCR_DAR);
  }

  if (canp->config->monitor) {
    SET_BIT(canp->can->CCCR, FDCAN_CCCR_MON);
  }
  if (canp->config->loopback) {
    SET_BIT(canp->can->CCCR, FDCAN_CCCR_TEST);
    SET_BIT(canp->can->TEST, FDCAN_TEST_LBCK);
  }

  nvicEnableVector(FDCAN1_IT0_IRQn, 28); // The default in the manual
  //Enable interrupts
  SET_BIT(canp->can->IE, FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF0LE | FDCAN_IE_TCE | FDCAN_IE_TEFLE);
  // Use FDCAN1 interrupt line 0
  SET_BIT(canp->can->ILE, FDCAN_ILE_EINT0);

  // Start it up
  //CLEAR_BIT(canp->can->CCCR, FDCAN_CCCR_CCE); Happens automatically with init
  //clear
  CLEAR_BIT(canp->can->CCCR, FDCAN_CCCR_INIT);

  // TODO: Timeout for exiting INIT
  while(READ_BIT(canp->can->CCCR, FDCAN_CCCR_INIT)) {
    osalThreadSleepS(1);
  }
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop(CANDriver *canp) {

  if (canp->state == CAN_READY) {
    CLEAR_REG(canp->can->IE);
    WRITE_REG(canp->can->IR, 0xFFF); // Reset all interrupt register
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if STM32_CAN_USE_CAN1 == TRUE
    if (&CAND1 == canp) {
      rccDisableFDCAN1();
      SET_BIT(canp->can->CCCR, FDCAN_CCCR_CSR);

    }
#endif
  }
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox) {

  switch (mailbox) {
  case CAN_ANY_MAILBOX:
  case 1:
  case 2:
  case 3:
  default:
    return (READ_BIT(canp->can->TXFQS, FDCAN_TXFQS_TFQF) == 0);
    // slots free:
    // return (READ_REG(canp->can->TXFQS) & FDCAN_TXFQS_TFFL_Msk) >> FDCAN_TXFQS_TFFL_Pos;
  }
}

uint8_t dlc_to_bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
  12, 16, 20, 24, 32, 48, 64};


/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] mailbox   mailbox number,  @p CAN_ANY_MAILBOX for any mailbox
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp,
                      canmbx_t mailbox,
                      const CANTxFrame *ctfp) {

  (void)mailbox;
  osalDbgAssert(dlc_to_bytes[ctfp->DLC] <= CAN_MAX_DLC_BYTES,
      "TX Packet is too large, increase CAN_MAX_DLC_BYTES");
  uint32_t *tx_address = canp->ram_base + SRAMCAN_TFQSA / sizeof(canp->ram_base);
  WRITE_REG(*tx_address++, ctfp->header32[0]);
  WRITE_REG(*tx_address++, ctfp->header32[1]);

  WRITE_REG(*tx_address++, ctfp->data32[0]);
  WRITE_REG(*tx_address++, ctfp->data32[1]);

  // Add TX request
  SET_BIT(canp->can->TXBAR, 1);


}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        RX FIFO\mailbox is empty.
 * @retval true         RX FIFO\mailbox is not empty.
 *
 * @notapi
 */
bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {
  switch (mailbox) {
    case CAN_ANY_MAILBOX:
      return can_lld_is_rx_nonempty(canp, 1) || can_lld_is_rx_nonempty(canp, 2);
    case 1:
      return (READ_REG_MASK_VALUE(canp->can->RXF0S, FDCAN_RXF0S_F0FL) != 0);
    case 2:
      return (READ_REG_MASK_VALUE(canp->can->RXF1S, FDCAN_RXF1S_F1FL) != 0);
  }
  return true;
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp,
                     canmbx_t mailbox,
                     CANRxFrame *crfp) {

  if (mailbox == CAN_ANY_MAILBOX) {
    if (can_lld_is_rx_nonempty(canp, 1)) {
      mailbox = 1;
    }
    else if (can_lld_is_rx_nonempty(canp, 2)) {
      mailbox = 2;
    }
    else {
      return;
    }
  }

  // GET index, add it and the length to the rx_address
  // uint32_t get_index = (READ_REG(canp->can->RXF0S) & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
  uint32_t get_index = READ_REG_MASK_VALUE(canp->can->RXF0S, FDCAN_RXF0S_F0GI);
  uint32_t *rx_address = canp->ram_base + (SRAMCAN_RF0SA + get_index * SRAMCAN_RF0_SIZE) / sizeof(canp->ram_base);
  crfp->header32[0] = READ_REG(*rx_address++); 
  crfp->header32[1] = READ_REG(*rx_address++); 
  crfp->data32[0] = READ_REG(*rx_address++); 
  crfp->data32[1] = READ_REG(*rx_address++); 

  // Acknowledge receipt using RXF0A
  WRITE_REG(canp->can->RXF0A, (get_index << FDCAN_RXF0A_F0AI_Pos) & FDCAN_RXF0A_F0AI_Msk);

  // Re-enable RX FIFO message arrived interrupt once the FIFO is emptied, see
  // can_lld_rx0_handler.
  if (!can_lld_is_rx_nonempty(canp, mailbox)) {
    SET_BIT(canp->can->IE, FDCAN_IE_RF0FE | FDCAN_IE_RF0NE);
  }

}

/**
 * @brief   Tries to abort an ongoing transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number
 *
 * @notapi
 */
void can_lld_abort(CANDriver *canp,
                   canmbx_t mailbox) {

  (void)canp;
  (void)mailbox;
}

#if (CAN_USE_SLEEP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {

  (void)canp;

}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {

  (void)canp;

}
#endif /* CAN_USE_SLEEP_MOD == TRUEE */

#endif /* HAL_USE_CAN == TRUE */

/** @} */
