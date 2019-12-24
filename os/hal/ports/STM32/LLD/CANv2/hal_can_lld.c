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
  uint32_t *addr = (uint32_t *) SRAMCAN_BASE + SRAMCAN_FLSSA + num * 1U;
  WRITE_REG(*addr, filter->data32);

  /*
  // Set filter count to SRAMCAN_FLS_NBR
  MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_LSS,
          FDCAN_RXGFC_LSS & (SRAMCAN_FLS_NBR << FDCAN_RXGFC_LSS_Pos));
  */
  // Standard filter enable 1 filter
  MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_LSS, (num + 1) << FDCAN_RXGFC_LSS_Pos);

  MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_ANFE_Msk,
      canp->config->anfe << FDCAN_RXGFC_ANFE_Pos);
  MODIFY_REG(canp->can->RXGFC, FDCAN_RXGFC_ANFS_Msk,
      canp->config->anfs << FDCAN_RXGFC_ANFS_Pos);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

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

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

#if STM32_CAN_USE_CAN1 == TRUE
  rccResetFDCAN1();

  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.can = FDCAN1;
  rccEnableFDCAN1(true);  // Stays on in sleep

  // Zero out the SRAM
  uint32_t * addr;
  for(addr=(uint32_t *)SRAMCAN_BASE;
          addr<(uint32_t *)(SRAMCAN_BASE + SRAMCAN_SIZE); addr+=1U)
  {
      *addr = (uint32_t) 0U;
  }
  //
  SET_BIT(CAND1.can->CCCR, FDCAN_CCCR_INIT);
  while(READ_BIT(CAND1.can->CCCR,FDCAN_CCCR_INIT) != 1) {
    osalThreadSleepS(1);
  }
  SET_BIT(CAND1.can->CCCR, FDCAN_CCCR_CCE);
  can_lld_set_filters(&CAND1, NULL, 0);


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

  // Start it up
  //CLEAR_BIT(canp->can->CCCR, FDCAN_CCCR_CCE); Happens automatically with init
  //clear
  CLEAR_BIT(canp->can->CCCR, FDCAN_CCCR_INIT);
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

  (void)canp;
  (void)mailbox;
  (void)ctfp;
  uint32_t *tx_address = (uint32_t *) (SRAMCAN_BASE + SRAMCAN_TFQSA);
  //WRITE_REG(SRAMCAN_BASE, 0);
  WRITE_REG(*tx_address, ctfp->header32[0]);
  tx_address += 1U;
  WRITE_REG(*tx_address, ctfp->header32[1]);
  tx_address += 1U;

  WRITE_REG(*tx_address, ctfp->data32[0]);
  tx_address += 1U;
  WRITE_REG(*tx_address, ctfp->data32[1]);
  tx_address += 1U;

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
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {

  (void)canp;
  (void)mailbox;

  switch (mailbox) {
  case CAN_ANY_MAILBOX:
  case 1:
  case 2:
  default:
    return (!READ_BIT(canp->can->RXF0S, FDCAN_RXF0S_F0F));
  }
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

  (void)canp;
  (void)mailbox;
  (void)crfp;
  // GET index, add it and the length to the rx_address
  uint32_t get_index = (READ_REG(canp->can->RXF0S) & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
  uint32_t *rx_address = (uint32_t *) (SRAMCAN_BASE + SRAMCAN_RF0SA + get_index * 18U);
  crfp->header32[0] = READ_REG(*rx_address); 
  rx_address += 1U;
  crfp->header32[1] = READ_REG(*rx_address); 
  rx_address += 1U;
  crfp->data32[0] = READ_REG(*rx_address); 
  rx_address += 1U;
  crfp->data32[1] = READ_REG(*rx_address); 

  // Acknowledge receipt using RXF0A
  WRITE_REG(canp->can->RXF0A, (get_index << FDCAN_RXF0A_F0AI_Pos) & FDCAN_RXF0A_F0AI_Msk);

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
