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

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

/* The looback time should be calculated based on bitrates and frame sizes */
#define CAN_LPBK_ROUNDTRIP_MS 10

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static int BLINK_SLEEP = 500;
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearLine(LINE_LED_LD2);
    chThdSleepMilliseconds(BLINK_SLEEP);
    palSetLine(LINE_LED_LD2);
    chThdSleepMilliseconds(BLINK_SLEEP);
  }
}


void test_poll_interface_and_events(CANRxFrame *crfp, CANTxFrame *ctfp) {
  event_listener_t rx_nonempty;
  chEvtRegister(&CAND1.rxfull_event, &rx_nonempty, EVENT_MASK(0));

  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) == 0);

  /* No packet should be waiting, true indicates empty RX FIFO */
  osalDbgCheck(canTryReceiveI(&CAND1, 1, crfp) == true);

  /* TX buffer should  be empty, false indicates empty TX buffer */
  osalDbgCheck(canTryTransmitI(&CAND1, CAN_ANY_MAILBOX, ctfp) == false);
  /* Wait for loopback packet to show up in RX buffer */
  osalThreadSleepS(TIME_MS2I(CAN_LPBK_ROUNDTRIP_MS));
  osalDbgCheck(canTryReceiveI(&CAND1, 1, crfp) == false);
  osalDbgCheck(crfp->SID == ctfp->SID);
  osalDbgCheck(crfp->data32[0] == ctfp->data32[0]);
  osalDbgCheck(crfp->data32[1] == ctfp->data32[1]);
  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) != 0);

  /* Buffer should now be empty again */
  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) == 0);

  osalDbgCheck(canTryTransmitI(&CAND1, CAN_ANY_MAILBOX, ctfp) == false);

  /* Message transmit, loopback, reception is not instantaneous! */
  osalThreadSleepS(TIME_MS2I(CAN_LPBK_ROUNDTRIP_MS));
  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) != 0);

  /* Event does not get set again until after re-enabling the interrupt (by
   * receiving all frames in the RX FIFO) */
  osalDbgCheck(canTryTransmitI(&CAND1, CAN_ANY_MAILBOX, ctfp) == false);
  osalThreadSleepS(TIME_MS2I(CAN_LPBK_ROUNDTRIP_MS));
  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) == 0);

  osalDbgCheck(canTryTransmitI(&CAND1, CAN_ANY_MAILBOX, ctfp) == false);
  osalThreadSleepS(TIME_MS2I(CAN_LPBK_ROUNDTRIP_MS));
  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) == 0);

  /* Buffer should now be full */

  osalDbgCheck(canTryReceiveI(&CAND1, 1, crfp) == false);
  osalDbgCheck(canTryReceiveI(&CAND1, 1, crfp) == false);
  osalDbgCheck(canTryReceiveI(&CAND1, 1, crfp) == false);
  osalDbgCheck(canTryReceiveI(&CAND1, 1, crfp) == true);
  osalDbgCheck(chEvtGetAndClearFlags(&rx_nonempty) == 1);

  chEvtUnregister(&CAND1.rxfull_event, &rx_nonempty);
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity.
   */

  /* Config for FDCAN peripheral */
  CANConfig test = {
    .anfs = 1,  // Accept unmatched standard packets in FIFO0
    .anfe = 1,  // Accept unmatched extended packets in FIFO0
    .dar = 0, // Disable automatic reply
    .monitor = 1, // Disable TX pin
    .loopback = 1, // Enable loopack
    .brs = 1, // Enable bit-rate switching
    .fd = 1, // Enable CANFD
  };

  /* Start CAN and apply the configuration in test */
  canStart(&CAND1, &test);

  /* Create a receiving CAN frame object, with zeroes to simplify debugging. */
  CANRxFrame crf = {0};
  /* Create a transmission CAN frame object, with zeroes to simplify 
   * debugging. */
  CANTxFrame ctf = {0};

  /* Fill out the transmission object. */
  ctf.SID = 1;
  ctf.XTD = 0;
  ctf.DLC = 8;
  ctf.data32[0] = 0xdeadbeef;
  ctf.data32[1] = 0xdeadbeef;

  test_poll_interface_and_events(&crf, &ctf);

  /* Reset RX message */
  crf.SID = 0;
  crf.data32[0] = 0;
  crf.data32[0] = 1;

  /* Test Interrupt-based interface */
  sysinterval_t wait = TIME_MS2I(CAN_LPBK_ROUNDTRIP_MS);
  osalDbgCheck(canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &ctf, wait) == MSG_OK);
  osalDbgCheck(canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &crf, wait) == MSG_OK);

  osalDbgCheck(crf.SID == ctf.SID);
  osalDbgCheck(crf.data32[0] == ctf.data32[0]);
  osalDbgCheck(crf.data32[1] == ctf.data32[1]);

  while (true) {
   if (palReadLine(LINE_INPUT_A12)) {
      BLINK_SLEEP = 2000;
   }
   else {
      BLINK_SLEEP = 125;
   }
   //chThdSleepMilliseconds(250);
 }
}