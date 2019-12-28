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
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  CANConfig test = {
    .anfs = 1,  // Accept unmatched standard packets in FIFO0
    .anfe = 1,  // Accept unmatched extended packets in FIFO0
    .dar = 0, // Disable automatic reply
    .monitor = 1, // Disable TX pin
    .loopback = 1, // Enable loopack
  };

  canStart(&CAND1, &test);

  CANRxFrame crfp = {0};

  bool my_var;

  //my_var = canTryReceiveI(&CAND1, CAN_ANY_MAILBOX, &crfp);

  CANTxFrame ctfp = {0};
  ctfp.SID = 1;
  ctfp.XTD = 0;
  ctfp.DLC = 8;
  ctfp.data32[0] = 0xdeadbeef;
  ctfp.data32[1] = 0xdeadbeef;

  // my_var = canTryTransmitI(&CAND1, CAN_ANY_MAILBOX, &ctfp);
  msg_t x1;
  sysinterval_t wait = TIME_MS2I(250);
  x1 = canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &ctfp, wait);
  // my_var = canTryReceiveI(&CAND1, 1, &crfp);
  x1 = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &crfp, wait);

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
