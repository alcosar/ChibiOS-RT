/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"

#define uart0 ((BaseSequentialStream *)&SD1)

/*
 * SPI configuration (1MHz, CPH=0, CPO=0).
 */
static SPIConfig spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI0SEL,
  CR0_DSS8BIT | CR0_FSPIFF | CR0_CLOCKRATE(2),
  39
};

/*
 * LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 256);
static msg_t Thread1(void *arg) {

  (void)arg;
  while (TRUE) {
    palClearPad(GPIOF, GPIOF_LED_RED);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOF, GPIOF_LED_RED);

    palClearPad(GPIOF, GPIOF_LED_BLUE);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOF, GPIOF_LED_BLUE);

    palClearPad(GPIOF, GPIOF_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOF, GPIOF_LED_GREEN);
  }
  return 0;
}

/*
 * Entry point, note, the main() function is already a thread in the system
 * on entry.
 */
int main(int argc, char **argv) {

  (void)argc;
  (void)argv;

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
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);           /* Default: 38400,8,N,1. */
  spiStart(&SPID1, &spicfg);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (TRUE) {
    char rx = 0;
    char tx;

    chThdSleepMilliseconds(500);
    chprintf(uart0, "Hello World\r\n");
    if (!palReadPad(GPIOF, GPIOF_SW1)) {
      tx = sdGet(&SD1);
      spiSelect(&SPID1);
      spiExchange(&SPID1, 1, &tx, &rx);
      spiUnselect(&SPID1);
      sdPut(&SD1, rx);
    }
    if (!palReadPad(GPIOF, GPIOF_SW2))
      TestThread(&SD1);
  }
  return 0;
}
