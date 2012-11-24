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

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config = {
 {VAL_GPIOADATA, VAL_GPIOADEN, VAL_GPIOADIR, VAL_GPIOAPUR},
 {VAL_GPIOBDATA, VAL_GPIOBDEN, VAL_GPIOBDIR, VAL_GPIOBPUR},
 {VAL_GPIOCDATA, VAL_GPIOCDEN, VAL_GPIOCDIR, VAL_GPIOCPUR},
 {VAL_GPIODDATA, VAL_GPIODDEN, VAL_GPIODDIR, VAL_GPIODPUR},
 {VAL_GPIOEDATA, VAL_GPIOEDEN, VAL_GPIOEDIR, VAL_GPIOEPUR},
 {VAL_GPIOFDATA, VAL_GPIOFDEN, VAL_GPIOFDIR, VAL_GPIOFPUR},
};
#endif

/*
 * Early initialization code.
 * This initialization is performed just after reset before BSS and DATA
 * segments initialization.
 */
void __early_init(void) {

  LM4F_clock_init();
}

/*
 * Late initialization code.
 * This initialization is performed after BSS and DATA segments initialization
 * and before invoking the main() function.
 */
void boardInit(void) {
#if 0
  /*
   * HAL initialization.
   */
  halInit();

  /*
   * ChibiOS/RT initialization.
   */
  chSysInit();
#endif
}
