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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for Texas Instruments STELLARIS_LAUNCHPAD Board.
 */

/*
 * Board identifiers.
 */
#define STELLARIS_LAUNCHPAD
#define BOARD_NAME "STELLARIS_LAUNCHPAD"

/*
 * Board frequencies.
 */
#define SYSOSCCLK               16000000
#define LM4F_SYSCLK             80000000

/*
 * User Push Buttons.
 */

#define GPIOF_SW1               4
#define GPIOF_SW2               0

/*
 * User LEDs
 */

#define GPIOF_LED_RED           1
#define GPIOF_LED_BLUE          2
#define GPIOF_LED_GREEN         3

#define GPIOA_SPI0SEL           3

#define VAL_GPIOADIR            (1 << GPIOA_SPI0SEL)
#define VAL_GPIOADATA           (1 << GPIOA_SPI0SEL)
#define VAL_GPIOADEN            0xff
#define VAL_GPIOAPUR            (1 << GPIOA_SPI0SEL)

#define VAL_GPIOBDIR            0x00000000
#define VAL_GPIOBDATA           0x00000000
#define VAL_GPIOBDEN            0xff
#define VAL_GPIOBPUR            0x00

#define VAL_GPIOCDIR            0x00000000
#define VAL_GPIOCDATA           0x00000000
#define VAL_GPIOCDEN            0xff
#define VAL_GPIOCPUR            0x00

#define VAL_GPIODDIR            0x00000000
#define VAL_GPIODDATA           0x00000000
#define VAL_GPIODDEN            0xff
#define VAL_GPIODPUR            0x00

#define VAL_GPIOEDIR            0x00000000
#define VAL_GPIOEDATA           0x00000000
#define VAL_GPIOEDEN            0xff
#define VAL_GPIOEPUR            0x00

#define VAL_GPIOFDATA           0x00000000
#define VAL_GPIOFDEN            0xff
#define VAL_GPIOFDIR            (PAL_PORT_BIT(GPIOF_LED_RED) | \
				PAL_PORT_BIT(GPIOF_LED_BLUE) | \
				PAL_PORT_BIT(GPIOF_LED_GREEN))
#define VAL_GPIOFPUR            (PAL_PORT_BIT(GPIOF_SW1) | \
                                PAL_PORT_BIT(GPIOF_SW2))

/*
 * Pin definitions.
 */

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
