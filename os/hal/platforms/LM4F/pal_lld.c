/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.
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

/**
 * @file    LM4F/pal_lld.c
 * @brief   LM4F GPIO low level driver code.
 *
 * @addtogroup LM4F_PAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/
/**
 * @brief LM4F I/O ports configuration.
 * @details GPIO unit registers initialization.
 *
 * @param[in] config the LM4F ports configuration
 */
void _pal_lld_init(const PALConfig *config) {
  /* enable clock gating control for PORTF */
  SYSCTL->RCGCGPIO |= (1 << 5);
  /* XXX wait after enabling clock - otherwise BusFault exeption occured */
  __NOP();
  __NOP();
  __NOP();
  /*
   * PF0 can be used as external NMI, and is under commit protection
   * (as well as PD7). A special sequence is required to change AFSEL,
   * PUR, PDR, DEN registers.
   */
  GPIOF->LOCK = 0x4C4F434B;     /* unlock commit register */
  GPIOF->CR = 1;                /* enable to change settings */
  GPIOF->LOCK = 0;              /* any value locks access again */

  GPIOF->DIR |= config->PF.dir;
  GPIOF->DEN |= config->PF.den;
  GPIOF->PUR |= config->PF.pur;
}

/**
 * @brief Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port the port identifier
 * @param[in] mask the group mask
 * @param[in] mode the mode
 *
 * @note This function is not meant to be invoked directly by the application
 *       code.
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {

  switch (mode) {
  case PAL_MODE_RESET:
  case PAL_MODE_INPUT:
    port->DIR &= ~mask;
    break;
  case PAL_MODE_UNCONNECTED:
    palSetPort(port, PAL_WHOLE_PORT);
  case PAL_MODE_OUTPUT_PUSHPULL:
    port->DIR |=  mask;
    break;
  }
}

#endif /* CH_HAL_USE_PAL */

/** @} */
