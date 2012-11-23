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

/**
 * @file    LM4F/hal_lld.c
 * @brief   LM4F HAL subsystem low level driver source.
 *
 * @addtogroup LM4F_HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

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
 * @brief   Low level HAL driver initialization.
 */
void hal_lld_init(void) {

  /* SysTick initialization using the system clock.*/
  SysTick->LOAD = LM4F_SYSCLK / CH_FREQUENCY - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;
}

/**
 * @brief   LM4F clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 */
void LM4F_clock_init(void) {
  uint32_t rcc = SYSCTL->RCC;
  uint32_t rcc2 = SYSCTL->RCC2;
  volatile uint32_t i;

  /* use RCC2, RCC2 register fealds override the RCC */
  rcc2 |= 1 << SYSCTL_RCC2_USERCC2;

  /* bypass the PLL */
  rcc |= 1 << SYSCTL_RCC_BYPASS;

  /* bypass the system clock divider */
  rcc &= ~(1 << SYSCTL_RCC_USESYSDIV);
  SYSCTL->RCC = rcc;

  rcc2 |= 1 << SYSCTL_RCC2_BYPASS2;
  SYSCTL->RCC2 = rcc2;

  /*
   * select the crystal value and oscillator source
   * enable power to PLL and enable its output
   */
  rcc &= ~(SYSCTL_RCC_XTAL_MASK | SYSCTL_RCC_OSCSRC_MASK |
           (1 << SYSCTL_RCC_PWRDN));
  rcc |= XTAL << SYSCTL_RCC_XTAL |
         (OSCSRC << SYSCTL_RCC_OSCSRC & SYSCTL_RCC_OSCSRC_MASK);
  SYSCTL->RCC = rcc;

  rcc2 &= ~(SYSCTL_RCC2_OSCSRC2_MASK | (1 << SYSCTL_RCC2_PWRDN2));
  rcc2 |= OSCSRC << SYSCTL_RCC2_OSCSRC2 | DIV400 << SYSCTL_RCC2_DIV400;
  SYSCTL->RCC2 = rcc2;
  for(i = 100000; i; i--)
	  ;

  /* clear SYSDIV bits */
  rcc &= ~SYSCTL_RCC_SYSDIV_MASK;

  /* select system divider and enable to use it */
  rcc |= (SYSDIV << SYSCTL_RCC_SYSDIV &  SYSCTL_RCC_SYSDIV_MASK) |
          SYSCTL_RCC_USESYSDIV;

  SYSCTL->RCC = rcc;

  rcc2 &= ~SYSCTL_RCC2_SYSDIV2_MASK;
  rcc2 |= SYSDIV << SYSCTL_RCC2_SYSDIV2;
  SYSCTL->RCC2 = rcc2;
//  for(i = 100000; i; i--)
	  ;

  /* wait for the PLL to lock */
  while ((SYSCTL->RIS & (1 << SYSCTL_RIS_PLLLRIS)) == 0)
    ;

  /* enable use of the PLL */
  SYSCTL->RCC &= ~(1 << SYSCTL_RCC_BYPASS);
  SYSCTL->RCC2 &= ~(1 << SYSCTL_RCC2_BYPASS2);
}

/** @} */
