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
 * @file    LM4F/hal_lld.h
 * @brief   HAL subsystem low level driver header template.
 *
 * @addtogroup LM4F_HAL
 * @{
 */

#ifndef _HAL_LLD_H_
#define _HAL_LLD_H_

#include "LM4F120H5QR.h"
#include "nvic.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Platform name.
 */
#define PLATFORM_NAME           "LM4F"

#define IRCOSCCLK               16000000    /**< High speed internal clock. */

#define SYSPLLCLKSEL_IRCOSC     0           /**< Internal RC oscillator
                                                 clock source.              */
#define SYSPLLCLKSEL_SYSOSC     1           /**< System oscillator clock
                                                 source.                    */
/* Clock sources */
#define OSCSRC_MOSC		0   /* main oscillator */
#define OSCSRC_PIOSC		1   /* precision internal oscillator */
#define OSCSRC_PIOSC_BY_4	2   /* precision internal oscillator / 4 */
#define OSCSRC_30		3   /* 30 kHz internal oscillator */
#define OSCSRC_32		7   /* 32.768 kHz internal oscillator */

#define OSCSRC                  OSCSRC_MOSC  /* use main oscillator */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   System PLL clock source.
 */
#if !defined(LM4F_PLLCLK_SOURCE) || defined(__DOXYGEN__)
#define LM4F_PLLCLK_SOURCE   SYSPLLCLKSEL_SYSOSC
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* XXX
 * System clock divisor may be evaluated by the folowing expression
 * but it dosen't check for valid system clocks
 */
#if LM4F_SYSCLK <= 66666666
#define SYSDIV	(200000000 / LM4F_SYSCLK - 1)
#define DIV400	0
#else
#define SYSDIV	(400000000 / LM4F_SYSCLK - 1)
#define DIV400	1
#endif

/*
 * valid crystal values when using PLL, 0-5 reserved
 */
#if SYSOSCCLK == 4000000
#define XTAL                    0x06  /* 4.0 MHz */
#elif SYSOSCCLK == 4096000
#define XTAL                    0x07  /* 4.096 MHz */
#elif SYSOSCCLK == 4915200
#define XTAL                    0x08  /* 4.9152 MHz */
#elif SYSOSCCLK == 5000000
#define XTAL                    0x09  /* 5.0 MHz */
#elif SYSOSCCLK == 5120000
#define XTAL                    0x0a  /* 5.12 MHz */
#elif SYSOSCCLK == 6000000
#define XTAL                    0x0b  /* 6.0 MHz */
#elif SYSOSCCLK == 6144000
#define XTAL                    0x0c  /* 6.144 MHz */
#elif SYSOSCCLK == 7372800
#define XTAL                    0x0d  /* 7.3728 MHz */
#elif SYSOSCCLK == 8000000
#define XTAL                    0x0e  /* 8.0 MHz */
#elif SYSOSCCLK == 8192000
#define XTAL                    0x0f  /* 8.192 MHz */
#elif SYSOSCCLK == 10000000
#define XTAL                    0x10  /* 10.0 MHz */
#elif SYSOSCCLK == 12000000
#define XTAL                    0x11  /* 12.0 MHz */
#elif SYSOSCCLK == 12288000
#define XTAL                    0x12  /* 12.288 MHz */
#elif SYSOSCCLK == 13560000
#define XTAL                    0x13  /* 13.56 MHz */
#elif SYSOSCCLK == 14318180
#define XTAL                    0x14  /* 14.31818 MHz */
#elif SYSOSCCLK == 16000000
#define XTAL                    0x15  /* 16.0 MHz */
#elif SYSOSCCLK == 16384000
#define XTAL                    0x16  /* 16.384 MHz */
#elif SYSOSCCLK == 18000000
#define XTAL                    0x17  /* 18.0 MHz */
#elif SYSOSCCLK == 20000000
#define XTAL                    0x18  /* 20.0 MHz */
#elif SYSOSCCLK == 24000000
#define XTAL                    0x19  /* 24.0 MHz */
#elif SYSOSCCLK == 25000000
#define XTAL                    0x1a  /* 25.0 MHz */
#else
#error "invalid SYSOSCCLK specified"
#endif

/*
 * Definitions of the bit fields in RCC (Run-Mode Clock Configuration
 * register
 */
#define SYSCTL_RCC_MOSCDIS	0
#define SYSCTL_RCC_IOSCDIS	1
#define SYSCTL_RCC_OSCSRC	4
#define SYSCTL_RCC_XTAL		6
#define SYSCTL_RCC_BYPASS	11
#define SYSCTL_RCC_OEN		12
#define SYSCTL_RCC_PWRDN	13
#define SYSCTL_RCC_USESYSDIV	22
#define SYSCTL_RCC_SYSDIV	23
#define SYSCTL_RCC_ACG		27

#define SYSCTL_RCC_OSCSRC_MASK	(0x03 << 4)
#define SYSCTL_RCC_XTAL_MASK	(0x1f << 6)
#define SYSCTL_RCC_SYSDIV_MASK	(0x0f << 23)

/*
 * Definitions of the bit fields in RCC2 (Run-Mode Clock Configuration 2
 * register
 */
#define SYSCTL_RCC2_OSCSRC2	4
#define SYSCTL_RCC2_BYPASS2	11
#define SYSCTL_RCC2_PWRDN2	13
#define SYSCTL_RCC2_USBPWRDN	14
#define SYSCTL_RCC2_SYSDIV2LSB	22
#define SYSCTL_RCC2_DIV400	30
#define SYSCTL_RCC2_USERCC2	31

#define SYSCTL_RCC2_OSCSRC2_MASK (0x07 << 4)
#if DIV400 == 0
#define SYSCTL_RCC2_SYSDIV2_MASK (0x3f << 23)
#define SYSCTL_RCC2_SYSDIV2	23
#else
#define SYSCTL_RCC2_SYSDIV2_MASK (0x7f << 22)
#define SYSCTL_RCC2_SYSDIV2	22
#endif

#define SYSCTL_RIS_PLLLRIS	6  /* PLL lock raw interrupt status */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void LM4F_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* _HAL_LLD_H_ */

/** @} */
