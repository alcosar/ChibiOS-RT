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
 * @file    LM4F/serial_lld.c
 * @brief   LM4F low level serial driver code.
 *
 * @addtogroup LM4F_SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_LM4F_UART0 || defined(__DOXYGEN__)
/** @brief UART0 serial driver identifier.*/
SerialDriver SD1;
#endif

#if USE_LM4F_UART1 || defined(__DOXYGEN__)
/** @brief UART1 serial driver identifier.*/
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  WLEN_8 | FEN,
  RXIFLSEL | TXIFLSEL
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  UART0_Type *u = sdp->uart;
  uint32_t div;  /* baud rate divisor */

  /* disable the UART before any ot the control registers are reprogrammed */
  u->CTL &= ~UARTEN;
  div = (((LM4F_SYSCLK * 8) / config->sc_speed) + 1) / 2;
  u->IBRD = div / 64;   /* integer portion of the baud rate divisor */
  u->FBRD = div % 64;   /* fractional portion of the baud rate divisor */
  u->LCRH = config->sc_lcrh;   /* set data format */
  u->IFLS = config->sc_ifls;
  u->CTL |= RXE | TXE | UARTEN;
  u->IM |= TXIM | RXIM | RTIM;   /* interrupts enable */
}

/**
 * @brief   UART de-initialization.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(UART0_Type *u) {

  u->CTL &= ~UARTEN;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] err       UART LSR register value
 */
static void set_error(SerialDriver *sdp, IOREG32 err) {
  flagsmask_t sts = 0;

  if (err & OEMIS)
    sts |= SD_OVERRUN_ERROR;
  if (err & PEMIS)
    sts |= SD_PARITY_ERROR;
  if (err & FEMIS)
    sts |= SD_FRAMING_ERROR;
  if (err & BEMIS)
    sts |= SD_BREAK_DETECTED;
  chSysLockFromIsr();
  chnAddFlagsI(sdp, sts);
  chSysUnlockFromIsr();
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] u         pointer to an UART I/O block
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  UART0_Type *u = sdp->uart;
  uint16_t mis = u->MIS;

  u->ICR = mis;		/* clear interrupts */
  if (mis & (OEMIS | BEMIS | PEMIS | FEMIS))
    set_error(sdp, mis);
  if ((mis & RXMIS) || (mis &  RTMIS)) {
    chSysLockFromIsr();
    if (chIQIsEmptyI(&sdp->iqueue))
      chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
    chSysUnlockFromIsr();
    while ((u->FR & RXFE) == 0) {
      chSysLockFromIsr();
      if (chIQPutI(&sdp->iqueue, u->DR) < Q_OK)
        chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
      chSysUnlockFromIsr();
    }
  }
  if (mis & TXMIS) {
    while ((u->FR & TXFF) == 0) {
      msg_t b;
      chSysLockFromIsr();
      b = chOQGetI(&sdp->oqueue);
      chSysUnlockFromIsr();
      if (b < Q_OK) {
        u->IM &= ~TXIM;
        chSysLockFromIsr();
        chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
        chSysUnlockFromIsr();
        break;
      }
      u->DR = b;
    }
  }
}

/**
 * @brief   Attempts a TX FIFO preload.
 */
static void preload(SerialDriver *sdp) {
  UART0_Type *u = sdp->uart;

  while ((u->FR & TXFF) == 0) {
    msg_t b = chOQGetI(&sdp->oqueue);
    if (b < Q_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      return;
    }
    u->DR = b;
  }
  u->IM |= TXIM;   /* transmit interrupt enable */
}

/**
 * @brief   Driver SD1 output notification.
 */
#if USE_LM4F_UART0 || defined(__DOXYGEN__)
static void notify1(GenericQueue *qp) {

  (void)qp;
  preload(&SD1);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 IRQ handler.
 */
#if USE_LM4F_UART0 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(Vector54) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 */
void sd_lld_init(void) {

#if USE_LM4F_UART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = UART0;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if USE_LM4F_UART0
    if (&SD1 == sdp) {
      SYSCTL->RCGCUART |= (1 << 0);  /* enable UART0 module */
      SYSCTL->RCGCGPIO |= (1 << 0);  /* enable clock to GPIOA */
      __NOP();
      __NOP();
      __NOP();
      GPIOA->AFSEL |= (1 << 0) | (1 << 1);
      GPIOA->DEN |= (1 << 0) | (1 << 1);
      GPIOA->PCTL |= (1 << 0) | (1 << 4);
      nvicEnableVector(UART0_IRQn, CORTEX_PRIORITY_MASK(LM4F_UART0_PRIORITY));
    }
#endif
  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if USE_LM4F_UART0
    if (&SD1 == sdp) {
      SYSCTL->RCGCUART &= ~(1 << 0);  /* disable UART0 module */
      nvicDisableVector(UART0_IRQn);
      return;
    }
#endif
  }
}

#endif /* CH_HAL_USE_SERIAL */

/** @} */
