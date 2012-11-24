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
 * @file    LM4F/serial_lld.h
 * @brief   LM4F low level serial driver header.
 *
 * @addtogroup LM4F_SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/* UARTLCRH (UART Line Control Register) bit definitions */
#define SPS	(1 << 7)  /* stick parity select */
#define WLEN_8	(3 << 5)  /* word length 8 bits*/
#define WLEN_7	(2 << 5)  /* word length 7 bits*/
#define WLEN_6	(1 << 5)  /* word length 6 bits*/
#define WLEN_5	(0 << 5)  /* word length 5 bits*/
#define FEN	(1 << 4)  /* enable FIFO */
#define STP2	(1 << 3)  /* two stop bits select */
#define EPS	(1 << 2)  /* even parity select */
#define PEN	(1 << 1)  /* parity enable */
#define BRK	(1 << 0)  /* send break */

/* UARTCTL (UART Control Register) bit definitions */
#define RXE	(1 << 9)  /* receive enable */
#define TXE	(1 << 8)  /* transmit enable */
#define LBE	(1 << 7)  /* loop back enable */
#define UARTEN	(1 << 0)  /* UART enable */

/* UARTFR (UART Flag Register) bit definitions */
#define TXFE	(1 << 7)  /* transmit FIFO empty */
#define RXFF	(1 << 6)  /* receive FIFO full */
#define TXFF	(1 << 5)  /* transmit FIFO full */
#define RXFE	(1 << 4)  /* receive FIFO empty */
#define BUSY	(1 << 3)  /* UART busy */

/* UARTMIS (UART Masked Interrupt Status) bit definitions */
#define OEMIS   (1 << 10) /* overrun error */
#define PEMIS   (1 << 9)  /* parity error */
#define FEMIS   (1 << 8)  /* frame error */
#define BEMIS   (1 << 7)  /* break error */
#define RTMIS   (1 << 6)  /* receive time-out */
#define TXMIS   (1 << 5)  /* transmit */
#define RXMIS   (1 << 4)  /* receive */

/* UARTIM (UART Interrupt Mask) bit definitions */
#define OEMI    (1 << 10) /* overrun error */
#define PEMI    (1 << 9)  /* parity error */
#define FEMI    (1 << 8)  /* frame error */
#define BEMI    (1 << 7)  /* break error */
#define RTIM    (1 << 6)  /* receive time-out */
#define TXIM    (1 << 5)  /* transmit */
#define RXIM    (1 << 4)  /* receive */

#define RX_FIFO_SIZE  4
#define TX_FIFO_SIZE  0
/* UARTIFLS (UART Interrupt FIFO Level Select) bit definitions */
#define RXIFLSEL (RX_FIFO_SIZE << 3)
#define TXIFLSEL (TX_FIFO_SIZE << 0)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   UART0 driver enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p TRUE .
 */
#if !defined(USE_LM4F_UART0) || defined(__DOXYGEN__)
#define USE_LM4F_UART0           TRUE
#endif

/**
 * @brief   FIFO preload parameter.
 * @details Configuration parameter, this values defines how many bytes are
 *          preloaded in the HW transmit FIFO for each interrupt, the maximum
 *          value is 14 the minimum is 1.
 * @note    An high value reduces the number of interrupts generated but can
 *          also increase the worst case interrupt response time because the
 *          preload loops.
 */
#if !defined(LM4F_UART_FIFO_PRELOAD) || defined(__DOXYGEN__)
#define LM4F_UART_FIFO_PRELOAD   14
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(LM4F_UART0_PRIORITY) || defined(__DOXYGEN__)
#define LM4F_UART0_PRIORITY      5
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (LM4F_UART_FIFO_PRELOAD < 1) || (LM4F_UART_FIFO_PRELOAD > 16)
#error "invalid LM4F_UART_FIFO_PRELOAD setting"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   LM4F Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct {
  /**
   * @brief Bit rate.
   */
  uint32_t                  sc_speed;
  /**
   * @brief Initialization value for the LCRH (Line Control) register.
   */
  uint32_t                  sc_lcrh;
  /**
   * @brief Initialization value for the IFLS (Interrupt FIFO Level Select)
   * register.
   */
  uint32_t                  sc_ifls;
} SerialConfig;

/**
 * @brief @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  InputQueue                iqueue;                                         \
  /* Output queue.*/                                                        \
  OutputQueue               oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the USART registers block.*/                                \
  UART0_Type                *uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_LM4F_UART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
