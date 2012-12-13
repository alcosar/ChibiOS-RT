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
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    LM4F120/spi_lld.c
 * @brief   LM4F120 low level SPI driver code.
 *
 * @addtogroup SPI
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if LM4F120_SPI_USE_SSI0 || defined(__DOXYGEN__)
/** @brief SPI1 driver identifier.*/
SPIDriver SPID1;
#endif

#if LM4F120_SPI_USE_SSI1 || defined(__DOXYGEN__)
/** @brief SPI2 driver identifier.*/
SPIDriver SPID2;
#endif

#if LM4F120_SPI_USE_SSI2 || defined(__DOXYGEN__)
/** @brief SPI3 driver identifier.*/
SPIDriver SPID3;
#endif

#if LM4F120_SPI_USE_SSI3 || defined(__DOXYGEN__)
/** @brief SPI4 driver identifier.*/
SPIDriver SPID4;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Preloads the transmit FIFO.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void ssi_fifo_preload(SPIDriver *spip) {
  SSI0_Type *ssi = spip->ssi;
  uint32_t n = spip->txcnt > LM4F120_SSI_FIFO_DEPTH ?
               LM4F120_SSI_FIFO_DEPTH : spip->txcnt;

  while(((ssi->SR & SSISR_TNF) != 0) && (n > 0)) {
    if (spip->txptr != NULL) {
      if ((ssi->CR0 & CR0_DSSMASK) > CR0_DSS8BIT) {
        const uint16_t *p = spip->txptr;
        ssi->DR = *p++;
        spip->txptr = p;
      }
      else {
        const uint8_t *p = spip->txptr;
        ssi->DR = *p++;
        spip->txptr = p;
      }
    }
    else
      ssi->DR = 0xffffffff;
    n--;
    spip->txcnt--;
  }
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 */
static void spi_serve_interrupt(SPIDriver *spip) {
  SSI0_Type *ssi = spip->ssi;

  if ((ssi->MIS & SSIMIS_ROR) != 0) {
    /* The overflow condition should never happen because priority is given
       to receive but a hook macro is provided anyway...*/
    LM4F120_SPI_SSI_ERROR_HOOK(spip);
  }
  ssi->ICR = SSIICR_RTIC | SSIICR_RORIC;
  while ((ssi->SR & SSISR_RNE) != 0) {
    if (spip->rxptr != NULL) {
      if ((ssi->CR0 & CR0_DSSMASK) > CR0_DSS8BIT) {
        uint16_t *p = spip->rxptr;
        *p++ = ssi->DR;
        spip->rxptr = p;
      }
      else {
        uint8_t *p = spip->rxptr;
        *p++ = ssi->DR;
        spip->rxptr = p;
      }
    }
    else
      (void)ssi->DR;
    if (--spip->rxcnt == 0) {
      chDbgAssert(spip->txcnt == 0,
                  "spi_serve_interrupt(), #1", "counter out of synch");
      /* Stops the IRQ sources.*/
      ssi->IM = 0;
      /* Portable SPI ISR code defined in the high level driver, note, it is
         a macro.*/
      _spi_isr_code(spip);
      return;
    }
  }
  ssi_fifo_preload(spip);
  if (spip->txcnt == 0)
    ssi->IM = SSIIM_ROR | SSIIM_RT | SSIIM_RX;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if LM4F120_SPI_USE_SSI0 || defined(__DOXYGEN__)
/**
 * @brief   SSI0 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector5C) {

  CH_IRQ_PROLOGUE();

  spi_serve_interrupt(&SPID1);

  CH_IRQ_EPILOGUE();
}
#endif

#if LM4F120_SPI_USE_SSI1 || defined(__DOXYGEN__)
/**
 * @brief   SSI1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(VectorC8) {

  CH_IRQ_PROLOGUE();

  spi_serve_interrupt(&SPID2);

  CH_IRQ_EPILOGUE();
}
#endif

#if LM4F120_SPI_USE_SSI2 || defined(__DOXYGEN__)
/**
 * @brief   SSI1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector124) {

  CH_IRQ_PROLOGUE();

  spi_serve_interrupt(&SPID3);

  CH_IRQ_EPILOGUE();
}
#endif

#if LM4F120_SPI_USE_SSI3 || defined(__DOXYGEN__)
/**
 * @brief   SSI1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector128) {

  CH_IRQ_PROLOGUE();

  spi_serve_interrupt(&SPID4);

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if LM4F120_SPI_USE_SSI0
  spiObjectInit(&SPID1);
  SPID1.ssi = SSI0;
#endif /* LM4F120_SPI_USE_SSI0 */

#if LM4F120_SPI_USE_SSI1
  spiObjectInit(&SPID2);
  SPID2.ssi = SSI1;
#endif /* LM4F120_SPI_USE_SSI1 */

#if LM4F120_SPI_USE_SSI2
  spiObjectInit(&SPID3);
  SPID3.ssi = SSI2;
#endif /* LM4F120_SPI_USE_SSI2 */

#if LM4F120_SPI_USE_SSI3
  spiObjectInit(&SPID4);
  SPID4.ssi = SSI3;
#endif /* LM4F120_SPI_USE_SSI3 */
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  if (spip->state == SPI_STOP) {
    /* Clock activation.*/
#if LM4F120_SPI_USE_SSI0
    if (&SPID1 == spip) {

      /* XXX GPIOA errata */
      //GPIOA->AFSEL &= ~PINS_SSI0;
      //GPIOA->DEN &= ~PINS_SSI0;

      SYSCTL->RCGCSSI |= (1 << 0);  /* enable SSI0 module */
      SYSCTL->RCGCGPIO |= (1 << 0); /* enable clock to GPIOA */
      GPIOA->AFSEL |= PINS_SSI0;
      GPIOA->DEN |= PINS_SSI0;
      GPIOA->PCTL |= ALTFUNC_SSI0;
      nvicEnableVector(SSI0_IRQn,
                       CORTEX_PRIORITY_MASK(LM4F120_SPI_SSI0_IRQ_PRIORITY));
    }
#endif
#if LM4F120_SPI_USE_SSI1
    if (&SPID2 == spip) {
      SYSCTL->RCGCSSI |= (1 << 1);   /* enable SSI1 module */
      SYSCTL->RCGCGPIO |= (1 << 0);  /* enable clock to GPIOD */
      GPIOD->AFSEL |= PINS_SSI1;
      GPIOD->DEN |= PINS_SSI1;
      GPIOD->PCTL |= ALFUNC_SSI1;
      nvicEnableVector(SSI1_IRQn,
                       CORTEX_PRIORITY_MASK(LM4F120_SPI_SSI1_IRQ_PRIORITY));
    }
#endif
  }
  /* set master operation mode */
  spip->ssi->CR1 = 0;
  /* clock configuration - system clock */
  spip->ssi->CC = 0;
  spip->ssi->ICR  = SSIICR_RTIC | SSIICR_RORIC;
  /* configure the clock prescale divisor */
  spip->ssi->CPSR = spip->config->cpsr;
  /* serial clock rate, phase/polarity, protocol mode, data size */
  spip->ssi->CR0  = spip->config->cr0;
  /* enable SSI */
  spip->ssi->CR1  = SSICR1_SSE;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state != SPI_STOP) {
    spip->ssi->CR1  = 0;
    spip->ssi->CR0  = 0;
    spip->ssi->CPSR = 0;
#if LM4F120_SPI_USE_SSI0
    if (&SPID1 == spip) {
      nvicDisableVector(SSI0_IRQn);
    }
#endif
#if LM4F120_SPI_USE_SSI1
    if (&SPID2 == spip) {
      nvicDisableVector(SSI1_IRQn);
    }
#endif
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  palClearPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  palSetPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This function transmits a series of idle words on the SPI bus and
 *          ignores the received data. This function can be invoked even
 *          when a slave select signal has not been yet asserted.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  spip->rxptr = NULL;
  spip->txptr = NULL;
  spip->rxcnt = spip->txcnt = n;
  ssi_fifo_preload(spip);
  spip->ssi->IM = SSIIM_ROR | SSIIM_RT | SSIIM_TX | SSIIM_RX;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->rxptr = rxbuf;
  spip->txptr = txbuf;
  spip->rxcnt = spip->txcnt = n;
  ssi_fifo_preload(spip);
  spip->ssi->IM = SSIIM_ROR | SSIIM_RT | SSIIM_TX | SSIIM_RX;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  spip->rxptr = NULL;
  spip->txptr = txbuf;
  spip->rxcnt = spip->txcnt = n;
  ssi_fifo_preload(spip);
  spip->ssi->IM = SSIIM_ROR | SSIIM_RT | SSIIM_TX | SSIIM_RX;
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  spip->rxptr = rxbuf;
  spip->txptr = NULL;
  spip->rxcnt = spip->txcnt = n;
  ssi_fifo_preload(spip);
  spip->ssi->IM = SSIIM_ROR | SSIIM_RT | SSIIM_TX | SSIIM_RX;
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spip->ssi->DR = (uint32_t)frame;
  while ((spip->ssi->SR & SSISR_RNE) == 0)
    ;
  return (uint16_t)spip->ssi->DR;
}

#endif /* HAL_USE_SPI */

/** @} */
