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
 * @file    LM4F120/spi_lld.h
 * @brief   LM4F120 low level SPI driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI_LLD_H_
#define _SPI_LLD_H_

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Hardware FIFO depth.
 */
#define LM4F120_SSI_FIFO_DEPTH  8

#define CR0_DSSMASK             0x0F
#define CR0_DSS4BIT             3
#define CR0_DSS5BIT             4
#define CR0_DSS6BIT             5
#define CR0_DSS7BIT             6
#define CR0_DSS8BIT             7
#define CR0_DSS9BIT             8
#define CR0_DSS10BIT            9
#define CR0_DSS11BIT            0xA
#define CR0_DSS12BIT            0xB
#define CR0_DSS13BIT            0xC
#define CR0_DSS14BIT            0xD
#define CR0_DSS15BIT            0xE
#define CR0_DSS16BIT            0xF
#define CR0_FSPIFF              (0 << 4)  /* Freescale SPI Frame Format */
#define CR0_TISSFF              (1 << 4)  /* TI synchronous serial FF */
#define CR0_MWFF                (2 << 4)  /* MICROWIRE Frame Format */
#define CR0_CLOCKRATE(n)        ((n) << 8)

#define PINS_SSI0               ((1 << 2) | (1 << 4) | (1 << 5))
#define ALTFUNC_SSI0            ((2 << 8) | (2 << 16) | (2 << 20))

#define PINS_SSI1               ((1 << 0) | (1 << 2) | (1 << 3))
#define ALTFUNC_SSI1            ((2 << 0) | (2 << 8) | (2 << 12))

#define PINS_SSI2               ((1 << 4) | (1 << 6) | (1 << 7))
#define ALTFUNC_SSI2            ((2 << 16) | (2 << 24) | (2 << 28))

#define PINS_SSI3               ((1 << 0) | (1 << 2) | (1 << 3))
#define ALTFUNC_SSI3            ((1 << 0) | (1 << 8) | (1 << 12))

/* CR0 bit definitions */
#define SSICR0_DSS              (1 << 0)   /* data size select */
#define SSICR0_FRF              (1 << 4)   /* frame format select */
#define SSICR0_SPO              (1 << 6)   /* serial clock polarity */
#define SSICR0_SPH              (1 << 7)   /* serial clock phase */
#define SSICR0_SCR              (1 << 8)   /* serial clock rate */

/* CR1 bit definitions */
#define SSICR1_LBM              (1 << 0)   /* loopback mode */
#define SSICR1_SSE              (1 << 1)   /* synchronous serial port enable */
#define SSICR1_MS               (1 << 2)   /* master/slave select */
#define SSICR1_SOD              (1 << 3)   /* slave mode output disable */
#define SSICR1_EOT              (1 << 4)   /* end of transmission */
#define SSICR1_SLBY6            (1 << 5)   /* slave bypass mode */

/* SSI status register bit definitions */
#define SSISR_TFE               (1 << 0)   /* transmit FIFO empty */
#define SSISR_TNF               (1 << 1)   /* transmit FIFO not full */
#define SSISR_RNE               (1 << 2)   /* receive FIFO not empty */
#define SSISR_RFF               (1 << 3)   /* receive FIFO full */
#define SSISR_BSY               (1 << 4)   /* ssi busy */

/* SSI interrupt mask register bit definitions */
#define SSIIM_ROR               (1 << 0)   /* receive FIFO overrun interrupt */
#define SSIIM_RT                (1 << 1)   /* receive time-out */
#define SSIIM_RX                (1 << 2)   /* receive FIFO interrupt */
#define SSIIM_TX                (1 << 3)   /* transmit FIFO interrupt */

/* SSI masked interrupt status register bit definitions */
#define SSIMIS_ROR              (1 << 0)   /* receive FIFO overrun interrupt */
#define SSIMIS_RT               (1 << 1)   /* receive time-out */
#define SSIMIS_RX               (1 << 2)   /* receive FIFO interrupt */
#define SSIMIS_TX               (1 << 3)   /* transmit FIFO interrupt */

/* SSI interrupt clear */
#define SSIICR_RORIC            (1 << 0)   /* receive overrun interrupt clear */
#define SSIICR_RTIC             (1 << 1)   /* receive timeout interrupt clear */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for device SSI0 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LM4F120_SPI_USE_SSI0) || defined(__DOXYGEN__)
#define LM4F120_SPI_USE_SSI0                TRUE
#endif

/**
 * @brief   SPI2 driver enable switch.
 * @details If set to @p TRUE the support for device SSI1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LM4F120_SPI_USE_SSI1) || defined(__DOXYGEN__)
#define LM4F120_SPI_USE_SSI1                FALSE
#endif

/**
 * @brief   SPI0 interrupt priority level setting.
 */
#if !defined(LM4F120_SPI_SSI0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LM4F120_SPI_SSI0_IRQ_PRIORITY       5
#endif

/**
 * @brief   SPI1 interrupt priority level setting.
 */
#if !defined(LM4F120_SPI_SSI1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define LM4F120_SPI_SSI1_IRQ_PRIORITY       5
#endif

/**
 * @brief   Overflow error hook.
 * @details The default action is to stop the system.
 */
#if !defined(LM4F120_SPI_SSI_ERROR_HOOK) || defined(__DOXYGEN__)
#define LM4F120_SPI_SSI_ERROR_HOOK(spip)    chSysHalt()
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct SPIDriver SPIDriver;

/**
 * @brief   SPI notification callback type.
 *
 * @param[in] spip      pointer to the @p SPIDriver object triggering the
 *                      callback
 */
typedef void (*spicallback_t)(SPIDriver *spip);

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /**
   * @brief Operation complete callback or @p NULL.
   */
  spicallback_t         end_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief The chip select line port.
   */
  ioportid_t            ssport;
  /**
   * @brief The chip select line pad number.
   */
  uint16_t              sspad;
  /**
   * @brief SSI CR0 initialization data.
   */
  uint16_t              cr0;
  /**
   * @brief SSI CPSR initialization data.
   */
  uint32_t              cpsr;
} SPIConfig;

/**
 * @brief   Structure representing a SPI driver.
 */
struct SPIDriver {
  /**
   * @brief Driver state.
   */
  spistate_t            state;
  /**
   * @brief Current configuration data.
   */
  const SPIConfig       *config;
#if SPI_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief Waiting thread.
   */
  Thread                *thread;
#endif /* SPI_USE_WAIT */
#if SPI_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_USE_MUTEXES || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the bus.
   */
  Mutex                 mutex;
#elif CH_USE_SEMAPHORES
  Semaphore             semaphore;
#endif
#endif /* SPI_USE_MUTUAL_EXCLUSION */
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the SSI registers block.
   */
  SSI0_Type             *ssi;
  /**
   * @brief Number of bytes yet to be received.
   */
  uint32_t              rxcnt;
  /**
   * @brief Receive pointer or @p NULL.
   */
  void                  *rxptr;
  /**
   * @brief Number of bytes yet to be transmitted.
   */
  uint32_t              txcnt;
  /**
   * @brief Transmit pointer or @p NULL.
   */
  const void            *txptr;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if LM4F120_SPI_USE_SSI0 && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif

#if LM4F120_SPI_USE_SSI1 && !defined(__DOXYGEN__)
extern SPIDriver SPID2;
#endif

#if LM4F120_SPI_USE_SSI2 && !defined(__DOXYGEN__)
extern SPIDriver SPID3;
#endif

#if LM4F120_SPI_USE_SSI3 && !defined(__DOXYGEN__)
extern SPIDriver SPID4;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  void spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
  void spi_lld_ignore(SPIDriver *spip, size_t n);
  void spi_lld_exchange(SPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _SPI_LLD_H_ */

/** @} */
