/*
 * Copyright (c) 2021-2021, Bluetrum Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Data             Version     Notes
 * 2021-11-19       v1.0        First version
 */

#ifndef __DRIVER_SPI_H__
#define __DRIVER_SPI_H__

#ifdef  __cplusplus
extern "C"
{
#endif

#include "driver_common.h"

#define BLUE_SPI_API_VERSION                BLUE_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* API version */

#define BLUE_SPI_CONTROL_POS                 0
#define BLUE_SPI_CONTROL_MASK               (0xFFUL << BLUE_SPI_CONTROL_POS)

/*----- SPI Control Codes: Mode -----*/
#define BLUE_SPI_MODE_INACTIVE              (0x00UL << BLUE_SPI_CONTROL_POS)     ///< SPI Inactive
#define BLUE_SPI_MODE_MASTER                (0x01UL << BLUE_SPI_CONTROL_POS)     ///< SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
#define BLUE_SPI_MODE_SLAVE                 (0x02UL << BLUE_SPI_CONTROL_POS)     ///< SPI Slave  (Output on MISO, Input on MOSI)
#define BLUE_SPI_MODE_MASTER_SIMPLEX        (0x03UL << BLUE_SPI_CONTROL_POS)     ///< SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
#define BLUE_SPI_MODE_SLAVE_SIMPLEX         (0x04UL << BLUE_SPI_CONTROL_POS)     ///< SPI Slave  (Output/Input on MISO)

#define BLUE_SPI_SET_MAPPING                (0x20UL << BLUE_SPI_CONTROL_POS)     ///< Set SPI mapping; arg = group (from 1 to 5);
                                                                                 ///< (data_in clk data_out) G1: PA2 PA3 PA4; G2: PA5 PA6 PA7
                                                                                 ///< G3: PB0 PB1 PB2; G4: PE5 PE6 PE7; G%: PF0 PF1 PF2
#define BLUE_SPI_ENABLE                     (0x21UL << BLUE_SPI_CONTROL_POS)

/****** SPI Event *****/
#define BLUE_SPI_EVENT_TRANSFER_COMPLETE    (1UL << 0)      ///< Data Transfer completed
#define BLUE_SPI_EVENT_DATA_LOST            (1UL << 1)      ///< Data lost: Receive overflow / Transmit underflow
#define BLUE_SPI_EVENT_MODE_FAULT           (1UL << 2)      ///< Master Mode Fault (SS deactivated when Master)

// Function documentation
/**
  \fn           blue_driver_version_t (*get_version) (void)
  \brief        Get driver version.
  \return       blue_driver_version_t

  \fn           int32_t (*init) (blue_spi_signal_event_t cb_event)
  \brief        Initialize SPI.
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*deinit) (void)
  \brief        De-initialize SPI.
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*power) (uint32_t state)
  \brief        Control SPI Interface Power.
  \param[in]    state   Power state
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*send) (const void *data, uint32_t num)
  \brief        Start sending data to SPI transmitter.
  \param[in]    data    Pointer to buffer with data to send to SPI transmitter
  \param[in]    num     Number of data items to send
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*recv) (void *data, uint32_t num)
  \brief        Start receiving data from SPI receiver.
  \param[out]   data    Pointer to buffer for data to receive from SPI receiver
  \param[in]    num     Number of data items to receive
  \return       status(OK:0, ERROR:<0)
  
  \fn           int32_t (*send_byte) (uint8_t data, int32_t timeout)
  \brief        Start sending one byte of data to SPI transmitter.
  \param[in]    data    Data
  \param[in]    timeout Timeout
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*recv_byte) (int32_t timeout)
  \brief        Start receiving one byte of data from SPI receiver.
  \param[in]    timeout Timeout
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*control) (uint32_t control, uint32_t arg)
  \brief        Control SPI Interface.
  \param[in]    control Operation(e.g. BLUE_SPI_ENABLE)
  \param[in]    arg     Argument of operation (optional)
  \return       status(OK:0, ERROR:<0)

  \fn           void (*blue_spi_signal_event_t) (uint32_t event)
  \brief        Signal SPI Events.
  \param[in]    event   Event(e.g. BLUE_SPI_EVENT_TRANSFER_COMPLETE)
  \return       none
*/
typedef void (*blue_spi_signal_event_t)(uint32_t event);

struct blue_drv_spi
{
    blue_driver_version_t   (*get_version)      (void);
    int32_t                 (*init)             (blue_spi_signal_event_t cb_event);
    int32_t                 (*deinit)           (void);
    int32_t                 (*power)            (uint32_t state);
    int32_t                 (*send)             (const void *data, uint32_t num);
    int32_t                 (*recv)             (void *data, uint32_t num);
    int32_t                 (*send_byte)        (uint8_t data, int32_t timeout);
    int32_t                 (*recv_byte)        (int32_t timeout);
    int32_t                 (*control)          (uint32_t control, uint32_t arg);
};

typedef const struct blue_drv_spi *blue_drv_spi_t;
extern const struct blue_drv_spi blue_spi1;

#ifdef  __cplusplus
}
#endif

#endif /* __DRIVER_SPI_H__ */
