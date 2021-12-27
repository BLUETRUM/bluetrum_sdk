/*
 * Copyright (c) 2021-2021, Bluetrum Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Data             Version     Notes
 * 2021-12-24       v1.0        First version
 */

#ifndef __DRIVER_UPDATE_H__
#define __DRIVER_UPDATE_H__

#ifdef  __cplusplus
extern "C"
{
#endif

#include "driver_common.h"

#define BLUE_UPDATE_API_VERSION                     BLUE_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* API version */

#define BLUE_UPDATE_CONTROL_POS                     (0)
#define BLUE_UPDATE_CONTROL_MASK                    (0xFFUL << BLUE_UPDATE_CONTROL_POS)

#define BLUE_UPDATE_CUSTOM_PARTITION_ENABLE         (0x00UL << BLUE_UPDATE_CONTROL_POS) ///> Update custom partition
#define BLUE_UPDATE_UART_ENABLE                     (0x01UL << BLUE_UPDATE_CONTROL_POS) ///> Update UART ENABLE

/****** Update Information ******/
#define BLUE_UPDATE_INFORMATION_MACHINE_STATE       (1UL << 0)

/****** Update Event ******/
#define BLUE_UPDATE_EVENT_UART_INIT                 (1UL << 0)

enum update_state
{
    UPDATE_STATE_WAIT_INIT = 0,
    UPDATE_STATE_1,
    UPDATE_STATE_2,
    UPDATE_STATE_3,
    UPDATE_STATE_4,
    UPDATE_STATE_5,
    UPDATE_STATE_ERR,
};

struct update_uart_info
{
    uint32_t uart_baud;     ///> The minimum value is 115200
    uint32_t uart_pin_map;  ///> UART pin mapping
    uint8_t  uart_rx_bit;   ///> From (1 << 0) to (1 << 7)
    uint8_t  uart_tx_bit;   ///> From (1 << 0) to (1 << 7)
    uint8_t  uart_select;   ///> UART0:0, UART1: 1, UART2: 2
    uint8_t  reserved;
};

typedef struct update_uart_info *update_uart_info_t;
typedef void (*update_uart_tx_byte_t)(uint8_t c);
typedef bool (*update_uart_rx_byte_t)(uint8_t *c);

struct update_uart_trans_init
{
    update_uart_info_t uart_info_p;
    update_uart_rx_byte_t rx_func_p;
    update_uart_tx_byte_t tx_func_p;
};
typedef struct update_uart_trans_init *update_uart_trans_init_t;

struct update_custom_partition
{
    uint32_t flash_size;
    uint32_t update_flash_start_address;
    uint32_t update_flash_end_address;
    uint32_t update_ram_start_address;
    uint32_t update_ram_end_address;
};
typedef struct update_custom_partition *update_custom_partition_t;

// Function documentation
/**  
  \fn           blue_driver_version_t (*get_version) (void)
  \brief        Get driver version.
  \return       blue_driver_version_t

  \fn           int32_t (*init) (blue_update_signal_event_t cb_event, blue_update_allocator_t upd_alloc)
  \brief        Initialize update.
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (deinit) (void)
  \brief        De-initialize update.
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*machine) (void)
  \brief        The update state machine.
  \return       status(OK:machine state, ERROR:<0)

  \fn           int32_t (*get_information) (uint32_t information, uint32_t *arg)
  \brief        Get the information of update.
  \param[in]    information Information(e.g. BLUE_UPDATE_INFORMATION_MACHINE_STATE)
  \param[in]    arg         Argument of operation (optional)
  \return       status(OK:0, ERROR:<0)

  \fn           int32_t (*control) (uint32_t control, uint32_t arg)
  \brief        Control update.
  \param[in]    control Operation(e.g. BLUE_UPDATE_CUSTOM_PARTITION_ENABLE)
  \param[in]    arg     Argument of operation (optional)
  \return       status(OK:0, ERROR:<0)

  \fn           void (*blue_update_signal_event_t) (uint32_t event)
  \brief        Signal update events.
  \param[in]    event   Event(e.g. BLUE_UPDATE_EVENT_UART_INIT)
  \return       none

  \fn           int32_t (*blue_update_allocator_t) (void **buf, uint32_t size)
  \brief        Update allocator.
  \param[in]    buf     Point to the point of update ram
  \param[in]    size    Ram required for update
  \return       status(OK:0, ERROR:<0)
*/
typedef void    (*blue_update_signal_event_t)   (uint32_t event);
typedef int32_t (*blue_update_allocator_t)      (void **buf, uint32_t size);

struct blue_drv_update
{
    blue_driver_version_t   (*get_version)      (void);
    int32_t                 (*init)             (blue_update_signal_event_t cb_event, blue_update_allocator_t upd_alloc);
    int32_t                 (*deinit)           (void);
    int32_t                 (*machine)          (void);
    int32_t                 (*get_information)  (uint32_t information, uint32_t *arg);
    int32_t                 (*control)          (uint32_t control, uint32_t arg);
};
extern struct blue_drv_update blue_update;

#ifdef  __cplusplus
}
#endif

#endif /* __DRIVER_UPDATE_H__ */
