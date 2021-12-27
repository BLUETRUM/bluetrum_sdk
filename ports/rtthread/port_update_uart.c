#include <rtthread.h>
#include <board.h>
#include <driver_update.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

void my_printf(const char *format, ...);
void os_interrupt_enter(void);
void os_interrupt_leave(void);

#define UPDATE_UART_BUF_SIZE        1024
#define UPDATE_UART_BUF_PTR         update_uart_buf
static uint8_t update_uart_buf[UPDATE_UART_BUF_SIZE];

#define UART_RX_BIT     BIT(3)
#define UART_TX_BIT     BIT(4)

struct uart_update
{
    uint8_t *rx_buf;
    uint16_t w_idx;
    uint16_t r_idx;
    uint16_t rx_len;
};
typedef struct uart_update *uart_update_t;
static struct uart_update _update;

typedef void (*isr_t)(void);
isr_t register_isr(int vector, isr_t isr);

static void update_uart_rx_reset(void)
{
    memset(&_update, 0, sizeof(_update));
}

static void uart1_putchar(uint8_t c)
{
    while (!(UART1CON & UART_FLAG_TXPND));
    UART1DATA = c;
}

RT_SECTION(".com_text")
static void print_r(void *data, uint32_t size)
{
    uint8_t *ptr = (uint8_t *)data;
    for (uint32_t i = 0; i < size; i++) {
        if ((i+1) % 16 != 0) {
            my_printf("%02x ", ptr[i]);
        } else {
            my_printf("%02x\n", ptr[i]);
        }
    }
}

RT_SECTION(".com_text")
static void print_update_info(uint32_t tag)
{
    my_printf("%d _update %p %d %d %p %d\n", tag, &_update, _update.r_idx, _update.w_idx, _update.rx_buf, _update.rx_len);
}

static void print_update_data(uint32_t tag)
{
    my_printf("tag=%d\n", tag);
    print_r(_update.rx_buf, UPDATE_UART_BUF_SIZE);
}

RT_SECTION(".com_text")
static void check_update_rxbuf(uint32_t tag)
{
    if (_update.rx_buf != UPDATE_UART_BUF_PTR) {
        my_printf("\n\n");
        print_update_info(tag);
        print_update_data(tag);
        while(1);
    }
}

// AT(.com_text.str2)
// static const char str2[] = "[%x";
RT_SECTION(".com_text")
static bool uart1_read_fifo(uint8_t *c)
{
    check_update_rxbuf(2);
    if (_update.r_idx != _update.w_idx) {
        *c = _update.rx_buf[_update.r_idx & _update.rx_len];
        // my_printf(str2, _update.rx_buf[_update.r_idx & _update.rx_len]);
        // my_printf(" %d %d]", _update.r_idx, _update.w_idx);
        _update.r_idx = (_update.r_idx + 1) % _update.rx_len;
        return true;
    }
    return false;
}

// AT(.com_text.str)
// static const char str[] = "<%02x %d";

RT_SECTION(".com_text")
static void uart1_interrupt_func(void)
{
    // static uint8_t cnt = 0;
    GPIOA ^= BIT(0);
    os_interrupt_enter();
    check_update_rxbuf(1);
    if (UART1CON & UART_FLAG_RXPND) {
        UART1CPND = UART_FLAG_RXPND;
        _update.rx_buf[_update.w_idx & _update.rx_len] = UART1DATA;
        // if (cnt++ > 10) {
        //     cnt = 0;
        //     my_printf(str, _update.rx_buf[_update.w_idx & _update.rx_len], _update.w_idx);
        // }
        _update.w_idx = (_update.w_idx + 1) % _update.rx_len;
    }
    os_interrupt_leave();
}

static void uart1_interrupt_init(void)
{
    UART1CON   |= UART_RXIT_ENABLE;
    UART1CPND   = UART_FLAG_RXPND;

    register_isr(IRQ_UART0_2_VECTOR, uart1_interrupt_func);
}

static void update_uart_port_init(void)
{
    GPIOADE     |=  UART_RX_BIT;
    GPIOAPU     |=  UART_RX_BIT;
    GPIOADIR    |=  UART_RX_BIT;
    GPIOAFEN    |=  UART_RX_BIT;
    GPIOADRV    |=  UART_RX_BIT;

    GPIOADE     |=  UART_TX_BIT;
    GPIOADIR    &= ~UART_TX_BIT;
    GPIOAFEN    |=  UART_TX_BIT;
    GPIOADRV    |=  UART_TX_BIT;

    FUNCMCON0   |=  0xf << 28;
    FUNCMCON0   |=  0x2 << 28;
    FUNCMCON0   |=  0xf << 24;
    FUNCMCON0   |=  0x2 << 24;
}

static void update_uart_interrupt_init(void)
{
    update_uart_rx_reset();
    _update.rx_buf = UPDATE_UART_BUF_PTR;
    _update.rx_len = UPDATE_UART_BUF_SIZE;
    uart1_interrupt_init();
}

static void update_uart_init(uint32_t baud)
{
    if (UART1CON & UART_MODULE_ENABLE) {
        while (!(UART1CON & UART_FLAG_TXPND));
    }

    // UART1
    UART1CON = 0;
    CLKGAT0 |= BIT(21);
    uint32_t baud_cfg = (26000000/2)/baud;
    UART1BAUD = (baud_cfg << 16) | baud_cfg;
    UART1CON = UART_MODULE_ENABLE | UART_SB2_ENABLE | UART_CLK_SRC1 | UART_RX_ENABLE;

    update_uart_port_init();
    update_uart_interrupt_init();
}

RT_SECTION(".com_text")
static bool update_uart_read_fifo(uint8_t *c)
{
    return uart1_read_fifo(c);
}

static void update_uart_putchar(uint8_t c)
{
    uart1_putchar(c);
}

static void blue_update_callbacks(uint32_t event)
{
    switch (event) {
    case 1:
        update_uart_init(115200);
        break;
    }
}

#define UPDATE_THREAD_PRIORITY          1
#define UPDATE_THREAD_STACK_SIZE        512
#define UPDATE_THREAD_TIMESLICE         1

void os_stop_forever(void);
static rt_thread_t upd_tid = NULL;

extern uint32_t __update_reuse_ram_addr;
#define LOAD_ADDR   ((uint32_t)&__update_reuse_ram_addr)
uint32_t update_get_load_addr(void)
{
    return LOAD_ADDR;
}

static int32_t blue_update_allocator(void **buf, uint32_t size)
{
    *buf = rt_malloc(size);
    my_printf("%s size=%d buf=0x%p\n", __func__, size, *buf);
}

#define UART_RX1_G2_PA3             ((2 << 28)|(0 << 0))
#define UART_TX1_G2_PA4             ((2 << 24)|(0 << 2))

static struct update_uart_info _uart_info = {
    .uart_baud = 115200,
    .uart_pin_map = (UART_RX1_G2_PA3 | UART_TX1_G2_PA4),
    .uart_rx_bit = BIT(3),
    .uart_tx_bit = BIT(4),
    .uart_select = 1,
};

static struct update_uart_trans_init _uart_init = {
    .uart_info_p = (void *)&_uart_info,
    .rx_func_p = update_uart_read_fifo,
    .tx_func_p = update_uart_putchar,
};

struct update_custom_partition _partition =
{
    .flash_size = 0x100000,
    .download_start_address = 0x100000 - 0x5000,
    .download_end_address = 0x100000,
};

static void update_thread_entry(void *parameter)
{
    if (blue_update.init(blue_update_callbacks, blue_update_allocator) < 0) {
        my_printf("blue_update init failed\n");
        return;
    }

    my_printf("blue_update.get_version() api=0x%x drv=0x%x\n", blue_update.get_version()->api, blue_update.get_version()->drv);
    int32_t ret = blue_update.control(BLUE_UPDATE_UART_ENABLE, (uint32_t)&_uart_init);
    my_printf("BLUE_UPDATE_UART_ENABLE ret = %d\n", ret);
    
    ret = blue_update.control(BLUE_UPDATE_CUSTOM_PARTITION_ENABLE, (uint32_t)&_partition);

    // os stop
    os_stop_forever();

    uint32_t machine_state = UPDATE_STATE_WAIT_INIT;
    blue_update.get_information(BLUE_UPDATE_INFORMATION_MACHINE_STATE, &machine_state);
    my_printf("machine_state=%d\n", machine_state);

    while (1) {
        if ((blue_update.machine() & 0x7fffffff) < UPDATE_STATE_ERR) {
            continue;
        }

        my_printf("update error!\n");
        while(1);
    }
}

static int update_thread_init(void)
{
    // Initial thread
    upd_tid = rt_thread_create("upd",
                               update_thread_entry,
                               NULL,
                               UPDATE_THREAD_STACK_SIZE,
                               UPDATE_THREAD_PRIORITY,
                               UPDATE_THREAD_TIMESLICE);

    if (upd_tid != NULL)
        rt_thread_startup(upd_tid);
}
// MSH_CMD_EXPORT(update_thread_init, "update");
INIT_APP_EXPORT(update_thread_init);
