#include <rtthread.h>
#include <rthw.h>
#include "board.h"
#include <string.h>

void os_cache_init(void);
extern volatile rt_uint8_t rt_interrupt_nest;

static struct rt_mutex mutex_spiflash = {0};
static struct rt_mutex mutex_cache = {0};

extern uint32_t __update_reuse_ram_addr;
#define LOAD_ADDR   ((uint32_t)&__update_reuse_ram_addr)
void os_stop_forever(void)
{
    rt_enter_critical();
    rt_console_set_device(RT_NULL);
    TMR0CON &= ~BIT(7);
    PICEN &= BIT(IRQ_UART0_2_VECTOR);
    extern rt_thread_t rt_current_thread;
    rt_current_thread = RT_NULL;
    uint8_t *ram_addr = (uint8_t *)LOAD_ADDR;
    memset(ram_addr, 0, 0x7a00 - (LOAD_ADDR - 0x50000));
}

RT_SECTION(".irq.cache")
void cache_init(void)
{
    os_cache_init();
    rt_mutex_init(&mutex_spiflash, "flash_mutex", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&mutex_cache, "cache_mutex", RT_IPC_FLAG_PRIO);
}

RT_SECTION(".irq.cache")
void os_spiflash_lock(void)
{
    if ((rt_thread_self() != RT_NULL) && (rt_interrupt_nest == 0))
    {
        rt_mutex_take(&mutex_spiflash, RT_WAITING_FOREVER);
    }
}

RT_SECTION(".irq.cache")
void os_spiflash_unlock(void)
{
    if ((rt_thread_self() != RT_NULL) && (rt_interrupt_nest == 0))
    {
        rt_mutex_release(&mutex_spiflash);
    }
}

RT_SECTION(".irq.cache")
void os_cache_lock(void)
{
    if ((rt_thread_self() != RT_NULL) && (rt_interrupt_nest == 0))
    {
        rt_mutex_take(&mutex_cache, RT_WAITING_FOREVER);
    }
}

RT_SECTION(".irq.cache")
void os_cache_unlock(void)
{
    if ((rt_thread_self() != RT_NULL) && (rt_interrupt_nest == 0))
    {
        rt_mutex_release(&mutex_cache);
    }
}

RT_SECTION(".irq")
void os_interrupt_enter(void)
{
    if (rt_thread_self() != RT_NULL) {
        rt_interrupt_enter();
    }
}

RT_SECTION(".irq")
void os_interrupt_leave(void)
{
    if (rt_thread_self() != RT_NULL) {
        rt_interrupt_leave();
    }
}

typedef void (*isr_t)(void);
RT_SECTION(".irq")
isr_t register_isr(int vector, isr_t isr)
{
    char buf[8] = {0};
    rt_snprintf(buf, sizeof(buf), "sys%d", vector);
    rt_isr_handler_t handle = (rt_isr_handler_t)isr;
    rt_hw_interrupt_install(vector, handle, RT_NULL, buf);
}

static void timer0_delay_start(void)
{
    TMR0CNT = 0;
    TMR0PR = 0xffffffff;
}

static void timer0_delay_wait(uint32_t ticks)
{
    while (TMR0CNT < ticks);
}

static bool timer0_delay_check(uint32_t ticks)
{
    if (TMR0CNT > ticks) {
        return true;
    }

    return false;
}

uint32_t hal_get_ticks(void)
{
    return rt_tick_get();
}

void hal_mdelay(uint32_t nms)
{
    if (rt_thread_self() != RT_NULL) {
        rt_thread_mdelay(nms);
    } else {
        timer0_delay_start();
        timer0_delay_wait(get_sysclk_nhz() / RT_TICK_PER_SECOND * nms); // TODO: overflow
    }
}

void hal_udelay(uint32_t nus)
{
    if (rt_thread_self() != RT_NULL) {
        rt_hw_us_delay(nus);
    } else {
        timer0_delay_start();
        timer0_delay_wait(get_sysclk_nhz() / RT_TICK_PER_SECOND / 1000 * nus); // TODO: overflow
    }
}

#ifdef RT_USING_CONSOLE
void hal_printf(const char *fmt, ...)
{
    rt_device_t console = rt_console_get_device();

    va_list args;
    rt_size_t length;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    if (length > RT_CONSOLEBUF_SIZE - 1)
        length = RT_CONSOLEBUF_SIZE - 1;
#ifdef RT_USING_DEVICE
    if (console == RT_NULL)
    {
        rt_hw_console_output(rt_log_buf);
    }
    else
    {
        rt_uint16_t old_flag = console->open_flag;

        console->open_flag |= RT_DEVICE_FLAG_STREAM;
        rt_device_write(console, 0, rt_log_buf, length);
        console->open_flag = old_flag;
    }
#else
    rt_hw_console_output(rt_log_buf);
#endif
    va_end(args);
}
#endif
