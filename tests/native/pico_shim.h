/* Minimal, deliberately strict peripheral boundary for the actual firmware C.
 * No playback decisions live here. Tests control clock time and FIFO consumption.
 * Queue copies have SDK semantics; a full queue rejects rather than overwrites.
 * This is deterministic single-thread execution, NOT a multicore emulator.
 */
#ifndef TIMINGBOX_PICO_SHIM_H
#define TIMINGBOX_PICO_SHIM_H
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <setjmp.h>
typedef unsigned uint;
#define count_of(a) (sizeof(a)/sizeof((a)[0]))
#define UART0_IRQ 0
#define UART_PARITY_NONE 0
#define GPIO_FUNC_UART 2
#define pio0 0
#define uart0 0
typedef int PIO;
typedef struct { unsigned size, depth, read, level; unsigned char data[32][32]; } queue_t;
typedef struct { unsigned base, count, set_count, div_int, div_frac; } pio_sm_config;
static uint32_t fake_time, fifo[4], submitted[20000];
static unsigned fifo_level, submitted_count, violations, dropped, context;
static unsigned uart_context_errors, block_on_empty, tx_count, rx_pos, rx_count;
static unsigned char rx[8192], tx[16384];
static uint32_t gpio_levels, gpio_directions, gpio_functions, output_base, output_count;
static bool active;
static uint32_t active_duration, active_mask;
static unsigned clock_requests;
static unsigned irq_handler_registrations, uart_irq_enabled, cpu_uart_irq_enabled;
static bool startup_through_core0;
static unsigned tx_capacity = 16384;
static jmp_buf trap;
static bool trap_enabled;
static const char *trap_reason;
static void fault(const char *reason) {
    trap_reason=reason;
    if (trap_enabled) longjmp(trap,1);
    fprintf(stderr,"Peripheral contract violation: %s\n",reason); exit(3);
}
static void queue_init(queue_t *q,unsigned size,unsigned depth) {
    if(size>32 || depth>32 || !depth) fault("invalid queue configuration");
    memset(q,0,sizeof(*q)); q->size=size; q->depth=depth;
}
static bool queue_is_empty(queue_t *q) { return !q->level; }
static bool queue_is_full(queue_t *q) { return q->level==q->depth; }
static unsigned queue_get_level(queue_t *q) { return q->level; }
static bool queue_try_add(queue_t *q,const void *p) {
    if(queue_is_full(q)) { dropped++; return false; }
    memcpy(q->data[(q->read+q->level)%q->depth],p,q->size); q->level++; return true;
}
static bool queue_try_remove(queue_t *q,void *p) {
    if(!q->level) return false;
    memcpy(p,q->data[q->read],q->size); q->read=(q->read+1)%q->depth; q->level--; return true;
}
static bool uart_is_readable(int id) {
    (void)id;
    if(context!=0) uart_context_errors++;
    return rx_pos<rx_count;
}
static unsigned char uart_getc(int id) {
    (void)id;
    if(rx_pos==rx_count) { block_on_empty++; fault("UART read would block awaiting a missing packet byte"); }
    return rx[rx_pos++];
}
static bool uart_is_writable(int id) { (void)id; if(context!=0) uart_context_errors++; return tx_count<tx_capacity; }
static void uart_putc_raw(int id,unsigned char byte) {
    (void)id;if(context!=0) uart_context_errors++;
    if(tx_count>=tx_capacity || tx_count>=sizeof(tx)) fault("UART write would block");
    tx[tx_count++]=byte;
}
static bool pio_sm_is_tx_fifo_empty(PIO p,unsigned sm) { (void)p; return sm==0 ? fifo_level==0 : true; }
static void pio_sm_put(PIO p,unsigned sm,uint32_t value) {
    (void)p;
    if(sm==1) {clock_requests++; return;}
    if(fifo_level==4) { violations++; return; }
    if(submitted_count==count_of(submitted)) fault("test event budget exhausted");
    fifo[fifo_level++]=value; submitted[submitted_count++]=value;
}
static void pio_sm_put_blocking(PIO p,unsigned sm,uint32_t v) { pio_sm_put(p,sm,v); }
static uint32_t pio_sm_get_blocking(PIO p,unsigned sm) {
    (void)p; if(sm!=1) fault("unexpected PIO RX read");
    return 0xFFFFFFu-(fake_time&0xFFFFFFu);
}
static void pio_sm_drain_tx_fifo(PIO p,unsigned sm) { (void)p; if(sm==0) fifo_level=0; }
static void pio_sm_clear_fifos(PIO p,unsigned sm) { pio_sm_drain_tx_fifo(p,sm); }
static void pio_sm_restart(PIO p,unsigned sm) { (void)p; if(sm==0) active=false; }
static void pio_sm_set_enabled(PIO p,unsigned sm,bool enabled) { (void)p;(void)sm;(void)enabled; }
/* An entry can be active while the FIFO already contains its successor. Tests
 * choose when the active entry completes, without assuming PIO timing is correct. */
static bool consume_entry(void) {
    if(fifo_level<2) return false;
    active=true; active_duration=fifo[0]; active_mask=fifo[1];
    memmove(fifo,fifo+2,(fifo_level-2)*sizeof(*fifo)); fifo_level-=2; return true;
}
static void gpio_put(unsigned pin,bool v) { if(v) gpio_levels|=1u<<pin; else gpio_levels&=~(1u<<pin); }
static void gpio_init(unsigned pin) { (void)pin; }
static void gpio_set_dir(unsigned pin,bool v) { if(v) gpio_directions|=1u<<pin; else gpio_directions&=~(1u<<pin); }
static void gpio_set_function(unsigned pin,unsigned f) { if(f!=GPIO_FUNC_UART) gpio_functions|=1u<<pin; }
static void pio_gpio_init(PIO p,unsigned pin) { (void)p; gpio_functions|=1u<<pin; }
static void pio_sm_set_pins_with_mask(PIO p,unsigned sm,uint32_t v,uint32_t m) { (void)p;(void)sm; gpio_levels=(gpio_levels&~m)|(v&m); }
static void pio_sm_set_pindirs_with_mask(PIO p,unsigned sm,uint32_t v,uint32_t m) { (void)p;(void)sm; gpio_directions=(gpio_directions&~m)|(v&m); }
static pio_sm_config pinDriver_program_get_default_config(unsigned o) { (void)o; return (pio_sm_config){0}; }
static pio_sm_config pianolaClock_program_get_default_config(unsigned o) { (void)o; return (pio_sm_config){0}; }
static void sm_config_set_out_pins(pio_sm_config *c,unsigned b,unsigned n) { if(n>32 || b>31) fault("invalid PIO OUT range"); c->base=b;c->count=n; }
static void sm_config_set_set_pins(pio_sm_config *c,unsigned b,unsigned n) { (void)b; if(n>5) fault("PIO SET count exceeds five pins"); c->set_count=n; }
static void sm_config_set_clkdiv_int_frac8(pio_sm_config *c,unsigned i,unsigned f) { c->div_int=i;c->div_frac=f; }
static void pio_sm_init(PIO p,unsigned sm,unsigned o,const pio_sm_config *c) { (void)p;(void)o; if(sm==0) {output_base=c->base;output_count=c->count;} }
static void stdio_init_all(void) {}
static void sleep_us(unsigned n) { (void)n; }
static void uart_init(int id,unsigned baud) { (void)id;(void)baud; }
static void uart_set_hw_flow(int id,bool a,bool b) { (void)id;(void)a;(void)b; }
static void uart_set_format(int id,unsigned a,unsigned b,unsigned c) { (void)id;(void)a;(void)b;(void)c; }
static void irq_set_exclusive_handler(unsigned irq,void (*fn)(void)) { (void)irq;(void)fn;irq_handler_registrations++; }
static void irq_set_enabled(unsigned irq,bool v) { (void)irq;cpu_uart_irq_enabled=v; }
static void uart_set_irq_enables(int id,bool a,bool b) { (void)id;uart_irq_enabled=a||b; }
static const int pinDriver_program=0, pianolaClock_program=1;
static bool pio_claim_free_sm_and_add_program_for_gpio_range(const int *a,PIO *p,uint *sm,uint *o,uint b,uint n,bool v) { (void)a;(void)p;(void)b;(void)n;(void)v;*sm=0;*o=0;return true; }
static bool pio_claim_free_sm_and_add_program(const int *a,PIO *p,uint *sm,uint *o) { (void)a;(void)p;*sm=1;*o=0;return true; }
static void pio_enable_sm_mask_in_sync(PIO p,uint32_t m) { (void)p;(void)m; }
static void multicore_launch_core1(void (*fn)(void)) { (void)fn; if(!startup_through_core0) fault("firmware startup reached core-1 launch"); }
static void tight_loop_contents(void) { if(startup_through_core0) fault("core-0 foreground loop reached"); }
#include "pianolaPinDriver.pio.h"
#include "pianola24bitClock.pio.h"
#endif
