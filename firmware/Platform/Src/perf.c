#include "perf.h"
#include "stm32f4xx.h"

#include <stdio.h>
#include <string.h>

extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;

#define STACK_PATTERN 0xA5A5A5A5u

static uint32_t s_busy_cyc;
static uint32_t s_window_start;

void perf_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_busy_cyc = 0;
    s_window_start = DWT->CYCCNT;
}

uint32_t perf_now(void)
{
    return DWT->CYCCNT;
}

uint32_t perf_cycles_to_us(uint32_t cyc)
{
    return cyc / (SystemCoreClock / 1000000u);
}

uint32_t perf_us_to_cycles(uint32_t us)
{
    return us * (SystemCoreClock / 1000000u);
}

void perf_stat_reset(perf_stat_t *s)
{
    if (!s)
        return;
    s->count = 0;
    s->min_cyc = 0;
    s->max_cyc = 0;
    s->sum_cyc = 0;
}

void perf_stat_add(perf_stat_t *s, uint32_t cyc)
{
    if (!s)
        return;
    if (s->count == 0 || cyc < s->min_cyc)
        s->min_cyc = cyc;
    if (cyc > s->max_cyc)
        s->max_cyc = cyc;
    s->sum_cyc += cyc;
    s->count++;
}

void perf_stat_print(const perf_stat_t *s)
{
    if (!s || !s->name)
        return;
    if (s->count == 0)
    {
        printf("STAT name=%s n=0\r\n", s->name);
        return;
    }
    uint32_t avg = (uint32_t)(s->sum_cyc / s->count);
    printf("STAT name=%s n=%lu min_us=%lu avg_us=%lu max_us=%lu\r\n",
           s->name,
           (unsigned long)s->count,
           (unsigned long)perf_cycles_to_us(s->min_cyc),
           (unsigned long)perf_cycles_to_us(avg),
           (unsigned long)perf_cycles_to_us(s->max_cyc));
}

void perf_evt(const char *name)
{
    if (!name)
        return;
    printf("EVT t_us=%lu name=%s\r\n",
           (unsigned long)perf_cycles_to_us(perf_now()),
           name);
}

void perf_load_add_busy(uint32_t cyc)
{
    s_busy_cyc += cyc;
}

void perf_load_poll(void)
{
    uint32_t now = perf_now();
    uint32_t window = now - s_window_start;
    if (window >= SystemCoreClock)
    {
        uint32_t pct = (uint32_t)(((uint64_t)s_busy_cyc * 100u) / window);
        printf("LOAD pct=%lu\r\n", (unsigned long)pct);
        s_busy_cyc = 0;
        s_window_start = now;
    }
}

static uint32_t *stack_bottom(void)
{
    return (uint32_t *)((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size);
}

void perf_stack_paint(void)
{
    uint32_t *p = stack_bottom();
    uint32_t *sp = (uint32_t *)__get_MSP();
    while (p < (sp - 16))
        *p++ = STACK_PATTERN;
}

uint32_t perf_stack_used_bytes(void)
{
    uint32_t *p = stack_bottom();
    while (p < (uint32_t *)&_estack && *p == STACK_PATTERN)
        p++;
    return (uint32_t)&_estack - (uint32_t)p;
}

uint32_t perf_stack_reserve_bytes(void)
{
    return (uint32_t)&_Min_Stack_Size;
}
