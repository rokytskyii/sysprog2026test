/**
 * @file perf.h
 * @brief Вимірювання часу (DWT CYCCNT), статистики задач, stack painting.
 * @note Заголовок без залежностей від HAL — можна включати з App.
 */
#ifndef PERF_H
#define PERF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        const char *name;
        uint32_t count;
        uint32_t min_cyc;
        uint32_t max_cyc;
        uint64_t sum_cyc;
    } perf_stat_t;

    void perf_init(void);
    uint32_t perf_now(void);
    uint32_t perf_cycles_to_us(uint32_t cyc);
    uint32_t perf_us_to_cycles(uint32_t us);

    void perf_stat_reset(perf_stat_t *s);
    void perf_stat_add(perf_stat_t *s, uint32_t cyc);
    void perf_stat_print(const perf_stat_t *s);

    /** Мітка події для хост-аналізу: EVT t_us=... name=... */
    void perf_evt(const char *name);

    /** CPU load: додати зайняті цикли; раз на ~1 с друкує LOAD pct=... */
    void perf_load_add_busy(uint32_t cyc);
    void perf_load_poll(void);

    void perf_stack_paint(void);
    uint32_t perf_stack_used_bytes(void);
    uint32_t perf_stack_reserve_bytes(void);

#ifdef __cplusplus
}
#endif

#endif /* PERF_H */
