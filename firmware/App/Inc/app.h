#ifndef APP_H
#define APP_H

#include "bus_interfaces.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Ініціалізація логіки реєстратора інцидентів та проведення Self-Test
     */
    void app_init(const i2c_interface_t *i2c_bus,
                  const sccb_interface_t *sccb_bus,
                  const spi_interface_t *spi_bus,
                  const pwm_interface_t *pwm_bus,
                  void (*delay_ms)(uint32_t));

    /**
     * @brief Основна періодична функція виконання логіки (викликається в циклі)
     * @param current_tick_ms Поточний системний час у мілісекундах
     */
    void app_run_step(uint32_t current_tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */