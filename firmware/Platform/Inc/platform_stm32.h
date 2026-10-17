#ifndef PLATFORM_STM32_H
#define PLATFORM_STM32_H

#include "bus_interfaces.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Ініціалізація платформних інтерфейсів шин Nucleo-F411RE
     */
    void platform_init(void);

    /**
     * @brief Отримання екземплярів інтерфейсів периферійних шин
     */
    const i2c_interface_t *platform_get_i2c1(void);
    const sccb_interface_t *platform_get_sccb(void);
    const spi_interface_t *platform_get_spi2(void);
    const pwm_interface_t *platform_get_servo_pwm(void);

    /**
     * @brief Платформна функція мілісекундної затримки
     */
    void platform_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_STM32_H */