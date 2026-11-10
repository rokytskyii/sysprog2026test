#ifndef PLATFORM_STM32_H
#define PLATFORM_STM32_H

#include "bus_interfaces.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void platform_init(void);

    const i2c_interface_t *platform_get_i2c1(void);
    const sccb_interface_t *platform_get_sccb(void);
    const spi_interface_t *platform_get_spi2(void);
    const pwm_interface_t *platform_get_servo_pwm(void);
    const board_io_interface_t *platform_get_board_io(void);
    const camera_parallel_interface_t *platform_get_camera_parallel(void);
    const uart_rx_interface_t *platform_get_cli_uart(void);

    void platform_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_STM32_H */
