#ifndef APP_H
#define APP_H

#include "bus_interfaces.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Ініціалізація продукту Incident Recorder + Self-Test
     */
    void app_init(const i2c_interface_t *i2c_bus,
                  const sccb_interface_t *sccb_bus,
                  const spi_interface_t *spi_bus,
                  const pwm_interface_t *pwm_bus,
                  const board_io_interface_t *board_io,
                  const camera_parallel_interface_t *cam_parallel,
                  const uart_rx_interface_t *cli_uart,
                  void (*delay_ms)(uint32_t));

    /**
     * @brief Один крок super loop (усі задачі з мітками часу)
     */
    void app_run_step(uint32_t current_tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
