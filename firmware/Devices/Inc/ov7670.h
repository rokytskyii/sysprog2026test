#ifndef OV7670_H
#define OV7670_H

#include "bus_interfaces.h"

#define OV7670_SCCB_ADDR 0x21
#define OV7670_REG_PID 0x0A
#define OV7670_REG_VER 0x0B

#define OV7670_QQVGA_W 160u
#define OV7670_QQVGA_H 120u
#define OV7670_FRAME_BYTES (OV7670_QQVGA_W * OV7670_QQVGA_H)

typedef struct
{
    const sccb_interface_t *sccb;
    const camera_parallel_interface_t *parallel;
    uint8_t dev_addr;
    void (*delay_ms)(uint32_t ms);
} ov7670_t;

bool ov7670_init(ov7670_t *dev,
                 const sccb_interface_t *sccb,
                 const camera_parallel_interface_t *parallel,
                 uint8_t dev_addr,
                 void (*delay_ms)(uint32_t ms));
bool ov7670_read_id(ov7670_t *dev, uint8_t *pid, uint8_t *ver);
bool ov7670_configure_qqvga_yuv(ov7670_t *dev);
bool ov7670_capture_gray(ov7670_t *dev, uint8_t *dst, uint32_t timeout_ms);

#endif /* OV7670_H */
