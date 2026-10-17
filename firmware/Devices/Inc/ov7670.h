#ifndef OV7670_H
#define OV7670_H

#include "bus_interfaces.h"

#define OV7670_SCCB_ADDR 0x21
#define OV7670_REG_PID 0x0A
#define OV7670_REG_VER 0x0B

typedef struct
{
    const sccb_interface_t *sccb;
    uint8_t dev_addr;
} ov7670_t;

bool ov7670_init(ov7670_t *dev, const sccb_interface_t *sccb, uint8_t dev_addr);
bool ov7670_read_id(ov7670_t *dev, uint8_t *pid, uint8_t *ver);

#endif /* OV7670_H */