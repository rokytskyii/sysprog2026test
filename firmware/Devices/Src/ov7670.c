#include "ov7670.h"

bool ov7670_init(ov7670_t *dev, const sccb_interface_t *sccb, uint8_t dev_addr)
{
    if (!dev || !sccb)
        return false;
    dev->sccb = sccb;
    dev->dev_addr = dev_addr;
    return true;
}

bool ov7670_read_id(ov7670_t *dev, uint8_t *pid, uint8_t *ver)
{
    if (!dev || !dev->sccb || !pid || !ver)
        return false;
    if (!dev->sccb->read_reg(dev->dev_addr, OV7670_REG_PID, pid))
        return false;
    if (!dev->sccb->read_reg(dev->dev_addr, OV7670_REG_VER, ver))
        return false;
    return true;
}