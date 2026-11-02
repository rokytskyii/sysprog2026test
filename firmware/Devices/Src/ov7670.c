#include "ov7670.h"

typedef struct
{
    uint8_t reg;
    uint8_t val;
} ov7670_reg_t;

/* Мінімальний набір для QQVGA YUV (DCW /4) */
static const ov7670_reg_t s_qqvga_yuv[] = {
    {0x12, 0x80}, /* COM7 soft reset — потім delay у configure */
};

static const ov7670_reg_t s_qqvga_yuv_after_reset[] = {
    {0x11, 0x01}, /* CLKRC */
    {0x3A, 0x04}, /* TSLB */
    {0x12, 0x00}, /* COM7: YUV */
    {0x8C, 0x00}, /* RGB444 off */
    {0x40, 0xC0}, /* COM15 */
    {0x0C, 0x0C}, /* COM3: DCW enable */
    {0x3E, 0x1A}, /* COM14: manual scaling, PCLK/4 */
    {0x70, 0x3A}, /* SCALING_XSC */
    {0x71, 0x35}, /* SCALING_YSC */
    {0x72, 0x22}, /* DCWCTR: /4 */
    {0x73, 0xF2}, /* PCLK_DIV */
    {0xA2, 0x02}, /* SCALING_PCLK_DELAY */
    {0x17, 0x16}, /* HSTART */
    {0x18, 0x04}, /* HSTOP */
    {0x19, 0x02}, /* VSTART */
    {0x1A, 0x7A}, /* VSTOP */
    {0x32, 0x80}, /* HREF */
    {0x1E, 0x00}, /* MVFP */
    {0x15, 0x00}, /* COM10 */
};

bool ov7670_init(ov7670_t *dev,
                 const sccb_interface_t *sccb,
                 const camera_parallel_interface_t *parallel,
                 uint8_t dev_addr,
                 void (*delay_ms)(uint32_t ms))
{
    if (!dev || !sccb || !parallel || !delay_ms)
        return false;
    dev->sccb = sccb;
    dev->parallel = parallel;
    dev->dev_addr = dev_addr;
    dev->delay_ms = delay_ms;
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

bool ov7670_configure_qqvga_yuv(ov7670_t *dev)
{
    if (!dev || !dev->sccb || !dev->delay_ms)
        return false;

    for (unsigned i = 0; i < sizeof(s_qqvga_yuv) / sizeof(s_qqvga_yuv[0]); i++)
    {
        if (!dev->sccb->write_reg(dev->dev_addr, s_qqvga_yuv[i].reg, s_qqvga_yuv[i].val))
            return false;
    }
    dev->delay_ms(50);

    for (unsigned i = 0; i < sizeof(s_qqvga_yuv_after_reset) / sizeof(s_qqvga_yuv_after_reset[0]); i++)
    {
        if (!dev->sccb->write_reg(dev->dev_addr, s_qqvga_yuv_after_reset[i].reg, s_qqvga_yuv_after_reset[i].val))
            return false;
        for (volatile int d = 0; d < 200; d++)
        {
        }
    }
    return true;
}

bool ov7670_capture_gray(ov7670_t *dev, uint8_t *dst, uint32_t timeout_ms)
{
    if (!dev || !dev->parallel || !dst)
        return false;
    return dev->parallel->capture_gray(dst, OV7670_QQVGA_W, OV7670_QQVGA_H, timeout_ms);
}
