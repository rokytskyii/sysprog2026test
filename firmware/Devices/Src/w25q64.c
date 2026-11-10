#include "w25q64.h"
#include <string.h>

#define CMD_WRITE_ENABLE 0x06
#define CMD_READ_STATUS_1 0x05
#define CMD_WRITE_STATUS_1 0x01
#define CMD_PAGE_PROGRAM 0x02
#define CMD_READ_DATA 0x03
#define CMD_SECTOR_ERASE 0x20
#define CMD_JEDEC_ID 0x9F
#define CMD_RELEASE_POWER_DOWN 0xAB

#define SR1_BUSY 0x01u
#define SR1_WEL 0x02u
#define SR1_BP_MASK 0x1Cu

static void w25q64_write_enable(w25q64_t *dev)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    dev->bus->cs_select();
    dev->bus->transfer(&cmd, NULL, 1);
    dev->bus->cs_deselect();
}

bool w25q64_read_status1(w25q64_t *dev, uint8_t *status)
{
    if (!dev || !dev->bus || !status)
        return false;

    /* Одна транзакція: 0x05 + dummy clock → статус у другому байті */
    uint8_t tx[2] = {CMD_READ_STATUS_1, 0xFF};
    uint8_t rx[2] = {0};
    dev->bus->cs_select();
    bool ok = dev->bus->transfer(tx, rx, 2);
    dev->bus->cs_deselect();
    if (!ok)
        return false;
    *status = rx[1];
    return true;
}

bool w25q64_is_busy(w25q64_t *dev)
{
    uint8_t status = 0xFF;
    if (!w25q64_read_status1(dev, &status))
        return true;
    return (status & SR1_BUSY) != 0u;
}

static void w25q64_wait_busy(w25q64_t *dev)
{
    while (w25q64_is_busy(dev))
    {
    }
}

static bool w25q64_clear_block_protect(w25q64_t *dev)
{
    uint8_t sr1 = 0;
    if (!w25q64_read_status1(dev, &sr1))
        return false;

    if ((sr1 & SR1_BP_MASK) == 0u)
        return true; /* вже знято */

    w25q64_write_enable(dev);
    uint8_t cmd[2] = {CMD_WRITE_STATUS_1, (uint8_t)(sr1 & (uint8_t)~SR1_BP_MASK)};
    /* також скидаємо SRP0 якщо був у sr1 bit7 — лишаємо інші службові біти обережно:
     * пишемо BP=0, решту зберігаємо крім BUSY/WEL */
    cmd[1] = (uint8_t)(sr1 & (uint8_t)~(SR1_BP_MASK | SR1_BUSY | SR1_WEL));

    dev->bus->cs_select();
    bool ok = dev->bus->transfer(cmd, NULL, 2);
    dev->bus->cs_deselect();
    if (!ok)
        return false;
    w25q64_wait_busy(dev);
    return true;
}

static void w25q64_release_power_down(w25q64_t *dev)
{
    uint8_t cmd = CMD_RELEASE_POWER_DOWN;
    dev->bus->cs_select();
    dev->bus->transfer(&cmd, NULL, 1);
    dev->bus->cs_deselect();
    /* tRES1 ~3 мкс — груба пауза */
    for (volatile uint32_t i = 0; i < 1000u; i++)
    {
    }
}

bool w25q64_init(w25q64_t *dev, const spi_interface_t *bus)
{
    if (!dev || !bus)
        return false;
    dev->bus = bus;

    w25q64_release_power_down(dev);

    uint8_t mfg = 0, type = 0, cap = 0;
    if (!w25q64_read_jedec_id(dev, &mfg, &type, &cap))
        return false;
    if (!(mfg == W25Q64_MANUFACTURER_WINBOND &&
          type == W25Q64_MEMORY_TYPE &&
          cap == W25Q64_CAPACITY_64MBIT))
        return false;

    (void)w25q64_clear_block_protect(dev);
    return true;
}

bool w25q64_read_jedec_id(w25q64_t *dev, uint8_t *mfg, uint8_t *type, uint8_t *cap)
{
    if (!dev || !dev->bus)
        return false;

    uint8_t tx[4] = {CMD_JEDEC_ID, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};

    dev->bus->cs_select();
    bool ok = dev->bus->transfer(tx, rx, 4);
    dev->bus->cs_deselect();

    if (ok)
    {
        if (mfg)
            *mfg = rx[1];
        if (type)
            *type = rx[2];
        if (cap)
            *cap = rx[3];
        return true;
    }
    return false;
}

bool w25q64_erase_sector_4k_start(w25q64_t *dev, uint32_t addr)
{
    if (!dev || !dev->bus)
        return false;

    w25q64_write_enable(dev);
    uint8_t sr = 0;
    if (!w25q64_read_status1(dev, &sr) || (sr & SR1_WEL) == 0u)
        return false;

    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)};

    dev->bus->cs_select();
    bool ok = dev->bus->transfer(cmd, NULL, 4);
    dev->bus->cs_deselect();
    return ok;
}

bool w25q64_erase_sector_4k(w25q64_t *dev, uint32_t addr)
{
    if (!w25q64_erase_sector_4k_start(dev, addr))
        return false;
    w25q64_wait_busy(dev);
    return true;
}

bool w25q64_page_program(w25q64_t *dev, uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (!dev || !dev->bus || !data || len == 0 || len > 256)
        return false;

    /* Не перетинати межу сторінки 256 B */
    if (((addr & 0xFFu) + len) > 256u)
        return false;

    w25q64_write_enable(dev);
    uint8_t sr = 0;
    if (!w25q64_read_status1(dev, &sr) || (sr & SR1_WEL) == 0u)
        return false;

    uint8_t header[4] = {
        CMD_PAGE_PROGRAM,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)};

    dev->bus->cs_select();
    bool ok = dev->bus->transfer(header, NULL, 4);
    if (ok)
        ok = dev->bus->transfer(data, NULL, len);
    dev->bus->cs_deselect();
    if (!ok)
        return false;

    w25q64_wait_busy(dev);
    return true;
}

bool w25q64_read_data(w25q64_t *dev, uint32_t addr, uint8_t *data, uint16_t len)
{
    if (!dev || !dev->bus || !data || len == 0)
        return false;

    uint8_t header[4] = {
        CMD_READ_DATA,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)};

    dev->bus->cs_select();
    bool ok = dev->bus->transfer(header, NULL, 4);
    if (ok)
        ok = dev->bus->transfer(NULL, data, len);
    dev->bus->cs_deselect();
    return ok;
}

bool w25q64_self_test(w25q64_t *dev, uint32_t test_addr)
{
    const uint8_t test_pattern[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t read_back[4] = {0};

    /* вирівняти на сектор */
    test_addr &= ~0xFFFu;

    if (!w25q64_erase_sector_4k(dev, test_addr))
        return false;
    if (!w25q64_page_program(dev, test_addr, test_pattern, 4))
        return false;
    if (!w25q64_read_data(dev, test_addr, read_back, 4))
        return false;

    return (memcmp(test_pattern, read_back, 4) == 0);
}
