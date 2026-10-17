#include "w25q64.h"
#include <string.h>

#define CMD_WRITE_ENABLE 0x06
#define CMD_READ_STATUS_1 0x05
#define CMD_PAGE_PROGRAM 0x02
#define CMD_READ_DATA 0x03
#define CMD_SECTOR_ERASE 0x20
#define CMD_JEDEC_ID 0x9F

static void w25q64_write_enable(w25q64_t *dev)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    dev->bus->cs_select();
    dev->bus->transfer(&cmd, NULL, 1);
    dev->bus->cs_deselect();
}

static void w25q64_wait_busy(w25q64_t *dev)
{
    uint8_t cmd = CMD_READ_STATUS_1;
    uint8_t status = 0;
    do
    {
        dev->bus->cs_select();
        dev->bus->transfer(&cmd, NULL, 1);
        dev->bus->transfer(NULL, &status, 1);
        dev->bus->cs_deselect();
    } while (status & 0x01); // Біт 0 (BUSY) активний під час запису/стирання
}

bool w25q64_init(w25q64_t *dev, const spi_interface_t *bus)
{
    if (!dev || !bus)
        return false;
    dev->bus = bus;

    uint8_t mfg = 0, type = 0, cap = 0;
    if (!w25q64_read_jedec_id(dev, &mfg, &type, &cap))
    {
        return false;
    }
    return (mfg == W25Q64_MANUFACTURER_WINBOND &&
            type == W25Q64_MEMORY_TYPE &&
            cap == W25Q64_CAPACITY_64MBIT);
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

bool w25q64_erase_sector_4k(w25q64_t *dev, uint32_t addr)
{
    if (!dev || !dev->bus)
        return false;

    w25q64_write_enable(dev);

    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)};

    dev->bus->cs_select();
    dev->bus->transfer(cmd, NULL, 4);
    dev->bus->cs_deselect();

    w25q64_wait_busy(dev);
    return true;
}

bool w25q64_page_program(w25q64_t *dev, uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (!dev || !dev->bus || !data || len == 0 || len > 256)
        return false;

    w25q64_write_enable(dev);

    uint8_t header[4] = {
        CMD_PAGE_PROGRAM,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)};

    dev->bus->cs_select();
    dev->bus->transfer(header, NULL, 4);
    dev->bus->transfer(data, NULL, len);
    dev->bus->cs_deselect();

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
    dev->bus->transfer(header, NULL, 4);
    dev->bus->transfer(NULL, data, len);
    dev->bus->cs_deselect();

    return true;
}

bool w25q64_self_test(w25q64_t *dev, uint32_t test_addr)
{
    const uint8_t test_pattern[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t read_back[4] = {0};

    if (!w25q64_erase_sector_4k(dev, test_addr))
        return false;
    if (!w25q64_page_program(dev, test_addr, test_pattern, 4))
        return false;
    if (!w25q64_read_data(dev, test_addr, read_back, 4))
        return false;

    return (memcmp(test_pattern, read_back, 4) == 0);
}