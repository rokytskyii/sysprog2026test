#ifndef W25Q64_H
#define W25Q64_H

#include "bus_interfaces.h"

#define W25Q64_MANUFACTURER_WINBOND 0xEF
#define W25Q64_MEMORY_TYPE 0x40
#define W25Q64_CAPACITY_64MBIT 0x17

typedef struct
{
    const spi_interface_t *bus;
} w25q64_t;

bool w25q64_init(w25q64_t *dev, const spi_interface_t *bus);
bool w25q64_read_jedec_id(w25q64_t *dev, uint8_t *manufacturer, uint8_t *mem_type, uint8_t *capacity);
bool w25q64_erase_sector_4k(w25q64_t *dev, uint32_t addr);
bool w25q64_page_program(w25q64_t *dev, uint32_t addr, const uint8_t *data, uint16_t len);
bool w25q64_read_data(w25q64_t *dev, uint32_t addr, uint8_t *data, uint16_t len);
bool w25q64_self_test(w25q64_t *dev, uint32_t test_addr);

#endif /* W25Q64_H */