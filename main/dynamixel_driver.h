#ifndef DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

void dxl_uart_init(void);

bool dxl_read(uint8_t id,
              uint16_t address,
              uint16_t size,
              uint32_t *value);

bool dxl_write(uint8_t id,
               uint16_t address,
               uint16_t size,
               uint32_t value);

#endif