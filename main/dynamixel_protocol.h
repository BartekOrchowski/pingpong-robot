
#pragma once

#include <stddef.h>
#include <stdint.h>

#define DXL_INST_PING   0x01
#define DXL_INST_READ   0x02
#define DXL_INST_WRITE  0x03

size_t dxl_build_packet(uint8_t id,
                        uint8_t instruction,
                        const uint8_t *params,
                        size_t param_count,
                        uint8_t *out,
                        size_t capacity);
