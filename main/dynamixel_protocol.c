
#include "dynamixel_protocol.h"

// CRC-16 for DYNAMIXEL Protocol 2.0
static uint16_t dxl_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x8005;
            else
                crc = crc << 1;
        }
    }

    return crc;
}

// Build a DYNAMIXEL Protocol 2.0 instruction packet
size_t dxl_build_packet(uint8_t id,
                        uint8_t instruction,
                        const uint8_t *params,
                        size_t param_count,
                        uint8_t *out,
                        size_t capacity)
{
    if (!out || id > 0xFE || capacity < 10 ||
        param_count > 128 || (param_count && !params))
        return 0;

    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = 0xFD;
    out[3] = 0x00;
    out[4] = id;
    out[5] = 0;
    out[6] = 0;
    out[7] = instruction;

    size_t n = 8;

    // Add parameters and apply byte stuffing
    for (size_t i = 0; i < param_count; i++)
    {
        if (n + 3 > capacity)
            return 0;

        out[n++] = params[i];

        if (n >= 11 &&
            out[n - 3] == 0xFF &&
            out[n - 2] == 0xFF &&
            out[n - 1] == 0xFD)
        {
            if (n + 3 > capacity)
                return 0;

            out[n++] = 0xFD;
        }
    }

    uint16_t length = (uint16_t)(n - 7 + 2);
    out[5] = length & 0xFF;
    out[6] = (length >> 8) & 0xFF;

    uint16_t crc = dxl_crc16(out, n);

    out[n++] = crc & 0xFF;
    out[n++] = (crc >> 8) & 0xFF;

    return n;
}
