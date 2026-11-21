#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

#include "dynamixel_driver.h"

#define DXL_UART        UART_NUM_2
#define DXL_TX_PIN      17
#define DXL_RX_PIN      16
#define DXL_BAUDRATE    57600
#define RX_BUFFER_SIZE  1024

static const char *TAG = "DXL_DRIVER";



// CRC-16 used by DYNAMIXEL Protocol 2.0
static uint16_t dxl_crc(const uint8_t *data, size_t length)
{
    uint16_t crc = 0;

    for (size_t i = 0; i < length; i++) {

        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++) {

            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void dxl_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = DXL_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_param_config(
        DXL_UART,
        &uart_config
    ));

    ESP_ERROR_CHECK(uart_set_pin(
        DXL_UART,
        DXL_TX_PIN,
        DXL_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_ERROR_CHECK(uart_driver_install(
        DXL_UART,
        RX_BUFFER_SIZE,
        0,
        0,
        NULL,
        0
    ));

    ESP_LOGI(TAG, "UART initialized");
}

bool dxl_read(uint8_t id,
                     uint16_t address,
                     uint16_t size,
                     uint32_t *value)
{
    // This version reads registers containing 1-4 bytes
    if (size == 0 || size > 4 || value == NULL) {
        return false;
    }

    uint8_t packet[14] = {
        0xFF, 0xFF, 0xFD, 0x00,
        id,
        0x07, 0x00,
        0x02,

        (uint8_t)(address & 0xFF),
        (uint8_t)(address >> 8),

        (uint8_t)(size & 0xFF),
        (uint8_t)(size >> 8),

        0x00, 0x00
    };

    // Calculate CRC
    uint16_t crc = dxl_crc(packet, 12);

    packet[12] = crc & 0xFF;
    packet[13] = crc >> 8;

    // Clear previous data and send request
    uart_flush_input(DXL_UART);

    if (uart_write_bytes(DXL_UART,
                         packet,
                         sizeof(packet)) != sizeof(packet)) {
        return false;
    }

    if (uart_wait_tx_done(DXL_UART,
                         pdMS_TO_TICKS(100)) != ESP_OK) {
        return false;
    }

    // Receive echo and status packet
    uint8_t rx_data[128];

    int length = uart_read_bytes(
        DXL_UART,
        rx_data,
        sizeof(rx_data),
        pdMS_TO_TICKS(100)
    );

    // Search for the status packet
    for (int i = 0; i + 11 <= length; i++) {

        const uint8_t *p = &rx_data[i];

        if (p[0] != 0xFF ||
            p[1] != 0xFF ||
            p[2] != 0xFD ||
            p[3] != 0x00 ||
            p[4] != id ||
            p[7] != 0x55) {

            continue;
        }

        uint16_t packet_length =
            p[5] | ((uint16_t)p[6] << 8);

        int total = packet_length + 7;

        if (packet_length < 4 ||
            i + total > length) {
            continue;
        }

        // Validate CRC before removing byte stuffing
        uint16_t received_crc =
            p[total - 2] |
            ((uint16_t)p[total - 1] << 8);

        if (dxl_crc(p, total - 2) != received_crc) {
            continue;
        }

        // Remove byte stuffing
        uint8_t body[128];
        int body_length = 0;

        for (int j = 7; j < total - 2; j++) {

            body[body_length++] = p[j];

            if (j >= 9 &&
                j + 1 < total - 2 &&
                p[j - 2] == 0xFF &&
                p[j - 1] == 0xFF &&
                p[j] == 0xFD &&
                p[j + 1] == 0xFD) {

                j++;
            }
        }

        uint8_t error = body[1];

        // Ignore Alert for diagnostic reads,
        // but reject instruction errors
        if (error & 0x7F) {

            ESP_LOGW(TAG,
                     "Instruction error: 0x%02X",
                     (unsigned)error);

            return false;
        }

        if (body_length != size + 2) {
            continue;
        }

        // Convert Little Endian bytes to a value
        *value = 0;

        for (int j = 0; j < size; j++) {

            *value |=
                (uint32_t)body[2 + j] << (8 * j);
        }

        return true;
    }

    ESP_LOGW(TAG,
             "Read failed, address: %u",
             (unsigned)address);

    return false;
}

bool dxl_write(uint8_t id,
                      uint16_t address,
                      uint16_t size,
                      uint32_t value)
{
    if (size == 0 || size > 4) {
        return false;
    }

    // Instruction and parameters
    uint8_t body[7] = {
        0x03,   // WRITE instruction
        address & 0xFF,
        address >> 8
    };

    // Convert value to Little Endian
    for (int i = 0; i < size; i++) {
        body[3 + i] = (value >> (8 * i)) & 0xFF;
    }

    uint8_t packet[32] = {
        0xFF, 0xFF, 0xFD, 0x00,
        id,
        0x00, 0x00
    };

    int index = 7;

    // Copy instruction and parameters
    // with byte stuffing
    for (int i = 0; i < size + 3; i++) {

        packet[index++] = body[i];

        if (i >= 2 &&
            body[i - 2] == 0xFF &&
            body[i - 1] == 0xFF &&
            body[i] == 0xFD) {

            packet[index++] = 0xFD;
        }
    }

    // Packet length
    uint16_t packet_length = index - 7 + 2;

    packet[5] = packet_length & 0xFF;
    packet[6] = packet_length >> 8;

    // Calculate CRC
    uint16_t crc = dxl_crc(packet, index);

    packet[index++] = crc & 0xFF;
    packet[index++] = crc >> 8;

    // Send packet
    uart_flush_input(DXL_UART);

    if (uart_write_bytes(DXL_UART,
                         packet,
                         index) != index) {
        return false;
    }

    if (uart_wait_tx_done(DXL_UART,
                         pdMS_TO_TICKS(100)) != ESP_OK) {
        return false;
    }

    // Receive echo and status packet
    uint8_t rx_data[128];

    int length = uart_read_bytes(
        DXL_UART,
        rx_data,
        sizeof(rx_data),
        pdMS_TO_TICKS(100)
    );

    // Search for WRITE status packet
    for (int i = 0; i + 11 <= length; i++) {

        const uint8_t *p = &rx_data[i];

        if (p[0] != 0xFF ||
            p[1] != 0xFF ||
            p[2] != 0xFD ||
            p[3] != 0x00 ||
            p[4] != id ||
            p[5] != 0x04 ||
            p[6] != 0x00 ||
            p[7] != 0x55) {

            continue;
        }

        uint16_t received_crc =
            p[9] | ((uint16_t)p[10] << 8);

        if (dxl_crc(p, 9) != received_crc) {
            continue;
        }

        uint8_t error = p[8];

        if (error & 0x7F) {

            ESP_LOGE(TAG,
                     "WRITE instruction error: 0x%02X",
                     (unsigned)error);

            return false;
        }

        if (error & 0x80) {

            ESP_LOGW(TAG,
                     "Hardware alert active");
        }

        ESP_LOGI(TAG,
                 "WRITE accepted: address=%u, value=%lu",
                 (unsigned)address,
                 (unsigned long)value);

        return true;
    }

    ESP_LOGE(TAG,
             "WRITE response not received, address=%u",
             (unsigned)address);

    return false;
}

