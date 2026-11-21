
// #include <stddef.h>
// #include <btstack_port_esp32.h>
// #include <btstack_run_loop.h>
// #include <btstack_stdio_esp32.h>
// #include <uni.h>
// #include "dynamixel_protocol.h"

// #include "sdkconfig.h"

// #ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
// #error "Bluepad32 custom platform must be enabled"
// #endif

// struct uni_platform* get_my_platform(void);

// int app_main(void)
// {
// #ifdef CONFIG_ESP_CONSOLE_UART
// #ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
//     btstack_stdio_init();
// #endif
// #endif

//     uint8_t ping[16];

//     size_t len = dxl_build_packet(
//         1, DXL_INST_PING, NULL, 0, ping, sizeof(ping)
//     );

//     if (len == 10 && ping[8] == 0x19 && ping[9] == 0x4E)
//     {
//         logi("DYNAMIXEL: PING packet CRC OK\n");
//     }
//     else
//     {
//         logi("DYNAMIXEL: Packet generation ERROR\n");
//     }
//     // Initialize Bluetooth stack
//     btstack_init();

//     // Register our custom controller platform
//     uni_platform_set_custom(get_my_platform());

//     // Initialize Bluepad32
//     uni_init(0, NULL);

//     // Start processing Bluetooth events
//     btstack_run_loop_execute();

//     return 0;
// }


#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

#define DXL_UART        UART_NUM_2
#define DXL_TX_PIN      17
#define DXL_RX_PIN      16
#define DXL_BAUDRATE    57600
#define RX_BUFFER_SIZE  1024

static const char *TAG = "DXL_TEST";

// PING packet - DYNAMIXEL Protocol 2.0, ID = 1
static const uint8_t ping_packet[] = {
    0xFF, 0xFF, 0xFD, 0x00,
    0x01,
    0x03, 0x00,
    0x01,
    0x19, 0x4E
};

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

static void dxl_uart_init(void)
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

// Search for a valid PING response
static void dxl_check_response(
    const uint8_t *data,
    int length
)
{
    for (int i = 0; i + 14 <= length; i++) {

        const uint8_t *p = &data[i];

        if (p[0] != 0xFF ||
            p[1] != 0xFF ||
            p[2] != 0xFD ||
            p[3] != 0x00 ||
            p[4] != 0x01 ||
            p[5] != 0x07 ||
            p[6] != 0x00 ||
            p[7] != 0x55) {

            continue;
        }

        uint16_t received_crc =
            p[12] | ((uint16_t)p[13] << 8);

        if (dxl_crc(p, 12) != received_crc) {
            ESP_LOGW(TAG, "Invalid CRC");
            continue;
        }

        uint16_t model =
            p[9] | ((uint16_t)p[10] << 8);

        ESP_LOGI(TAG, "DYNAMIXEL RESPONSE OK");
        ESP_LOGI(TAG, "ID: %u", p[4]);
        ESP_LOGI(TAG, "Error: 0x%02X", p[8]);
        ESP_LOGI(TAG, "Model number: %u", model);
        ESP_LOGI(TAG, "Firmware version: %u", p[11]);

        return;
    }

    ESP_LOGW(TAG, "No valid status packet");
}

void app_main(void)
{
    uint8_t rx_data[128];

    dxl_uart_init();

    ESP_LOGI(TAG, "Starting DYNAMIXEL PING test");

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {

        uart_flush_input(DXL_UART);

        ESP_LOGI(TAG, "Sending PING");

        ESP_LOG_BUFFER_HEX(
            TAG,
            ping_packet,
            sizeof(ping_packet)
        );

        uart_write_bytes(
            DXL_UART,
            ping_packet,
            sizeof(ping_packet)
        );

        uart_wait_tx_done(
            DXL_UART,
            pdMS_TO_TICKS(100)
        );

        int length = uart_read_bytes(
            DXL_UART,
            rx_data,
            sizeof(rx_data),
            pdMS_TO_TICKS(100)
        );

        if (length > 0) {

            ESP_LOGI(TAG, "Received %d bytes", length);

            ESP_LOG_BUFFER_HEX(
                TAG,
                rx_data,
                length
            );

            dxl_check_response(rx_data, length);

        } else {

            ESP_LOGW(TAG, "No response");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
