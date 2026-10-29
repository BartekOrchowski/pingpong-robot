
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "PINGPONG_ROBOT";

void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Ping-Pong Robot - System Start");
    ESP_LOGI(TAG, "ESP-IDF and FreeRTOS initialized");
    ESP_LOGI(TAG, "================================");

    uint32_t counter = 0;

    while (1)
    {
        ESP_LOGI(TAG, "System running | Counter: %lu", (unsigned long)counter++);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
