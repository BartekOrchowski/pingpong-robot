#include "drive_control.h"
#include "dynamixel_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

// Motor configuration
#define DXL_ID                  1

// Dynamixel registers
#define REG_OPERATING_MODE      11
#define REG_TORQUE_ENABLE       64
#define REG_GOAL_VELOCITY       104

// Controller configuration
#define AXIS_MAX                512
#define AXIS_DEADZONE           40
#define MAX_GOAL_VELOCITY       50

// Task configuration
#define DRIVE_PERIOD_MS         50
#define CONTROLLER_TIMEOUT_MS   500

static const char *TAG = "DRIVE_CONTROL";

typedef struct {

    int32_t axis_y;
    TickType_t timestamp;
    bool connected;

} drive_command_t;

static QueueHandle_t drive_queue = NULL;


// Convert joystick position to goal velocity
static int32_t calculate_velocity(int32_t axis_y)
{
    // Limit input range
    if (axis_y > AXIS_MAX) {
        axis_y = AXIS_MAX;
    }

    if (axis_y < -AXIS_MAX) {
        axis_y = -AXIS_MAX;
    }

    // Joystick deadzone
    if (axis_y >= -AXIS_DEADZONE &&
        axis_y <= AXIS_DEADZONE) {

        return 0;
    }

    // Determine rotation direction
    int32_t magnitude;
    int32_t direction;

    if (axis_y < 0) {

        magnitude = -axis_y;
        direction = 1;

    } else {

        magnitude = axis_y;
        direction = -1;
    }

    // Remove deadzone and scale velocity
    magnitude -= AXIS_DEADZONE;

    int32_t velocity =
        magnitude * MAX_GOAL_VELOCITY /
        (AXIS_MAX - AXIS_DEADZONE);

    return direction * velocity;
}


// Receive new controller data
void drive_control_submit(int32_t axis_y)
{
    if (drive_queue == NULL) {
        return;
    }

    drive_command_t command = {

        .axis_y = axis_y,
        .timestamp = xTaskGetTickCount(),
        .connected = true
    };

    xQueueOverwrite(drive_queue, &command);
}


// Stop request after controller disconnection
void drive_control_stop(void)
{
    if (drive_queue == NULL) {
        return;
    }

    drive_command_t command = {

        .axis_y = 0,
        .timestamp = xTaskGetTickCount(),
        .connected = false
    };

    xQueueOverwrite(drive_queue, &command);
}


// Initialize motor configuration
static bool configure_motor(void)
{
    uint32_t value;

    // Keep torque disabled during integration test
    if (!dxl_write(DXL_ID, REG_TORQUE_ENABLE, 1, 0)) {
        return false;
    }

    // Verify torque state
    if (!dxl_read(DXL_ID, REG_TORQUE_ENABLE, 1, &value) ||
        value != 0) {

        return false;
    }

    // Read current operating mode
    if (!dxl_read(DXL_ID, REG_OPERATING_MODE, 1, &value)) {
        return false;
    }

    // Configure velocity control mode
    if (value != 1) {

        if (!dxl_write(DXL_ID, REG_OPERATING_MODE, 1, 1)) {
            return false;
        }
    }

    // Verify operating mode
    if (!dxl_read(DXL_ID, REG_OPERATING_MODE, 1, &value) ||
        value != 1) {

        return false;
    }

    // Set initial velocity to zero
    if (!dxl_write(DXL_ID, REG_GOAL_VELOCITY, 4, 0)) {
        return false;
    }

    if (!dxl_read(DXL_ID, REG_GOAL_VELOCITY, 4, &value) ||
        value != 0) {

        return false;
    }

    // Additional motor diagnostics
    if (dxl_read(DXL_ID, 44, 4, &value)) {
        ESP_LOGI(TAG, "Velocity Limit: %lu",
                (unsigned long)value);
    }

    if (dxl_read(DXL_ID, 36, 2, &value)) {
        ESP_LOGI(TAG, "PWM Limit: %lu",
                (unsigned long)value);
    }

    if (dxl_read(DXL_ID, 63, 1, &value)) {
        ESP_LOGI(TAG, "Shutdown: 0x%02lX",
                (unsigned long)value);
    }

    if (dxl_read(DXL_ID, 98, 1, &value)) {
        ESP_LOGI(TAG, "Bus Watchdog: %ld",
                (long)(int8_t)value);
    }

    if (dxl_read(DXL_ID, 70, 1, &value)) {
        ESP_LOGI(TAG, "Hardware Error: 0x%02lX",
                (unsigned long)value);
    }

    // Test fixed velocity without controller input
    bool test_write = dxl_write(
        DXL_ID, REG_GOAL_VELOCITY, 4, 32
    );

    value = 0;

    bool test_read = dxl_read(
        DXL_ID, REG_GOAL_VELOCITY, 4, &value
    );

    ESP_LOGI(TAG,
            "FIXED TEST: WRITE=%d READ=%d TARGET=32 READBACK=%ld",
            (int)test_write,
            (int)test_read,
            (long)(int32_t)value);

    // Restore zero velocity
    if (!dxl_write(DXL_ID, REG_GOAL_VELOCITY, 4, 0)) {
        return false;
    }

    if (!dxl_read(DXL_ID, REG_GOAL_VELOCITY, 4, &value) ||
        value != 0) {
        return false;
    }

    ESP_LOGI(TAG, "Motor configured");
    ESP_LOGI(TAG, "Velocity Control Mode active");
    ESP_LOGI(TAG, "Torque disabled - integration test");

    return true;
}


// FreeRTOS drive task
static void drive_task(void *arg)
{
    (void)arg;

    if (!configure_motor()) {

        ESP_LOGE(TAG, "Motor configuration failed");

        dxl_write(DXL_ID, REG_TORQUE_ENABLE, 1, 0);

        vTaskDelete(NULL);
        return;
    }

    drive_command_t command;

    int32_t last_velocity = 0;

    while (1) {

        int32_t target_velocity = 0;
        int32_t axis_y = 0;

        // Read latest controller command
        if (xQueuePeek(drive_queue, &command, 0) == pdTRUE) {

            TickType_t elapsed =
                xTaskGetTickCount() - command.timestamp;

            // Controller connection and timeout
            if (command.connected &&
                elapsed < pdMS_TO_TICKS(CONTROLLER_TIMEOUT_MS)) {

                axis_y = command.axis_y;

                target_velocity =
                    calculate_velocity(axis_y);
            }
        }

        // Send command only if velocity has changed
        if (target_velocity != last_velocity) {

            uint32_t readback = 0;

            bool write_ok = dxl_write(
                DXL_ID,
                REG_GOAL_VELOCITY,
                4,
                (uint32_t)target_velocity
            );

            bool read_ok = write_ok && dxl_read(
                DXL_ID,
                REG_GOAL_VELOCITY,
                4,
                &readback
            );

            ESP_LOGI(TAG,
            "DEBUG: WRITE=%d READ=%d TARGET=%ld READBACK=%ld",
            (int)write_ok,
            (int)read_ok,
            (long)target_velocity,
            (long)(int32_t)readback);

            if (read_ok &&
                readback == (uint32_t)target_velocity) {

                last_velocity = target_velocity;

                ESP_LOGI(
                    TAG,
                    "LY=%ld | Goal=%ld | Readback=%ld | RPM=%.2f",
                    (long)axis_y,
                    (long)target_velocity,
                    (long)(int32_t)readback,
                    target_velocity * 0.229
                );

            } else {

                ESP_LOGE(TAG, "Drive communication failed");

                dxl_write(DXL_ID, REG_GOAL_VELOCITY, 4, 0);
                dxl_write(DXL_ID, REG_TORQUE_ENABLE, 1, 0);

                vTaskDelete(NULL);
                return;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DRIVE_PERIOD_MS));
    }
}


// Initialize drive control module
bool drive_control_start(void)
{
    drive_queue = xQueueCreate(
        1,
        sizeof(drive_command_t)
    );

    if (drive_queue == NULL) {

        ESP_LOGE(TAG, "Queue creation failed");
        return false;
    }

    dxl_uart_init();

    if (xTaskCreate(
            drive_task,
            "drive_task",
            4096,
            NULL,
            5,
            NULL
        ) != pdPASS) {

        ESP_LOGE(TAG, "Drive task creation failed");

        vQueueDelete(drive_queue);
        drive_queue = NULL;

        return false;
    }

    ESP_LOGI(TAG, "Drive control started");

    return true;
}