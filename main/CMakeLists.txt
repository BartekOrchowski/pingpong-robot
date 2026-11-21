idf_component_register(
    SRCS
        "main.c"
        "my_platform.c"
        "dynamixel_protocol.c"
        "dynamixel_driver.c"
        "drive_control.c"
    INCLUDE_DIRS "."
    REQUIRES
        bluepad32
        btstack
        esp_driver_uart
        freertos
)