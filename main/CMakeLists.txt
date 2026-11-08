idf_component_register(
    SRCS
        "main.c"
        "my_platform.c"
        "dynamixel_protocol.c"
    INCLUDE_DIRS "."
    REQUIRES bluepad32 btstack
)