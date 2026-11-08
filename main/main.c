
#include <stddef.h>
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>
#include "dynamixel_protocol.h"

#include "sdkconfig.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Bluepad32 custom platform must be enabled"
#endif

struct uni_platform* get_my_platform(void);

int app_main(void)
{
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

    uint8_t ping[16];

    size_t len = dxl_build_packet(
        1, DXL_INST_PING, NULL, 0, ping, sizeof(ping)
    );

    if (len == 10 && ping[8] == 0x19 && ping[9] == 0x4E)
    {
        logi("DYNAMIXEL: PING packet CRC OK\n");
    }
    else
    {
        logi("DYNAMIXEL: Packet generation ERROR\n");
    }
    // Initialize Bluetooth stack
    btstack_init();

    // Register our custom controller platform
    uni_platform_set_custom(get_my_platform());

    // Initialize Bluepad32
    uni_init(0, NULL);

    // Start processing Bluetooth events
    btstack_run_loop_execute();

    return 0;
}
