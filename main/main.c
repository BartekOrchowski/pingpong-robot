
#include <stddef.h>
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

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
