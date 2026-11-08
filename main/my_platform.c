
#include <stdint.h>
#include <uni.h>

// Called during platform initialization
static void controller_init(int argc, const char** argv)
{
    (void)argc;
    (void)argv;

    logi("PINGPONG: Controller platform initialized\n");
}

// Called when Bluepad32 initialization is complete
static void controller_init_complete(void)
{
    logi("PINGPONG: Starting Bluetooth scanning\n");

    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
}

// Bluetooth connection established
static void controller_connected(uni_hid_device_t* d)
{
    logi("PINGPONG: Controller connected: %p\n", d);
}

// Bluetooth connection lost
static void controller_disconnected(uni_hid_device_t* d)
{
    logi("PINGPONG: Controller disconnected: %p\n", d);
}

// Controller is ready to send input data
static uni_error_t controller_ready(uni_hid_device_t* d)
{
    logi("PINGPONG: Controller ready: %p\n", d);
    return UNI_ERROR_SUCCESS;
}

// Process incoming controller data
static void controller_data(uni_hid_device_t* d,
                            uni_controller_t* ctl)
{
    (void)d;

    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
        return;

    const uni_gamepad_t* gp = &ctl->gamepad;

    // Limit diagnostic logging
    static uint32_t counter = 0;
    if (++counter % 10 != 0)
        return;

    logi("LX=%d LY=%d RX=%d RY=%d "
         "LT=%d RT=%d BUTTONS=0x%04x DPAD=0x%02x\n",
         gp->axis_x,
         gp->axis_y,
         gp->axis_rx,
         gp->axis_ry,
         gp->brake,
         gp->throttle,
         (unsigned int)gp->buttons,
         (unsigned int)gp->dpad);
}

static const uni_property_t* controller_property(uni_property_idx_t idx)
{
    (void)idx;
    return NULL;
}

static void controller_oob_event(uni_platform_oob_event_t event,
                                 void* data)
{
    (void)event;
    (void)data;
}

// Register callback functions
struct uni_platform* get_my_platform(void)
{
    static struct uni_platform platform = {
        .name = "pingpong-robot",
        .init = controller_init,
        .on_init_complete = controller_init_complete,
        .on_device_connected = controller_connected,
        .on_device_disconnected = controller_disconnected,
        .on_device_ready = controller_ready,
        .on_controller_data = controller_data,
        .on_oob_event = controller_oob_event,
        .get_property = controller_property,
    };

    return &platform;
}
