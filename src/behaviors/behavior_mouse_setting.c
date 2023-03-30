#define DT_DRV_COMPAT zmk_behavior_mouse_setting

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>

#include <dt-bindings/zmk/mouse_settings.h>
#include <zmk/mouse_ps2.h>
#include <zmk/mouse.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event)
{
    switch (binding->param1) {
        case MS_SAMPLING_RATE_INCR:
            return zmk_ps2_set_sampling_rate_incr();
        case MS_SAMPLING_RATE_DECR:
            return zmk_ps2_set_sampling_rate_decr();
    }

    return -ENOTSUP;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event)
{
    return ZMK_BEHAVIOR_OPAQUE;
}

// Initialization Function
static int zmk_behavior_mouse_setting_init(const struct device *dev) {
    return 0;
};

static const struct behavior_driver_api
zmk_behavior_mouse_setting_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released
};

DEVICE_DT_INST_DEFINE(
    0,
    zmk_behavior_mouse_setting_init, NULL,
    NULL, NULL,
    APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
    &zmk_behavior_mouse_setting_driver_api
);
// #endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
