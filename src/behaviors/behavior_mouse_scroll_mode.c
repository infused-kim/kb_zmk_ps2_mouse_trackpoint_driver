#define DT_DRV_COMPAT zmk_behavior_mouse_scroll_mode

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/mouse_ps2.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    zmk_mouse_ps2_scroll_mode_enable();

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    zmk_mouse_ps2_scroll_mode_disable();

    return ZMK_BEHAVIOR_OPAQUE;
}

// Initialization Function
static int zmk_behavior_mouse_scroll_mode_init(const struct device *dev) { return 0; };

static const struct behavior_driver_api zmk_behavior_mouse_scroll_mode_driver_api = {
    .binding_pressed = on_keymap_binding_pressed, .binding_released = on_keymap_binding_released};

BEHAVIOR_DT_INST_DEFINE(0, zmk_behavior_mouse_scroll_mode_init, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &zmk_behavior_mouse_scroll_mode_driver_api);
