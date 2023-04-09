/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_ps2_mouse

#include <device.h>
#include <devicetree.h>
#include <drivers/ps2.h>
#include <sys/util.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <dt-bindings/zmk/mouse.h>
#include <zmk/events/mouse_move_state_changed.h>
#include <zmk/behavior_queue.h>
#include <dt-bindings/zmk/mouse.h>
#include <dt-bindings/zmk/keys.h>
#include <zmk/events/mouse_tick.h>
#include <drivers/gpio.h>
#include <stdlib.h>

// #if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Settings
 */

#define PS2_MOUSE_INIT_ATTEMPTS 10
#define PS2_MOUSE_TIMEOUT_CMD_BUFFER K_MSEC(500)

/*
 * PS/2 Defines
 */

// According to the `IBM TrackPoint System Version 4.0 Engineering
// Specification`...
// "The POR shall be timed to occur 600 ms ± 20 % from the time power is
//  applied to the TrackPoint controller."
#define PS2_MOUSE_POWER_ON_RESET_TIME K_MSEC(600)

// Common PS/2 Mouse commands
#define PS2_MOUSE_CMD_GET_SECONDARY_ID "\xe1"
#define PS2_MOUSE_CMD_GET_SECONDARY_ID_RESP_LEN 2

#define PS2_MOUSE_CMD_GET_DEVICE_ID "\xf2"
#define PS2_MOUSE_CMD_GET_DEVICE_ID_RESP_LEN 1

#define PS2_MOUSE_CMD_SET_SAMPLING_RATE "\xf3"
#define PS2_MOUSE_CMD_SET_SAMPLING_RATE_RESP_LEN 0
#define PS2_MOUSE_CMD_SET_SAMPLING_RATE_DEFAULT 100

#define PS2_MOUSE_CMD_ENABLE_REPORTING "\xf4"
#define PS2_MOUSE_CMD_ENABLE_REPORTING_RESP_LEN 0

#define PS2_MOUSE_CMD_DISABLE_REPORTING "\xf5"
#define PS2_MOUSE_CMD_DISABLE_REPORTING_RESP_LEN 0

#define PS2_MOUSE_CMD_RESEND "\xfe"
#define PS2_MOUSE_CMD_RESEND_RESP_LEN 0

#define PS2_MOUSE_CMD_RESET "\xff"
#define PS2_MOUSE_CMD_RESET_RESP_LEN 0

// Trackpoint Commands
// They can be found in the `IBM TrackPoint System Version 4.0 Engineering
// Specification` (YKT3Eext.pdf)...

#define PS2_MOUSE_CMD_TP_GET_CONFIG_BYTE "\xe2\x80\x2c"
#define PS2_MOUSE_CMD_TP_GET_CONFIG_BYTE_RESP_LEN 1

#define PS2_MOUSE_CMD_TP_SET_CONFIG_BYTE "\xe2\x81\x2c"
#define PS2_MOUSE_CMD_TP_SET_CONFIG_BYTE_RESP_LEN 0

#define PS2_MOUSE_CMD_TP_GET_SENSITIVITY "\xe2\x80\x4a"
#define PS2_MOUSE_CMD_TP_GET_SENSITIVITY_RESP_LEN 1

#define PS2_MOUSE_CMD_TP_SET_SENSITIVITY "\xe2\x81\x4a"
#define PS2_MOUSE_CMD_TP_SET_SENSITIVITY_RESP_LEN 0
#define PS2_MOUSE_CMD_TP_SET_SENSITIVITY_MIN 0.0
#define PS2_MOUSE_CMD_TP_SET_SENSITIVITY_MAX 1.999
#define PS2_MOUSE_CMD_TP_SET_SENSITIVITY_DEFAULT 1.0

#define PS2_MOUSE_CMD_TP_GET_NEG_INERTIA "\xe2\x80\x4d"
#define PS2_MOUSE_CMD_TP_GET_NEG_INERTIA_RESP_LEN 1

#define PS2_MOUSE_CMD_TP_SET_NEG_INERTIA "\xe2\x81\x4d"
#define PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_RESP_LEN 0
#define PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_MIN 0
#define PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_MAX 255
#define PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_DEFAULT 0x06

#define PS2_MOUSE_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED "\xe2\x80\x4d"
#define PS2_MOUSE_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN 1

#define PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED "\xe2\x81\x4d"
#define PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN 0
#define PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MIN 0
#define PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MAX 255
#define PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_DEFAULT 0x61

#define PS2_MOUSE_CMD_TP_GET_PTS_THRESHOLD "\xe2\x80\x5c"
#define PS2_MOUSE_CMD_TP_GET_PTS_THRESHOLD_RESP_LEN 1

#define PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD "\xe2\x81\x5c"
#define PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_RESP_LEN 0
#define PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_MIN 0
#define PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_MAX 255
#define PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_DEFAULT 0x08

// Trackpoint Config Bits
#define PS2_MOUSE_TP_CONFIG_BIT_PRESS_TO_SELECT 0x00
#define PS2_MOUSE_TP_CONFIG_BIT_RESERVED 0x01
#define PS2_MOUSE_TP_CONFIG_BIT_BUTTON2 0x02
#define PS2_MOUSE_TP_CONFIG_BIT_INVERT_X 0x03
#define PS2_MOUSE_TP_CONFIG_BIT_INVERT_Y 0x04
#define PS2_MOUSE_TP_CONFIG_BIT_INVERT_Z 0x05
#define PS2_MOUSE_TP_CONFIG_BIT_SWAP_XY 0x06
#define PS2_MOUSE_TP_CONFIG_BIT_FORCE_TRANSPARENT 0x07

// Responses
#define PS2_MOUSE_RESP_SELF_TEST_PASS 0xaa
#define PS2_MOUSE_RESP_SELF_TEST_FAIL 0xfc

/*
 * ZMK Defines
 */

#define PS2_MOUSE_BUTTON_L_IDX 0
#define PS2_MOUSE_BUTTON_R_IDX 1
#define PS2_MOUSE_BUTTON_M_IDX 3

#define PS2_MOUSE_THREAD_STACK_SIZE 1024
#define PS2_MOUSE_THREAD_PRIORITY 10

/*
 * Global Variables
 */

typedef enum
{
    PS2_MOUSE_PACKET_MODE_PS2_DEFAULT,
    PS2_MOUSE_PACKET_MODE_SCROLL,
} zmk_ps2_mouse_packet_mode;

struct zmk_ps2_mouse_config {
	const struct device *ps2_device;
	const char *rst_gpio_name;
	gpio_pin_t rst_pin;
};

struct zmk_ps2_mouse_packet {
    int16_t mov_x;
    int16_t mov_y;
    int8_t scroll;
    bool overflow_x;
    bool overflow_y;
    bool button_l;
    bool button_m;
    bool button_r;
};

struct zmk_ps2_mouse_data {
	const struct device *rst_gpio;	/* GPIO used for Power-On-Reset line */

    K_THREAD_STACK_MEMBER(thread_stack, PS2_MOUSE_THREAD_STACK_SIZE);
    struct k_thread thread;

    zmk_ps2_mouse_packet_mode packet_mode;
    uint8_t cmd_buffer[4];
    int cmd_idx;
    struct zmk_ps2_mouse_packet prev_packet;
    struct k_work_delayable cmd_buffer_timeout;

    // Stores the x and y coordinates between reporting to the os
    struct vector2d move_speed;
    struct vector2d scroll_speed;
    struct k_timer mouse_timer;
    struct k_work mouse_tick;

    bool button_l_is_held;
    bool button_m_is_held;
    bool button_r_is_held;

    bool activity_reporting_on;

    uint8_t sampling_rate;
    float tp_sensitivity;
    uint8_t tp_neg_inertia;
    uint8_t tp_value6;
    uint8_t tp_pts_threshold;
};


static const struct zmk_ps2_mouse_config zmk_ps2_mouse_config = {
    .ps2_device = DEVICE_DT_GET(DT_INST_PHANDLE(0, ps2_device)),

#if DT_INST_NODE_HAS_PROP(0, rst_gpios)
    .rst_gpio_name = DT_INST_GPIO_LABEL(0, rst_gpios),
    .rst_pin = DT_INST_GPIO_PIN(0, rst_gpios),
#else
    .rst_gpio_name = NULL,
    .rst_pin = 0,
#endif

};

static struct zmk_ps2_mouse_data zmk_ps2_mouse_data = {
    .rst_gpio = NULL,
    .packet_mode = PS2_MOUSE_PACKET_MODE_PS2_DEFAULT,
    .cmd_idx = 0,
    .prev_packet = {
        .button_l = false,
        .button_r = false,
        .button_m = false,
        .overflow_x = 0,
        .overflow_y = 0,
        .mov_x = 0,
        .mov_y = 0,
        .scroll = 0,
    },

    .button_l_is_held = false,
    .button_m_is_held = false,
    .button_r_is_held = false,

    .move_speed = {0},
    .scroll_speed = {0},

    // Data reporting is disabled on init
    .activity_reporting_on = false,

    // PS2 devices initialize with this rate
    .sampling_rate = PS2_MOUSE_CMD_SET_SAMPLING_RATE_DEFAULT,
    .tp_sensitivity = PS2_MOUSE_CMD_TP_SET_SENSITIVITY_DEFAULT,
    .tp_neg_inertia = PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_DEFAULT,
    .tp_value6 = PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_DEFAULT,
    .tp_pts_threshold = PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_DEFAULT,
};

static int allowed_sampling_rates[] = {
    10,
    20,
    40,
    60,
    80,
    100,
    200,
};

/*
 * Helpers
 */

#define PS2_GPIO_GET_BIT(data, bit_pos) ( (data >> bit_pos) & 0x1 )
#define PS2_GPIO_SET_BIT(data, bit_val, bit_pos) ( \
	data |= (bit_val) << bit_pos \
)

int array_get_elem_index(int elem, int *array, size_t array_size)
{
    int elem_index = -1;
    for(int i = 0; i < array_size; i++) {
        if(array[i] == elem) {
            elem_index = i;
            break;
        }
    }

    return elem_index;
}

int array_get_next_elem(int elem, int *array, size_t array_size)
{
    int elem_index = array_get_elem_index(elem, array, array_size);
    if(elem_index == -1) {
        return -1;
    }

    int next_index = elem_index + 1;
    if(next_index >= array_size) {
        return -1;
    }

    return array[next_index];
}

int array_get_prev_elem(int elem, int *array, size_t array_size)
{
    int elem_index = array_get_elem_index(elem, array, array_size);
    if(elem_index == -1) {
        return -1;
    }

    int prev_index = elem_index - 1;
    if(prev_index < 0 || prev_index >= array_size) {
        return -1;
    }

    return array[prev_index];
}

/*
 * Mouse Activity Packet Reading
 */

void zmk_ps2_mouse_activity_process_cmd(zmk_ps2_mouse_packet_mode packet_mode,
                                        uint8_t cmd_state,
                                        uint8_t cmd_x,
                                        uint8_t cmd_y,
                                        uint8_t cmd_extra);
void zmk_ps2_mouse_activity_abort_cmd();
void zmk_ps2_mouse_activity_move_mouse(int16_t mov_x, int16_t mov_y);
void zmk_ps2_mouse_activity_scroll(int8_t scroll_y);
void zmk_ps2_mouse_activity_click_buttons(bool button_l,
                                          bool button_m,
                                          bool button_r);
void zmk_ps2_mouse_activity_reset_cmd_buffer();

struct zmk_ps2_mouse_packet
zmk_ps2_mouse_activity_parse_cmd_buffer(zmk_ps2_mouse_packet_mode packet_mode,
                                        uint8_t cmd_state,
                                        uint8_t cmd_x,
                                        uint8_t cmd_y,
                                        uint8_t cmd_extra);

// Called by the PS/2 driver whenver the mouse sends a byte and
// reporting is enabled through `zmk_ps2_activity_reporting_enable`.
void zmk_ps2_mouse_activity_callback(const struct device *ps2_device,
                                     uint8_t byte)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    k_work_cancel_delayable(&data->cmd_buffer_timeout);

    // LOG_DBG("Received mouse movement data: 0x%x", byte);

    data->cmd_buffer[data->cmd_idx] = byte;

    if(data->cmd_idx == 0) {

        // Bit 3 of the first command byte should always be 1
        // If it is not, then we are definitely out of alignment.
        // So we ask the device to resend the entire 3-byte command
        // again.
        int alignment_bit = PS2_GPIO_GET_BIT(byte, 3);
        if(alignment_bit != 1) {

            zmk_ps2_mouse_activity_abort_cmd();
            return;
        }
    } else if(data->cmd_idx == 1) {
        // Do nothing
    } else if(
        (data->packet_mode == PS2_MOUSE_PACKET_MODE_PS2_DEFAULT &&
         data->cmd_idx == 2) ||
        (data->packet_mode == PS2_MOUSE_PACKET_MODE_SCROLL &&
         data->cmd_idx == 3)
    ) {

        zmk_ps2_mouse_activity_process_cmd(
            data->packet_mode,
            data->cmd_buffer[0],
            data->cmd_buffer[1],
            data->cmd_buffer[2],
            data->cmd_buffer[3]
        );
        zmk_ps2_mouse_activity_reset_cmd_buffer();
        return;
    }

    data->cmd_idx += 1;

	k_work_schedule(&data->cmd_buffer_timeout, PS2_MOUSE_TIMEOUT_CMD_BUFFER);
}

void zmk_ps2_mouse_activity_abort_cmd() {
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
    const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;
    const struct device *ps2_device = config->ps2_device;

    LOG_ERR(
        "PS/2 Mouse cmd buffer is out of aligment. Requesting resend."
    );

    data->cmd_idx = 0;
    ps2_write(ps2_device, PS2_MOUSE_CMD_RESEND[0]);
}

// Called if no new byte arrives within
// PS2_MOUSE_TIMEOUT_CMD_BUFFER
void zmk_ps2_mouse_activity_cmd_timout(struct k_work *item)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    LOG_DBG("Mouse movement cmd timed out on idx=%d", data->cmd_idx);

    // Reset the cmd buffer in case we are out of alignment.
    // This way if the mouse ever gets out of alignment, the user
    // can reset it by just not moving it for a second.
    zmk_ps2_mouse_activity_reset_cmd_buffer();
}

void zmk_ps2_mouse_activity_reset_cmd_buffer()
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    data->cmd_idx = 0;
    memset(data->cmd_buffer, 0x0, sizeof(data->cmd_buffer));
}

void zmk_ps2_mouse_activity_process_cmd(zmk_ps2_mouse_packet_mode packet_mode,
                                        uint8_t cmd_state,
                                        uint8_t cmd_x,
                                        uint8_t cmd_y,
                                        uint8_t cmd_extra)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    LOG_DBG(
        "zmk_ps2_mouse_activity_process_cmd "
        "Got state=0x%x x=0x%x, y=0x%x, extra=0x%x",
        cmd_state, cmd_x, cmd_y, cmd_extra
    );

    struct zmk_ps2_mouse_packet packet;
    packet = zmk_ps2_mouse_activity_parse_cmd_buffer(
        packet_mode,
        cmd_state, cmd_x, cmd_y, cmd_extra
    );

    int x_delta = abs(data->prev_packet.mov_x - packet.mov_x);
    int y_delta = abs(data->prev_packet.mov_y - packet.mov_y);

    LOG_DBG(
        "Got mouse activity cmd "
        "(mov_x=%d, mov_y=%d, o_x=%d, o_y=%d, scroll=%d, "
        "b_l=%d, b_m=%d, b_r=%d) and ("
        "x_delta=%d, y_delta=%d)",
        packet.mov_x, packet.mov_y, packet.overflow_x, packet.overflow_y,
        packet.scroll, packet.button_l, packet.button_m, packet.button_r,
        x_delta, y_delta
    );

    // If the mouse exceeds the allowed threshold of movement, it's probably
    // a mistransmission or misalignment.
    // But we only do this check if there was prior movement that wasn't reset
    // in `zmk_ps2_mouse_activity_cmd_timout`.
    if((data->move_speed.x != 0 && data->move_speed.y != 0) &&
       (x_delta > 150  || y_delta > 150))
    {
        LOG_WRN(
            "Detected malformed packet with "
            "(mov_x=%d, mov_y=%d, o_x=%d, o_y=%d, scroll=%d, "
            "b_l=%d, b_m=%d, b_r=%d) and ("
            "x_delta=%d, y_delta=%d)",
            packet.mov_x, packet.mov_y, packet.overflow_x, packet.overflow_y,
            packet.scroll, packet.button_l, packet.button_m, packet.button_r,
            x_delta, y_delta
        );
        // zmk_ps2_mouse_activity_abort_cmd();
        return;
    }

    zmk_ps2_mouse_activity_move_mouse(packet.mov_x, packet.mov_y);
    zmk_ps2_mouse_activity_scroll(packet.scroll);

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_ENABLE_CLICKING)
        zmk_ps2_mouse_activity_click_buttons(
            packet.button_l, packet.button_m, packet.button_r
        );
    #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_ENABLE_CLICKING) */

    data->prev_packet = packet;
}

struct zmk_ps2_mouse_packet
zmk_ps2_mouse_activity_parse_cmd_buffer(zmk_ps2_mouse_packet_mode packet_mode,
                                        uint8_t cmd_state,
                                        uint8_t cmd_x,
                                        uint8_t cmd_y,
                                        uint8_t cmd_extra)
{
    struct zmk_ps2_mouse_packet packet;

    packet.button_l = PS2_GPIO_GET_BIT(cmd_state, 0);
    packet.button_r = PS2_GPIO_GET_BIT(cmd_state, 1);
    packet.button_m = PS2_GPIO_GET_BIT(cmd_state, 2);
    packet.overflow_x = PS2_GPIO_GET_BIT(cmd_state, 6);
    packet.overflow_y = PS2_GPIO_GET_BIT(cmd_state, 7);

    // The coordinates are delivered as a signed 9bit integers.
    // But a PS/2 packet is only 8 bits, so the most significant
    // bit with the sign is stored inside the state packet.
    //
    // Since we are converting the uint8_t into a int16_t
    // we must pad the unused most significant bits with
    // the sign bit.
    //
    // Example:
    //                              ↓ x sign bit
    //  - State: 0x18 (          0001 1000)
    //                             ↑ y sign bit
    //  - X:     0xfd (          1111 1101) / decimal 253
    //  - New X:      (1111 1111 1111 1101) / decimal -3
    //
    //  - Y:     0x02 (          0000 0010) / decimal 2
    //  - New Y:      (0000 0000 0000 0010) / decimal 2
    //
    // The code below creates a signed int and is from...
    // https://wiki.osdev.org/PS/2_Mouse
    packet.mov_x = cmd_x - ((cmd_state << 4) & 0x100);
    packet.mov_y = cmd_y - ((cmd_state << 3) & 0x100);

    // If packet mode scroll or scroll+5 buttons is used,
    // then the first 4 bit of the extra byte are used for the
    // scroll wheel. It is a signed number with the rango of
    // -8 to +7.
    if(packet_mode == PS2_MOUSE_PACKET_MODE_SCROLL) {
        packet.scroll = 0x0;

        PS2_GPIO_SET_BIT(
            packet.scroll,
            PS2_GPIO_GET_BIT(cmd_extra, 0),
            0
        );
        PS2_GPIO_SET_BIT(
            packet.scroll,
            PS2_GPIO_GET_BIT(cmd_extra, 1),
            1
        );
        PS2_GPIO_SET_BIT(
            packet.scroll,
            PS2_GPIO_GET_BIT(cmd_extra, 2),
            2
        );
        packet.scroll = cmd_extra - ((packet.scroll << 3) & 0x100);
    }

    return packet;
}


/*
 * Mouse Moving and Clicking
 */

// We don't send the mouse move over BT every time we get a PS2 packet,
// because it can send too fast.
// Here we just add up the coordinates and then we use a timer to actually
// send the movement once every CONFIG_ZMK_MOUSE_TICK_DURATION ms in
// zmk_ps2_mouse_tick_timer_handler.
void zmk_ps2_mouse_activity_move_mouse(int16_t mov_x, int16_t mov_y)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    data->move_speed.x += mov_x;
    data->move_speed.y += mov_y;
}

void zmk_ps2_mouse_activity_scroll(int8_t scroll_y)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    data->scroll_speed.y += scroll_y;
}

// Called using k_timer data->mouse_timer every x ms as configured with
// CONFIG_ZMK_MOUSE_TICK_DURATION
void zmk_ps2_mouse_tick_timer_cb(struct k_timer *dummy) {
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    // LOG_DBG("Submitting mouse work to queue");
    // Calls zmk_ps2_mouse_tick_timer_handler
    k_work_submit_to_queue(zmk_mouse_work_q(), &data->mouse_tick);
}

// Here is where we actually ask zmk to send the mouse movement to
// the OS.
static void zmk_ps2_mouse_tick_timer_handler(struct k_work *work)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    // LOG_DBG("Raising mouse tick event");

    if(data->move_speed.x == 0 && data->move_speed.y == 0 &&
       data->scroll_speed.x == 0 && data->scroll_speed.y == 0) {
        // LOG_DBG("Not raising mouse tick event as the mouse hasn't moved.");
        return;
    }

    struct vector2d mouse_move = data->move_speed;
    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_SWAP_XY)
        mouse_move.x = data->move_speed.y;
        mouse_move.y = data->move_speed.x;
    #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_SWAP_XY) */

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_INVERT_X)
        mouse_move.x = -mouse_move.x;
    #endif /* IS_ENABLED(ZMK_MOUSE_PS2_INVERT_X) */

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_INVERT_Y)
        mouse_move.y = -mouse_move.y;
    #endif /* IS_ENABLED(ZMK_MOUSE_PS2_INVERT_Y) */

    zmk_hid_mouse_movement_set(0, 0);

    // ZMK expects up movement to be negative, but PS2 sends it as positive
    zmk_hid_mouse_movement_update(mouse_move.x, -mouse_move.y);

    zmk_hid_mouse_scroll_set(0, 0);
    zmk_hid_mouse_scroll_update(data->scroll_speed.x, data->scroll_speed.y);

    zmk_endpoints_send_mouse_report();

    data->move_speed.x = 0;
    data->move_speed.y = 0;

    data->scroll_speed.x = 0;
    data->scroll_speed.y = 0;
}

void zmk_ps2_mouse_activity_click_buttons(bool button_l,
                                          bool button_m,
                                          bool button_r)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
    bool should_send_report = false;

    // TODO: Integrate this with the proper button mask instead
    // of hardcoding the mouse button indeces.
    // Check hid.c and zmk_hid_mouse_buttons_press() for more info.

    if(button_l == true && data->button_l_is_held == false) {
        LOG_DBG("Pressing button_l");
        zmk_hid_mouse_button_press(PS2_MOUSE_BUTTON_L_IDX);
        data->button_l_is_held = true;
        should_send_report = true;
    } else if(button_l == false && data->button_l_is_held == true) {
        LOG_DBG("Releasing button_l");
        zmk_hid_mouse_button_release(PS2_MOUSE_BUTTON_L_IDX);
        data->button_l_is_held = false;
        should_send_report = true;
    }

    if(button_m == true && data->button_m_is_held == false) {
        LOG_DBG("Pressing button_m");
        zmk_hid_mouse_button_press(PS2_MOUSE_BUTTON_M_IDX);
        data->button_m_is_held = true;
        should_send_report = true;
    } else if(button_m == false && data->button_m_is_held == true) {
        LOG_DBG("Releasing button_m");
        zmk_hid_mouse_button_release(PS2_MOUSE_BUTTON_M_IDX);
        data->button_m_is_held = false;
        should_send_report = true;
    }

    if(button_r == true && data->button_r_is_held == false) {
        LOG_DBG("Pressing button_r");
        zmk_hid_mouse_button_press(PS2_MOUSE_BUTTON_R_IDX);
        data->button_r_is_held = true;
        should_send_report = true;
    } else if(button_r == false && data->button_r_is_held == true) {
        LOG_DBG("Releasing button_r");
        zmk_hid_mouse_button_release(PS2_MOUSE_BUTTON_R_IDX);
        data->button_r_is_held = false;
        should_send_report = true;
    }

    // Since mouse clicks generate far few events than movement,
    // we send them right away instead of using the timer.
    if(should_send_report) {
        zmk_endpoints_send_mouse_report();
    }
}


/*
 * PS/2 Command Sending Wrapper
 */
int zmk_ps2_activity_reporting_enable();
int zmk_ps2_activity_reporting_disable();


struct zmk_ps2_send_cmd_resp {
    int err;
    char err_msg[50];
    uint8_t resp_buffer[8];
    int resp_len;
};

struct zmk_ps2_send_cmd_resp zmk_ps2_send_cmd(char *cmd,
                                              int cmd_len,
                                              uint8_t *arg,
                                              int resp_len,
                                              bool pause_reporting)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
    const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;
    const struct device *ps2_device = config->ps2_device;
    int err = 0;
    bool prev_activity_reporting_on = data->activity_reporting_on;

    struct zmk_ps2_send_cmd_resp resp = {
        .err = 0,
        .err_msg = "",
        .resp_len = 0,
    };
    memset(resp.resp_buffer, 0x0, sizeof(resp.resp_buffer));

    // Don't send the string termination NULL byte
    int cmd_bytes = cmd_len - 1;
    if(cmd_bytes < 1) {
        err = 1;
        snprintf(
            resp.err_msg, sizeof(resp.err_msg),
            "Cannot send cmd with less than 1 byte length"
        );

        return resp;
    }

    if(resp_len > sizeof(resp.resp_buffer)) {
        err = 2;
        snprintf(
            resp.err_msg, sizeof(resp.err_msg),
            "Response can't be longer than the resp_buffer (%d)",
            sizeof(resp.err_msg)
        );

        return resp;
    }

    if(pause_reporting == true && data->activity_reporting_on == true) {
        LOG_DBG("Disabling mouse activity reporting...");

        err = zmk_ps2_activity_reporting_disable();
        if(err) {
            resp.err = err;
            snprintf(
                resp.err_msg, sizeof(resp.err_msg),
                "Could not disable data reporting (%d)", err
            );
        }
    }

    if(resp.err == 0) {
        LOG_DBG("Sending cmd...");

        for(int i = 0; i < cmd_bytes; i++) {
            err = ps2_write(ps2_device, cmd[i]);
            if(err) {
                resp.err = err;
                snprintf(
                    resp.err_msg, sizeof(resp.err_msg),
                    "Could not send cmd byte %d/%d (%d)", i+1, cmd_bytes, err
                );
                break;
            }
        }
    }

    if(resp.err == 0 && arg != NULL) {
        LOG_DBG("Sending arg...");
        err = ps2_write(ps2_device, *arg);
        if(err) {
            resp.err = err;
            snprintf(
                resp.err_msg, sizeof(resp.err_msg),
                "Could not send arg (%d)", err
            );
        }
    }

    if(resp.err == 0 && resp_len > 0) {
        LOG_DBG("Reading response...");
        for(int i = 0; i < resp_len; i++) {
            err = ps2_read(ps2_device, &resp.resp_buffer[i]);
            if(err) {
                resp.err = err;
                snprintf(
                    resp.err_msg, sizeof(resp.err_msg),
                    "Could not read response cmd byte %d/%d (%d)",
                    i+1, resp_len, err
                );
                break;
            }
        }
    }

    if(pause_reporting == true && prev_activity_reporting_on == true) {
        LOG_DBG("Enabling mouse activity reporting...");

        err = zmk_ps2_activity_reporting_enable();
        if(err) {
            // Don' overwrite existing error
            if(resp.err == 0) {
                resp.err = err;
                snprintf(
                    resp.err_msg, sizeof(resp.err_msg),
                    "Could not re-enable data reporting (%d)", err
                );
            }
        }
    }

    return resp;
}


int zmk_ps2_activity_reporting_enable()
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
    const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;
    const struct device *ps2_device = config->ps2_device;

    if(data->activity_reporting_on == true) {
        return 0;
    }

    LOG_INF("Enabling mouse activity reporting...");

    uint8_t cmd = PS2_MOUSE_CMD_ENABLE_REPORTING[0];
    int err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR("Could not enable data reporting: %d", err);
        return err;
    }

    err = ps2_enable_callback(ps2_device);
    if(err) {
        LOG_ERR("Could not enable ps2 callback: %d", err);
        return err;
    }

    data->activity_reporting_on = true;

    return 0;
}

int zmk_ps2_activity_reporting_disable()
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
    const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;
    const struct device *ps2_device = config->ps2_device;

    if(data->activity_reporting_on == false) {
        return 0;
    }

    LOG_INF("Disabling mouse activity reporting...");

    uint8_t cmd = PS2_MOUSE_CMD_DISABLE_REPORTING[0];
    int err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR("Could not disable data reporting: %d", err);
        return err;
    }

    err = ps2_disable_callback(ps2_device);
    if(err) {
        LOG_ERR("Could not disable ps2 callback: %d", err);
        return err;
    }

    data->activity_reporting_on = false;

    return 0;
}

/*
 * PS/2 Commands
 */

int zmk_ps2_reset(const struct device *ps2_device) {
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_RESET,
        sizeof(PS2_MOUSE_CMD_RESET),
        NULL,
        PS2_MOUSE_CMD_RESET_RESP_LEN,
        false
    );
    if(resp.err) {
        LOG_ERR(
            "Could not send reset cmd"
        );
    }

    return resp.err;
}

int zmk_ps2_get_secondary_id(uint8_t *resp_byte_1,
                             uint8_t *resp_byte_2)
{
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_GET_SECONDARY_ID,
        sizeof(PS2_MOUSE_CMD_GET_SECONDARY_ID),
        NULL,
        PS2_MOUSE_CMD_GET_SECONDARY_ID_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not get secondary id"
        );
        return resp.err;
    }

    *resp_byte_1 = resp.resp_buffer[0];
    *resp_byte_2 = resp.resp_buffer[1];

    return 0;
}

int zmk_ps2_set_sampling_rate(uint8_t sampling_rate)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int rate_idx = array_get_elem_index(
        sampling_rate,
        allowed_sampling_rates, sizeof(allowed_sampling_rates)
    );
    if(rate_idx == -1) {
        LOG_ERR("Requested to set illegal sampling rate: %d", sampling_rate);
        return -1;
    }

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_SET_SAMPLING_RATE,
        sizeof(PS2_MOUSE_CMD_SET_SAMPLING_RATE),
        &sampling_rate,
        PS2_MOUSE_CMD_SET_SAMPLING_RATE_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not set sample rate to %d", sampling_rate
        );
        return resp.err;
    }

    data->sampling_rate = sampling_rate;

    return resp.err;
}

int zmk_ps2_set_sampling_rate_incr() {
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int next_rate = array_get_next_elem(
        data->sampling_rate,
        allowed_sampling_rates, sizeof(allowed_sampling_rates)
    );
    if(next_rate == -1) {
        LOG_ERR(
            "Could not increase sampling rate from %d. Already the max rate",
            data->sampling_rate
        );

        return -1;
    }

    LOG_DBG("zmk_ps2_send_cmd_sampling_rate_incr setting %d", next_rate);

    return zmk_ps2_set_sampling_rate(next_rate);
}

int zmk_ps2_set_sampling_rate_decr() {
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int prev_rate = array_get_prev_elem(
        data->sampling_rate,
        allowed_sampling_rates, sizeof(allowed_sampling_rates)
    );
    if(prev_rate == -1) {
        LOG_ERR(
            "Could not decrease sampling rate from %d. Already the min rate",
            data->sampling_rate
        );

        return -1;
    }

    LOG_DBG("zmk_ps2_send_cmd_sampling_rate_decr setting %d", prev_rate);
    return zmk_ps2_set_sampling_rate(prev_rate);
}

int zmk_ps2_get_device_id(uint8_t *device_id)
{

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_GET_DEVICE_ID,
        sizeof(PS2_MOUSE_CMD_GET_DEVICE_ID),
        NULL,
        1,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not get device id"
        );
        return resp.err;
    }

    *device_id = resp.resp_buffer[0];

    return 0;
}

int zmk_ps2_set_packet_mode(zmk_ps2_mouse_packet_mode mode)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    if(mode == PS2_MOUSE_PACKET_MODE_PS2_DEFAULT) {
        // Do nothing. Mouse devices enable this by
        // default.
        return 0;
    }

    bool prev_activity_reporting_on = data->activity_reporting_on;
    zmk_ps2_activity_reporting_disable();

    // Setting a mouse mode is a bit like using a cheat code
    // in a video game.
    // You have to send a specific sequence of sampling rates.
    if(mode == PS2_MOUSE_PACKET_MODE_SCROLL) {

        zmk_ps2_set_sampling_rate(200);
        zmk_ps2_set_sampling_rate(100);
        zmk_ps2_set_sampling_rate(80);
    }

    // Scroll mouse + 5 buttons mode can be enabled with the
    // following sequence, but since I don't have a mouse to
    // test it, I am commenting it out for now.
    // else if(mode == PS2_MOUSE_PACKET_MODE_SCROLL_5_BUTTONS) {

    //     zmk_ps2_set_sampling_rate(200);
    //     zmk_ps2_set_sampling_rate(200);
    //     zmk_ps2_set_sampling_rate(80);
    // }

    uint8_t device_id;
    int err = zmk_ps2_get_device_id(&device_id);
    if(err) {
        LOG_ERR(
            "Could not enable packet mode %d. Failed to get device id with "
            "error %d", mode, err
        );
    } else {
        if(device_id == 0x00) {
            LOG_ERR(
                "Could not enable packet mode %d. The device does not support "
                "it", mode
            );

            data->packet_mode = PS2_MOUSE_PACKET_MODE_PS2_DEFAULT;
            err = 1;
        } else if(device_id == 0x03 || device_id == 0x04) {
            LOG_INF(
                "Successfully activated packet mode %d. Mouse returned device "
                "id: %d", mode, device_id
            );

            data->packet_mode = PS2_MOUSE_PACKET_MODE_SCROLL;
            err = 0;
        }
        // else if(device_id == 0x04) {
        //     LOG_INF(
        //         "Successfully activated packet mode %d. Mouse returned device "
        //         "id: %d", mode, device_id
        //     );

        //     data->packet_mode = PS2_MOUSE_PACKET_MODE_SCROLL_5_BUTTONS;
        //     err = 0;
        // }
        else {
            LOG_ERR(
                "Could not enable packet mode %d. Received an invalid device "
                "id: %d", mode, device_id
            );

            data->packet_mode = PS2_MOUSE_PACKET_MODE_PS2_DEFAULT;
            err = 1;
        }
    }

    // Restore sampling rate to prev value
    zmk_ps2_set_sampling_rate(data->sampling_rate);

    if(prev_activity_reporting_on == true) {
        zmk_ps2_activity_reporting_enable();
    }

    return err;
}

/*
 * Trackpoint Commands
 */

bool zmk_ps2_is_device_trackpoint()
{
    bool ret = false;

    uint8_t second_id_1, second_id_2;
    int err = zmk_ps2_get_secondary_id(
        &second_id_1, &second_id_2
    );
    if(err) {
        // Not all devices implement this command.
        ret = false;
    } else {
        if(second_id_1 == 0x1) {
            ret = true;
        }
    }

    LOG_DBG("Connected device is a trackpoint: %d", ret);

    return ret;
}

int zmk_ps2_tp_get_config_byte(uint8_t *config_byte)
{
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_GET_CONFIG_BYTE,
        sizeof(PS2_MOUSE_CMD_TP_GET_CONFIG_BYTE),
        NULL,
        PS2_MOUSE_CMD_TP_GET_CONFIG_BYTE_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not read trackpoint config byte"
        );
        return resp.err;
    }

    *config_byte = resp.resp_buffer[0];

    return 0;
}

int zmk_ps2_tp_set_config_option(int config_bit, bool enabled, char *descr)
{
    uint8_t config_byte;
    int err = zmk_ps2_tp_get_config_byte(
        &config_byte
    );
    if(err) {
        return err;
    }

    bool is_enabled = PS2_GPIO_GET_BIT(
        config_byte,
        config_bit
    );

    if(is_enabled == enabled) {
        LOG_DBG(
            "Trackpoint %s was already %s... not doing anything.",
            descr, is_enabled ? "enabled" : "disabled"
        );
        return 0;
    }

    LOG_DBG(
        "Setting trackpoint %s: %s",
        descr, enabled ? "enabled" : "disabled"
    );

    PS2_GPIO_SET_BIT(config_byte, enabled, config_bit);

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_SET_CONFIG_BYTE,
        sizeof(PS2_MOUSE_CMD_TP_SET_CONFIG_BYTE),
        &config_byte,
        PS2_MOUSE_CMD_TP_SET_CONFIG_BYTE_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not set trackpoint %s to %s",
            descr, enabled ? "enabled" : "disabled"
        );
        return resp.err;
    }

    return 0;
}

int zmk_ps2_tp_press_to_select_set(bool enabled)
{
    int err = zmk_ps2_tp_set_config_option(
        PS2_MOUSE_TP_CONFIG_BIT_PRESS_TO_SELECT,
        enabled,
        "Press To Select"
    );

    return err;
}

int zmk_ps2_tp_invert_x_set(bool enabled)
{
    int err = zmk_ps2_tp_set_config_option(
        PS2_MOUSE_TP_CONFIG_BIT_INVERT_X,
        enabled,
        "Invert X"
    );

    return err;
}

int zmk_ps2_tp_invert_y_set(bool enabled)
{
    int err = zmk_ps2_tp_set_config_option(
        PS2_MOUSE_TP_CONFIG_BIT_INVERT_Y,
        enabled,
        "Invert Y"
    );

    return err;
}

int zmk_ps2_tp_swap_xy_set(bool enabled)
{
    int err = zmk_ps2_tp_set_config_option(
        PS2_MOUSE_TP_CONFIG_BIT_SWAP_XY,
        enabled,
        "Swap XY"
    );

    return err;
}

int zmk_ps2_tp_sensitivity_get(float *sensitivity)
{
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_GET_SENSITIVITY,
        sizeof(PS2_MOUSE_CMD_TP_GET_SENSITIVITY),
        NULL,
        PS2_MOUSE_CMD_TP_GET_SENSITIVITY_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not get trackpoint sensitivity"
        );
        return resp.err;
    }

    // Convert uint8_t to float
    // 0x80 (128) represents 1.0
    uint8_t sensitivity_int = resp.resp_buffer[0];
    float sensitivity_float = sensitivity_int / 0x80;
    *sensitivity = sensitivity_float;

    LOG_DBG("Trackpoint sensitivity is %f", sensitivity_float);

    return 0;
}

int zmk_ps2_tp_sensitivity_set(float sensitivity)
{
    if(sensitivity < PS2_MOUSE_CMD_TP_SET_SENSITIVITY_MIN ||
       sensitivity > PS2_MOUSE_CMD_TP_SET_SENSITIVITY_MAX)
    {
        LOG_ERR(
            "Invalid sensitivity value %f. Min: %f; Max: %f",
            sensitivity,
            PS2_MOUSE_CMD_TP_SET_SENSITIVITY_MIN,
            PS2_MOUSE_CMD_TP_SET_SENSITIVITY_MAX
        );
        return 1;
    }

    // Convert float to byte arg
    // 0x80 (128) represents 1.0
    uint8_t sensitivity_arg = 0x80 * sensitivity;

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_SET_SENSITIVITY,
        sizeof(PS2_MOUSE_CMD_TP_SET_SENSITIVITY),
        &sensitivity_arg,
        PS2_MOUSE_CMD_TP_SET_SENSITIVITY_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not set sensitivity to %f", sensitivity
        );
        return resp.err;
    }

    return 0;
}

int zmk_ps2_tp_sensitivity_incr(float incr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    float new_val = data->tp_sensitivity + incr_amount;

    LOG_INF("Setting trackpoint sensitivity to %f", new_val);
    int err = zmk_ps2_tp_sensitivity_set(new_val);

    return err;
}

int zmk_ps2_tp_sensitivity_decr(float decr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    float new_val = data->tp_sensitivity - decr_amount;

    LOG_INF("Setting trackpoint sensitivity to %f", new_val);
    int err = zmk_ps2_tp_sensitivity_set(new_val);

    return err;
}

int zmk_ps2_tp_negative_inertia_get(uint8_t *neg_inertia)
{
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_GET_NEG_INERTIA,
        sizeof(PS2_MOUSE_CMD_TP_GET_NEG_INERTIA),
        NULL,
        PS2_MOUSE_CMD_TP_GET_NEG_INERTIA_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not get trackpoint negative inertia"
        );
        return resp.err;
    }

    uint8_t neg_inertia_int = resp.resp_buffer[0];
    *neg_inertia = neg_inertia_int;

    LOG_DBG("Trackpoint negative inertia is %d", neg_inertia_int);

    return 0;
}

int zmk_ps2_tp_neg_inertia_set(int neg_inertia)
{
    if(neg_inertia < PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_MIN ||
       neg_inertia > PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_MAX)
    {
        LOG_ERR(
            "Invalid sensitivity value %d. Min: %d; Max: %d",
            neg_inertia,
            PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_MIN,
            PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_MAX
        );
        return 1;
    }

    uint8_t arg = neg_inertia;

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_SET_NEG_INERTIA,
        sizeof(PS2_MOUSE_CMD_TP_SET_NEG_INERTIA),
        &arg,
        PS2_MOUSE_CMD_TP_SET_NEG_INERTIA_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not set negative inertia to %d",
            neg_inertia
        );
        return resp.err;
    }

    return 0;
}

int zmk_ps2_tp_neg_inertia_incr(int incr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int new_val = data->tp_neg_inertia + incr_amount;

    LOG_INF("Setting negative inertia to %d", new_val);
    int err = zmk_ps2_tp_neg_inertia_set(new_val);

    return err;
}

int zmk_ps2_tp_neg_inertia_decr(int decr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int new_val = data->tp_neg_inertia - decr_amount;

    LOG_INF("Setting negative inertia to %d", new_val);
    int err = zmk_ps2_tp_neg_inertia_set(new_val);

    return err;
}

int zmk_ps2_tp_value6_upper_plateau_speed_get(uint8_t *value6)
{
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED,
        sizeof(PS2_MOUSE_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED),
        NULL,
        PS2_MOUSE_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not get trackpoint value6 upper plateau speed"
        );
        return resp.err;
    }

    uint8_t value6_int = resp.resp_buffer[0];
    *value6 = value6_int;

    LOG_DBG("Trackpoint value6 upper plateau speed is %d", value6_int);

    return 0;
}

int zmk_ps2_tp_value6_upper_plateau_speed_set(int value6)
{
    if(value6 < PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MIN ||
       value6 > PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MAX)
    {
        LOG_ERR(
            "Invalid value6 upper plateau speed value %d. Min: %d; Max: %d",
            value6,
            PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MIN,
            PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MAX
        );
        return 1;
    }

    uint8_t arg = value6;

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED,
        sizeof(PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED),
        &arg,
        PS2_MOUSE_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not set value6 upper plateau speed to %d",
            value6
        );
        return resp.err;
    }

    return 0;
}

int zmk_ps2_tp_value6_upper_plateau_speed_incr(int incr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int new_val = data->tp_value6 + incr_amount;

    LOG_INF("Setting value6 upper plateau speed to %d", new_val);
    int err = zmk_ps2_tp_value6_upper_plateau_speed_set(new_val);

    return err;
}

int zmk_ps2_tp_value6_upper_plateau_speed_decr(int decr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int new_val = data->tp_value6 - decr_amount;

    LOG_INF("Setting value6 upper plateau speed to %d", new_val);
    int err = zmk_ps2_tp_value6_upper_plateau_speed_set(new_val);

    return err;
}

int zmk_ps2_tp_pts_threshold_get(uint8_t *pts_threshold)
{
    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_GET_PTS_THRESHOLD,
        sizeof(PS2_MOUSE_CMD_TP_GET_PTS_THRESHOLD),
        NULL,
        PS2_MOUSE_CMD_TP_GET_PTS_THRESHOLD_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not get trackpoint press-to-select threshold"
        );
        return resp.err;
    }

    uint8_t pts_threshold_int = resp.resp_buffer[0];
    *pts_threshold = pts_threshold_int;

    LOG_DBG(
        "Trackpoint press-to-select threshold is %d", pts_threshold_int
    );

    return 0;
}

int zmk_ps2_tp_pts_threshold_set(int pts_threshold)
{
    if(pts_threshold < PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_MIN ||
       pts_threshold > PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_MAX)
    {
        LOG_ERR(
            "Invalid press-to-select threshold value %d. Min: %d; Max: %d",
            pts_threshold,
            PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_MIN,
            PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_MAX
        );
        return 1;
    }

    uint8_t arg = pts_threshold;

    struct zmk_ps2_send_cmd_resp resp = zmk_ps2_send_cmd(
        PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD,
        sizeof(PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD),
        &arg,
        PS2_MOUSE_CMD_TP_SET_PTS_THRESHOLD_RESP_LEN,
        true
    );
    if(resp.err) {
        LOG_ERR(
            "Could not set press-to-select threshold to %d",
            pts_threshold
        );
        return resp.err;
    }

    return 0;
}

int zmk_ps2_tp_pts_threshold_incr(int incr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int new_val = data->tp_pts_threshold + incr_amount;

    LOG_INF("Setting press-to-select threshold to %d", new_val);
    int err = zmk_ps2_tp_pts_threshold_set(new_val);

    return err;
}

int zmk_ps2_tp_pts_threshold_decr(int decr_amount)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    int new_val = data->tp_pts_threshold - decr_amount;

    LOG_INF("Setting press-to-select threshold to %d", new_val);
    int err = zmk_ps2_tp_pts_threshold_set(new_val);

    return err;
}

/*
 * Init
 */

static void zmk_ps2_mouse_init_thread(int dev_ptr, int unused);
int zmk_ps2_init_to_power_on_reset();
int zmk_ps2_init_wait_for_mouse(const struct device *dev);

static int zmk_ps2_mouse_init(const struct device *dev)
{
	LOG_DBG("Inside zmk_ps2_mouse_init");

	LOG_DBG("Creating ps2_mouse init thread.");
    k_thread_create(
        &zmk_ps2_mouse_data.thread,
        zmk_ps2_mouse_data.thread_stack,
        PS2_MOUSE_THREAD_STACK_SIZE,
        (k_thread_entry_t)zmk_ps2_mouse_init_thread,
        (struct device *)dev, 0, NULL,
        K_PRIO_COOP(PS2_MOUSE_THREAD_PRIORITY), 0, K_NO_WAIT
    );

	return 0;
}

static void zmk_ps2_mouse_init_thread(int dev_ptr, int unused) {
    const struct device *dev = INT_TO_POINTER(dev_ptr);
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
	const struct zmk_ps2_mouse_config *config = dev->config;
    int err;

    zmk_ps2_init_to_power_on_reset();

	LOG_INF("Waiting for mouse to connect...");
    err = zmk_ps2_init_wait_for_mouse(dev);
    if(err) {
        LOG_ERR(
            "Could not init a mouse in %d attempts. Giving up. "
            "Power cycle the mouse and reset zmk to try again.",
            PS2_MOUSE_INIT_ATTEMPTS
        );
        return;
    }

    #if CONFIG_ZMK_MOUSE_PS2_SAMPLING_RATE != \
            PS2_MOUSE_CMD_SET_SAMPLING_RATE_DEFAULT
        LOG_INF(
            "Setting sample rate to %d...",
            CONFIG_ZMK_MOUSE_PS2_SAMPLING_RATE
        );
        zmk_ps2_set_sampling_rate(CONFIG_ZMK_MOUSE_PS2_SAMPLING_RATE);
        if(err) {
            LOG_ERR(
                "Could not set sampling rate to %d: %d",
                CONFIG_ZMK_MOUSE_PS2_SAMPLING_RATE,
                err
            );
            return;
        }
    #endif /* CONFIG_ZMK_MOUSE_PS2_SAMPLING_RATE */

    if(zmk_ps2_is_device_trackpoint() == true) {
        LOG_INF("Device is a trackpoint");

        #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_TAP_TO_SELECT)
            LOG_INF("Enabling trackpoint press to select...");
            zmk_ps2_tp_press_to_select_set(true);
        #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_TAP_TO_SELECT) */

        #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_INVERT_X)
            LOG_INF("Inverting trackpoint x axis.");
            zmk_ps2_tp_invert_x_set(true);
        #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_INVERT_X) */

        #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_INVERT_Y)
            LOG_INF("Inverting trackpoint y axis.");
            zmk_ps2_tp_invert_y_set(true);
        #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_INVERT_Y) */

        #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_SWAP_XY)
            LOG_INF("Swapping trackpoint x and y axis.");
            zmk_ps2_tp_swap_xy_set(true);
        #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_TP_SWAP_XY) */

    }

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_SCROLL)
        LOG_INF("Enabling scroll mode.");
        zmk_ps2_set_packet_mode(PS2_MOUSE_PACKET_MODE_SCROLL);
    #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_SCROLL) */

    // Configure read callback
	LOG_DBG("Configuring ps2 callback...");
    err = ps2_config(config->ps2_device, &zmk_ps2_mouse_activity_callback);
    if(err) {
        LOG_ERR("Could not configure ps2 interface: %d", err);
        return ;
    }

	LOG_INF("Enabling data reporting and ps2 callback...");
    err = zmk_ps2_activity_reporting_enable();
    if(err) {
        LOG_ERR("Could not activate ps2 callback: %d", err);
    } else {
        LOG_DBG("Successfully activated ps2 callback");
    }

    k_timer_init(&data->mouse_timer, zmk_ps2_mouse_tick_timer_cb, NULL);
    k_timer_start(
        &data->mouse_timer, K_NO_WAIT, K_MSEC(CONFIG_ZMK_MOUSE_TICK_DURATION)
    );
    k_work_init(&data->mouse_tick, zmk_ps2_mouse_tick_timer_handler);
    k_work_init_delayable(
        &data->cmd_buffer_timeout, zmk_ps2_mouse_activity_cmd_timout
    );

	return;
}

// Power-On-Reset for trackpoints (and possibly other devices).
// From the `IBM TrackPoint System Version 4.0 Engineering
// Specification`...
// "The TrackPoint logic shall execute a Power On Reset (POR) when power is
//  applied to the device. The POR shall be timed to occur 600 ms ± 20 % from
//  the time power is applied to the TrackPoint controller. Activity on the
//  clock and data lines is ignored prior to the completion of the diagnostic
//  sequence. (See RESET mode of operation.)"
int zmk_ps2_init_to_power_on_reset()
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;
	const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;

    if(config->rst_gpio_name == NULL) {
        return 0;
    }

    LOG_INF("Performing Power-On-Reset...");

    if(data->rst_gpio == NULL) {
        data->rst_gpio = device_get_binding(config->rst_gpio_name);
        if (!data->rst_gpio) {
            LOG_ERR("Failed Power-On-Reset: Failed to get RST GPIO device...");
            return -EINVAL;
        }
    }

    //  Set reset pin low...
	int err = gpio_pin_configure(
		data->rst_gpio,
		config->rst_pin,
		(GPIO_OUTPUT_HIGH)
	);
	if (err) {
		LOG_ERR(
            "Failed Power-On-Reset: Failed to configure RST GPIO pin to "
            "output low (err %d)", err
        );
        return err;
	}

    // Wait 600ms
    k_sleep(PS2_MOUSE_POWER_ON_RESET_TIME);

    // Set pin high
    err = gpio_pin_set(data->rst_gpio, config->rst_pin, 0);
	if (err) {
		LOG_ERR(
            "Failed Power-On-Reset: Failed to set RST GPIO pin to "
            "low (err %d)", err
        );
        return err;
	}

    LOG_DBG("Finished Power-On-Reset successfully...");

    return 0;
}

int zmk_ps2_init_wait_for_mouse(const struct device *dev)
{
	const struct zmk_ps2_mouse_config *config = dev->config;
    int err;

    uint8_t read_val;

    for(int i = 0; i < PS2_MOUSE_INIT_ATTEMPTS; i++) {

        LOG_INF(
            "Trying to initialize mouse device (attempt %d / %d)",
            i+1, PS2_MOUSE_INIT_ATTEMPTS
        );

        // PS/2 Devices do a self-test and send the result when they power up.

        err = ps2_read(config->ps2_device, &read_val);
        if(err == 0) {
            if(read_val != PS2_MOUSE_RESP_SELF_TEST_PASS) {
                LOG_WRN("Got invalid PS/2 self-test result: 0x%x", read_val);
                continue;
            }

            LOG_INF("PS/2 Device passed self-test: 0x%x", read_val);

            // Read device id
            LOG_INF("Reading PS/2 device id...");
            err = ps2_read(config->ps2_device, &read_val);
            if(err) {
                LOG_WRN("Could not read PS/2 device id: %d", err);
            } else {
                if(read_val == 0) {
                    LOG_INF("Connected PS/2 device is a mouse...");
                    return 0;
                } else {
                    LOG_WRN("PS/2 device is not a mouse: 0x%x", read_val);
                    return 1;
                }
            }
        } else {
            LOG_DBG(
                "Could not read PS/2 device self-test result: %d. ", err
            );
        }

        // But when a zmk device is reset, it doesn't cut the power to external
        // devices. So the device acts as if it was never disconnected.
        // So we try sending the reset command.
        if(i % 2 == 0) {
            LOG_INF("Trying to reset PS2 device...");
            zmk_ps2_reset(config->ps2_device);
            continue;
        }

        k_sleep(K_SECONDS(5));
    }

    return 1;
}


DEVICE_DT_INST_DEFINE(
	0,
	&zmk_ps2_mouse_init,
	NULL,
	&zmk_ps2_mouse_data, &zmk_ps2_mouse_config,
	POST_KERNEL, 41,
	NULL
);

// #endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
