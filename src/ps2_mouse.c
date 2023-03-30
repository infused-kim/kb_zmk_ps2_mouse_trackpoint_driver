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

#define PS2_MOUSE_CMD_RESEND 0xfe
#define PS2_MOUSE_CMD_RESET 0xff
#define PS2_MOUSE_CMD_MODE_STREAM 0xea
#define PS2_MOUSE_CMD_GET_DEVICE_ID 0xf2
#define PS2_MOUSE_CMD_SET_SAMPLING_RATE 0xf3
#define PS2_MOUSE_CMD_ENABLE_REPORTING 0xf4
#define PS2_MOUSE_CMD_DISABLE_REPORTING 0xf5

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

    .button_l_is_held = false,
    .button_m_is_held = false,
    .button_r_is_held = false,

    .move_speed = {0},
    .scroll_speed = {0},

    // Data reporting is disabled on init
    .activity_reporting_on = false,

    // PS2 devices initialize with this rate
    .sampling_rate = 100,
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

    LOG_DBG("Received mouse movement data: 0x%x", byte);

    data->cmd_buffer[data->cmd_idx] = byte;

    if(data->cmd_idx == 0) {

        // Bit 3 of the first command byte should always be 1
        // If it is not, then we are definitely out of alignment.
        // So we ask the device to resend the entire 3-byte command
        // again.
        int alignment_bit = PS2_GPIO_GET_BIT(byte, 3);
        if(alignment_bit != 1) {
            LOG_ERR(
                "PS/2 Mouse cmd buffer is out of aligment. Requesting resend."
            );

            ps2_write(ps2_device, PS2_MOUSE_CMD_RESEND);
            data->cmd_idx = 0;
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

    LOG_DBG(
        "Got mouse activity cmd "
        "(mov_x=%d, mov_y=%d, o_x=%d, o_y=%d, scroll=%d, "
        "b_l=%d, b_m=%d, b_r=%d)",
        packet.mov_x, packet.mov_y, packet.overflow_x, packet.overflow_y,
        packet.scroll, packet.button_l, packet.button_m, packet.button_r
    );

    zmk_ps2_mouse_activity_move_mouse(packet.mov_x, packet.mov_y);
    zmk_ps2_mouse_activity_scroll(packet.scroll);
    zmk_ps2_mouse_activity_click_buttons(
        packet.button_l, packet.button_m, packet.button_r
    );
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

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_INVERT_X)
        mov_x = -mov_x;
    #endif /* IS_ENABLED(ZMK_MOUSE_PS2_INVERT_X) */

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_INVERT_Y)
        mov_y = -mov_y;
        LOG_DBG("Inverted mouse movement: %d", mov_y);
    #endif /* IS_ENABLED(ZMK_MOUSE_PS2_INVERT_Y) */

    data->move_speed.x += mov_x;

    // zmk expects y coordinates to be negative for up.
    data->move_speed.y += -mov_y;
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

    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_movement_update(data->move_speed.x, data->move_speed.y);

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
 * PS/2 Commands
 *
 * These functions only send the commands without any additional logic.
 */

int zmk_ps2_send_cmd_reset(const struct device *ps2_device) {
    int err;

    uint8_t cmd = PS2_MOUSE_CMD_RESET;
    LOG_DBG("Sending reset command: 0x%x", cmd);
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not reset: %d", err
        );
        return err;
    } else {
        LOG_DBG("Sent command succesfully: 0x%x", cmd);
    }

    return 0;
}

int zmk_ps2_send_cmd_data_reporting_on(const struct device *ps2_device) {
    int err;

    uint8_t cmd = PS2_MOUSE_CMD_ENABLE_REPORTING;
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not enable reporting for stream mode."
        );
        return err;
    }

    return 0;
}

int zmk_ps2_send_cmd_data_reporting_off(const struct device *ps2_device) {
    int err;

    uint8_t cmd = PS2_MOUSE_CMD_DISABLE_REPORTING;
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not disable reporting for stream mode."
        );
        return err;
    }

    return 0;
}

int zmk_ps2_send_cmd_set_sampling_rate(const struct device *ps2_device,
                                       uint8_t sampling_rate) {
    int err;

    uint8_t cmd = PS2_MOUSE_CMD_SET_SAMPLING_RATE;
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not send set sampling rate command: %d", err
        );
        return err;
    }
    err = ps2_write(ps2_device, sampling_rate);
    if(err) {
        LOG_ERR(
            "Could not send sampling rate value: %d", err
        );
        return err;
    }

    return 0;
}

int zmk_ps2_send_cmd_get_device_id(const struct device *ps2_device,
                                   uint8_t *resp_byte)
{
    int err;

    uint8_t cmd = PS2_MOUSE_CMD_GET_DEVICE_ID;
    err = ps2_write(ps2_device, cmd);
    if(err) {
        LOG_ERR(
            "Could not send set sampling rate command: %d", err
        );
        return err;
    }

    uint8_t read_val;
    err = ps2_read(ps2_device, &read_val);
    if(err) {
        return err;
    }

    *resp_byte = read_val;

    return 0;
}

/*
 * Helper functions
 *
 * These functions send commands and do necessary checks like disabling
 * data reporting and callbacks before sending the commands.
 */

int zmk_ps2_activity_reporting_enable(const struct device *ps2_device)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    if(data->activity_reporting_on == true) {
        return 0;
    }

    LOG_INF("Enabling mouse activity reporting...");

    int err = zmk_ps2_send_cmd_data_reporting_on(ps2_device);
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

int zmk_ps2_activity_reporting_disable(const struct device *ps2_device)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    if(data->activity_reporting_on == false) {
        return 0;
    }

    LOG_INF("Disabling mouse activity reporting...");

    int err = zmk_ps2_send_cmd_data_reporting_off(ps2_device);
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

int zmk_ps2_set_sampling_rate(const struct device *ps2_device,
                              uint8_t sampling_rate)
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

    bool prev_activity_reporting_on = data->activity_reporting_on;
    zmk_ps2_activity_reporting_disable(ps2_device);

    int err = zmk_ps2_send_cmd_set_sampling_rate(
        ps2_device, sampling_rate
    );

    if(err == 0) {
        data->sampling_rate = sampling_rate;
    }

    if(prev_activity_reporting_on == true) {
        zmk_ps2_activity_reporting_enable(ps2_device);
    }

    return err;
}

int zmk_ps2_set_sampling_rate_incr() {
    const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;
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

    return zmk_ps2_set_sampling_rate(config->ps2_device, next_rate);
}

int zmk_ps2_set_sampling_rate_decr() {
    const struct zmk_ps2_mouse_config *config = &zmk_ps2_mouse_config;
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
    return zmk_ps2_set_sampling_rate(config->ps2_device, prev_rate);
}

int zmk_ps2_set_packet_mode(const struct device *ps2_device,
                            zmk_ps2_mouse_packet_mode mode)
{
    struct zmk_ps2_mouse_data *data = &zmk_ps2_mouse_data;

    if(mode == PS2_MOUSE_PACKET_MODE_PS2_DEFAULT) {
        // Do nothing. Mouse devices enable this by
        // default.
        return 0;
    }

    bool prev_activity_reporting_on = data->activity_reporting_on;
    zmk_ps2_activity_reporting_disable(ps2_device);

    // Setting a mouse mode is a bit like using a cheat code
    // in a video game.
    // You have to send a specific sequence of sampling rates.
    if(mode == PS2_MOUSE_PACKET_MODE_SCROLL) {

        zmk_ps2_send_cmd_set_sampling_rate(
            ps2_device, 200
        );
        zmk_ps2_send_cmd_set_sampling_rate(
            ps2_device, 100
        );
        zmk_ps2_send_cmd_set_sampling_rate(
            ps2_device, 80
        );
    }

    // Scroll mouse + 5 buttons mode can be enabled with the
    // following sequence, but since I don't have a mouse to
    // test it, I am commenting it out for now.
    // else if(mode == PS2_MOUSE_PACKET_MODE_SCROLL_5_BUTTONS) {

    //     zmk_ps2_send_cmd_set_sampling_rate(
    //         ps2_device, 200
    //     );
    //     zmk_ps2_send_cmd_set_sampling_rate(
    //         ps2_device, 200
    //     );
    //     zmk_ps2_send_cmd_set_sampling_rate(
    //         ps2_device, 80
    //     );
    // }

    uint8_t device_id;
    int err = zmk_ps2_send_cmd_get_device_id(ps2_device, &device_id);
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
    zmk_ps2_set_sampling_rate(ps2_device, data->sampling_rate);

    if(prev_activity_reporting_on == true) {
        zmk_ps2_activity_reporting_enable(ps2_device);
    }

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

	// LOG_INF("Setting sample rate...");
    // zmk_ps2_set_sampling_rate(config->ps2_device, 200);

    #if IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_SCROLL)
        LOG_INF("Enabling scroll mode.");
        zmk_ps2_set_packet_mode(
            config->ps2_device, PS2_MOUSE_PACKET_MODE_SCROLL
        );
    #endif /* IS_ENABLED(CONFIG_ZMK_MOUSE_PS2_SCROLL) */

    // Configure read callback
	LOG_DBG("Configuring ps2 callback...");
    err = ps2_config(config->ps2_device, &zmk_ps2_mouse_activity_callback);
    if(err) {
        LOG_ERR("Could not configure ps2 interface: %d", err);
        return ;
    }

	LOG_INF("Enabling data reporting and ps2 callback...");
    err = zmk_ps2_activity_reporting_enable(config->ps2_device);
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
		(GPIO_OUTPUT_LOW)
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
    err = gpio_pin_set(data->rst_gpio, config->rst_pin, 1);
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
            zmk_ps2_send_cmd_reset(config->ps2_device);
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
