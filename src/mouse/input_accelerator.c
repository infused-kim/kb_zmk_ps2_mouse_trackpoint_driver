/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_accelerator

#include <stdlib.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/input_accelerator.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// The maximum amount of time in ms between mouse packets
// that we consider a continuous movement. Used for the
// calculation of movement speed and acc_rate curve
#define INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL 200

// The default rate to use for interval based speed calc
#define INPUT_ACCELERATOR_DEFAULT_SENDING_RATE 100

// The nearest sigmoid y value we treat as the beginning of the sigmoid curve
#define SIGMOID_FUNCTION_FROM_ORIGIN_EPSILON 0.01

struct vector2d {
    int32_t x;
    int32_t y;
};

struct vector2df {
    float x;
    float y;
};

struct input_accelerator_config {
    char *acc_curve_name;
    int acc_factor_base;
    int acc_factor_max;
    int acc_rate;
    int start_offset;
    int max_speed;
    int divisor;

    bool enable_interval_based_speed;
    int input_default_sending_rate;
};

struct input_accelerator_data {
    const struct device *dev;
    struct vector2d prev_mov;
    int64_t prev_mov_received_at;
    struct vector2df move_remainder;

    // Config settings converted to float values
    float acc_factor_base;
    float acc_factor_max;
    float acc_rate;
    int start_offset;
    int max_speed;
    int divisor;
};

float zmk_input_acc_get_interval_based_speed(float mov, int64_t mov_interval,
                                             int input_default_sending_rate);
float zmk_mouse_ps2_sigmoid_function_from_origin(float x, float limit, float slope, float epsilon);
float zmk_input_acc_get_acceleration_factor_none(float speed, float acc_factor_base,
                                                 float acc_factor_max, float acc_rate,
                                                 int start_offset, int max_speed,
                                                 bool should_round);
float zmk_input_acc_get_acceleration_factor_sigmoid(float speed, float acc_factor_base,
                                                    float acc_factor_max, float acc_rate,
                                                    int start_offset, int max_speed,
                                                    bool should_round);
float zmk_input_acc_get_acc_factor_with_max_speed(float acc_factor, float speed, int max_speed,
                                                  float acc_factor_base);
float zmk_input_acc_roundf_2d(float value);
bool zmk_input_acc_is_new_mov(float mov, float prev_mov, int64_t mov_interval);
int zmk_input_acc_get_direction_factor(int mov_val);

struct input_accelerator_result zmk_accelerate_input(const struct device *accelerator_dev,
                                                     int32_t x, int32_t y) {
    const struct input_accelerator_config *config = accelerator_dev->config;
    struct input_accelerator_data *data = accelerator_dev->data;

    // Rename variables for clarity
    int32_t orig_x = x;
    int32_t orig_y = y;

    int64_t mov_received_at = k_uptime_get();
    int64_t mov_interval = mov_received_at - data->prev_mov_received_at;

    float speed_x = (float)orig_x;
    float speed_y = (float)orig_y;

    // Check the comments in `zmk_input_acc_get_interval_based_speed` for why
    // this is necessary.
    if (config->enable_interval_based_speed) {
        speed_x = zmk_input_acc_get_interval_based_speed(speed_x, mov_interval,
                                                         config->input_default_sending_rate);
        speed_y = zmk_input_acc_get_interval_based_speed(speed_y, mov_interval,
                                                         config->input_default_sending_rate);
    }

    // We calculate the acceleration curve on the combined speed of
    // both the x and y movements. Otherwise we would skew the
    // movement direction and make diagonal movement way too
    // fast.
    float speed = sqrtf(speed_x * speed_x + speed_y * speed_y);

    // We use the speed to calculate the acceleration factor, which we
    // then apply to the actual movement data
    float acc_factor = 1;
    if (strcmp(config->acc_curve_name, "none") == 0) {
        acc_factor = zmk_input_acc_get_acceleration_factor_none(
            speed, data->acc_factor_base, data->acc_factor_max, data->acc_rate, data->start_offset,
            data->max_speed, true);
    } else if (strcmp(config->acc_curve_name, "sigmoid") == 0) {
        acc_factor = zmk_input_acc_get_acceleration_factor_sigmoid(
            speed, data->acc_factor_base, data->acc_factor_max, data->acc_rate, data->start_offset,
            data->max_speed, true);
    } else {
        LOG_ERR("Unknown acceleration curve: %s", config->acc_curve_name);
    }

    // Calculate the actual movement using the acceleration factor
    float acc_x = (float)speed_x * acc_factor;
    float acc_y = (float)speed_y * acc_factor;

    // Apply the divisor in case the movement needs to be significantly slowed down,
    // such as when the mouse movement is converted into scrolling.
    float with_divisor_x = acc_x / data->divisor;
    float with_divisor_y = acc_y / data->divisor;

    // If the previous packet was more than 200ms ago or the direction
    // of the movement changed (e.g. pos to neg), then we consider this
    // a new movement.
    bool is_new_mov_x = zmk_input_acc_is_new_mov(orig_x, data->prev_mov.x, mov_interval);
    bool is_new_mov_y = zmk_input_acc_is_new_mov(orig_y, data->prev_mov.y, mov_interval);

    // The zephyr input system uses integers. So, if we need to send
    // a fractional movement like 4.5 we round it down and then store the
    // remainder, which we then add to the next movement.
    float with_buffer_x = with_divisor_x;
    if (is_new_mov_x == false) {
        with_buffer_x += data->move_remainder.x;
    }
    float with_buffer_y = with_divisor_y;
    if (is_new_mov_y == false) {
        with_buffer_y += data->move_remainder.y;
    }

    // Make low acc_factor more responsive.
    //
    // When the acc_factor is set below 1, all movement is effectively
    // decelerated. Movement can start feeling sluggish, because it might take
    // a few movement packets until the decelerated values add up to 1 and the
    // cursor finally makes its initial move.
    //
    // So, here we make sure that whenever a new movement starts we send the
    // value 1 immediately and only decelerate future movements in the same
    // direction.
    if (is_new_mov_x == true && orig_x != 0 && fabsf(with_buffer_x) < 1) {
        int direction_factor_x = zmk_input_acc_get_direction_factor(orig_x);
        with_buffer_x = 1 * direction_factor_x;
    }
    if (is_new_mov_y == true && orig_y != 0 && fabsf(with_buffer_y) < 1) {
        int direction_factor_y = zmk_input_acc_get_direction_factor(orig_y);
        with_buffer_y = 1 * direction_factor_y;
    }

    // The zephyr input system uses whole numbers. So, we round down.
    int32_t to_send_x = (int32_t)with_buffer_x;
    int32_t to_send_y = (int32_t)with_buffer_y;

    // Store remainder in buffer
    data->move_remainder.x = with_buffer_x - (float)to_send_x;
    data->move_remainder.y = with_buffer_y - (float)to_send_y;

#if IS_ENABLED(CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED)

    int acc_factor_pct = (int)(acc_factor * 100);

    LOG_INF("Input Accel %s - "
            "Interval: %4lld ms; "
            "Received X: %+3d, Y: %+3d; "
            "Speed: %+5.2f; "
            "Acceleration X: %+5.2f, Y: %+5.2f (%3d%%); "
            "Divisor X: %+5.2f, Y: %+5.2f (/ %3d); "
            "With Buffer X: %+5.2f, Y: %+5.2f; "
            "Final X: %+3d, Y: %+3d; "
            "New Buffer X: %+4.2f, Y: %+4.2f; ",
            config->acc_curve_name, mov_interval, orig_x, orig_y, speed, acc_x, acc_y,
            acc_factor_pct, with_divisor_x, with_divisor_y, data->divisor, with_buffer_x,
            with_buffer_y, to_send_x, to_send_y, data->move_remainder.x, data->move_remainder.y);

#endif // CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED

    data->prev_mov.x = orig_x;
    data->prev_mov.y = orig_y;
    data->prev_mov_received_at = mov_received_at;

    struct input_accelerator_result ret = {.x = to_send_x, .y = to_send_y};

    return ret;
}

// Calculates the speed of a movement for input devices that don't have a
// consistent sending rate, such as PS/2 mice and Trackpoints.
//
// Devices, with consistent reporting rates, send data like this:
//   -  0ms: 5
//   - 10ms: 3
//   - 20ms: 1
//   - 30ms: 0
//   - 40ms: 0
//   - 50ms: 0
//   - 60ms: 1
//
// In contrast, PS/2 devices report slow movements by slowing down the speed
// at which the movement reports are sent, like this:
//   -  0ms: 5
//   - 10ms: 3
//   - 20ms: 1
//   - 60ms: 1
//
// Even though both movements at 20ms and 60ms have the value 1, they are not
// equal in speed. The 20ms movement traversed 1 unit in 10ms and has a speed
// of 0.1 units/ms. On the other hand the movement at 60ms has traversed 1
// unit in 40ms and has a speed of 0.025 units/ms.
//
// This function calculates the correct speed based on the actual sending
// interval.
float zmk_input_acc_get_interval_based_speed(float mov, int64_t mov_interval,
                                             int input_default_sending_rate) {

    if (input_default_sending_rate <= 0) {
        LOG_ERR("The configured input_default_sending_rate is invalid: %d. Using %d as the new "
                "rate, which might be incorrect.",
                input_default_sending_rate, INPUT_ACCELERATOR_DEFAULT_SENDING_RATE);
        input_default_sending_rate = INPUT_ACCELERATOR_DEFAULT_SENDING_RATE;
    }

    int64_t default_interval = 1000 / input_default_sending_rate;

    int64_t interval = default_interval;
    if (mov_interval != 0 && mov_interval <= INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL) {
        interval = mov_interval;
    }

    float speed = mov / interval * default_interval;

    return speed;
}

// Sets acc_factor to acc_factor_base and if necessary limits to max speed
float zmk_input_acc_get_acceleration_factor_none(float speed, float acc_factor_base,
                                                 float acc_factor_max, float acc_rate,
                                                 int start_offset, int max_speed,
                                                 bool should_round) {

    float acc_factor = acc_factor_base;

    acc_factor =
        zmk_input_acc_get_acc_factor_with_max_speed(acc_factor, speed, max_speed, acc_factor_base);

    if (should_round) {
        // Rounded to two decimal places
        acc_factor = zmk_input_acc_roundf_2d(acc_factor);
    }

    return acc_factor;
}

float zmk_input_acc_get_acceleration_factor_sigmoid(float speed, float acc_factor_base,
                                                    float acc_factor_max, float acc_rate,
                                                    int start_offset, int max_speed,
                                                    bool should_round) {

    float speed_abs = fabsf(speed);
    float speed_offset = speed_abs - start_offset;

    float sigmoid_limit = acc_factor_max - acc_factor_base;

    float acc_factor = zmk_mouse_ps2_sigmoid_function_from_origin(
        speed_offset, sigmoid_limit, acc_rate, SIGMOID_FUNCTION_FROM_ORIGIN_EPSILON);

    acc_factor += acc_factor_base;

    acc_factor =
        zmk_input_acc_get_acc_factor_with_max_speed(acc_factor, speed, max_speed, acc_factor_base);

    if (should_round) {
        // Rounded to two decimal places
        acc_factor = zmk_input_acc_roundf_2d(acc_factor);
    }

    return acc_factor;
}

// Standard sigmoid curve function
float zmk_mouse_ps2_sigmoid_function(float x, float limit, float slope) {
    return limit / (1 + expf(-slope * x));
}

// Adjusts a standard sigmoid function so that its rise starts at x=0
// instead of the negative half of the graph.
float zmk_mouse_ps2_sigmoid_function_from_origin(float x, float limit, float slope, float epsilon) {

    // Make sure the graph starts with y=0
    if (x <= 0) {
        return 0;
    }

    // Sigmoid approaches 0, but never really reaches it.
    // Here we calculate the x value where the sigmoid is epsilon, such as
    // 0.01.
    float start_at = -logf(limit / epsilon) / slope;
    float offset_x = x + start_at;

    // We find where the sigmoid function starts within 0.01 of 0,
    // but it will be something like y=0.013. So we lower the y
    // values by that amount to make sure the curve transitions
    // smoothly from the offset y value
    float offset_y = zmk_mouse_ps2_sigmoid_function(start_at, limit, slope);

    float y = zmk_mouse_ps2_sigmoid_function(offset_x, limit, slope) - offset_y;

    return y;
}

float zmk_input_acc_get_acc_factor_with_max_speed(float acc_factor, float speed, int max_speed,
                                                  float acc_factor_base) {

    float capped_acc_factor = acc_factor;

    if (max_speed <= 0) {
        return acc_factor;
    }

    float speed_abs = fabsf(speed);
    float sensitive_speed = speed_abs * acc_factor;
    if (sensitive_speed > max_speed) {
        if (speed_abs == 0) {
            capped_acc_factor = acc_factor_base;
        } else {
            capped_acc_factor = max_speed / speed_abs;
        }
    }

    return acc_factor;
}

// Round float to 2 digits
float zmk_input_acc_roundf_2d(float value) { return roundf(value * 100) / 100; }

// Returns 1 if positive and 0 if negative. Can be used to multiply another movement and get the
// correct direction.
int zmk_input_acc_get_direction_factor(int mov_val) {
    if (mov_val == 0) {
        return 0;
    } else {
        return abs(mov_val) / mov_val;
    }
}

// Returns 1 if the direction of the current and prev movement are not the
// same.
bool zmk_input_acc_direction_changed(float mov, float prev_mov) {

    if (mov == 0 && prev_mov == 0) {
        return false;
    }

    // If one value is negative and the other is positive then the result will
    // be negative, therefore direction did change.
    // If the result is 0 and the prev statement was not true, then it means
    // one of the directions was 0 and it changed
    if (mov * prev_mov <= 0) {
        return true;
    }

    return false;
}

// Returns true if either the direction of the mov changed or more than 200ms
// passed since the previous move.
bool zmk_input_acc_is_new_mov(float mov, float prev_mov, int64_t mov_interval) {

    bool is_new_direction = zmk_input_acc_direction_changed(mov, prev_mov);

    if (is_new_direction || mov_interval > INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL) {
        return true;
    }

    return false;
}

#define IL_INST(n)                                                                                 \
    static const struct input_accelerator_config config_##n = {                                    \
        .acc_curve_name = DT_INST_PROP(n, acceleration_curve),                                     \
        .acc_factor_base = DT_INST_PROP(n, acceleration_factor_base),                              \
        .acc_factor_max = DT_INST_PROP(n, acceleration_factor_max),                                \
        .acc_rate = DT_INST_PROP(n, acceleration_factor_rate),                                     \
        .start_offset = DT_INST_PROP(n, acceleration_start_offset),                                \
        .max_speed = DT_INST_PROP(n, max_speed),                                                   \
        .divisor = DT_INST_PROP(n, divisor),                                                       \
        .enable_interval_based_speed = DT_INST_PROP(n, enable_interval_based_speed),               \
        .input_default_sending_rate = DT_INST_PROP(n, input_default_sending_rate),                 \
    };                                                                                             \
    static struct input_accelerator_data data_##n = {                                              \
        .dev = DEVICE_DT_INST_GET(n),                                                              \
        .prev_mov = {x : 0, y : 0},                                                                \
        .prev_mov_received_at = 0,                                                                 \
        .move_remainder = {x : 0, y : 0},                                                          \
    };                                                                                             \
                                                                                                   \
    static int zmk_input_accelerator_init_##n(const struct device *dev) {                          \
        struct input_accelerator_data *data = dev->data;                                           \
        const struct input_accelerator_config *config = dev->config;                               \
                                                                                                   \
        LOG_DBG("Initialized input accelerator: %s", dev->name);                                   \
                                                                                                   \
        data->acc_factor_base = (float)config->acc_factor_base / 100.0;                            \
        data->acc_factor_max = (float)config->acc_factor_max / 100.0;                              \
        data->acc_rate = (float)config->acc_rate / 100.0;                                          \
        data->start_offset = config->start_offset;                                                 \
        data->max_speed = config->max_speed;                                                       \
        data->divisor = config->divisor;                                                           \
                                                                                                   \
        return 0;                                                                                  \
    }                                                                                              \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &zmk_input_accelerator_init_##n, NULL, &data_##n, &config_##n,        \
                          POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IL_INST)
