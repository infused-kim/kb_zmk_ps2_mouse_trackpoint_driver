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

#include <drivers/input_processor.h>
#include <zmk/input_accelerator.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// The maximum amount of time in ms between mouse packets
// that we consider a continuous movement. Used for the
// calculation of movement speed and acc_rate curve
#define INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL 200

// The default rate to use for interval based speed calc
#define INPUT_ACCELERATOR_DEFAULT_SENDING_RATE 100

struct vector2d {
    int32_t x;
    int32_t y;
};

struct vector2df {
    float x;
    float y;
};

// Same order as the `acceleration-curve` enum in the binding
enum input_accelerator_curve {
    INPUT_ACCELERATOR_CURVE_NONE,
    INPUT_ACCELERATOR_CURVE_SIGMOID,
};

struct input_accelerator_config {
    enum input_accelerator_curve acc_curve;
    char *acc_curve_name;
    int acc_factor_base;
    int acc_factor_max;
    int acc_rate;
    int start_offset;
    int max_speed;

    bool enable_interval_based_speed;
    int input_default_sending_rate;
};

// Intermediate values of `zmk_input_acc_accelerate_axis`, kept for logging
struct input_acc_axis_result {
    float acc;
    float with_buffer;
    int32_t to_send;
};

// One axis of the report that the input processor is currently receiving
struct input_acc_processor_axis {
    int32_t mov;
    float speed;
    struct input_acc_axis_result res;
};

// The report that the input processor is currently receiving. A report
// consists of all events up to and including the event with the sync flag.
struct input_acc_processor_report {
    bool started;
    int64_t received_at;
    int64_t mov_interval;
    struct input_acc_processor_axis x;
    struct input_acc_processor_axis y;
};

struct input_accelerator_data {
    // The last non-zero movement of each axis, used to detect direction changes
    struct vector2d prev_mov;
    int64_t prev_mov_received_at;
    struct vector2df move_remainder;

    // Only used by the input processor
    float processor_acc_factor;
    struct input_acc_processor_report processor_report;

    // For logging of move summary
    int64_t continuous_move_start_time;
    float continuous_move_max_speed;
    struct k_work_delayable continuous_move_timeout_delay;

    // Config settings converted to float values
    float acc_factor_base;
    float acc_factor_max;
    float acc_rate;

    // Calculated once from the config, see
    // `zmk_input_acc_get_sigmoid_start_offset_addition`
    float sigmoid_start_offset_addition;
};

static void zmk_input_acc_processor_start_report(const struct input_accelerator_config *config,
                                                 struct input_accelerator_data *data);
static void zmk_input_acc_processor_accelerate_event(const struct input_accelerator_config *config,
                                                     struct input_accelerator_data *data,
                                                     struct input_event *event);
static void zmk_input_acc_processor_finish_report(const struct input_accelerator_config *config,
                                                  struct input_accelerator_data *data);
static float zmk_input_acc_get_axis_speed(const struct input_accelerator_config *config,
                                          int32_t mov, int64_t mov_interval);
static float zmk_input_acc_get_acc_factor(const struct input_accelerator_config *config,
                                          const struct input_accelerator_data *data, float speed);
static struct input_acc_axis_result
zmk_input_acc_accelerate_axis(const struct input_accelerator_data *data, int32_t orig,
                              int32_t prev_orig, float speed, float acc_factor,
                              int64_t mov_interval, float *remainder);
static void zmk_input_acc_log_move(const struct input_accelerator_config *config,
                                   struct input_accelerator_data *data, int64_t mov_received_at,
                                   int64_t mov_interval, int32_t orig_x, int32_t orig_y,
                                   float speed, float acc_factor,
                                   const struct input_acc_axis_result *res_x,
                                   const struct input_acc_axis_result *res_y);
static void zmk_input_acc_update_prev_mov(struct input_accelerator_data *data, int32_t mov_x,
                                          int32_t mov_y);
static float zmk_input_acc_get_interval_based_speed(float mov, int64_t mov_interval,
                                                    int input_default_sending_rate);
static float zmk_input_acc_sigmoid_function_with_offset(float x, float limit, float acc_rate,
                                                        float start_offset,
                                                        float start_offset_addition);
static float zmk_input_acc_get_acceleration_factor_none(float speed, float acc_factor_base,
                                                        int max_speed);
static float zmk_input_acc_get_acceleration_factor_sigmoid(float speed, float acc_factor_base,
                                                           float acc_factor_max, float acc_rate,
                                                           int start_offset,
                                                           float start_offset_addition,
                                                           int max_speed);
static float zmk_input_acc_get_acc_factor_with_max_speed(float acc_factor, float speed,
                                                         int max_speed);
static bool zmk_input_acc_is_new_mov(float mov, float prev_mov, int64_t mov_interval);
static int zmk_input_acc_get_direction_factor(int mov_val);

struct input_accelerator_result zmk_accelerate_input(const struct device *accelerator_dev,
                                                     int32_t x, int32_t y) {
    const struct input_accelerator_config *config = accelerator_dev->config;
    struct input_accelerator_data *data = accelerator_dev->data;

    // Rename variables for clarity
    int32_t orig_x = x;
    int32_t orig_y = y;

    int64_t mov_received_at = k_uptime_get();
    int64_t mov_interval = mov_received_at - data->prev_mov_received_at;

    float speed_x = zmk_input_acc_get_axis_speed(config, orig_x, mov_interval);
    float speed_y = zmk_input_acc_get_axis_speed(config, orig_y, mov_interval);

    // We calculate the acceleration curve on the combined speed of
    // both the x and y movements. Otherwise we would skew the
    // movement direction and make diagonal movement way too
    // fast.
    float speed = sqrtf(speed_x * speed_x + speed_y * speed_y);

    // We use the speed to calculate the acceleration factor, which we
    // then apply to the actual movement data
    float acc_factor = zmk_input_acc_get_acc_factor(config, data, speed);

    struct input_acc_axis_result res_x = zmk_input_acc_accelerate_axis(
        data, orig_x, data->prev_mov.x, speed_x, acc_factor, mov_interval, &data->move_remainder.x);
    struct input_acc_axis_result res_y = zmk_input_acc_accelerate_axis(
        data, orig_y, data->prev_mov.y, speed_y, acc_factor, mov_interval, &data->move_remainder.y);

    zmk_input_acc_log_move(config, data, mov_received_at, mov_interval, orig_x, orig_y, speed,
                           acc_factor, &res_x, &res_y);

    zmk_input_acc_update_prev_mov(data, orig_x, orig_y);
    data->prev_mov_received_at = mov_received_at;

    struct input_accelerator_result ret = {.x = res_x.to_send, .y = res_y.to_send};

    return ret;
}

// Input processor API.
//
// Unlike `zmk_accelerate_input`, which receives the x and y movement of a
// report together, an input processor receives one event at a time and can't
// hold back the x event until the y event arrives. So, each axis is
// accelerated as soon as it arrives, using the acceleration factor that was
// calculated from the combined speed of the previous report.
//
// Shares its state with `zmk_accelerate_input`, so a single accelerator node
// must not be used as input processor and `move-accelerator` at the same time.
static int zmk_input_acc_handle_event(const struct device *dev, struct input_event *event,
                                      uint32_t param1, uint32_t param2,
                                      struct zmk_input_processor_state *state) {
    const struct input_accelerator_config *config = dev->config;
    struct input_accelerator_data *data = dev->data;
    struct input_acc_processor_report *report = &data->processor_report;

    if (event->type == INPUT_EV_REL && (event->code == INPUT_REL_X || event->code == INPUT_REL_Y)) {
        if (!report->started) {
            zmk_input_acc_processor_start_report(config, data);
        }

        zmk_input_acc_processor_accelerate_event(config, data, event);
    }

    if (event->sync && report->started) {
        zmk_input_acc_processor_finish_report(config, data);
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static void zmk_input_acc_processor_start_report(const struct input_accelerator_config *config,
                                                 struct input_accelerator_data *data) {
    struct input_acc_processor_report *report = &data->processor_report;

    report->started = true;
    report->received_at = k_uptime_get();
    report->mov_interval = report->received_at - data->prev_mov_received_at;

    // The previous report's acceleration factor doesn't apply to a new movement
    if (report->mov_interval > INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL) {
        data->processor_acc_factor = zmk_input_acc_get_acc_factor(config, data, 0);
    }
}

// Accelerates the movement of a single REL_X or REL_Y event and stores the
// axis values in the current report.
static void zmk_input_acc_processor_accelerate_event(const struct input_accelerator_config *config,
                                                     struct input_accelerator_data *data,
                                                     struct input_event *event) {
    struct input_acc_processor_report *report = &data->processor_report;

    struct input_acc_processor_axis *axis;
    int32_t prev_mov;
    float *remainder;

    if (event->code == INPUT_REL_X) {
        axis = &report->x;
        prev_mov = data->prev_mov.x;
        remainder = &data->move_remainder.x;
    } else {
        axis = &report->y;
        prev_mov = data->prev_mov.y;
        remainder = &data->move_remainder.y;
    }

    axis->mov = event->value;
    axis->speed = zmk_input_acc_get_axis_speed(config, axis->mov, report->mov_interval);
    axis->res =
        zmk_input_acc_accelerate_axis(data, axis->mov, prev_mov, axis->speed,
                                      data->processor_acc_factor, report->mov_interval, remainder);

    event->value = axis->res.to_send;
}

// Calculates the acceleration factor for the next report from the combined
// speed of both axes, the same way `zmk_accelerate_input` does. An axis that
// wasn't part of the report counts as no movement.
static void zmk_input_acc_processor_finish_report(const struct input_accelerator_config *config,
                                                  struct input_accelerator_data *data) {
    struct input_acc_processor_report *report = &data->processor_report;

    float speed = sqrtf(report->x.speed * report->x.speed + report->y.speed * report->y.speed);

    zmk_input_acc_log_move(config, data, report->received_at, report->mov_interval, report->x.mov,
                           report->y.mov, speed, data->processor_acc_factor, &report->x.res,
                           &report->y.res);

    data->processor_acc_factor = zmk_input_acc_get_acc_factor(config, data, speed);

    zmk_input_acc_update_prev_mov(data, report->x.mov, report->y.mov);
    data->prev_mov_received_at = report->received_at;

    // Start the next report from scratch: Sets `report->started = false` and counts both
    // axes as unmoved until their events arrive.
    *report = (struct input_acc_processor_report){0};
}

// Speed of a single axis.
static float zmk_input_acc_get_axis_speed(const struct input_accelerator_config *config,
                                          int32_t mov, int64_t mov_interval) {
    float speed = (float)mov;

    // Check the comments in `zmk_input_acc_get_interval_based_speed` for why
    // this is necessary.
    if (config->enable_interval_based_speed) {
        speed = zmk_input_acc_get_interval_based_speed(speed, mov_interval,
                                                       config->input_default_sending_rate);
    }

    return speed;
}

// Acceleration factor for a speed, using the configured curve.
static float zmk_input_acc_get_acc_factor(const struct input_accelerator_config *config,
                                          const struct input_accelerator_data *data, float speed) {
    // Read the curve as enum index from the devicetree instead of comparing
    // the curve name with strcmp on every report. The binding's enum already
    // guarantees a valid value.
    switch (config->acc_curve) {
    case INPUT_ACCELERATOR_CURVE_SIGMOID:
        return zmk_input_acc_get_acceleration_factor_sigmoid(
            speed, data->acc_factor_base, data->acc_factor_max, data->acc_rate,
            config->start_offset, data->sigmoid_start_offset_addition, config->max_speed);
    case INPUT_ACCELERATOR_CURVE_NONE:
    default:
        return zmk_input_acc_get_acceleration_factor_none(speed, data->acc_factor_base,
                                                          config->max_speed);
    }
}

// Applies the acceleration factor to a single axis and updates its remainder.
static struct input_acc_axis_result
zmk_input_acc_accelerate_axis(const struct input_accelerator_data *data, int32_t orig,
                              int32_t prev_orig, float speed, float acc_factor,
                              int64_t mov_interval, float *remainder) {
    struct input_acc_axis_result res;

    // Calculate the actual movement using the acceleration factor
    res.acc = speed * acc_factor;

    // If the previous packet was more than 200ms ago or the direction
    // of the movement changed (e.g. pos to neg), then we consider this
    // a new movement.
    bool is_new_mov = zmk_input_acc_is_new_mov(orig, prev_orig, mov_interval);

    // The zephyr input system uses integers. So, if we need to send
    // a fractional movement like 4.5 we round it down and then store the
    // remainder, which we then add to the next movement.
    res.with_buffer = res.acc;
    if (is_new_mov == false) {
        res.with_buffer += *remainder;
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
    if (is_new_mov == true && orig != 0 && fabsf(res.with_buffer) < 1) {
        int direction_factor = zmk_input_acc_get_direction_factor(orig);
        res.with_buffer = 1 * direction_factor;
    }

    // The zephyr input system uses whole numbers. So, we round down.
    res.to_send = (int32_t)res.with_buffer;

    // Store remainder in buffer
    *remainder = res.with_buffer - (float)res.to_send;

    return res;
}

// Logs each accelerated movement and tracks the continuous move for the
// summary logged by `zmk_input_accelerator_continuous_move_timeout_work`.
static void zmk_input_acc_log_move(const struct input_accelerator_config *config,
                                   struct input_accelerator_data *data, int64_t mov_received_at,
                                   int64_t mov_interval, int32_t orig_x, int32_t orig_y,
                                   float speed, float acc_factor,
                                   const struct input_acc_axis_result *res_x,
                                   const struct input_acc_axis_result *res_y) {
#if IS_ENABLED(CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED)

    if (data->continuous_move_start_time == 0) {
        data->continuous_move_start_time = mov_received_at;
    }

    if (data->continuous_move_max_speed < speed) {
        data->continuous_move_max_speed = speed;
    }

    // If no further movement packets come in, then this work will end the
    // continuous move.
    k_work_reschedule(&data->continuous_move_timeout_delay,
                      K_MSEC(INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL));

    int acc_factor_pct = (int)(acc_factor * 100);

    LOG_INF("Input Accel %s - "
            "Interval: %4lld ms; "
            "Received X: %+3d, Y: %+3d; "
            "Speed: %5.2f; "
            "Acceleration X: %+5.2f, Y: %+5.2f (%3d%%); "
            "With Buffer X: %+5.2f, Y: %+5.2f; "
            "Final X: %+3d, Y: %+3d; "
            "New Buffer X: %+4.2f, Y: %+4.2f; ",
            config->acc_curve_name, mov_interval, orig_x, orig_y, speed, res_x->acc, res_y->acc,
            acc_factor_pct, res_x->with_buffer, res_y->with_buffer, res_x->to_send, res_y->to_send,
            data->move_remainder.x, data->move_remainder.y);

#endif // CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED
}

// Remembers the direction of each axis for `zmk_input_acc_is_new_mov`. An
// axis without movement keeps its previous direction, so that briefly
// pausing an axis, e.g. during a mostly horizontal movement, doesn't count as
// a new movement.
static void zmk_input_acc_update_prev_mov(struct input_accelerator_data *data, int32_t mov_x,
                                          int32_t mov_y) {
    if (mov_x != 0) {
        data->prev_mov.x = mov_x;
    }

    if (mov_y != 0) {
        data->prev_mov.y = mov_y;
    }
}

#if IS_ENABLED(CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED)

static void zmk_input_accelerator_continuous_move_timeout_work(struct k_work *item) {

    struct k_work_delayable *d_work = k_work_delayable_from_work(item);

    struct input_accelerator_data *data =
        CONTAINER_OF(d_work, struct input_accelerator_data, continuous_move_timeout_delay);

    int64_t current_time = k_uptime_get();
    int64_t continuous_move_period = current_time - data->continuous_move_start_time -
                                     INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL;

    LOG_INF(
        "Accelerated continuous mouse move ended after %lldms with unaccelerated max speed: %4.2f",
        continuous_move_period, data->continuous_move_max_speed);

#if !IS_ENABLED(CONFIG_CBPRINTF_COMPLETE)
    LOG_WRN("Enable CONFIG_CBPRINTF_COMPLETE=y to see proper speed values instead of *float*.");
#endif // CONFIG_CBPRINTF_COMPLETE

    data->continuous_move_start_time = 0;
    data->continuous_move_max_speed = 0;
}

#endif // CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED

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
static float zmk_input_acc_get_interval_based_speed(float mov, int64_t mov_interval,
                                                    int input_default_sending_rate) {

    if (input_default_sending_rate <= 0) {
        LOG_ERR("The configured input_default_sending_rate is invalid: %d. Using %d as the new "
                "rate, which might be incorrect.",
                input_default_sending_rate, INPUT_ACCELERATOR_DEFAULT_SENDING_RATE);
        input_default_sending_rate = INPUT_ACCELERATOR_DEFAULT_SENDING_RATE;
    }

    float default_interval = 1000.0f / input_default_sending_rate;

    float interval = default_interval;
    if (mov_interval != 0 && mov_interval <= INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL) {
        interval = mov_interval;
    }

    float speed = mov / interval * default_interval;

    return speed;
}

// Sets acc_factor to acc_factor_base and if necessary limits to max speed
static float zmk_input_acc_get_acceleration_factor_none(float speed, float acc_factor_base,
                                                        int max_speed) {

    float acc_factor = acc_factor_base;

    acc_factor = zmk_input_acc_get_acc_factor_with_max_speed(acc_factor, speed, max_speed);

    return acc_factor;
}

static float zmk_input_acc_get_acceleration_factor_sigmoid(float speed, float acc_factor_base,
                                                           float acc_factor_max, float acc_rate,
                                                           int start_offset,
                                                           float start_offset_addition,
                                                           int max_speed) {

    float speed_abs = fabsf(speed);
    float sigmoid_limit = acc_factor_max - acc_factor_base;

    float acc_factor = zmk_input_acc_sigmoid_function_with_offset(
        speed_abs, sigmoid_limit, acc_rate, start_offset, start_offset_addition);

    acc_factor = acc_factor + acc_factor_base;

    acc_factor = zmk_input_acc_get_acc_factor_with_max_speed(acc_factor, speed, max_speed);

    return acc_factor;
}

// Standard sigmoid curve function
static float zmk_input_acc_sigmoid_function(float x, float limit, float acc_rate) {
    return limit / (1 + expf(-acc_rate * x));
}

static float zmk_input_acc_sigmoid_function_inverse(float y, float limit, float acc_rate) {

    if (y <= 0) {
        y = 0.001;
    }

    if (y >= limit) {
        y = limit - 0.001;
    }

    return -1 / acc_rate * logf((limit - y) / y);
}

// Shifts the sigmoid curve to the right, so that it starts rising at the
// start offset. See `zmk_input_acc_get_sigmoid_start_offset_addition`.
static float zmk_input_acc_sigmoid_function_with_offset(float x, float limit, float acc_rate,
                                                        float start_offset,
                                                        float start_offset_addition) {

    float final_offset = x - start_offset + start_offset_addition;

    float acc_factor = zmk_input_acc_sigmoid_function(final_offset, limit, acc_rate);

    return acc_factor;
}

// The sigmoid curve is always centered at 0 (meaning x0 = limit/2). We
// can shift the curve to the right by subtracting x values.
//
// We want the sigmoid curve to start with a specific y value after the
// offset. We use 2.5% of the total acceleration-increase-range (the limit)
// for that.
//
// This way, we position the curve in such a way that we can see a meaningful,
// yet gradual, increase of the acceleration curve after the offset.
//
// So, we determine the x value where the sigmoid function returns that
// increase. And then we shift the curve to the right so that the curve
// has the 2.5% increase at exactly the offset x value.
//
// The result only depends on the config, so it's calculated once at init.
static float zmk_input_acc_get_sigmoid_start_offset_addition(float limit, float acc_rate) {
    float start_offset_y_value = limit * 0.025;

    return zmk_input_acc_sigmoid_function_inverse(start_offset_y_value, limit, acc_rate);
}

static float zmk_input_acc_get_acc_factor_with_max_speed(float acc_factor, float speed,
                                                         int max_speed) {

    if (max_speed <= 0) {
        return acc_factor;
    }

    // `speed_abs` can't be 0 here, because the accelerated speed would be 0
    // as well and can't exceed a positive `max_speed`.
    float speed_abs = fabsf(speed);
    float sensitive_speed = speed_abs * acc_factor;
    if (sensitive_speed > max_speed) {
        return max_speed / speed_abs;
    }

    return acc_factor;
}

// Returns 1 if positive and 0 if negative. Can be used to multiply another movement and get the
// correct direction.
static int zmk_input_acc_get_direction_factor(int mov_val) {
    if (mov_val == 0) {
        return 0;
    } else {
        return abs(mov_val) / mov_val;
    }
}

// Returns true if the current and previous movement go in opposite
// directions (one is positive and the other negative). A movement of 0 has no
// direction, so starting or stopping an axis doesn't count as a change.
static bool zmk_input_acc_direction_changed(float mov, float prev_mov) {
    return mov * prev_mov < 0;
}

// Returns true if either the direction of the mov changed or more than 200ms
// passed since the previous move.
static bool zmk_input_acc_is_new_mov(float mov, float prev_mov, int64_t mov_interval) {

    bool is_new_direction = zmk_input_acc_direction_changed(mov, prev_mov);

    if (is_new_direction || mov_interval > INPUT_ACCELERATOR_CONTINUOUS_MOVE_MAX_INTERVAL) {
        return true;
    }

    return false;
}

static int zmk_input_accelerator_init(const struct device *dev,
                                      const struct input_accelerator_config *config,
                                      struct input_accelerator_data *data) {

    LOG_DBG("Initialized input accelerator: %s", dev->name);

    data->acc_factor_base = (float)config->acc_factor_base / 100.0;
    data->acc_factor_max = (float)config->acc_factor_max / 100.0;
    data->acc_rate = (float)config->acc_rate / 100.0;

    if (config->acc_curve == INPUT_ACCELERATOR_CURVE_SIGMOID) {
        data->sigmoid_start_offset_addition = zmk_input_acc_get_sigmoid_start_offset_addition(
            data->acc_factor_max - data->acc_factor_base, data->acc_rate);
    }

#if IS_ENABLED(CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED)
    k_work_init_delayable(&data->continuous_move_timeout_delay,
                          zmk_input_accelerator_continuous_move_timeout_work);
#endif // CONFIG_ZMK_INPUT_ACCELERATOR_LOG_ENABLED

    return 0;
}

static const struct zmk_input_processor_driver_api zmk_input_accelerator_processor_api = {
    .handle_event = zmk_input_acc_handle_event,
};

#define IL_INST(n)                                                                                 \
    static const struct input_accelerator_config config_##n = {                                    \
        .acc_curve = DT_INST_ENUM_IDX(n, acceleration_curve),                                      \
        .acc_curve_name = DT_INST_PROP(n, acceleration_curve),                                     \
        .acc_factor_base = DT_INST_PROP(n, acceleration_factor_base),                              \
        .acc_factor_max = DT_INST_PROP(n, acceleration_factor_max),                                \
        .acc_rate = DT_INST_PROP(n, acceleration_factor_rate),                                     \
        .start_offset = DT_INST_PROP(n, acceleration_start_offset),                                \
        .max_speed = DT_INST_PROP(n, max_speed),                                                   \
        .enable_interval_based_speed = DT_INST_PROP(n, enable_interval_based_speed),               \
        .input_default_sending_rate = DT_INST_PROP(n, input_default_sending_rate),                 \
    };                                                                                             \
    static struct input_accelerator_data data_##n = {                                              \
        .prev_mov = {x : 0, y : 0},                                                                \
        .prev_mov_received_at = 0,                                                                 \
        .move_remainder = {x : 0, y : 0},                                                          \
        .continuous_move_start_time = 0,                                                           \
        .continuous_move_max_speed = 0,                                                            \
    };                                                                                             \
                                                                                                   \
    static int zmk_input_accelerator_init_##n(const struct device *dev) {                          \
        struct input_accelerator_data *data = dev->data;                                           \
        const struct input_accelerator_config *config = dev->config;                               \
                                                                                                   \
        return zmk_input_accelerator_init(dev, config, data);                                      \
    }                                                                                              \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &zmk_input_accelerator_init_##n, NULL, &data_##n, &config_##n,        \
                          POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,                           \
                          &zmk_input_accelerator_processor_api);

DT_INST_FOREACH_STATUS_OKAY(IL_INST)
