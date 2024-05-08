"""
Python functions that generate plots for acceleration curves. To be used with
the jupyter notebook mouse_acceleration_models.

Requirements:
    - pip install pandas matplotlib
"""

import math
import pandas as pd
import matplotlib.pyplot as plt


# Default value for functions
D_EPSILON = 0.01
D_MAX_SPEED = 0
D_OFFSET = 0
D_SHOULD_ROUND = False
D_X_TITLE = 'x'
D_ACC_FACTOR_TITLE = 'acc_factor'
D_OUTPUT_TITLE = 'output'
D_ZOOM_MAX_X = 30


def sigmoid_function(x, limit, slope):
    '''
    Standard sigmoid function
    '''

    return (limit) / (1 + math.exp(-slope * x))


def sigmoid_function_from_origin(x, limit, slope, epsilon=D_EPSILON):
    '''
    Adjusts a standard sigmoid function so that its rise starts at x=0
    instead of the negative half of the graph.
    '''

    # Make sure the graph starts with y=0
    if x <= 0:
        return 0

    # Sigmoid approaches 0, but never really reaches it.
    # Here we calculate the x value where the sigmoid is epsilon, such as 0.01.
    start_at = -math.log((limit) / epsilon) / slope

    # Then we shift the whole range by that amount to make the curve start
    # to rise at x=0
    offset_x = x + start_at

    # We find where the sigmoid function starts within 0.01 of 0,
    # but it will be something like y=0.013. So we lower the y
    # values by that amount to make sure the curve transitions
    # smoothly from the offset y value
    offset_y = sigmoid_function(start_at, limit, slope)

    y = sigmoid_function(offset_x, limit, slope) - offset_y

    return y


def get_acc_factor_with_max_speed(acc_factor,
                                  speed,
                                  max_speed,
                                  acc_factor_base):
    capped_acc_factor = acc_factor

    if max_speed is None or max_speed <= 0:
        return acc_factor

    speed_abs = abs(speed)
    sensitive_speed = speed_abs * acc_factor
    if sensitive_speed > max_speed:
        if speed_abs == 0:
            capped_acc_factor = acc_factor_base
        else:
            capped_acc_factor = max_speed / speed_abs

    return capped_acc_factor


def calc_acceleration_factor_sigmoid_acceleration_factor(speed,
                                                         acc_factor_base,
                                                         acc_factor_max,
                                                         acc_rate,
                                                         max_speed=D_MAX_SPEED,
                                                         start_offset=D_OFFSET,
                                                         should_round=D_SHOULD_ROUND):
    speed_abs = abs(speed)
    speed_offset = speed_abs - start_offset

    sigmoid_limit = acc_factor_max - acc_factor_base

    acc_factor = sigmoid_function_from_origin(
        speed_offset, sigmoid_limit, acc_rate, epsilon=D_EPSILON
    )

    acc_factor = acc_factor + acc_factor_base * 1

    acc_factor = get_acc_factor_with_max_speed(
        acc_factor,
        speed,
        max_speed,
        acc_factor_base,
    )

    if should_round is True:
        acc_factor = round(acc_factor, 2)

    return acc_factor


def calc_acceleration_factor_sigmoid_output_speed(speed,
                                                  acc_factor_base,
                                                  acc_factor_max,
                                                  acc_rate,
                                                  max_speed=D_MAX_SPEED,
                                                  start_offset=D_OFFSET,
                                                  should_round=D_SHOULD_ROUND):
    speed_abs = abs(speed)

    if speed_abs < 1:
        return 1

    # Cap speed to 1 in start_offset range
    if speed_abs <= start_offset:
        return 1 / speed

    speed_offset = speed_abs - start_offset
    sigmoid_limit = max_speed - 1

    speed_new = sigmoid_function_from_origin(
        speed_offset, sigmoid_limit, acc_rate, epsilon=D_EPSILON
    )

    speed_new = speed_new + 1
    acc_factor = speed_new / speed_abs

    if should_round is True:
        acc_factor = round(acc_factor, 2)

    return acc_factor


def gen_acceleration_factor_range(curve,
                                  range_start,
                                  range_end,
                                  acc_factor_base,
                                  acc_factor_max,
                                  acc_rate,
                                  max_speed=D_MAX_SPEED,
                                  start_offset=D_OFFSET,
                                  should_round=D_SHOULD_ROUND):

    if curve == 'sigmoid_accelaration_factor':
        calc_curve = calc_acceleration_factor_sigmoid_acceleration_factor
    elif curve == 'sigmoid_output_speed':
        calc_curve = calc_acceleration_factor_sigmoid_output_speed
    else:
        raise ValueError(f'Unknown curve: {curve}')

    x_values = [speed for speed in range(range_start, range_end)]
    y_values = []
    for x in x_values:
        y = calc_curve(
            x,
            acc_factor_base,
            acc_factor_max,
            acc_rate,
            max_speed=max_speed,
            start_offset=start_offset,
            should_round=should_round,
        )
        y_values.append(y)

    return (x_values, y_values)


def gen_acceleration_factor_df(curve,
                               range_start,
                               range_end,
                               acc_factor_base,
                               acc_factor_max,
                               acc_rate,
                               max_speed=D_MAX_SPEED,
                               start_offset=D_OFFSET,
                               should_round=D_SHOULD_ROUND,
                               x_title=D_X_TITLE,
                               acc_factor_title=D_ACC_FACTOR_TITLE,
                               output_title=D_OUTPUT_TITLE):

    if curve == 'sigmoid_accelaration_factor':
        range_func = gen_acceleration_factor_sigmoid_accelaration_factor_range
    elif curve == 'sigmoid_output_speed':
        range_func = gen_acceleration_factor_sigmoid_output_speed_range
    else:
        raise ValueError(f'Unknown curve: {curve}')

    x_values, y_values = range_func(
        range_start,
        range_end,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
    )

    output_values = []
    for speed, acc_factor in enumerate(y_values):
        output = speed * acc_factor
        output_values.append(output)

    df = pd.DataFrame({
        x_title: x_values,
        acc_factor_title: y_values,
        output_title: output_values
    })

    return df


def gen_acceleration_factor_sigmoid_accelaration_factor_range(range_start,
                                                              range_end,
                                                              acc_factor_base,
                                                              acc_factor_max,
                                                              acc_rate,
                                                              max_speed=D_MAX_SPEED,
                                                              start_offset=D_OFFSET,
                                                              should_round=D_SHOULD_ROUND):
    return gen_acceleration_factor_range(
        'sigmoid_accelaration_factor',
        range_start,
        range_end,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
    )


def gen_acceleration_factor_sigmoid_df(range_start,
                                       range_end,
                                       acc_factor_base,
                                       acc_factor_max,
                                       acc_rate,
                                       max_speed=D_MAX_SPEED,
                                       start_offset=D_OFFSET,
                                       should_round=D_SHOULD_ROUND,
                                       x_title=D_X_TITLE,
                                       acc_factor_title=D_ACC_FACTOR_TITLE,
                                       output_title=D_OUTPUT_TITLE):
    return gen_acceleration_factor_df(
        'sigmoid',
        range_start,
        range_end,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
        x_title=x_title,
        acc_factor_title=acc_factor_title,
        output_title=output_title,
    )


def gen_acceleration_factor_sigmoid_output_speed_range(range_start,
                                         range_end,
                                         acc_factor_base,
                                         acc_factor_max,
                                         acc_rate,
                                         max_speed=D_MAX_SPEED,
                                         start_offset=D_OFFSET,
                                         should_round=D_SHOULD_ROUND):
    return gen_acceleration_factor_range(
        'sigmoid_output_speed',
        range_start,
        range_end,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
    )


def gen_acceleration_factor_sigmoid_output_speed_df(range_start,
                                      range_end,
                                      acc_factor_base,
                                      acc_factor_max,
                                      acc_rate,
                                      max_speed=D_MAX_SPEED,
                                      start_offset=D_OFFSET,
                                      should_round=D_SHOULD_ROUND,
                                      x_title=D_X_TITLE,
                                      acc_factor_title=D_ACC_FACTOR_TITLE,
                                      output_title=D_OUTPUT_TITLE):
    return gen_acceleration_factor_df(
        'scroll',
        range_start,
        range_end,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
        x_title=x_title,
        acc_factor_title=acc_factor_title,
        output_title=output_title,
    )


def plot_acc_factor(df_col_x,
                    df_col_acc_factor,
                    title='Acceleration Factor Curve',
                    x_label='Input Speed (units/ms)',
                    y_label='Acceleration Factor',
                    plot_label='Acceleration Factor',
                    show_legend=True,
                    show_grid=True,
                    start_offset=None,
                    ax=None,
                    **plot_kwargs):
    if ax is None:
        ax = plt.figure(figsize=(10, 5)).add_subplot(1, 1, 1)

    plot_sens = ax.plot(
        df_col_x, df_col_acc_factor,
        label=plot_label, **plot_kwargs
    )

    if start_offset is not None:
        ax.axvline(
            x=start_offset, color='red', linestyle=':',
            label='Transition Start (Offset)',
        )

    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(show_grid)

    if show_legend:
        ax.legend()

    return plot_sens


def plot_output_speed(df_col_x,
                      df_col_output,
                      plot_original=True,
                      start_offset=None,
                      title='Output Speed',
                      x_label='Input Speed (units/ms)',
                      y_label='Output Speed (units/ms)',
                      plot_label_original='Original Speed',
                      plot_label_output='Accelerated Speed',
                      show_legend=True,
                      show_grid=True,
                      ax=None,
                      plot_output_kwargs={},
                      plot_original_kwargs={}):
    if ax is None:
        ax = plt.figure(figsize=(10, 5)).add_subplot(1, 1, 1)

    ax.plot(
        df_col_x, df_col_output,
        label=plot_label_output, **plot_output_kwargs
    )

    if plot_original:
        ax.plot(
            df_col_x, df_col_x,
            label=plot_label_original, **plot_original_kwargs,
        )

    if start_offset is not None:
        ax.axvline(
            x=start_offset, color='red', linestyle=':',
            label='Transition Start (Offset)',
        )

    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(show_grid)

    if show_legend:
        ax.legend()


def display_curve(curve,
                  acc_factor_base,
                  acc_factor_max,
                  acc_rate,
                  max_speed,
                  start_offset,
                  zoom_max_x=D_ZOOM_MAX_X,
                  should_round=D_SHOULD_ROUND):

    df = gen_acceleration_factor_df(
        curve,
        0,
        254,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
    )

    df_zoomed = df.head(zoom_max_x)

    plot_acc_factor(
        df_zoomed['x'], df_zoomed['acc_factor'], start_offset=start_offset,
        title='Acceleration Factor (Zoomed)',
    )
    plot_output_speed(
        df_zoomed['x'], df_zoomed['output'], start_offset=start_offset,
        title='Output Speed (Zoomed)',
    )

    plot_acc_factor(df['x'], df['acc_factor'], start_offset=start_offset)
    plot_output_speed(df['x'], df['output'], start_offset=start_offset)

    return df
