"""
Python functions that generate plots for acceleration curves. To be used with
the jupyter notebook mouse_acceleration_models.

Requirements:
    - pip install pandas matplotlib
"""

import math
import pandas as pd
import matplotlib.pyplot as plt


# Default values for functions
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


def sigmoid_function_inverse(y, limit, slope, raise_error=True):
    '''
    Inverse of the sigmoid function. Takes a y value as input and returns the
    corresponding x value.
    '''

    if raise_error is False:
        if y <= 0:
            y = 0.001

        if y >= limit:
            y = limit - 0.001
    else:
        if y <= 0 or y >= limit:
            raise ValueError("y must be within the range (0, limit)")

    return -1 / slope * math.log((limit - y) / y)


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


def calc_acceleration_factor_sigmoid_pure(speed,
                                          acc_factor_base,
                                          acc_factor_max,
                                          acc_rate,
                                          max_speed=D_MAX_SPEED,
                                          start_offset=D_OFFSET,
                                          should_round=D_SHOULD_ROUND):
    speed_offset = speed + start_offset
    sigmoid_limit = acc_factor_max - acc_factor_base

    acc_factor = sigmoid_function(speed_offset, sigmoid_limit, acc_rate)
    acc_factor = acc_factor + acc_factor_base * 1

    return acc_factor


def calc_acceleration_factor_sigmoid(speed,
                                     acc_factor_base,
                                     acc_factor_max,
                                     acc_rate,
                                     max_speed=D_MAX_SPEED,
                                     start_offset=D_OFFSET,
                                     should_round=D_SHOULD_ROUND):
    speed_abs = abs(speed)
    sigmoid_limit = acc_factor_max - acc_factor_base

    # The sigmoid curve is always centered at 0 (meaning x0 = limit/2). We
    # can shift the curve to the right by subtracting x values.
    #
    # We want to position the curve in such a way that we can see a meaningful,
    # yet gradual, increase of the acceleration curve after the offset. We use
    # 2.5% of the total acceleration-increase-range for that.
    #
    # So, we determine the x value where the sigmoid function returns that
    # increase. And then we shift the curve to the right so that the curve
    # has the 2.5% increase at exactly the offset x value.
    start_offset_y_value = sigmoid_limit * 0.025
    start_offset_addition = sigmoid_function_inverse(
        start_offset_y_value, sigmoid_limit, acc_rate
    )
    speed_offset = speed_abs - start_offset + start_offset_addition

    acc_factor = sigmoid_function(
        speed_offset, sigmoid_limit, acc_rate
    )

    acc_factor = acc_factor + acc_factor_base

    acc_factor = get_acc_factor_with_max_speed(
        acc_factor,
        speed,
        max_speed,
        acc_factor_base,
    )

    if should_round is True:
        acc_factor = round(acc_factor, 2)

    return acc_factor


def calc_acceleration_factor_scroll(speed,
                                    acc_factor_base,
                                    acc_factor_max,
                                    acc_rate,
                                    max_speed=D_MAX_SPEED,
                                    start_offset=D_OFFSET,
                                    should_round=D_SHOULD_ROUND):
    speed_abs = abs(speed)

    if speed_abs < 1:
        return 1

    speed_new = 1
    if speed_abs >= start_offset:
        sigmoid_limit = max_speed - 1

        # The sigmoid curve is always centered at 0 (meaning x0 = limit/2). We
        # can shift the curve to the right by subtracting x values.
        #
        # We want to position the curve in such a way that we can see a meaningful,
        # yet gradual, increase of the acceleration curve after the offset.
        #
        # For scrolling we use a smaller rate of just 1% increase (compared to 2.5 for
        # mouse movement in the sigmoid curve).
        #
        # So, we determine the x value where the sigmoid function returns that
        # increase. And then we shift the curve to the right so that the curve
        # has the 2.5% increase at exactly the offset x value.
        start_offset_y_value = sigmoid_limit * 0.01
        start_offset_addition = sigmoid_function_inverse(
            start_offset_y_value, sigmoid_limit, acc_rate
        )
        speed_offset = speed_abs - start_offset + start_offset_addition

        speed_new = sigmoid_function(
            speed_offset, sigmoid_limit, acc_rate
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

    if curve == 'sigmoid_pure':
        calc_curve = calc_acceleration_factor_sigmoid_pure
    elif curve == 'sigmoid':
        calc_curve = calc_acceleration_factor_sigmoid
    elif curve == 'scroll':
        calc_curve = calc_acceleration_factor_scroll
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

    x_values, y_values = gen_acceleration_factor_range(
        curve,
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
    for index, x in enumerate(x_values):
        output = x * y_values[index]
        output_values.append(output)

    df = pd.DataFrame({
        x_title: x_values,
        acc_factor_title: y_values,
        output_title: output_values
    })

    return df


def gen_acceleration_factor_sigmoid_range(range_start,
                                                              range_end,
                                                              acc_factor_base,
                                                              acc_factor_max,
                                                              acc_rate,
                                                              max_speed=D_MAX_SPEED,
                                                              start_offset=D_OFFSET,
                                                              should_round=D_SHOULD_ROUND):
    return gen_acceleration_factor_range(
        'sigmoid',
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


def gen_acceleration_factor_scroll_range(range_start,
                                         range_end,
                                         acc_factor_base,
                                         acc_factor_max,
                                         acc_rate,
                                         max_speed=D_MAX_SPEED,
                                         start_offset=D_OFFSET,
                                         should_round=D_SHOULD_ROUND):
    return gen_acceleration_factor_range(
        'scroll',
        range_start,
        range_end,
        acc_factor_base,
        acc_factor_max,
        acc_rate,
        max_speed=max_speed,
        start_offset=start_offset,
        should_round=should_round,
    )


def gen_acceleration_factor_scroll_df(range_start,
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
                  range_start=0,
                  range_end=254,
                  zoom_max_x=D_ZOOM_MAX_X,
                  should_round=D_SHOULD_ROUND):

    df = gen_acceleration_factor_df(
        curve,
        range_start,
        range_end,
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
