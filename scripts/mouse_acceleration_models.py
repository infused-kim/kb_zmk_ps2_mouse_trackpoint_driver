"""
Python functions that generate plots for acceleration curves. To be used with
the jupyter notebook mouse_acceleration_models.

Requirements:
    - pip install pandas matplotlib
"""

import math
import pandas as pd
import matplotlib.pyplot as plt


def sigmoid_function(x, limit, slope):
    '''
    Standard sigmoid function
    '''

    return (limit) / (1 + math.exp(-slope * x))


def sigmoid_function_from_origin(x, limit, slope, epsilon=0.01):
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

    # We find where the sigmoid function starts within 0.1 of 0,
    # but it will be something like y=0.013. So we lower the y
    # values by that amount to make sure the curve transitions
    # smoothly from the offset y value
    offset_y = sigmoid_function(start_at, limit, slope)

    y = sigmoid_function(offset_x, limit, slope) - offset_y

    return y


def calc_sensitivity_curve_sigmoid(x,
                                   base_sensitivity,
                                   max_sensitivity,
                                   accel_slope,
                                   max_speed=None,
                                   offset=0,
                                   should_round=True):
    x_offset = x - offset
    sigmoid_limit = max_sensitivity - base_sensitivity

    sensitivity = sigmoid_function_from_origin(
        x_offset, sigmoid_limit, accel_slope, epsilon=0.01
    )

    sensitivity = sensitivity + base_sensitivity * 1

    if max_speed is not None:
        speed = x * sensitivity
        if speed > max_speed:
            if x == 0:
                sensitivity = base_sensitivity
            else:
                sensitivity = max_speed / x

    if should_round is True:
        sensitivity = round(sensitivity, 2)

    return sensitivity


def gen_sensitivity_curve_sigmoid_range(range_start,
                                        range_end,
                                        base_sensitivity,
                                        max_sensitivity,
                                        accel_slope,
                                        max_speed=None,
                                        offset=0,
                                        should_round=True):
    x_values = [x for x in range(range_start, range_end)]
    y_values = []
    for x in x_values:
        y = calc_sensitivity_curve_sigmoid(
            x,
            base_sensitivity,
            max_sensitivity,
            accel_slope,
            max_speed=max_speed,
            offset=offset,
            should_round=should_round,
        )
        y_values.append(y)

    return (x_values, y_values)


def gen_sensitivity_curve_sigmoid_df(range_start,
                                     range_end,
                                     base_sensitivity,
                                     max_sensitivity,
                                     accel_slope,
                                     max_speed=None,
                                     offset=0,
                                     should_round=True,
                                     x_title='x',
                                     sensitivity_title='sensitivity',
                                     output_title='output'):
    x_values, sensitivity_values = gen_sensitivity_curve_sigmoid_range(
        range_start,
        range_end,
        base_sensitivity,
        max_sensitivity,
        accel_slope,
        max_speed=max_speed,
        offset=offset,
        should_round=should_round,
    )

    output_values = []
    for x, sensitivity in enumerate(sensitivity_values):
        output = x * sensitivity
        output_values.append(output)

    df = pd.DataFrame({
        x_title: x_values,
        sensitivity_title: sensitivity_values,
        output_title: output_values
    })

    return df


def plot_sensitivity(df_col_x,
                     df_col_sensitivity,
                     title='Sensitivity Curve',
                     x_label='Input Speed (units/ms)',
                     y_label='Sensitivity',
                     plot_label='Sensitivity',
                     show_legend=True,
                     show_grid=True,
                     offset=None,
                     ax=None,
                     **plot_kwargs):
    if ax is None:
        ax = plt.figure(figsize=(10, 5)).add_subplot(1, 1, 1)

    plot_sens = ax.plot(
        df_col_x, df_col_sensitivity,
        label=plot_label, **plot_kwargs
    )

    if offset is not None:
        ax.axvline(
            x=offset, color='red', linestyle=':',
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
                      offset=None,
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

    if offset is not None:
        ax.axvline(
            x=offset, color='red', linestyle=':',
            label='Transition Start (Offset)',
        )

    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(show_grid)

    if show_legend:
        ax.legend()
