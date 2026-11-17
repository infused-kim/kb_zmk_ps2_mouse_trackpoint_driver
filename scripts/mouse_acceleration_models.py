#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "matplotlib",
#     "pandas",
#     "ipykernel",
# ]
# ///
#
# Find the venv with:
# uv python find --script scripts/mouse_acceleration_models.py
"""
Python functions that generate plots for acceleration curves. To be used with
the jupyter notebook mouse_acceleration_models.
"""

import math
from typing import Any, Literal

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.axes import Axes
from matplotlib.lines import Line2D

# Default values for functions
D_EPSILON = 0.01
D_MAX_SPEED = 0
D_OFFSET = 0
D_SHOULD_ROUND = False
D_X_TITLE = 'x'
D_ACC_FACTOR_TITLE = 'acc_factor'
D_OUTPUT_TITLE = 'output'
D_ZOOM_MAX_X = 30

CurveName = Literal['sigmoid_pure', 'sigmoid']


def sigmoid_function(x: float, limit: float, acc_rate: float) -> float:
    """
    Standard sigmoid function
    """

    return (limit) / (1 + math.exp(-acc_rate * x))


def sigmoid_function_inverse(
    y: float, limit: float, acc_rate: float, *, raise_error: bool = True
) -> float:
    """
    Inverse of the sigmoid function. Takes a y value as input and returns the
    corresponding x value.
    """

    if raise_error is False:
        if y <= 0:
            y = 0.001

        if y >= limit:
            y = limit - 0.001
    elif y <= 0 or y >= limit:
        message = 'y must be within the range (0, limit)'
        raise ValueError(message)

    return -1 / acc_rate * math.log((limit - y) / y)


def sigmoid_function_with_offset(
    x: float,
    limit: float,
    acc_rate: float,
    start_offset: float,
    start_offset_y_value: float,
) -> float:
    # The sigmoid curve is always centered at 0 (meaning x0 = limit/2). We
    # can shift the curve to the right by subtracting x values.
    #
    # This function allows us to pass the y value we wish the sigmoid curve
    # to start with after the offset.
    #
    # A reasonable value, for example, is 2.5% of the limit.
    #
    # This way, we position the curve in such a way that we can see a
    # meaningful, yet gradual, increase of the acceleration curve after the
    # offset.
    #
    # So, we determine the x value where the sigmoid function returns that
    # increase. And then we shift the curve to the right so that the curve
    # has the 2.5% increase at exactly the offset x value.
    start_offset_addition = sigmoid_function_inverse(
        start_offset_y_value, limit, acc_rate
    )
    speed_offset = x - start_offset + start_offset_addition

    acc_factor = sigmoid_function(speed_offset, limit, acc_rate)

    return acc_factor


def get_acc_factor_with_max_speed(
    acc_factor: float,
    speed: float,
    max_speed: float | None,
    acc_factor_base: float,
) -> float:
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


def calc_acceleration_factor_sigmoid_pure(
    speed: float,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    *,
    # Unused, but part of the signature that all curves share
    max_speed: float | None = D_MAX_SPEED,  # noqa: ARG001
    start_offset: float = D_OFFSET,
    should_round: bool = D_SHOULD_ROUND,  # noqa: ARG001
) -> float:
    speed_offset = speed + start_offset
    sigmoid_limit = acc_factor_max - acc_factor_base

    acc_factor = sigmoid_function(speed_offset, sigmoid_limit, acc_rate)
    acc_factor = acc_factor + acc_factor_base * 1

    return acc_factor


def calc_acceleration_factor_sigmoid(
    speed: float,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    *,
    max_speed: float | None = D_MAX_SPEED,
    start_offset: float = D_OFFSET,
    should_round: bool = D_SHOULD_ROUND,
) -> float:
    speed_abs = abs(speed)
    sigmoid_limit = acc_factor_max - acc_factor_base

    start_offset_y_value = sigmoid_limit * 0.025
    acc_factor = sigmoid_function_with_offset(
        speed_abs, sigmoid_limit, acc_rate, start_offset, start_offset_y_value
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


def gen_acceleration_factor_range(
    curve: CurveName,
    range_start: int,
    range_end: int,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    *,
    max_speed: float | None = D_MAX_SPEED,
    start_offset: float = D_OFFSET,
    should_round: bool = D_SHOULD_ROUND,
) -> tuple[list[int], list[float]]:

    if curve == 'sigmoid_pure':
        calc_curve = calc_acceleration_factor_sigmoid_pure
    elif curve == 'sigmoid':
        calc_curve = calc_acceleration_factor_sigmoid
    else:
        message = f'Unknown curve: {curve}'
        raise ValueError(message)

    x_values = list(range(range_start, range_end))
    y_values: list[float] = []
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


def gen_acceleration_factor_df(
    curve: CurveName,
    range_start: int,
    range_end: int,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    *,
    max_speed: float | None = D_MAX_SPEED,
    start_offset: float = D_OFFSET,
    should_round: bool = D_SHOULD_ROUND,
    x_title: str = D_X_TITLE,
    acc_factor_title: str = D_ACC_FACTOR_TITLE,
    output_title: str = D_OUTPUT_TITLE,
) -> pd.DataFrame:

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

    output_values: list[float] = []
    for index, x in enumerate(x_values):
        output = x * y_values[index]
        output_values.append(output)

    df = pd.DataFrame(
        {
            x_title: x_values,
            acc_factor_title: y_values,
            output_title: output_values,
        }
    )

    return df


def gen_acceleration_factor_sigmoid_range(
    range_start: int,
    range_end: int,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    *,
    max_speed: float | None = D_MAX_SPEED,
    start_offset: float = D_OFFSET,
    should_round: bool = D_SHOULD_ROUND,
) -> tuple[list[int], list[float]]:
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


def gen_acceleration_factor_sigmoid_df(
    range_start: int,
    range_end: int,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    *,
    max_speed: float | None = D_MAX_SPEED,
    start_offset: float = D_OFFSET,
    should_round: bool = D_SHOULD_ROUND,
    x_title: str = D_X_TITLE,
    acc_factor_title: str = D_ACC_FACTOR_TITLE,
    output_title: str = D_OUTPUT_TITLE,
) -> pd.DataFrame:
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


def plot_acc_factor(
    df_col_x: pd.Series,
    df_col_acc_factor: pd.Series,
    title: str = 'Acceleration Factor Curve',
    x_label: str = 'Input Speed (units/ms)',
    y_label: str = 'Acceleration Factor',
    plot_label: str = 'Acceleration Factor',
    *,
    show_legend: bool = True,
    show_grid: bool = True,
    start_offset: float | None = None,
    ax: Axes | None = None,
    # Passed through to matplotlib, which doesn't type them either
    **plot_kwargs: Any,  # noqa: ANN401
) -> list[Line2D]:
    if ax is None:
        ax = plt.figure(figsize=(10, 5)).add_subplot(1, 1, 1)

    plot_sens = ax.plot(
        df_col_x, df_col_acc_factor, label=plot_label, **plot_kwargs
    )

    if start_offset is not None:
        ax.axvline(
            x=start_offset,
            color='red',
            linestyle=':',
            label='Transition Start (Offset)',
        )

    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(show_grid)

    if show_legend:
        ax.legend()

    return plot_sens


def plot_output_speed(
    df_col_x: pd.Series,
    df_col_output: pd.Series,
    *,
    plot_original: bool = True,
    start_offset: float | None = None,
    title: str = 'Output Speed',
    x_label: str = 'Input Speed (units/ms)',
    y_label: str = 'Output Speed (units/ms)',
    plot_label_original: str = 'Original Speed',
    plot_label_output: str = 'Accelerated Speed',
    show_legend: bool = True,
    show_grid: bool = True,
    ax: Axes | None = None,
    plot_output_kwargs: dict[str, Any] | None = None,
    plot_original_kwargs: dict[str, Any] | None = None,
) -> None:
    if plot_output_kwargs is None:
        plot_output_kwargs = {}
    if plot_original_kwargs is None:
        plot_original_kwargs = {}

    if ax is None:
        ax = plt.figure(figsize=(10, 5)).add_subplot(1, 1, 1)

    ax.plot(
        df_col_x, df_col_output, label=plot_label_output, **plot_output_kwargs
    )

    if plot_original:
        ax.plot(
            df_col_x,
            df_col_x,
            label=plot_label_original,
            **plot_original_kwargs,
        )

    if start_offset is not None:
        ax.axvline(
            x=start_offset,
            color='red',
            linestyle=':',
            label='Transition Start (Offset)',
        )

    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(show_grid)

    if show_legend:
        ax.legend()


def display_curve(
    curve: CurveName,
    acc_factor_base: float,
    acc_factor_max: float,
    acc_rate: float,
    max_speed: float | None,
    start_offset: float,
    *,
    range_start: int = 0,
    range_end: int = 254,
    zoom_max_x: int = D_ZOOM_MAX_X,
    should_round: bool = D_SHOULD_ROUND,
) -> pd.DataFrame:

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
        df_zoomed['x'],
        df_zoomed['acc_factor'],
        start_offset=start_offset,
        title='Acceleration Factor (Zoomed)',
    )
    plot_output_speed(
        df_zoomed['x'],
        df_zoomed['output'],
        start_offset=start_offset,
        title='Output Speed (Zoomed)',
    )

    plot_acc_factor(df['x'], df['acc_factor'], start_offset=start_offset)
    plot_output_speed(df['x'], df['output'], start_offset=start_offset)

    return df
