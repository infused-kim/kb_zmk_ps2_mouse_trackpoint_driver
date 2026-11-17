#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "matplotlib",
#     "pandas",
#     "ipykernel",
#     "rich",
#     "typer",
# ]
# ///
#
# Find the venv with:
# uv python find --script scripts/mouse_acceleration_models.py
"""
Model of the input accelerator's curve, to plot it and try out settings.

Run it as a CLI. The options are named like the `zmk,input-accelerator`
devicetree properties without the `acceleration-` prefix and use the same
units, so you can copy values from the config:

    ./mouse_acceleration_models.py plot
    ./mouse_acceleration_models.py plot --factor-max 150
    ./mouse_acceleration_models.py plot -b 80 -m 130 -r 40 -o 12
    ./mouse_acceleration_models.py table --step 5
    ./mouse_acceleration_models.py all

`plot` draws the graphs directly in terminals that support the kitty graphics
protocol (Ghostty, kitty, iTerm2, WezTerm) and checks for it first. Inside
tmux, it needs `set -g allow-passthrough on`. Use `--output file.png` to save
it instead.

The functions can also be imported, e.g. by the jupyter notebook
mouse_acceleration_models.
"""

import base64
import fcntl
import io
import math
import os
import select
import shutil
import struct
import sys
import termios
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Annotated, Any, Literal

import matplotlib.pyplot as plt
import matplotlib.style
import pandas as pd
import typer
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from rich.console import Console
from rich.table import Table

#
# Defaults
#


# Match the TrackPoint's `input_accelerator_move` node in the zmk-config
DEFAULT_ACCELERATION_FACTOR_BASE = 80
DEFAULT_ACCELERATION_FACTOR_MAX = 130
DEFAULT_ACCELERATION_FACTOR_RATE = 40
DEFAULT_ACCELERATION_START_OFFSET = 12
DEFAULT_MAX_SPEED = -1

# Highest input speed that the commands show
DEFAULT_END = 60
DEFAULT_TABLE_STEP = 2
DEFAULT_PLOT_WIDTH = 120

INPUT_SPEED_LABEL = 'Input speed (units per report)'

# Defaults of the functions used by the notebook
D_EPSILON = 0.01
D_MAX_SPEED = DEFAULT_MAX_SPEED
D_OFFSET = 0
D_SHOULD_ROUND = False
D_X_TITLE = 'x'
D_ACC_FACTOR_TITLE = 'acc_factor'
D_OUTPUT_TITLE = 'output'
D_ZOOM_MAX_X = 30


#
# CLI
#

app = typer.Typer(
    help='Plot and tabulate the input accelerator curve.',
    no_args_is_help=True,
    add_completion=False,
)
console = Console()
err_console = Console(stderr=True)


class Theme(str, Enum):
    dark = 'dark'
    light = 'light'


# Groups of options in --help
ACCELERATION_PANEL = 'Acceleration settings'
GRAPH_PANEL = 'Graph and table'
OUTPUT_PANEL = 'Output'

# Options shared by the commands. Named like the `zmk,input-accelerator`
# devicetree properties without the `acceleration-` prefix and in the same
# units.
FactorBaseOption = Annotated[
    int,
    typer.Option(
        '--factor-base',
        '-b',
        help='Start of the curve in percent',
        rich_help_panel=ACCELERATION_PANEL,
    ),
]
FactorMaxOption = Annotated[
    int,
    typer.Option(
        '--factor-max',
        '-m',
        help='End of the curve in percent',
        rich_help_panel=ACCELERATION_PANEL,
    ),
]
FactorRateOption = Annotated[
    int,
    typer.Option(
        '--factor-rate',
        '-r',
        help='Steepness of the curve in hundredths',
        rich_help_panel=ACCELERATION_PANEL,
    ),
]
StartOffsetOption = Annotated[
    int,
    typer.Option(
        '--start-offset',
        '-o',
        help='Input speed where the curve starts rising',
        rich_help_panel=ACCELERATION_PANEL,
    ),
]
MaxSpeedOption = Annotated[
    int,
    typer.Option(
        '--max-speed',
        '-s',
        help='Maximum output speed, -1 disables it',
        rich_help_panel=ACCELERATION_PANEL,
    ),
]
EndOption = Annotated[
    int,
    typer.Option(
        '--end',
        help='Highest input speed to show',
        rich_help_panel=GRAPH_PANEL,
    ),
]
StepOption = Annotated[
    int,
    typer.Option(
        '--step',
        help='Input speed step between table rows',
        rich_help_panel=GRAPH_PANEL,
    ),
]
OutputOption = Annotated[
    Path | None,
    typer.Option(
        '--output',
        help='Save the plot as PNG instead of showing it in the terminal',
        rich_help_panel=OUTPUT_PANEL,
    ),
]
OutputSeparateOption = Annotated[
    bool,
    typer.Option(
        '--output-separate',
        help='Show or save each graph as its own image. With '
        '--output plot.png, writes plot-factor.png, plot-gain.png and '
        'plot-output.png',
        rich_help_panel=OUTPUT_PANEL,
    ),
]
ExcludeSettingsOption = Annotated[
    bool,
    typer.Option(
        '--exclude-settings',
        help='Leave out the settings above the graphs',
        rich_help_panel=GRAPH_PANEL,
    ),
]
ThemeOption = Annotated[
    Theme,
    typer.Option(
        '--theme', help='Colors of the plot', rich_help_panel=GRAPH_PANEL
    ),
]
WidthOption = Annotated[
    int,
    typer.Option(
        '--width',
        help='Maximum width of the plot in terminal columns',
        rich_help_panel=GRAPH_PANEL,
    ),
]


@app.command()
def plot(
    *,
    factor_base: FactorBaseOption = DEFAULT_ACCELERATION_FACTOR_BASE,
    factor_max: FactorMaxOption = DEFAULT_ACCELERATION_FACTOR_MAX,
    factor_rate: FactorRateOption = DEFAULT_ACCELERATION_FACTOR_RATE,
    start_offset: StartOffsetOption = DEFAULT_ACCELERATION_START_OFFSET,
    max_speed: MaxSpeedOption = DEFAULT_MAX_SPEED,
    end: EndOption = DEFAULT_END,
    output: OutputOption = None,
    output_separate: OutputSeparateOption = False,
    exclude_settings: ExcludeSettingsOption = False,
    theme: ThemeOption = Theme.dark,
    width: WidthOption = DEFAULT_PLOT_WIDTH,
) -> None:
    """
    Plot the acceleration factor, gain and output speed.
    """

    curve = CurveSettings(
        factor_base,
        factor_max,
        factor_rate,
        start_offset,
        max_speed,
    )
    plot_options = PlotOptions(
        end, output_separate, not exclude_settings, theme, width
    )
    show_plot(curve, plot_options, output)


@app.command()
def table(
    *,
    factor_base: FactorBaseOption = DEFAULT_ACCELERATION_FACTOR_BASE,
    factor_max: FactorMaxOption = DEFAULT_ACCELERATION_FACTOR_MAX,
    factor_rate: FactorRateOption = DEFAULT_ACCELERATION_FACTOR_RATE,
    start_offset: StartOffsetOption = DEFAULT_ACCELERATION_START_OFFSET,
    max_speed: MaxSpeedOption = DEFAULT_MAX_SPEED,
    end: EndOption = DEFAULT_END,
    step: StepOption = DEFAULT_TABLE_STEP,
) -> None:
    """
    Show the acceleration factor, gain and output speed as a table.

    Gain values above acceleration-factor-max are highlighted.
    """

    curve = CurveSettings(
        factor_base,
        factor_max,
        factor_rate,
        start_offset,
        max_speed,
    )
    print_table(curve, end, step)


@app.command('all')
def show_all(
    *,
    factor_base: FactorBaseOption = DEFAULT_ACCELERATION_FACTOR_BASE,
    factor_max: FactorMaxOption = DEFAULT_ACCELERATION_FACTOR_MAX,
    factor_rate: FactorRateOption = DEFAULT_ACCELERATION_FACTOR_RATE,
    start_offset: StartOffsetOption = DEFAULT_ACCELERATION_START_OFFSET,
    max_speed: MaxSpeedOption = DEFAULT_MAX_SPEED,
    end: EndOption = DEFAULT_END,
    step: StepOption = DEFAULT_TABLE_STEP,
    output: OutputOption = None,
    output_separate: OutputSeparateOption = False,
    exclude_settings: ExcludeSettingsOption = False,
    theme: ThemeOption = Theme.dark,
    width: WidthOption = DEFAULT_PLOT_WIDTH,
) -> None:
    """
    Show the plot and the table.
    """

    curve = CurveSettings(
        factor_base,
        factor_max,
        factor_rate,
        start_offset,
        max_speed,
    )
    plot_options = PlotOptions(
        end, output_separate, not exclude_settings, theme, width
    )
    show_plot(curve, plot_options, output)
    print_table(curve, end, step)


@dataclass
class PlotOptions:
    """
    How to draw the graphs.
    """

    end: int
    separate: bool
    include_settings: bool
    theme: Theme
    width: int


# PNGs by graph name, or `{None: png}` for the combined plot
PlotPngs = dict[str | None, bytes]


def show_plot(
    curve: 'CurveSettings', plot_options: PlotOptions, output: Path | None
) -> None:
    pngs = render_plot_pngs(curve, plot_options)

    if output is not None:
        save_pngs(pngs, output)
        return

    if not sys.stdout.isatty():
        err_console.print(
            "[red]stdout is not a terminal, so the plot can't be shown. "
            'Use --output to save it as PNG.[/red]'
        )
        raise typer.Exit(1)

    if not terminal_supports_kitty_graphics():
        err_console.print(
            "[red]This terminal doesn't support the kitty graphics protocol, "
            "so the plot can't be shown. Use a terminal that does, like "
            'Ghostty, kitty or iTerm2, or use --output to save it as PNG.'
            '[/red]'
        )
        raise typer.Exit(1)

    for png in pngs.values():
        columns, rows = fit_image_to_terminal(png, plot_options.width)
        show_png_in_terminal(png, columns, rows)


def save_pngs(pngs: PlotPngs, output: Path) -> None:
    """
    Writes the PNGs from `render_plot_pngs`. The combined plot goes to
    `output` itself, a separate graph to `output` with the graph's name added,
    e.g. plot-gain.png.
    """

    for name, png in pngs.items():
        if name is None:
            path = output
        else:
            suffix = output.suffix or '.png'
            path = output.with_name(f'{output.stem}-{name}{suffix}')

        path.write_bytes(png)
        console.print(f'Wrote {path}')


def print_table(curve: 'CurveSettings', end: int, step: int) -> None:
    console.print('\n'.join(curve.describe()), style='dim', highlight=False)

    tbl = Table(caption='Gain values above acceleration-factor-max are yellow')
    tbl.add_column('Speed', justify='right')
    tbl.add_column('Factor', justify='right')
    tbl.add_column('Gain', justify='right')
    tbl.add_column('Output', justify='right')

    for speed in range(0, end + 1, step):
        tbl.add_row(
            str(speed),
            f'{curve.acc_factor(speed):.2f}',
            format_gain(curve, speed),
            f'{curve.output_speed(speed):.1f}',
        )

    console.print(tbl)


@dataclass
class CurveSettings:
    """
    Sigmoid curve settings, named like the devicetree properties and in the
    same units: factors in percent and the rate in hundredths.
    """

    acceleration_factor_base: int
    acceleration_factor_max: int
    acceleration_factor_rate: int
    acceleration_start_offset: int
    max_speed: int

    def acc_factor(self, speed: float) -> float:
        return calc_acceleration_factor_sigmoid(
            speed,
            self.acceleration_factor_base / 100,
            self.acceleration_factor_max / 100,
            self.acceleration_factor_rate / 100,
            max_speed=self.max_speed,
            start_offset=self.acceleration_start_offset,
        )

    def output_speed(self, speed: float) -> float:
        return speed * self.acc_factor(speed)

    def gain(self, speed: float, h: float = 0.01) -> float:
        """
        How much the output speed changes when the input speed changes (the
        slope of the output speed), calculated numerically.
        """

        low = max(speed - h, 0)
        high = speed + h
        return (self.output_speed(high) - self.output_speed(low)) / (
            high - low
        )

    def describe(self) -> list[str]:
        """
        The settings as devicetree properties, one per line.
        """

        lines = [
            f'acceleration-factor-base = <{self.acceleration_factor_base}>;',
            f'acceleration-factor-max = <{self.acceleration_factor_max}>;',
            f'acceleration-factor-rate = <{self.acceleration_factor_rate}>;',
            f'acceleration-start-offset = <{self.acceleration_start_offset}>;',
        ]
        if self.max_speed > 0:
            lines.append(f'max-speed = <{self.max_speed}>;')

        return lines


def format_gain(curve: CurveSettings, speed: float) -> str:
    gain = curve.gain(speed)
    text = f'{gain:.2f}'

    if gain > curve.acceleration_factor_max / 100 + 0.005:
        text = f'[yellow]{text}[/yellow]'

    return text


def render_plot_pngs(
    curve: CurveSettings, plot_options: PlotOptions
) -> PlotPngs:
    """
    Renders the graphs as PNGs. Returns a dict from the graph's name to its
    PNG, or `{None: png}` for the combined plot.
    """

    with matplotlib.style.context(get_plot_style(plot_options.theme)):
        if plot_options.separate:
            return render_separate_plot_pngs(curve, plot_options)
        return {None: render_combined_plot_png(curve, plot_options)}


def render_combined_plot_png(
    curve: CurveSettings, plot_options: PlotOptions
) -> bytes:
    """
    Renders the factor, gain and output speed graphs below each other into
    one PNG.
    """

    speeds = get_plot_speeds(plot_options.end)

    fig = Figure(figsize=(10, 11), dpi=150, layout='constrained')
    ax_factor, ax_gain, ax_output = fig.subplots(3, 1, sharex=True)
    if plot_options.include_settings:
        set_plot_title(fig, curve)

    draw_factor_graph(ax_factor, curve, speeds)
    draw_gain_graph(ax_gain, curve, speeds)
    draw_output_graph(ax_output, curve, speeds)
    ax_output.set_xlabel(INPUT_SPEED_LABEL)

    return figure_to_png(fig)


def render_separate_plot_pngs(
    curve: CurveSettings, plot_options: PlotOptions
) -> PlotPngs:
    """
    Renders each graph into its own PNG. Returns a dict from the graph's name
    to the PNG.
    """

    speeds = get_plot_speeds(plot_options.end)
    graphs: dict[str, DrawGraph] = {
        'factor': draw_factor_graph,
        'gain': draw_gain_graph,
        'output': draw_output_graph,
    }
    pngs: PlotPngs = {}

    for name, draw_graph in graphs.items():
        fig = Figure(figsize=(10, 4.5), dpi=150, layout='constrained')
        ax = fig.subplots()
        if plot_options.include_settings:
            set_plot_title(fig, curve)
        draw_graph(ax, curve, speeds)
        ax.set_xlabel(INPUT_SPEED_LABEL)
        pngs[name] = figure_to_png(fig)

    return pngs


def get_plot_speeds(end: int) -> list[float]:
    return [i / 10 for i in range(end * 10 + 1)]


def get_plot_style(theme: Theme) -> str:
    if theme == Theme.dark:
        return 'dark_background'
    return 'default'


def set_plot_title(fig: Figure, curve: CurveSettings) -> None:
    settings = curve.describe()
    fig.suptitle(
        ' '.join(settings[:3]) + '\n' + ' '.join(settings[3:]),
        fontsize='medium',
    )


def draw_factor_graph(
    ax: Axes, curve: CurveSettings, speeds: list[float]
) -> None:
    ax.plot(
        speeds,
        [curve.acc_factor(s) for s in speeds],
        label='acceleration factor',
    )
    draw_factor_max_line(ax, curve)
    finish_graph(ax, curve)
    ax.set_ylabel('Acceleration factor\n(output / input)')


def draw_gain_graph(
    ax: Axes, curve: CurveSettings, speeds: list[float]
) -> None:
    ax.plot(speeds, [curve.gain(s) for s in speeds], label='gain')
    draw_factor_max_line(ax, curve)
    finish_graph(ax, curve)
    ax.set_ylabel('Gain\n(change of output speed)')


def draw_output_graph(
    ax: Axes, curve: CurveSettings, speeds: list[float]
) -> None:
    ax.plot(
        speeds,
        [curve.output_speed(s) for s in speeds],
        label='accelerated',
    )
    ax.plot(speeds, speeds, ':', color='gray', label='unaccelerated')

    if curve.max_speed > 0:
        ax.axhline(
            curve.max_speed,
            color='orange',
            linestyle=':',
            label='max-speed',
        )

    finish_graph(ax, curve)
    ax.set_ylabel('Output speed')


# Draws one graph into the given axes
DrawGraph = Callable[[Axes, CurveSettings, list[float]], None]


def draw_factor_max_line(ax: Axes, curve: CurveSettings) -> None:
    ax.axhline(
        curve.acceleration_factor_max / 100,
        color='gray',
        linestyle='--',
        label='acceleration-factor-max',
    )


def finish_graph(ax: Axes, curve: CurveSettings) -> None:
    """
    Adds the start offset line, the grid and the legend.
    """

    ax.axvline(
        curve.acceleration_start_offset,
        color='red',
        linestyle=':',
        label='acceleration-start-offset',
    )
    ax.grid(alpha=0.3)
    ax.legend(fontsize='small')


def figure_to_png(fig: Figure) -> bytes:
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png')
    return buffer.getvalue()


def terminal_supports_kitty_graphics(timeout: float = 1) -> bool:
    """
    Asks the terminal whether it supports the kitty graphics protocol.

    Sends a graphics query followed by a request for the primary device
    attributes, which every terminal answers. A terminal with graphics support
    answers the query first. If only the device attributes come back, it
    doesn't support it.
    """

    # tmux answers the device attributes itself and doesn't forward the
    # graphics answer reliably, so assume the outer terminal supports it.
    if os.environ.get('TMUX'):
        return True

    try:
        tty = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
    except OSError:
        return False

    old_attributes = termios.tcgetattr(tty)
    try:
        # Read the answer byte by byte without echoing it
        attributes = termios.tcgetattr(tty)
        attributes[3] &= ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(tty, termios.TCSANOW, attributes)

        # i=31: an id for the answer, a=q: query only, t=d: direct data,
        # f=24: RGB, s=1,v=1: a 1x1 pixel image, AAAA: its base64 data
        os.write(tty, b'\033_Gi=31,s=1,v=1,a=q,t=d,f=24;AAAA\033\\\033[c')

        answer = read_device_attributes_answer(tty, timeout)
    finally:
        termios.tcsetattr(tty, termios.TCSANOW, old_attributes)
        os.close(tty)

    return b'\033_Gi=31;OK' in answer


def read_device_attributes_answer(tty: int, timeout: float) -> bytes:
    """
    Reads from the terminal until the device attributes answer
    (`ESC [ ? ... c`) arrived or `timeout` seconds passed. Returns everything
    read, including any answers that came before it.
    """

    answer = b''
    deadline = time.monotonic() + timeout

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return answer

        readable, _, _ = select.select([tty], [], [], remaining)
        if not readable:
            return answer

        answer += os.read(tty, 1024)

        if b'\033[?' in answer and answer.endswith(b'c'):
            return answer


def fit_image_to_terminal(png: bytes, max_columns: int) -> tuple[int, int]:
    """
    Returns the size in terminal cells (columns, rows) to show the image at.
    The image is as wide as the terminal or `max_columns`, but small enough
    that it fits on the screen without scrolling.
    """

    terminal_size = shutil.get_terminal_size()
    image_width, image_height = struct.unpack('>II', png[16:24])
    cell_width, cell_height = get_terminal_cell_size()

    columns = min(terminal_size.columns, max_columns)
    rows = math.ceil(
        columns * cell_width * image_height / image_width / cell_height
    )

    # Leave a line for the shell prompt
    max_rows = terminal_size.lines - 1
    if rows > max_rows:
        columns = math.floor(columns * max_rows / rows)
        rows = max_rows

    return columns, rows


def get_terminal_cell_size() -> tuple[float, float]:
    """
    Returns the size of a terminal cell in pixels as (width, height).
    """

    try:
        packed = fcntl.ioctl(sys.stdout.fileno(), termios.TIOCGWINSZ, bytes(8))
        rows, columns, width, height = struct.unpack('HHHH', packed)
        if rows and columns and width and height:
            return width / columns, height / rows
    except OSError:
        pass

    # The terminal doesn't report pixel sizes. Cells are usually about twice
    # as tall as they are wide.
    return 1, 2


def show_png_in_terminal(png: bytes, columns: int, rows: int) -> None:
    """
    Shows a PNG image with the kitty graphics protocol, scaled to the given
    size in terminal cells, and moves the cursor below it.

    Terminals differ in where they leave the cursor after an image, so this
    reserves the rows first, draws the image without moving the cursor and
    then moves the cursor below the image itself.
    """

    # Scroll the screen if needed, so all rows of the image are visible, and
    # go back to the first row
    sys.stdout.write('\n' * rows + f'\033[{rows}A\r')

    data = base64.standard_b64encode(png).decode('ascii')
    chunks = [data[i : i + 4096] for i in range(0, len(data), 4096)]

    for index, chunk in enumerate(chunks):
        more = 1 if index < len(chunks) - 1 else 0
        if index == 0:
            # a=T: transmit and show, f=100: PNG, q=2: no responses,
            # C=1: don't move the cursor
            control = f'a=T,f=100,q=2,C=1,c={columns},r={rows},m={more}'
        else:
            control = f'm={more}'

        escape = f'\033_G{control};{chunk}\033\\'

        if os.environ.get('TMUX'):
            # tmux only passes escape sequences through when they are wrapped
            escape = (
                '\033Ptmux;' + escape.replace('\033', '\033\033') + '\033\\'
            )

        sys.stdout.write(escape)

    sys.stdout.write(f'\033[{rows}B\r')
    sys.stdout.flush()


#
# Curve model. Keep in sync with src/mouse/input_accelerator.c
#


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


def sigmoid_function(x: float, limit: float, acc_rate: float) -> float:
    """
    Standard sigmoid function
    """

    return (limit) / (1 + math.exp(-acc_rate * x))


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


#
# Functions used by the jupyter notebook
#

CurveName = Literal['sigmoid_pure', 'sigmoid']


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


if __name__ == '__main__':
    app()
