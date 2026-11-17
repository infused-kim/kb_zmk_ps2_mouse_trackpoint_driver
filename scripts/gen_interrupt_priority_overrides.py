#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "devicetree",
#     "typer",
# ]
# ///
#
# Find the venv with:
# uv python find --script scripts/gen_interrupt_priority_overrides.py
"""
Script for generating interrupt adjustments for zephyr dts files.
"""

from pathlib import Path
from typing import Annotated

import typer
from devicetree import dtlib

app = typer.Typer(add_completion=False)


def get_interrupt_nodes(dt: dtlib.DT) -> list[dtlib.Node]:
    return [node for node in dt.node_iter() if 'interrupts' in node.props]


def get_node_label(node: dtlib.Node) -> str:
    label = node.labels[0]
    return label


def gen_interrupt_adjustment_str(node: dtlib.Node, priority_incr: int) -> str:
    label = get_node_label(node)
    interrupt_val1, interrupt_priority = node.props['interrupts'].to_nums()

    if label == 'gpiote':
        new_priority = 0
    else:
        new_priority = interrupt_priority + priority_incr

    adj_str = f"""
&{label} {{
    interrupts = < {interrupt_val1} {new_priority} >;
}};
    """
    return adj_str


def gen_interrupt_adjustment(dts_path: Path, priority_incr: int) -> str:
    dt = dtlib.DT(str(dts_path))
    interrupt_nodes = get_interrupt_nodes(dt)

    adjustments = ''
    for node in interrupt_nodes:
        adj_str = gen_interrupt_adjustment_str(node, priority_incr)
        if get_node_label(node) == 'gpiote':
            adjustments = adj_str + adjustments
        else:
            adjustments += adj_str

    return adjustments


@app.command()
def main(
    dts_path: Annotated[
        Path,
        typer.Argument(metavar='DTS_PATH', help='Path to the zephyr.dts file'),
    ],
    priority_incr: Annotated[
        int,
        typer.Option(
            '--priority-incr',
            '-p',
            help='By how much to increase the priority',
        ),
    ] = 2,
) -> None:
    """
    Generate interrupt adjustments for zephyr.dts files.
    """

    adj = gen_interrupt_adjustment(dts_path, priority_incr)
    typer.echo(adj)


if __name__ == '__main__':
    app()
