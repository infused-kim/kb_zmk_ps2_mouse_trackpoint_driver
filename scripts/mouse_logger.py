#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "pynput",
#     "typer",
# ]
# ///
#
# Find the venv with:
# uv python find --script scripts/mouse_logger.py
"""
Script that logs mouse movements to debug mouse performance issues.
"""

import statistics
import threading
import time

import typer
from pynput import mouse

# How long the mouse has to stop moving before the summary is printed
SUMMARY_DELAY_MS = 500

app = typer.Typer(add_completion=False)


@app.command()
def main() -> None:
    """
    Log mouse movements to debug mouse performance issues.

    Prints the distance and time interval of every movement, and a summary of
    the time intervals whenever the mouse stops moving.
    """

    logger = MouseLogger()
    with mouse.Listener(on_move=logger.on_move) as listener:
        listener.join()


class MouseLogger:
    """
    Keeps the state between mouse movements.
    """

    def __init__(self) -> None:
        # To calculate the mouse movement
        self.prev_x = 0
        self.prev_y = 0
        self.prev_time = time.time()
        self.prev_time_interval = 0.0

        # For the summary task
        self.time_intervals: list[float] = []
        self.lock = threading.Lock()
        self.delayed_task_thread: threading.Timer | None = None

    def on_move(self, x: int, y: int) -> None:
        distance_x = x - self.prev_x
        distance_y = y - self.prev_y

        current_time = time.time()
        time_interval = current_time - self.prev_time
        # Change compared to the previous interval, e.g. -20% from 10 to 8 ms
        try:
            time_interval_change = (
                f'({time_interval / self.prev_time_interval - 1:+9.2%})'
            )
        except ZeroDivisionError:
            time_interval_change = ''

        with self.lock:
            self.time_intervals.append(time_interval)

        typer.echo(
            f'Mouse moved distance - '
            f'X: {distance_x:+7.2f}, '
            f'Y: {distance_y:+7.2f}, '
            f'time interval: {time_interval * 1000:5.2f} ms '
            f'{time_interval_change}'
        )

        self.prev_x, self.prev_y = x, y
        self.prev_time = current_time
        self.prev_time_interval = time_interval

        self.schedule_summary_task(SUMMARY_DELAY_MS)

    def schedule_summary_task(self, delay_ms: int) -> None:
        if (
            self.delayed_task_thread is not None
            and self.delayed_task_thread.is_alive()
        ):
            self.delayed_task_thread.cancel()

        self.delayed_task_thread = threading.Timer(
            delay_ms / 1000, self.summary_task
        )
        self.delayed_task_thread.start()

    def summary_task(self) -> None:
        summary_intervals = []
        with self.lock:
            if self.time_intervals:
                # Ignore the first interval as it includes the time since the
                # previous movement session.
                #
                # And ignore the last 3 as they can be unintentional TP drift
                # movements.
                summary_intervals = self.time_intervals[1:-3]
                self.time_intervals.clear()

        # Too few movements for a summary
        if not summary_intervals:
            return

        median_interval = statistics.median(summary_intervals)
        average_interval = statistics.mean(summary_intervals)
        typer.echo(
            f'Time between movements '
            f'median: {median_interval * 1000:.2f} ms; '
            f'average: {average_interval * 1000:.2f} ms; '
            f'\n\n\n'
        )


if __name__ == '__main__':
    app()
