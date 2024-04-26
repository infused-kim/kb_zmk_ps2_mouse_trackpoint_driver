#!/usr/bin/env python3
"""
Script that logs mouse movements to debug mouse performance issues.

Requirements:
    - pip install pynput
"""

from pynput import mouse
import time
import threading
import statistics

# Global variables to calculate mouse movement
prev_x, prev_y = 0, 0
prev_time = time.time()

# Variables for summary task
time_intervals = []
lock = threading.Lock()
delayed_task_thread = None
prev_summary_interval = None


def summary_task():
    with lock:
        if time_intervals:
            median_interval = statistics.median(time_intervals)
            print(
                f'Median time between movements: {median_interval * 1000:.2f} '
                f'ms\n\n\n'
            )
            time_intervals.clear()


def schedule_summary_task(delay_ms):
    global delayed_task_thread
    if delayed_task_thread is not None and delayed_task_thread.is_alive():
        delayed_task_thread.cancel()

    delayed_task_thread = threading.Timer(delay_ms / 1000, summary_task)
    delayed_task_thread.start()


def on_move(x, y):
    global prev_x, prev_y, prev_time

    distance_x = x - prev_x
    distance_y = y - prev_y

    current_time = time.time()
    time_interval = current_time - prev_time

    with lock:
        time_intervals.append(time_interval)

    print(
        f'Mouse moved distance - '
        f'X: {distance_x:+7.2f}, '
        f'Y: {distance_y:+7.2f}, '
        f'time interval: {time_interval * 1000:5.2f} ms'
    )

    prev_x, prev_y = x, y
    prev_time = current_time

    schedule_summary_task(500)


with mouse.Listener(on_move=on_move) as listener:
    listener.join()
