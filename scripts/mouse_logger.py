#!/usr/bin/env python3

from pynput import mouse
import time

# Define global variables to store the previous mouse position
prev_x, prev_y = 0, 0
prev_time = time.time()

# Define a callback function to handle mouse movement events
def on_move(x, y):
    global prev_x, prev_y, prev_time

    # Calculate the distance moved
    distance_x = x - prev_x
    distance_y = y - prev_y

    # Get the current time
    current_time = time.time()

    # Calculate the time interval of movement
    time_interval = current_time - prev_time

    # Print the movement distance and time interval
    print('Mouse moved distance - X: {0}, Y: {1}, time interval: {2}'.format(distance_x, distance_y, time_interval))

    # Update the previous position and time
    prev_x, prev_y = x, y
    prev_time = current_time

# Create a mouse listener
with mouse.Listener(on_move=on_move) as listener:
    listener.join()
