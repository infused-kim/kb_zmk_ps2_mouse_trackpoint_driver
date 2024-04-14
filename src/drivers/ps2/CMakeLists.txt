# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_PS2_GPIO app PRIVATE ps2_gpio.c)
target_sources_ifdef(CONFIG_PS2_UART app PRIVATE ps2_uart.c)
