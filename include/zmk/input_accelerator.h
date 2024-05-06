/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#include <zephyr/device.h>

#pragma once

struct input_accelerator_result {
    int32_t x;
    int32_t y;
};

struct input_accelerator_result zmk_accelerate_input(const struct device *accelerator_dev,
                                                     int32_t x, int32_t y);
