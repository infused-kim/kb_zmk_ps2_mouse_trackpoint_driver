/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

int zmk_mouse_ps2_tp_sensitivity_change(int amount);
int zmk_mouse_ps2_tp_neg_inertia_change(int amount);
int zmk_mouse_ps2_tp_value6_upper_plateau_speed_change(int amount);
int zmk_mouse_ps2_tp_pts_threshold_change(int amount);

void zmk_mouse_ps2_scroll_mode_set(bool enabled);
void zmk_mouse_ps2_scroll_mode_enable();
void zmk_mouse_ps2_scroll_mode_disable();
void zmk_mouse_ps2_scroll_mode_toggle();
