/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

int zmk_ps2_tp_sensitivity_incr(int incr_amount);
int zmk_ps2_tp_sensitivity_decr(int decr_amount);

int zmk_ps2_tp_neg_inertia_incr(int incr_amount);
int zmk_ps2_tp_neg_inertia_decr(int decr_amount);

int zmk_ps2_tp_value6_upper_plateau_speed_incr(int incr_amount);
int zmk_ps2_tp_value6_upper_plateau_speed_decr(int decr_amount);

int zmk_ps2_tp_pts_threshold_incr(int incr_amount);
int zmk_ps2_tp_pts_threshold_decr(int decr_amount);
