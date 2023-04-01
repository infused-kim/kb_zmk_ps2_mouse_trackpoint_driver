/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

int zmk_ps2_set_sampling_rate_decr();
int zmk_ps2_set_sampling_rate_incr();

int zmk_ps2_tp_sensitivity_incr(float incr_amount);
int zmk_ps2_tp_sensitivity_decr(float decr_amount);

int zmk_ps2_tp_neg_inertia_incr(int incr_amount);
int zmk_ps2_tp_neg_inertia_decr(int decr_amount);

int zmk_ps2_tp_value6_upper_plateau_speed_incr(int incr_amount);
int zmk_ps2_tp_value6_upper_plateau_speed_decr(int decr_amount);

int zmk_ps2_tp_pts_threshold_incr(int incr_amount);
int zmk_ps2_tp_pts_threshold_decr(int decr_amount);
