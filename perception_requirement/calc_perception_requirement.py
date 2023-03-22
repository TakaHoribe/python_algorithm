#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

from stop_distance import StopDistCalculator, LateralDistCalculator, Param

TIME_DELAY = 0.5

MAX_ACC_W = 1.0
MIN_ACC_W = -3.0
MAX_JERK_W = 1.0
MIN_JERK_W = -1.0

MAX_ACC_S = 3.0
MIN_ACC_S = -3.0
MAX_JERK_S = 10.0
MIN_JERK_S = -1.5

MAX_LAT_ACC = 1.0
MAX_LAT_JERK = 1.0
TIME_TO_STEER = 2.0  # time to inform other vehicle before steering [s]

KMPH_TO_MPS = 1.0 / 3.6
V0_RANGE_KMPH = np.array([120, 100, 80, 60, 40, 20])
V0_RANGE_MPS = V0_RANGE_KMPH * KMPH_TO_MPS


VNPC_RANGE_KMPH = np.array([120, 100, 80, 60, 40, 20])
VNPC_RANGE_MPS = VNPC_RANGE_KMPH * KMPH_TO_MPS

weak_param = Param()
weak_param.max_acc = MAX_ACC_W
weak_param.min_acc = MIN_ACC_W
weak_param.max_jerk = MAX_JERK_W
weak_param.min_jerk = MIN_JERK_W
weak_param.delay_time = TIME_DELAY

strong_param = Param()
strong_param.max_acc = MAX_ACC_S
strong_param.min_acc = MIN_ACC_S
strong_param.max_jerk = MAX_JERK_S
strong_param.min_jerk = MIN_JERK_S
strong_param.delay_time = TIME_DELAY



def weak_stop_dist(stop_dist_margin=0.0):
    sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(weak_param)
    stop_dist_arr = []
    for v in V0_RANGE_MPS:
        sdc.set_params(v0=v)
        xe, t = sdc.calc()
        stop_dist_arr.append(xe + stop_dist_margin)
    print("weak stop distance [m] = " + ", ".join(map(str, stop_dist_arr)))

def strong_stop_dist(stop_dist_margin=0.0):
    sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(strong_param)
    stop_dist_arr = []
    for v in V0_RANGE_MPS:
        sdc.set_params(v0=v)
        xe, t = sdc.calc()
        stop_dist_arr.append(xe + stop_dist_margin)
    print("strong stop distance [m] = " + ", ".join(map(str, stop_dist_arr)))

def lateral_shift_dist(lateral_shift_dist, time_to_steer):
    ldc = LateralDistCalculator(max_lat_acc=MAX_LAT_ACC, max_lat_jerk=MAX_LAT_JERK, lat_dist=lateral_shift_dist)
    lat_time = ldc.calc_required_time()

    required_time = lat_time + TIME_DELAY + time_to_steer

    dist_arr_const_v = []
    for v in V0_RANGE_MPS:
        d = v * required_time
        dist_arr_const_v.append(d)
    print("lateral shift distance (const speed) [m] = " + ", ".join(map(str, dist_arr_const_v)))

    sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(weak_param)
    dist_arr_decel_v = []
    for v in V0_RANGE_MPS:
        d, v2 = sdc.calc_dist_after_decel_time(v0=v, t=required_time)
        dist_arr_decel_v.append(d)
    print("lateral shift distance (decel speed) [m] = " + ", ".join(map(str, dist_arr_decel_v)))

def intersection(time_to_pass):
    dist_arr = []
    for v in VNPC_RANGE_MPS:
        d = v * time_to_pass
        dist_arr.append(d)
    print("intersection required distance [m] = " + ", ".join(map(str, dist_arr)))

def intersection_npc_decel(time_to_pass, time_to_start_decel):
    sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(weak_param)

    t_const_speed = time_to_start_decel
    t_deceleration = time_to_pass - time_to_start_decel
    
    dist_arr = []
    v_arr = []
    for v in VNPC_RANGE_MPS:
        sdc.set_params(v0=v)
        dist_decel, v2 = sdc.calc_dist_after_decel_time(v0=v, t=t_deceleration)
        d = v * t_const_speed + dist_decel
        dist_arr.append(d)
        v_arr.append(v2 / KMPH_TO_MPS )
    print("intersection required distance (decel) [m] = " + ", ".join(map(str, dist_arr)))
    print("intersection required distance (decel) [km/h] = " + ", ".join(map(str, v_arr)))




if __name__ == '__main__':

    # obstacle stop
    weak_stop_dist(stop_dist_margin=5.0)
    strong_stop_dist(stop_dist_margin=5.0)

    # avoidance
    lateral_shift_dist(lateral_shift_dist=2.0, time_to_steer=2.0)

    # traffic light
    weak_stop_dist(stop_dist_margin=0.0)
    strong_stop_dist(stop_dist_margin=0.0)

    # intersection
    intersection(time_to_pass=9.0)
    intersection_npc_decel(time_to_pass=9.0, time_to_start_decel=4.5)