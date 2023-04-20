#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

from stop_distance import StopDistCalculator, LateralDistCalculator, Param

TIME_DELAY = 0.5

# ---------- Global Parameters ----------
MAX_ACC_W = 1.0
MIN_ACC_W = -1.0
MAX_JERK_W = 1.0
MIN_JERK_W = -1.0

MAX_ACC_M = 2.0
MIN_ACC_M = -2.0
MAX_JERK_M = 2.0
MIN_JERK_M = -2.0

MAX_ACC_S = 3.0
MIN_ACC_S = -5.0
MAX_JERK_S = 1000.0
MIN_JERK_S = -3.0

MAX_LAT_ACC = 1.0
MAX_LAT_JERK = 1.0
TIME_TO_STEER = 2.0  # time to inform other vehicle before steering [s]

LATERAL_MARGIN_OBJECT_IN_LANE = 3.0


# ---------- Local Parameters ----------

OBSTACLE_STOP__DISTANCE_MARGIN = 5.0

AVOIDANCE__SHIFT_DISTANCE = 2.0
AVOIDANCE__PREPARE_TIME = 0.0  # 考慮する必要なし
AVOIDANCE__NPC_DECEL_START_TIME_AFTER_STEERING = 3.0

TRAFFIC_LIGHT__STOP_DISTANCE = 5.0

INTERSECTION__RIGHT_MAX_SPEED = 20.0 / 3.6
INTERSECTION__RIGHT_DISTANCE = 25.0  # [m]
INTERSECTION__RIGHT_NPC_DECEL_START_TIME = 4.0  # [s]

INTERSECTION__LEFT_MAX_SPEED = 15.0 / 3.6
INTERSECTION__LEFT_DISTANCE = 15.0  # [m]
INTERSECTION__LEFT_NPC_DECEL_START_TIME = 4.0  # [s]

LANE_CHANGE__SHIFT_DISTANCE = 3.5
LANE_CHANGE__NPC_DECEL_START_TIME_AFTER_STEERING = 3.0


CROSSWALK__STOPLINE_TO_PEDESTRIAN_DISTANCE = 15.0

# ---------- For calculation ----------


KMPH_TO_MPS = 1.0 / 3.6
V0_RANGE_KMPH = np.array([120, 100, 80, 60, 40, 20])
V0_RANGE_MPS = V0_RANGE_KMPH * KMPH_TO_MPS


VNPC_RANGE_KMPH = np.array([120, 100, 80, 60, 40, 20])
VNPC_RANGE_MPS = VNPC_RANGE_KMPH * KMPH_TO_MPS

VNPC_MOTORBIKE_RANGE_KMPH = np.array([60, 50, 40, 30, 20, 10])
VNPC_MOTORBIKE_RANGE_MPS = VNPC_MOTORBIKE_RANGE_KMPH * KMPH_TO_MPS

weak_param = Param(MAX_ACC_W, MIN_ACC_W, MAX_JERK_W, MIN_JERK_W, TIME_DELAY)
mid_param = Param(MAX_ACC_M, MIN_ACC_M, MAX_JERK_M, MIN_JERK_M, TIME_DELAY)
strong_param = Param(MAX_ACC_S, MIN_ACC_S, MAX_JERK_S, MIN_JERK_S, TIME_DELAY)

DIST_RANGE_M = np.array([600, 500, 400, 300, 200, 160, 100])


def calc_stop_dist(param, stop_dist_margin=0.0):
    sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(param)
    stop_dist_arr = []
    for v in V0_RANGE_MPS:
        sdc.set_individual_param(v0=v)
        xe, t = sdc.calc()
        stop_dist_arr.append(xe + stop_dist_margin)
    return stop_dist_arr


def lateral_shift_dist(lateral_shift_dist, time_to_steer):
    ldc = LateralDistCalculator(
        max_lat_acc=MAX_LAT_ACC, max_lat_jerk=MAX_LAT_JERK, lat_dist=lateral_shift_dist)
    lat_time = ldc.calc_required_time()

    required_time = lat_time + TIME_DELAY + time_to_steer

    dist_arr_const_v = []
    for v in V0_RANGE_MPS:
        d = v * required_time
        dist_arr_const_v.append(d)

    sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(weak_param)
    dist_arr_decel_v = []
    for v in V0_RANGE_MPS:
        d, _ = sdc.calc_dist_after_decel_time(v0=v, t=required_time)
        dist_arr_decel_v.append(d)
    return (dist_arr_const_v, dist_arr_decel_v)


def calc_intersection_passing_time(ego_param, max_speed, intersection_distance):
    sdc = StopDistCalculator(v0=max_speed, a0=0.0, v_end=0.0, print=False)
    sdc.set_param(ego_param)
    sdc.set_individual_param(delay_time=0.0)  # overwrite with 0
    decel_dist, time_in_decel = sdc.calc()
    if decel_dist > intersection_distance:
        print("unsupported speed profile. intersection distance reaches before max speed")
        return 0.0
    dist_with_max_speed = intersection_distance - decel_dist
    time_in_const_speed = dist_with_max_speed / max_speed
    intersection_passing_time = TIME_DELAY + time_in_const_speed + time_in_decel
    return intersection_passing_time


def intersection(intersection_passing_time, vrange):
    dist_arr = []
    for v in vrange:
        d = v * intersection_passing_time
        dist_arr.append(d)
    return dist_arr


def intersection_npc_decel(intersection_passing_time, time_to_start_decel, vrange):
    t_const_speed = time_to_start_decel
    t_deceleration = intersection_passing_time - time_to_start_decel
    print("[intersection] (npc_decel) passing_time=%.3f [s], npc_decel_start=%.3f [s]" % (
        intersection_passing_time, time_to_start_decel))
    if t_deceleration < 0.0:
        print("[intersection] (npc_decel) warn! npc_decel_start time is too late")

    npc_sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    npc_sdc.set_param(weak_param)
    dist_arr = []
    v_arr = []
    for v in vrange:
        npc_sdc.set_individual_param(v0=v)
        dist_decel, v2 = npc_sdc.calc_dist_after_decel_time(
            v0=v, t=t_deceleration)
        d = v * t_const_speed + dist_decel
        dist_arr.append(d)
        v_arr.append(v2 / KMPH_TO_MPS)
    print("[intersection] (npc_decel) target vehicle speed after decel [km/h] = " +
          ", ".join(map(str, v_arr)))
    return dist_arr


def calc_npc_travel_dist_until_speed_match(ego_param, npc_decel_start_time):
    npc_sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    npc_sdc.set_param(weak_param)

    ego_sdc = StopDistCalculator(v0=0.0, a0=0.0, v_end=0.0, print=False)
    ego_sdc.set_param(ego_param)

    for ve0 in np.array([120, 100, 80, 60, 40, 20, 0]) * KMPH_TO_MPS:
        npc_dist_arr = []
        for vn0 in np.array([120, 100, 80, 60, 40, 20]) * KMPH_TO_MPS:
            t_total = 0.0
            dt = 0.01
            xe, ve, ae = 0.0, ve0, 0.0
            xn, vn, an = 0.0, vn0, 0.0
            while ve < vn:
                xe, ve, ae = ego_sdc.update_with_constraint(
                    t=dt, jerk=ego_param.max_jerk, a0=ae, v0=ve, x0=xe)
                # print("T=" + str(t_total) +"  ve=" + str(ve) + ", jerk = " + str(weak_param.max_jerk))
                npc_jeck = 0.0
                if t_total > npc_decel_start_time:
                    npc_jeck = weak_param.min_jerk
                xn, vn, an = npc_sdc.update_with_constraint(
                    t=dt, jerk=npc_jeck, a0=an, v0=vn, x0=xn)
                t_total = t_total + dt
            npc_dist_arr.append(xn)
        print("npc travel dist (ve=%.2f) [m] = " % (
            ve0 / KMPH_TO_MPS) + ", ".join(map(str, npc_dist_arr)))

    return xn


def calc_yaw_error_requirements(dist_ranges, lateral_margin, perception_erorrs):
    for perr in perception_erorrs:
        yaw_requirements = []
        for d in dist_ranges:
            margin_dist = max(lateral_margin - perr, 0.0)
            yaw = math.degrees(math.asin(margin_dist / d))
            yaw_requirements.append(yaw)
        print("localization yaw requirement (perception error=%.2f) [m] = " % (
            perr) + ", ".join(map(str, yaw_requirements)))


# ===== obstacle stop =====
def obstacle_stop():
    # stop distance
    dist = calc_stop_dist(
        param=weak_param, stop_dist_margin=OBSTACLE_STOP__DISTANCE_MARGIN)
    print("[obstacle stop] weak-brake stop distance [m] = " +
          ", ".join(map(str, dist)))

    dist = calc_stop_dist(
        param=mid_param, stop_dist_margin=OBSTACLE_STOP__DISTANCE_MARGIN)
    print("[obstacle stop] mid-brake stop distance [m] = " +
          ", ".join(map(str, dist)))

    dist = calc_stop_dist(param=strong_param,
                          stop_dist_margin=OBSTACLE_STOP__DISTANCE_MARGIN)
    print("[obstacle stop] strong-brake stop distance [m] = " +
          ", ".join(map(str, dist)))

    # perception-position and yaw-error
    perception_error = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
    calc_yaw_error_requirements(
        DIST_RANGE_M, LATERAL_MARGIN_OBJECT_IN_LANE, perception_error)


#  ===== avoidance =====
def avoidance():
    (dists_const_v, dists_decel_v) = lateral_shift_dist(lateral_shift_dist=AVOIDANCE__SHIFT_DISTANCE,
                                                        time_to_steer=AVOIDANCE__PREPARE_TIME)
    print("[obstacle avoidance] lateral shift distance (const speed) [m] = " +
          ", ".join(map(str, dists_const_v)))
    print("[obstacle avoidance] lateral shift distance (decel speed) [m] = " +
          ", ".join(map(str, dists_decel_v)))

    # perception-position and yaw-error
    perception_error = (0.0, 0.25, 0.5, 0.75)
    AVOIDANCE__LATERAL_MARGIN = 1.0
    AVOIDANCE__DIST_RANGE = (200, 150, 100, 50)
    calc_yaw_error_requirements(
        AVOIDANCE__DIST_RANGE, AVOIDANCE__LATERAL_MARGIN, perception_error)


#  ===== traffic light =====
def traffic_light():
    dist = calc_stop_dist(
        param=weak_param, stop_dist_margin=TRAFFIC_LIGHT__STOP_DISTANCE)
    print("[traffic light] weak-brake stop distance [m] = " +
          ", ".join(map(str, dist)))

    dist = calc_stop_dist(
        param=mid_param, stop_dist_margin=TRAFFIC_LIGHT__STOP_DISTANCE)
    print("[traffic light] mid-brake stop distance [m] = " +
          ", ".join(map(str, dist)))

    dist = calc_stop_dist(param=strong_param,
                          stop_dist_margin=TRAFFIC_LIGHT__STOP_DISTANCE)
    print("[traffic light] strong-brake stop distance [m] = " +
          ", ".join(map(str, dist)))


#  ===== intersection (turn right) =====
def intersection_right():
    passing_time_weak = calc_intersection_passing_time(ego_param=weak_param,
                                                       max_speed=INTERSECTION__RIGHT_MAX_SPEED,
                                                       intersection_distance=INTERSECTION__RIGHT_DISTANCE)

    passing_time_mid = calc_intersection_passing_time(ego_param=mid_param,
                                                      max_speed=INTERSECTION__RIGHT_MAX_SPEED,
                                                      intersection_distance=INTERSECTION__RIGHT_DISTANCE)

    dist = intersection(passing_time_weak, VNPC_RANGE_MPS)
    print("[intersection] weak acc. dist [m] = " + ", ".join(map(str, dist)))
    dist = intersection(passing_time_mid, VNPC_RANGE_MPS)
    print("[intersection] mid acc. dist [m] = " + ", ".join(map(str, dist)))

    dist = intersection_npc_decel(
        passing_time_weak, time_to_start_decel=INTERSECTION__RIGHT_NPC_DECEL_START_TIME, vrange=VNPC_RANGE_MPS)
    print("[intersection] weak acc with NPC decel. dist [m] = " +
          ", ".join(map(str, dist)))
    dist = intersection_npc_decel(
        passing_time_mid, time_to_start_decel=INTERSECTION__RIGHT_NPC_DECEL_START_TIME, vrange=VNPC_RANGE_MPS)
    print("[intersection] mid acc with NPC decel. dist [m] = " +
          ", ".join(map(str, dist)))


#  ===== intersection (turn left) =====
def intersection_left():
    passing_time_weak = calc_intersection_passing_time(ego_param=weak_param,
                                                       max_speed=INTERSECTION__LEFT_MAX_SPEED,
                                                       intersection_distance=INTERSECTION__LEFT_DISTANCE)

    dist = intersection(passing_time_weak, VNPC_MOTORBIKE_RANGE_MPS)
    print("[intersection-left] weak acc. dist [m] = " +
          ", ".join(map(str, dist)))

    dist = intersection_npc_decel(
        passing_time_weak, time_to_start_decel=INTERSECTION__LEFT_NPC_DECEL_START_TIME, vrange=VNPC_MOTORBIKE_RANGE_MPS)
    print("[intersection-left] weak acc with NPC decel. dist [m] = " +
          ", ".join(map(str, dist)))

    passing_time_mid = calc_intersection_passing_time(ego_param=mid_param,
                                                      max_speed=INTERSECTION__LEFT_MAX_SPEED,
                                                      intersection_distance=INTERSECTION__LEFT_DISTANCE)

    dist = intersection(passing_time_mid, VNPC_MOTORBIKE_RANGE_MPS)
    print("[intersection-left] mid acc. dist [m] = " + ", ".join(map(str, dist)))

    dist = intersection_npc_decel(
        passing_time_mid, time_to_start_decel=INTERSECTION__LEFT_NPC_DECEL_START_TIME, vrange=VNPC_MOTORBIKE_RANGE_MPS)
    print("[intersection-left] mid acc with NPC decel. dist [m] = " +
          ", ".join(map(str, dist)))

#  ===== lane change =====

def lane_change():
    # 自車が低速から加速。数秒後にNPCが減速。このとき2台が等速になるまでNPCが何m走行するか
    calc_npc_travel_dist_until_speed_match(weak_param, 100.0)
    calc_npc_travel_dist_until_speed_match(weak_param, LANE_CHANGE__NPC_DECEL_START_TIME_AFTER_STEERING)
    calc_npc_travel_dist_until_speed_match(mid_param, 1000.0)
    calc_npc_travel_dist_until_speed_match(mid_param, LANE_CHANGE__NPC_DECEL_START_TIME_AFTER_STEERING)
    calc_npc_travel_dist_until_speed_match(strong_param, 1000.0)
    calc_npc_travel_dist_until_speed_match(strong_param, LANE_CHANGE__NPC_DECEL_START_TIME_AFTER_STEERING)


#  ===== crosswalk =====
def crosswalk():
    # stop distance
    dist = calc_stop_dist(
        param=weak_param, stop_dist_margin=CROSSWALK__STOPLINE_TO_PEDESTRIAN_DISTANCE)
    print("[crosswalk] weak-brake stop distance [m] = " +
          ", ".join(map(str, dist)))

    dist = calc_stop_dist(
        param=mid_param, stop_dist_margin=CROSSWALK__STOPLINE_TO_PEDESTRIAN_DISTANCE)
    print("[crosswalk] mid-brake stop distance [m] = " +
          ", ".join(map(str, dist)))



if __name__ == '__main__':

    obstacle_stop()

    # avoidance()

    # traffic_light()
    # intersection_right()

    # intersection_left()
    # lane_change()

    # crosswalk()
