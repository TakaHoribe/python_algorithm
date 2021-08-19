#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from matplotlib import pyplot as plt




wheelbase = 1.335

def steer_to_yawrate(vx, steer):
    return vx * np.tan(steer) / wheelbase

def yawrate_to_steer(vx, wz):
    return np.arctan(wheelbase * wz / vx)

if __name__ == '__main__':

    steer_arr = np.array([0.363, 0.808, 0.609, 0.518, 0.799])
    vx_can_arr = np.array([1.520, 1.438, 1.330, 2.035, 1.927])
    wz_imu_arr = np.array([0.427, 0.964, 0.635, 0.814, 1.254])
    yawrate_from_steer = steer_to_yawrate(vx_can_arr, steer_arr)
    steer_from_imu = yawrate_to_steer(vx_can_arr, wz_imu_arr)

    # plt.plot(steer_arr, marker="o", label="steer can")
    # plt.plot(steer_from_imu, marker="o", label="steer from imu")
    # plt.plot(steer_from_imu / steer_arr, marker="o", label="ratio (imu/can)")
    # plt.legend()
    # plt.grid()
    # plt.show()

    # plt.plot(steer_arr, steer_arr / steer_from_imu, marker="o", label="")
    # plt.xlim((0.0, 1.0))
    # plt.legend()
    # plt.grid()
    # plt.show()

    coeff = np.polyfit(steer_arr, steer_arr / steer_from_imu, 1)
    print("coeff = ", coeff)
    poly1d_fn = np.poly1d(coeff)
    plt.plot(steer_arr, steer_arr / steer_from_imu, marker="o", label="")
    plt.plot(steer_arr, poly1d_fn(steer_arr), "--k", label="")
    plt.xlim((0.0, 1.0))
    plt.legend()
    plt.grid()
    plt.show()
