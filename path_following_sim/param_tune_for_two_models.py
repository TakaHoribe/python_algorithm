#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from ddeint import ddeint
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse

import simulator_core

velocity = 0.2

kp_range = np.arange(0.1, 1, 0.5)
kd_range = np.arange(0.1, 1, 0.5)

p_gain = 0.0 # set later
d_gain = 0.0 # set later

# Model-A
param_A = simulator_core.ModelParams(v=velocity, kp=p_gain, kd=d_gain, tau=0.1, delay = 0.1)

# Model-B
param_B = simulator_core.ModelParams(v=velocity, kp=p_gain, kd=d_gain, tau=0.5, delay = 0.5)

x0 = [0.1, 0.0, 0.0]
t = np.arange(0, 10, 0.01)

SIM_WITH_DELAY = True


def generate_fig(res_x_A, res_x_B, kp, kd):
    plt.rcParams["font.size"] = 15
    plt.rcParams["lines.linewidth"] = 2

    plt.figure(figsize=(10, 5))
    plt.subplot(1, 1, 1)
    plt.plot(t, res_x_A[:,0], label='model A')
    plt.plot(t, res_x_B[:,0], label='model B')
    plt.xlabel("Time [sec]")
    plt.ylabel("Lateral error [m]")
    plt.legend()

    plt.title("kp = {}, kd = {}".format(kp, kd))

    plt.grid()

if __name__ == '__main__':

    total_num = len(kp_range) * len(kd_range)
    print("total num = ", total_num)
    if total_num > 50:
        print("total sim num > 50. Too much. stop computing.")
        sys.exit()

    sim_A = simulator_core.Simulator()
    sim_B = simulator_core.Simulator()

    i = 0
    for kp in kp_range:
        for kd in kd_range:
            param_A.p_gain = kp
            param_A.d_gain = kd
            res_x_A, res_u_A = sim_A.run(x0, t, SIM_WITH_DELAY, param_A)

            param_B.p_gain = kp
            param_B.d_gain = kd
            res_x_B, res_u_B = sim_B.run(x0, t, SIM_WITH_DELAY, param_B)

            generate_fig(res_x_A, res_x_B, kp, kd)

            i = i + 1
            print("sim: {}/{} is done.".format(i, total_num))


    plt.show()

    # param_A = simulator.ModelParams(velocity, time_constant, p_gain, d_gain, time_delay)
    # sim_A = simulator.Simulator()
    # res_x, res_u = sim.run(x0, t, SIM_WITH_DELAY, param)

    # plot_result(res_x, res_u)
