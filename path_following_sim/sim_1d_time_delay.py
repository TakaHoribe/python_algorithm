#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ddeint import ddeint
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse

import simulator_core

# sim parameters
SIM_WITH_DELAY = False
time_constant = 0.01
time_delay = 0.3
velocity = 0.2
p_gain = 1.0
d_gain = 0.7
x0 = [0.1, 0.0, 0.0]

# option for SIM_WITH_DELAY
parser = argparse.ArgumentParser()
parser.add_argument('-D', '--sim_with_delay', action='store_true', help='set if sim with delay time is needed')
parser.add_argument('-v', '--velocity', help='sim parameter: velocity', type=float)
parser.add_argument('-d', '--time_delay', help='sim parameter: delay time', type=float)
parser.add_argument('-t', '--time_constant', help='sim parameter: time constant', type=float)
parser.add_argument('-kp', '--p_gain', help='sim parameter: p gain', type=float)
parser.add_argument('-kd', '--d_gain', help='sim parameter: d gain', type=float)
args = parser.parse_args()
SIM_WITH_DELAY = args.sim_with_delay
if args.velocity != None:
    velocity = args.velocity
if args.p_gain != None:
    p_gain = args.p_gain
if args.d_gain != None:
    d_gain = args.d_gain
if args.time_constant != None:
    time_constant = args.time_constant
if args.time_delay != None:
    time_delay = args.time_delay

print("run simulation: sim_with_delay = ", SIM_WITH_DELAY)
print("parameters: v = {},  tau = {},  delay = {},  kp = {},  kd = {}".format(velocity, time_constant, time_delay, p_gain, d_gain))


def plot_result(res_x, res_u):
    plt.rcParams["font.size"] = 18
    plt.rcParams["lines.linewidth"] = 2

    plt.figure(figsize=(20, 20))
    plt.subplot(3, 1, 1)
    plt.plot(t, res_x[:,0])
    plt.xlabel("Time [sec]")
    plt.ylabel("Lateral error [m]")
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(t, res_x[:,1] * 180.0 / 3.1415)
    plt.xlabel("Time [sec]")
    plt.ylabel("Heading error [deg]")
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(t, res_u, label='cmd wz')
    plt.plot(t, res_x[:, 2], label='act wz')
    plt.xlabel("Time [sec]")
    plt.ylabel("angular veocity [rad/s]")
    plt.legend()

    plt.grid()
    plt.show()


if __name__ == '__main__':

    t = np.arange(0, 10, 0.01)
    param = simulator_core.ModelParams(v=velocity, tau=time_constant, delay=time_delay, kp=p_gain, kd=d_gain)
    sim = simulator_core.Simulator()
    res_x, res_u = sim.run(x0, t, SIM_WITH_DELAY, param)

    plot_result(res_x, res_u)
