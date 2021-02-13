#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ddeint import ddeint
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse

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


def calc_control_input(kp, kd, laterr, theta):
    return - kp * laterr - kd * theta

def model_1d_delay(x, t, v, tau, kp, kd, delay):
    e, theta, w = x(t)
    e_delay, theta_delay, w_delay  = x(t-delay)
    u = calc_control_input(kp, kd, e_delay, theta_delay)
    dx = [0, 0, 0]
    dx[0] = v * math.sin(theta)
    dx[1] = w
    dx[2] = - (1.0 / tau) * (w - u)
    return dx


def model_1d(x, t, v, tau, kp, kd):
    e, theta, w = x
    u = calc_control_input(kp, kd, e, theta)
    dx = [0, 0, 0]
    dx[0] = v * math.sin(theta)
    dx[1] = w
    dx[2] = - (1.0 / tau) * (w - u)
    return dx

if __name__ == '__main__':


    t = np.arange(0, 10, 0.01)
    if SIM_WITH_DELAY:
        g = lambda t: x0 # history before t=0
        res_x = ddeint(model_1d_delay, g, t, fargs=(velocity, time_constant, p_gain, d_gain, time_delay))
    else:
        res_x = odeint(model_1d, x0, t, args=(velocity, time_constant, p_gain, d_gain))
    res_u = calc_control_input(p_gain, d_gain, np.array(res_x[:, 0]), np.array(res_x[:,1]))


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
