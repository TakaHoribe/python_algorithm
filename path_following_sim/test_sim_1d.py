#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def calc_control_input(kp, kd, laterr, theta):
    return - kp * laterr - kd * theta

def func(x, t, v, tau, kp, kd):
    e, theta, w = x
    u = calc_control_input(kp, kd, e, theta)

    dx = [0, 0, 0]
    dx[0] = v * math.sin(theta)
    dx[1] = w
    dx[2] = - (1.0 / tau) * (w - u)
    return dx


if __name__ == '__main__':

    # params = Params()
    velocity = 0.2
    time_constant = 0.5
    p_gain = 1.0
    d_gain = 0.7
    x0 = [0.1, 0.0, 0.0]
    t = np.arange(0, 10, 0.01)

    res_x = odeint(func, x0, t, args=(velocity, time_constant, p_gain, d_gain))
    res_u = calc_control_input(p_gain, d_gain, np.array(res_x[:, 0]), np.array(res_x[:,1]))

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
