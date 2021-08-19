#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ddeint import ddeint
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse

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

class  ModelParams():
    def __init__(self, v, tau, kp, kd, delay):
        self.velocity = v
        self.time_constant = tau
        self.p_gain = kp
        self.d_gain = kd
        self.time_delay = delay

class Simulator():
    def __init__(self):
        self.res_x = []
        self.res_u = []

    def run(self, x0, t, SIM_WITH_DELAY, p):
        if SIM_WITH_DELAY:
            g = lambda t: x0 # history before t=0
            res_x = ddeint(model_1d_delay, g, t, fargs=(p.velocity, p.time_constant, p.p_gain, p.d_gain, p.time_delay))
        else:
            res_x = odeint(model_1d, x0, t, args=(p.velocity, p.time_constant, p.p_gain, p.d_gain))
        res_u = calc_control_input(p.p_gain, p.d_gain, np.array(res_x[:, 0]), np.array(res_x[:,1]))

        return (res_x, res_u)
