#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from matplotlib import pyplot as plt


class Calculator():

    def __init__(self, x=0, v=0, a=0, dt=0.01):
        self.x = x
        self.v = v
        self.a = a
        self.dt = dt
        self.t = 0.0

    def integrate(self, j):
        self.a += j * self.dt
        self.v += self.a * self.dt
        self.x += self.v * self.dt
        self.t += self.dt

    def time(self):
        return self.t

if __name__ == '__main__':

    dt = 1.0e-5
    Tj = 2.3
    Ta = 4.8
    Tv = 6.1

    jerk = 0.23

    calculator = Calculator(dt=dt)

    while calculator.time() < Tj:
        calculator.integrate(jerk)
    x_1 = 1.0 / 6.0 * jerk * Tj**3
    print("x1: {}, sim: {}".format(x_1, calculator.x))

    while calculator.time() < Tj + Ta:
        calculator.integrate(0.0)
    x_2 = 1.0 / 6.0 * jerk * Tj**3 + 1.0 / 2.0 * jerk * Ta * Tj**2 + 1.0 / 2.0 * jerk * Ta**2 * Tj
    print("x2: {}, sim: {}".format(x_2, calculator.x))

    while calculator.time() < Tj + Ta + Tj:
        calculator.integrate(-jerk)
    x_3 = jerk * Tj**3 + 3.0 / 2.0 * jerk * Ta * Tj**2 + 1.0 / 2.0 * jerk * Ta**2 * Tj
    print("x3: {}, sim: {}".format(x_3, calculator.x))

    while calculator.time() < Tj + Ta + Tj + Tv:
        calculator.integrate(0.0)
    x_4 = jerk * Tj**3 + 3.0 / 2.0 * jerk * Ta * Tj**2 + 1.0 / 2.0 * jerk * Ta**2 * Tj + jerk*(Ta + Tj)*Tj*Tv
    v_4 = jerk * Tj * (Ta + Tj)
    print("x4: {}, sim: {}, v4: {}, sim: {}".format(x_4, calculator.x, v_4, calculator.v))

    while calculator.time() < Tj + Ta + Tj + Tv + Tj:
        calculator.integrate(-jerk)
    x_5 = 5.0 / 6.0 * jerk * Tj**3 + 3.0 / 2.0 * jerk * Ta * Tj**2 + 1.0 / 2.0 * jerk * Ta**2 * Tj + jerk*(Ta + Tj)*Tj*(Tv + Tj)
    v_5 = jerk * Tj * (Ta + 1.0 / 2.0 * Tj)
    print("x5: {}, sim: {}, v5: {}, sim: {}".format(x_5, calculator.x, v_5, calculator.v))

    while calculator.time() < Tj + Ta + Tj + Tv + Tj + Ta:
        calculator.integrate(0.0)
    x_6 = 11.0 / 6.0 * jerk * Tj**3 + 3.0 * jerk * Ta * Tj**2 + jerk * Ta**2 * Tj + jerk*(Ta + Tj)*Tj*Tv
    v_6 = 1.0 / 2.0 * jerk * Tj**2
    print("x6: {}, sim: {}, v6: {}, sim: {}".format(x_6, calculator.x, v_6, calculator.v))

    while calculator.time() < Tj + Ta + Tj + Tv + Tj + Ta + Tj:
        calculator.integrate(jerk)
    x_7 = 2.0 * jerk * Tj**3 + 3.0 * jerk * Ta * Tj**2 + jerk * Ta**2 * Tj + jerk*(Ta + Tj)*Tj*Tv
    print("x7: {}, sim: {}".format(x_7, calculator.x))


    