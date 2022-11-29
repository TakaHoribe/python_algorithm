#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from matplotlib import pyplot as plt


def calcFromJerk():
    jerk = 0.3
    alim = 0.5


    for Le in (0.5, 1.0, 2.0, 3.0, 4.0, 5.0):

        Tj = alim / jerk
        Ta = (math.sqrt(alim**4 + 4.0*(jerk**2)*Le*alim) - 3.0 * alim**2) / (2.0 * alim * jerk)
        if Ta > 0.0:
            print("jerk: {}, acc: {}, shift_length: {}, T_total: {}, T_jerk: {}, T_acc: {}".format(jerk, alim, Le, Tj*4.0 + Ta*2.0, Tj, Ta))
        else:
            Tj = (Le / (2.0 * jerk))**(1.0/3.0)
            print("jerk: {}, acc: {}, shift_length: {}, T_total: {}, T_jerk: {}, T_acc: {}".format(jerk, alim, Le, Tj*4.0, Tj, 0.0))

def calcFromTime():
    Tall = 7.0
    alim = 0.5

    for Le in (0.5, 1.0, 2.0, 3.0, 4.0, 5.0):

        Tj = Tall / 2.0 - 2.0 * Le / (alim * Tall)
        Ta = 4.0 * Le / (alim * Tall) - Tall / 2.0
        if Ta > 0:
            jerk = 2.0 * (alim**2) * Tall / (alim * Tall**2 - 4.0 * Le)
            print("T_total: {}[s], shift_length: {}[m], alim: {}[m/s2], jerk: {}[m/s3], T_jerk: {}[s], T_acc: {}[s]".format(Tall, Le, alim, jerk, Tj, Ta))
        else:
            jerk = 32.0 * Le / Tall**3
            print("T_total: {}[s], shift_length: {}[m], alim: {}[m/s2], jerk: {}[m/s3], T_jerk: {}[s], T_acc: {}[s]".format(Tall, Le, alim, jerk, Tj, 0.0))


if __name__ == '__main__':

    # JERKとACCから時間を計算
    calcFromJerk()


    # 時間とACCからjerkを計算
    calcFromTime()


jerk: 0.3, acc: 0.5, shift_length: 0.5, T_total: 3.7641, T_jerk: 0.9410, T_acc: 0.0
jerk: 0.3, acc: 0.5, shift_length: 1.0, T_total: 4.7425, T_jerk: 1.1856, T_acc: 0.0
jerk: 0.3, acc: 0.5, shift_length: 2.0, T_total: 5.9752, T_jerk: 1.4938, T_acc: 0.0
jerk: 0.3, acc: 0.5, shift_length: 3.0, T_total: 6.8413, T_jerk: 1.6666, T_acc: 0.087
jerk: 0.3, acc: 0.5, shift_length: 4.0, T_total: 7.5639, T_jerk: 1.6666, T_acc: 0.448
jerk: 0.3, acc: 0.5, shift_length: 5.0, T_total: 8.2071, T_jerk: 1.6666, T_acc: 0.770
