import numpy as np
import math
import matplotlib.pyplot as plt
import copy
import pandas as pd

class Times:
    def __init__(self, t1=0, t2=0, t3=0):
        self.dec_jerk_time = t1
        self.zero_jerk_time = t2
        self.acc_jerk_time = t3

class Params:
    def __init__(self, v0=0, v_end=0, a0=0, max_acc=0, min_acc=0, max_jerk=0, min_jerk=0):
        self.v0 = v0
        self.v_end = v_end
        self.a0 = a0
        self.max_acc = max_acc
        self.min_acc = min_acc
        self.max_jerk = max_jerk
        self.min_jerk  = min_jerk 


class StopDistCalculator:
    def __init__(self):
        return

    def update_a(self, t, jerk, a0):
        return a0 + jerk * t

    def update_v(self, t, jerk, a0, v0):
        return v0 + (a0 * t) + (0.5 * jerk * t * t)

    def update_x(self, t, jerk, a0, v0, x0):
        return x0 + (v0 * t) + (0.5 * a0 * t * t) + ((1.0 / 6.0) * jerk * t * t * t)
    
    def update(self, t, jerk, a0, v0, x0, t_offset):
        a = self.update_a(t, jerk, a0)
        v = self.update_v(t, jerk, a0, v0)
        x = self.update_x(t, jerk, a0, v0, x0)
        return (x, v, a)

    def valid_check(self, params):

        if params.a0 < params.min_acc:
            print("[invalid] a0 < a_min: a0: %f, a_min: %f" % (params.a0, params.min_acc))
            return False

        if params.v0 < params.v_end:
            print("[invalid] ved is larger than v0: v0: %f, ved: %f" % (params.v0, params.v_end))
            return False

        if params.min_jerk >= 0.0:
            print("[invalid] min_jerk is positive: %f" % params.min_jerk)
            return False

        if params.max_jerk <= 0.0:
            print("[invalid] max_jerk is negative: %f" % params.max_jerk)
            return False

        if params.min_acc >= 0.0:
            print("[invalid] min_acc is positive: %f" % params.min_acc)
            return False

        if params.max_acc <= 0.0:
            print("[invalid] max_acc is negative: %f" % params.max_acc)
            return False

        return True


    def calc(self, p):

        if (not self.valid_check(p)):
            return

        times = Times()
        times.zero_jerk_time = (p.v_end - p.v0 + 0.5 * (p.a0**2 - p.min_acc**2) / p.min_jerk + (0.5 * p.min_acc**2 / p.max_jerk)) / p.min_acc
        if (times.zero_jerk_time > 0):
            jerk_plan_type = 'dec_zero_acc'
            times.dec_jerk_time = (p.min_acc - p.a0) / p.min_jerk
            times.acc_jerk_time = -p.min_acc / p.max_jerk
        else:
            min_acc_actual = -np.sqrt(2 * (p.v_end - p.v0 + (0.5 * p.a0**2 / p.min_jerk)) * ((p.min_jerk * p.max_jerk) / (p.max_jerk - p.min_jerk)))
            
            if (p.a0 > min_acc_actual):
                jerk_plan_type = 'dec_acc'
                times.dec_jerk_time = (min_acc_actual - p.a0) / p.min_jerk
                times.acc_jerk_time = -min_acc_actual / p.max_jerk
            else:
                jerk_plan_type = 'acc'
                times.dec_jerk_time = 0.0
                times.acc_jerk_time = -p.a0 / p.max_jerk
        
        # print("t_dec_zero_acc = [%f, %f, %f]" % (times.dec_jerk_time, times.zero_jerk_time, times.acc_jerk_time))

        times.dec_jerk_time = max(times.dec_jerk_time, 0.0)
        times.zero_jerk_time = max(times.zero_jerk_time, 0.0)
        times.acc_jerk_time = max(times.acc_jerk_time, 0.0)


        (x1, v1, a1) = self.update(t=times.dec_jerk_time, jerk=params.min_jerk, a0=params.a0, v0=params.v0, x0=0.0, t_offset=0.0)
        (x2, v2, a2) = self.update(t=times.zero_jerk_time, jerk=0.0, a0=a1, v0=v1, x0=x1, t_offset=times.dec_jerk_time)
        (x3, v3, a3) = self.update(t=times.acc_jerk_time, jerk=params.max_jerk, a0=a2, v0=v2, x0=x2, t_offset=times.zero_jerk_time + times.dec_jerk_time)

        total_t = times.dec_jerk_time + times.zero_jerk_time + times.acc_jerk_time
        print("a0=%f, v0=%f, xe=%f, ve=%f, ae=%f, t=%f type=" % (p.a0, p.v0, x3, v3, a3, total_t), jerk_plan_type, ", times=[%f, %f, %f]" % (times.dec_jerk_time, times.zero_jerk_time, times.acc_jerk_time))
        


if __name__ == '__main__':

    KMPH2MPS = 1.0 / 3.6
    params = Params()
    params.v0 = 20.0 * KMPH2MPS
    params.v_end = 0.0 * KMPH2MPS
    params.a0 = -0.8
    params.min_acc = -1.0
    params.max_acc = 1.0
    params.max_jerk = 0.3
    params.min_jerk = -0.3

    for v0 in np.linspace(0.0, 60 * KMPH2MPS, 10, endpoint=True):
        for a0 in np.linspace(-1.0, 1.0, 10, endpoint=True):
            params.v0 = v0
            params.a0 = a0

            obj = StopDistCalculator()

            obj.calc(params)




