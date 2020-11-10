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
        self.dists = []
        self.vels = []
        self.accs = []
        self.jerks = []
        self.times = []

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
        self.dists.append(x)
        self.vels.append(v)
        self.accs.append(a)
        self.jerks.append(jerk)
        self.times.append(t + t_offset)

    def trapezoid_shape(self, params, times):

        # applying decel jerk
        a0 = params.a0
        v0 = params.v0
        x0 = 0.0
        for t in np.linspace(0.0, times.dec_jerk_time, 100, endpoint=True):
            jerk = params.min_jerk
            t_offset = 0.0
            self.update(t, jerk, a0, v0, x0, t_offset)

        # applying zero jerk
        a1 = self.accs[-1]
        v1 = self.vels[-1]
        x1 = self.dists[-1]
        for t in np.linspace(0.0, times.zero_jerk_time, 100, endpoint=True):
            jerk = 0.0
            t_offset = times.dec_jerk_time
            self.update(t, jerk, a1, v1, x1, t_offset)

        # applying decel jerk
        a2 = self.accs[-1]
        v2 = self.vels[-1]
        x2 = self.dists[-1]
        for t in np.linspace(0.0, times.acc_jerk_time, 100, endpoint=True):
            jerk = params.max_jerk
            t_offset = times.zero_jerk_time + times.dec_jerk_time
            self.update(t, jerk, a2, v2, x2, t_offset)

    def calc_stop_times(self, p):
        times = Times()
        print("%f, %f, %f" % (p.min_jerk, p.max_jerk, p.min_acc))
        times.zero_jerk_time = (p.v_end - p.v0 + (0.5 * p.a0 * p.a0 / p.min_jerk) - (0.5 * p.min_acc * p.min_acc / p.min_jerk) + (0.5 * p.min_acc * p.min_acc / p.max_jerk)) / p.min_acc
        jerk_plan_type = ''
        if (times.zero_jerk_time > 0):
            jerk_plan_type = 'dec_zero_acc'
            times.dec_jerk_time = (p.min_acc - p.a0) / p.min_jerk
            times.acc_jerk_time = -p.min_acc / p.max_jerk
        else:
            min_acc_actual = -np.sqrt(2 * (ved - p.v0 + (0.5 * p.a0 * p.a0 / p.min_jerk)) * ((p.min_jerk * p.max_jerk) / (p.max_jerk - p.min_jerk)))
            
            if (p.a0 > min_acc_actual):
                jerk_plan_type = 'dec_acc'
                times.dec_jerk_time = (min_acc_actual - p.a0) / p.min_jerk
                times.acc_jerk_time = (-(p.a0 + p.min_jerk * dec_jerk_time) / p.max_jerk)
            else:
                jerk_plan_type = 'acc'
                times.dec_jerk_time = 0.0
                times.acc_jerk_time = -p.a0 / p.max_jerk
        
        print("jerk_plan_type = " + jerk_plan_type)
        print("t_dec_zero_acc = [%f, %f, %f]" % (times.dec_jerk_time, times.zero_jerk_time, times.acc_jerk_time))

        times.dec_jerk_time = max(times.dec_jerk_time, 0.0)
        times.zero_jerk_time = max(times.zero_jerk_time, 0.0)
        times.acc_jerk_time = max(times.acc_jerk_time, 0.0)
        return times

    def calc_stop_dist(self, params):

        if params.a0 < params.min_acc:
            print("invalid a0 setting: a0: %f, a_min: %f" % (params.a0, params.a_min))
            return [0.0], [0.0], [0.0], [0.0], [0.0]

        if params.v0 <= params.v_end:
            print("ved is larger than v0: v0: %f, ved: %f" % (params.v0, params.v_end))
            return [0.0], [0.0], [0.0], [0.0], [0.0]

        times = self.calc_stop_times(params)
        self.trapezoid_shape(params, times)

    def visualize(self, params):

        plt.figure(figsize=(20, 10))
        
        plt.subplot(2, 3, 1)
        plt.plot(self.times, self.dists)
        plt.xlabel("Time [sec]")
        plt.ylabel("Distance [m]")
        plt.title("Distance [m]")
        print("(stop dist, stop time, end velocity, end acceleration) = (%f, %f, %f, %f)" % (self.dists[-1], self.times[-1], self.vels[-1], self.accs[-1]))

        plt.subplot(2, 3, 2)
        vels_kmh = list(map(lambda x: x * 3.6, self.vels))
        plt.plot(self.times, vels_kmh)
        plt.xlabel("Time [sec]")
        plt.ylabel("Velocity [km/h]")
        plt.title("Velocity [km/h]")

        plt.subplot(2, 3, 3)
        plt.plot(self.times, self.accs)
        plt.xlabel("Time [sec]")
        plt.ylabel("Acceleration [m/s^2]")
        plt.title("Acceleration [m/s^2]")

        plt.subplot(2, 3, 4)
        plt.plot(self.times, self.jerks)
        plt.xlabel("Time [sec]")
        plt.ylabel("Jerk [m/s^3]")
        plt.title("Jerk [m/s^3]")

        plt.subplot(2, 3, 5)
        plt.plot(self.dists, vels_kmh)
        plt.xlabel("Distance [m]")
        plt.ylabel("Velocity [km/h]")
        plt.title("Vel Dist")

        df = pd.DataFrame()
        df['dist'] = self.dists
        df['vel'] = self.vels
        df['acc'] = self.accs
        df['jerk'] = self.jerks
        df['time'] = self.times
        df.to_csv(str(params.v0 * 3.6) + "kmph_" + str(params.v_end * 3.6) + "kmph_" + str(params.a0) + "mps2_" + str(params.min_acc) + "mps2_" + str(params.max_jerk) + "mps3_" + str(params.min_jerk) + "mps3" + ".csv")

        plt.show()

if __name__ == '__main__':

    KMPH2MPS = 1.0 / 3.6
    params = Params()
    params.v0 = 20.0 * KMPH2MPS
    params.v_end = 0.0 * KMPH2MPS
    params.a0 = 0.5
    params.min_acc = -1.0
    params.max_jerk = 0.3
    params.min_jerk = -0.3

    obj = StopDistCalculator()

    obj.calc_stop_dist(params)
    obj.visualize(params)




