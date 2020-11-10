import numpy as np
import math
import matplotlib.pyplot as plt
import copy
import pandas as pd

class Times {
    def __init__(self):
        self.t_dec_jerk
        self.t_non_jerk
        self.t_acc_jerk
}

class Params {
    def __init__(self):
        self.v0
        self.v_end
        self.a0
        self.max_acc
        self.min_acc
        self.max_jerk
        self.min_jerk 
}


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

    def trapezoid_shape(self, v0, a0, a_min, j_acc, j_dec, t_min):

        t1 = max((a_min - a0) / j_dec, 0.0)
        t2 = max(t_min, 0.0)
        t3 = max(-a_min / j_acc, 0.0)
        print("t1: %f, t2: %f, t3: %f" % (t1, t2, t3))

        # 0 ~ t1
        x0 = 0.0
        for t in np.linspace(0.0, t1, 100, endpoint=True):
            jerk = j_dec
            t_offset = 0.0
            self.update(t, jerk, a0, v0, x0, t_offset)

        # t1 ~ t2
        a1 = self.accs[-1]
        v1 = self.vels[-1]
        x1 = self.dists[-1]
        for t in np.linspace(0.0, t2, 100, endpoint=True):
            jerk = 0.0
            t_offset = t1
            self.update(t, jerk, a1, v1, x1, t_offset)

        # t2 ~ t3
        a2 = self.accs[-1]
        v2 = self.vels[-1]
        x2 = self.dists[-1]
        for t in np.linspace(0.0, t3, 100, endpoint=True):
            jerk = j_acc
            t_offset = t1 + t2
            self.update(t, jerk, a2, v2, x2, t_offset)



    def triangle_shape_v1(self, v0, ved,  a0, a_min, j_acc, j_dec):

        a_min2_square = 2 * (ved - v0 + (0.5 * a0 * a0 / j_dec)) * ((j_dec * j_acc) / (j_acc - j_dec))
        a_min2 = -np.sqrt(a_min2_square)

        t1 = max((a_min2 - a0) / j_dec, 0.0)
        print("t1: %f" % t1)


        t2 = 0.0
        a1 = 0.0
        v1 = 0.0
        x1 = 0.0
        if t1 > 1e-3:
            print("v1")
            t2 = max(-a_min2 / j_acc, 0.0)

            # 0 ~ t1
            for t in np.linspace(0.0, t1, 100, endpoint=True):
                jerk = j_dec
                x0 = 0.0
                self.update(t, jerk, a0, v0, x0, t_offset)

            a1 = accs[-1]
            v1 = vels[-1]
            x1 = dists[-1]

        else:
            print("v2")
            t2 = max(-a0 / j_acc, 0.0)

            a1 = a0
            v1 = v0

            v_true = (0.5) * j_acc * t2 * t2
            if math.fabs(v_true - v0) > 0.03:
                print("invalid v0 setting: v_true: %f, v0: %f" % ((v_true * 3.6), (v0 * 3.6)))
                return [0.0], [0.0], [0.0], [0.0], [0.0]

        # t1 ~ t2
        for t in np.linspace(0.0, t2, 100, endpoint=True):
            jerk = j_acc
            self.update(t, jerk, a1, v1, x1, t1)



    def calc_stop_dist(self, v0, ved, a0, a_min, j_acc, j_dec):

        if a0 < a_min:
            print("invalid a0 setting: a0: %f, a_min: %f" % (a0, a_min))
            return [0.0], [0.0], [0.0], [0.0], [0.0]

        if v0 <= ved:
            print("ved is larger than v0: v0: %f, ved: %f" % (v0, ved))
            return [0.0], [0.0], [0.0], [0.0], [0.0]

        times = calc_stop_time_sequence()

        t_min = (ved - v0 + (0.5 * a0 * a0 / j_dec) - (0.5 * a_min * a_min / j_dec) + (0.5 * a_min * a_min / j_acc)) / a_min
        # print("t min : %f" % t_min)

        dists = []
        vels = []
        accs = []
        jerks = []
        times = []

        if t_min > 0:
            print("type: trapezoid")
            self.trapezoid_shape(v0, a0, a_min, j_acc, j_dec, t_min)
        else:
            print("type: triangle")
            self.triangle_shape_v1(v0, ved, a0, a_min, j_acc, j_dec)

    def visualize(self, v0_kmh, ved_kmh, a0, min_acc, acc_jerk, dec_jerk):

        self.calc_stop_dist(v0_kmh / 3.6, ved_kmh / 3.6,  a0, min_acc, acc_jerk, dec_jerk)

        plt.figure(figsize=(20, 10))
        
        plt.subplot(2, 3, 1)
        plt.plot(self.times, self.dists)
        plt.xlabel("Time [sec]")
        plt.ylabel("Distance [m]")
        plt.title("Distance [m]")
        print("(x, t, v, a) = (%f, %f, %f, %f)" % (self.dists[-1], self.times[-1], self.vels[-1], self.accs[-1]))

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
        df.to_csv(str(v0_kmh) + "kmph_" + str(ved_kmh) + "kmph_" + str(a0) + "mps2_" + str(min_acc) + "mps2_" + str(acc_jerk) + "mps3_" + str(dec_jerk) + "mps3" + ".csv")

        plt.show()

if __name__ == '__main__':

    stop_dist_calc = StopDistCalculator()

    v0_kmh = 20
    ved_kmh = 0
    a0 = 0.5
    min_acc = -1.0
    acc_jerk = 0.3
    dec_jerk = -0.3

    stop_dist_calc.visualize(v0_kmh, ved_kmh, a0, min_acc, acc_jerk, dec_jerk)




