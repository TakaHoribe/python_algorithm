import numpy as np
import math

class Times:
    def __init__(self, t1=0, t2=0, t3=0):
        self.dec_jerk_time = t1
        self.zero_jerk_time = t2
        self.acc_jerk_time = t3

class Param:
    def __init__(self, max_acc=0, min_acc=0, max_jerk=0, min_jerk=0, delay_time=0):
        self.max_acc = max_acc
        self.min_acc = min_acc
        self.max_jerk = max_jerk
        self.min_jerk  = min_jerk 
        self.delay_time = delay_time

class StopDistCalculator:
    def __init__(self, v0=None, v_end=None, a0=None, max_acc=None, min_acc=None, max_jerk=None, min_jerk=None, delay_time=None, print=False):
        self.param = Param()
        if v0 is not None:
            self.v0 = v0
        if v_end is not None:
            self.v_end = v_end
        if a0 is not None:
            self.a0 = a0
        if max_acc is not None:
            self.param.max_acc = max_acc
        if min_acc is not None:
            self.param.min_acc = min_acc
        if max_jerk is not None:
            self.param.max_jerk = max_jerk
        if min_jerk is not None:
            self.param.min_jerk  = min_jerk 
        if delay_time is not None:
            self.delay_time = delay_time
        if print is not None:
            self.enable_print = print
        return

    def set_param(self, param):
        self.param = param
        return

    def set_params(self, v0=None, v_end=None, a0=None, max_acc=None, min_acc=None, max_jerk=None, min_jerk=None, delay_time=None, print=None):
        if v0 is not None:
            self.v0 = v0
        if v_end is not None:
            self.v_end = v_end
        if a0 is not None:
            self.a0 = a0
        if max_acc is not None:
            self.param.max_acc = max_acc
        if min_acc is not None:
            self.param.min_acc = min_acc
        if max_jerk is not None:
            self.param.max_jerk = max_jerk
        if min_jerk is not None:
            self.param.min_jerk  = min_jerk 
        if delay_time is not None:
            self.delay_time = delay_time
        if print is not None:
            self.enable_print = print
        return

    def update(self, t, jerk, a0, v0, x0, t_offset):
        a = a0 + jerk * t
        v = v0 + (a0 * t) + (0.5 * jerk * t * t)
        x = x0 + (v0 * t) + (0.5 * a0 * t * t) + ((1.0 / 6.0) * jerk * t * t * t)
        return (x, v, a)

    def valid_check(self):
        if self.a0 < self.param.min_acc or self.param.max_acc < self.a0:
            if self.enable_print:
                print("[invalid] a0 exceeds constraints: a0: %f, a_min: %f, a_max: %f" % (self.a0, self.param.min_acc, self.param.max_acc))
            return False

        if self.v0 < self.v_end:
            if self.enable_print:
                print("[invalid] end velocity is larger than v0: v0: %f, ved: %f" % (self.v0, self.v_end))
            return False

        if self.param.min_jerk >= 0.0 or self.param.max_jerk <= 0.0:
            if self.enable_print:
                print("[invalid jerk] min_jerk: %f, max_jerk: %f" % (self.param.min_jerk, self.param.max_jerk))
            return False

        if self.param.min_acc >= 0.0 or self.param.max_acc <= 0.0:
            if self.enable_print:
                print("[invalid acc] min_acc: %f, max_acc: %f" % (self.param.min_acc, self.param.max_acc))
            return False

        return True


    def calc(self):

        if self.valid_check() is False:
            return (0.0, 0.0)
        
        p = self.param

        vd = self.v0 + p.delay_time * self.a0
        x0 = self.v0 * p.delay_time + 0.5 * self.a0 * (p.delay_time ** 2)


        times = Times()
        times.zero_jerk_time = (self.v_end - vd + 0.5 * (self.a0**2 - p.min_acc**2) / p.min_jerk + (0.5 * p.min_acc**2 / p.max_jerk)) / p.min_acc
        if (times.zero_jerk_time > 0):
            jerk_plan_type = 'dec_zero_acc'
            times.dec_jerk_time = (p.min_acc - self.a0) / p.min_jerk
            times.acc_jerk_time = -p.min_acc / p.max_jerk
        else:
            min_acc_actual = -np.sqrt(2 * (self.v_end - vd + (0.5 * self.a0**2 / p.min_jerk)) * ((p.min_jerk * p.max_jerk) / (p.max_jerk - p.min_jerk)))

            if (self.a0 > min_acc_actual):
                jerk_plan_type = 'dec_acc'
                times.dec_jerk_time = (min_acc_actual - self.a0) / p.min_jerk
                times.acc_jerk_time = -min_acc_actual / p.max_jerk
            else:
                jerk_plan_type = 'acc'
                times.dec_jerk_time = 0.0
                times.acc_jerk_time = -self.a0 / p.max_jerk

        # print("t_dec_zero_acc = [%f, %f, %f]" % (times.dec_jerk_time, times.zero_jerk_time, times.acc_jerk_time))

        times.dec_jerk_time = max(times.dec_jerk_time, 0.0)
        times.zero_jerk_time = max(times.zero_jerk_time, 0.0)
        times.acc_jerk_time = max(times.acc_jerk_time, 0.0)

        (x1, v1, a1) = self.update(t=times.dec_jerk_time, jerk=p.min_jerk, a0=self.a0, v0=vd, x0=x0, t_offset=0.0)
        (x2, v2, a2) = self.update(t=times.zero_jerk_time, jerk=0.0, a0=a1, v0=v1, x0=x1, t_offset=times.dec_jerk_time)
        (x3, v3, a3) = self.update(t=times.acc_jerk_time, jerk=p.max_jerk, a0=a2, v0=v2, x0=x2, t_offset=times.zero_jerk_time + times.dec_jerk_time)

        total_t = p.delay_time + times.dec_jerk_time + times.zero_jerk_time + times.acc_jerk_time
        if self.enable_print:
            print("a0=%f, v0=%f, vd=%f, xe=%f, ve=%f, ae=%f, t=%f type=" % (self.a0, self.v0, vd, x3, v3, a3, total_t), jerk_plan_type, ", times=[%f, %f, %f]" % (times.dec_jerk_time, times.zero_jerk_time, times.acc_jerk_time))

        return (x3, total_t)

    # apply decel jerk and acceleration for given time and return the velocity at the time
    def calc_dist_after_decel_time(self, v0, t):
        tj = self.param.min_acc / self.param.min_jerk
        ta = max(t - tj, 0.0)
        (x1, v1, a1) = self.update(t=tj, jerk=self.param.min_jerk, a0=0.0, v0=v0, x0=0.0, t_offset=0.0)
        (x2, v2, a2) = self.update(t=ta, jerk=0.0, a0=self.param.min_acc, v0=v1, x0=x1, t_offset=0.0)
        return x2, v2

    # apply accel jerk and acceleration for given time and return the velocity at the time
    def calc_dist_after_accel_time(self, v0, t):
        tj = self.param.max_acc / self.param.max_jerk
        ta = max(t - tj, 0.0)
        (x1, v1, a1) = self.update(t=tj, jerk=self.param.max_jerk, a0=0.0, v0=v0, x0=0.0, t_offset=0.0)
        (x2, v2, a2) = self.update(t=ta, jerk=0.0, a0=self.param.max_acc, v0=v1, x0=x1, t_offset=0.0)
        return x2, v2


class LateralDistCalculator:
    def __init__(self, max_lat_acc=None, max_lat_jerk=None, lat_dist=None, print=False):
        if max_lat_acc is not None:
            self.max_lat_acc = max_lat_acc
        if max_lat_jerk is not None:
            self.max_lat_jerk = max_lat_jerk
        if lat_dist is not None:
            self.lat_dist = lat_dist
        if print is not None:
            self.enable_print = print
        return

    def valid_check(self):
        if self.max_lat_acc <= 0.0 or self.max_lat_jerk <= 0.0:
            print("[invalid] parameters are invalid: max_lat_acc: %f, max_lat_jerk: %f" % (self.max_lat_acc, self.max_lat_jerk))
            return False
        return True

    def calc_required_time(self):
        if self.valid_check() is False:
            return 0.0
        tj = self.max_lat_acc / self.max_lat_jerk  # time with constant jerk

        # time with constant acceleration (zero jerk)
        a = self.max_lat_acc
        j = self.max_lat_jerk
        l = self.lat_dist
        ta = (np.sqrt(a * a + 4.0 * j * j * l / a) - 3.0 * a) / (2.0 * j)

        if ta < 0.0:
            tj = pow(l / (2.0 * j), 1.0 / 3.0)
            ta = 0.0

        t_total = 4.0 * tj + 2.0 * ta
        return t_total


if __name__ == '__main__':

    sdc = StopDistCalculator(v0=20, a0=0.0, v_end=0.0, max_acc=1.0, min_acc=-1.0, max_jerk=1.0, min_jerk=-1.0)
    sdc.calc()

    x2, v2 = sdc.calc_dist_after_accel_time(v0=0.0, t=9.0)
    print(x2, v2)