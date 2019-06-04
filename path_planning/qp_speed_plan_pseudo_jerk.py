import numpy as np
import math
import cvxopt
import scipy.interpolate as interp
import matplotlib.pyplot as plt
import copy

from cvxopt import matrix


def qp_velocity_planning(s, v, v0, a0, vn, an, w):

    # -- problem --
    # min   0.5 * x' * P * x + q' * x
    # s.t.  Ax = b,  Gx <= h

    N = len(s)

    P = np.matrix(np.zeros(shape=(N, N)))

    # == object function ==
    # -- w * jerk^2 term --
    for i in range(1, N-1):
        c0 = s[i-1] * s[i]
        c1 = s[i-1] * s[i+1]
        c2 = s[i] * s[i+1]
        d0 = (c0 - c1 - c2 + s[i+1]*s[i+1])
        d1 = (c0 + c1 - c2 - s[i-1]*s[i-1])
        d2 = (c0 - c1 + c2 - s[i]*s[i])
        e0 = 2 / d0
        e1 = -2 / d2
        e2 = -2 / d1
        # v[i] = e0 * v[i+1] + e1 * v[i] + e2 * v[i-1]
        e = np.matrix([e0, e1, e2])
        E = np.dot(e.T, e)
        P[i-1:i+2, i-1:i+2] += w * E

    # -- (v - v_r)^2 term --
    P += np.identity(N)
    q = -2.0 * v

    # == constraint ==

    # v[0] = v0, v[N] = vn, a[0] = a0, a[N] = an
    A = np.matrix(np.zeros(shape=(4, N)))
    b = np.matrix(np.zeros(shape=(4, 1)))

    # v[0] = v0
    A[0, 0] = 1
    b[0] = v0

    # a[0] = a0
    A[1, 0] = -1
    A[1, 1] = 1 
    b[1] = a0 * (s[1] - s[0])

    # v[n] = vn
    A[2, -1] = 1
    b[2] = vn

    # a[N] = an
    A[3, -1] = 1
    A[3, -2] = -1
    b[3] = an * (s[N-1] - s[N-2])

    # v[i] < v_max
    G = np.identity(N)
    h = np.matrix(v).reshape(N, 1)

    print 'A : \n', A
    print 'b : \n', b
    print 'G : \n', G
    print 'h : \n', h
    print 'P : \n', P
    print 'q : \n', q


    cvxopt.solvers.options['show_progress'] = True
    sol = cvxopt.solvers.qp(cvxopt.matrix(P), cvxopt.matrix(q), G=cvxopt.matrix(G), h=cvxopt.matrix(h), A=cvxopt.matrix(A), b=cvxopt.matrix(b))
    
    return sol['x']


def plot_vel(s, vmax, v):

    plt.subplot(1, 3, 1)
    plt.plot(s, vmax, label="v max")
    plt.plot(s, v, label="v")
    plt.title('velocity')
    plt.legend()
    
    a_arr = np.diff(v.T, n=1)
    plt.subplot(1, 3, 2)
    # plt.plot(s[1:], a_arr, label="accl")
    # plt.title('acceleration')
    # plt.legend()

    plt.show()

    print s
    print s[1:]
    print v
    print a_arr.reshape(1, N-1)








def func(a, b):
    a = 2.0
    b = 2.0
    return a, b

if __name__ == '__main__':

    ds = 0.1
    v_nominal = 1.0
    N = 100
    s_arr = np.array([i for i in range(N)]) * ds
    vmax_arr = np.ones(N) * v_nominal
    vmax_arr[-1] = 0.0

    # initial condition
    v0 = 0.8
    a0 = 0.5


    # terminal condition
    vn = 0.0
    an = 0.0

    # jerk weight
    w = 2.0

    v_arr = qp_velocity_planning(s_arr, vmax_arr, v0, a0, vn, an, w)
    
    # print 'v_arr : \n', v_arr
    # print 'vmax_arr : \n', vmax_arr

    plot_vel(s_arr, vmax_arr, v_arr)





    # print(s_arr)
    # print(v_arr)

