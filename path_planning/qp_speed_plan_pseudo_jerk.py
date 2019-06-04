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

    plt.figure()
    plt.plot(s, vmax, label="v max")
    plt.plot(s, v, label="v")
    plt.title('velocity')
    plt.legend()
    




def compare_jerk(s, v):

    N = len(s)

    j_approx_quad = np.array(np.zeros(shape=(N, 1)))
    j_discrete = np.array(np.zeros(shape=(N, 1)))
    j_d2vds2 = np.array(np.zeros(shape=(N, 1)))


    for i in range(2, N-2):

        # quadratic function approximation
        c0 = s[i-1] * s[i]
        c1 = s[i-1] * s[i+1]
        c2 = s[i] * s[i+1]
        d0 = (c0 - c1 - c2 + s[i+1]*s[i+1])
        d1 = (c0 + c1 - c2 - s[i-1]*s[i-1])
        d2 = (c0 - c1 + c2 - s[i]*s[i])
        e0 = 2 / d0
        e1 = -2 / d2
        e2 = -2 / d1
        j_approx_quad[i] = e0 * v[i+1] + e1 * v[i] + e2 * v[i-1]

        # pure discretization
        dvds = (v[i+1] - v[i-1]) / (s[i+1] - s[i-1])
        dvds_next = (v[i+2] - v[i]) / (s[i+2] - s[i]) # for dvds[i+1]
        dvds_prev = (v[i] - v[i-2]) / (s[i] - s[i-2]) # for dvds[i-1]
        d2vds2 = (dvds_next - dvds_prev) / (s[i+1] - s[i-1])
        j_discrete[i] = d2vds2 * (v[i] ** 2) + (dvds ** 2) * v[i]

        j_d2vds2[i] = d2vds2 * v[i]

    plt.figure()
    plt.plot(s, j_approx_quad, label="j_approx_quad")
    plt.plot(s, j_discrete, label="d/dt(dv/dt)")
    plt.plot(s, j_d2vds2, label="d/ds(dv/ds)", linestyle='dashed')
    plt.title('jerk')
    plt.legend()




if __name__ == '__main__':

    ds = 0.01
    v_nominal = 10.0
    N = 1000
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


    plot_vel(s_arr, vmax_arr, v_arr)
    compare_jerk(s_arr, v_arr)

    plt.show()



