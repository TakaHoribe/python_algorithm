import numpy as np
import math
import matplotlib.pyplot as plt
import copy




def integrate(vals, dt):
    ret = np.zeros(vals.size)
    for i in range(1,vals.size):
        ret[i] = ret[i-1] + vals[i] * dt
    return ret

def differentiate(vals, dt):
    ret = np.zeros(vals.size)
    for i in range(0,vals.size-1):
        ret[i] = (vals[i+1] - vals[i]) / dt
    ret[-1] = ret[-2]
    return ret

def tate_shrink(vals, scale):
    vals *= scale
    return vals

def tate_shrink(vals, scale):
    vals *= scale
    return vals

if __name__ == '__main__':


    N = 10000
    jerk = 0.01
    dt = 0.001
    jerks = np.zeros(N)
    NN = int(N/4)
    jerks[0:NN] += jerk
    jerks[NN:3*NN] -= jerk
    jerks[3*NN:] += jerk

    times = np.arange(0,N*dt, dt)



    accs = integrate(jerks, dt)
    vels = integrate(accs, dt)
    poss = integrate(vels, dt)

    # poss_shrinked = tate_shrink(poss, 0.5)
    # vels_tsd = differentiate(poss_shrinked, dt)
    # accs_tsd = differentiate(vels_tsd, dt)
    # jerks_tsd = differentiate(accs_tsd, dt)

    vels_ysd = differentiate(poss, 0.5*dt)
    accs_ysd = differentiate(vels_ysd, 0.5*dt)
    jerks_ysd = differentiate(accs_ysd, 0.5*dt)


    plt.plot(jerks)
    plt.plot(accs)
    plt.plot(vels)
    plt.plot(poss)
    # plt.plot(vels_tsd, '--')
    # plt.plot(accs_tsd, '--')
    # plt.plot(jerks_tsd, '--')

    plt.plot(vels_ysd, '--')
    plt.plot(accs_ysd, '--')
    plt.plot(jerks_ysd, '--')

    plt.grid()
    plt.show()

