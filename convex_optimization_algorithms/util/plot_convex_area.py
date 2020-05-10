#!/usr/bin/env python


import math
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg
from matplotlib import gridspec


def problemFunc(x, y, Q, c):
    return 0.5 * (Q[0, 0] * x**2 + Q[1, 1] * y**2 + 2 * Q[0, 1] * x * y) + c[0, 0] * x + c[1, 0] * y




Q = np.matrix([[2.0, 0.0],
                [0.0, 2.0]])
c = np.matrix([[0.0],
                [0.0]])



plt.figure(1, figsize=(12.0, 6.0))
gs = gridspec.GridSpec(1,3, width_ratios=[3,1,1]) 
plt.subplot(gs[0])



xmin = -2.0
xmax = 10.5
ymin = -2.0
ymax = 10.5
dx = 0.1
ep = 1.0e-5
x = np.arange(xmin, xmax + ep, dx)
y = np.arange(ymin, ymax + ep, dx)
X,Y = np.meshgrid(x, y)
t = np.arange(xmin, xmax, 0.1)
Z = problemFunc(X, Y, Q, c)
# plt.pcolor(X, Y, Z, cmap="Oranges")
# plt.contourf(X, Y, Z, 15, cmap="Oranges")
plt.plot_surface(X, Y, Z, cmap='bwr', linewidth=0)
# area1x = [-6, 1, 0, 0, 3, 6, 6, 12, 12, -6]
# area1y = [4, 4, 2, 0, 0, 3, 4, 4, -6, -6]
# plt.fill(area1x,area1y,color="gray",alpha=0.4)
# area1x = [-6, 12, 12, -6]
# area1y = [4, 4, 12, 12]
# plt.fill(area1x,area1y,color="gray",alpha=0.4)
plt.xlabel("x1")
plt.ylabel("x2")

plt.plot(t, np.zeros(len(t)), 'k')
plt.plot(np.zeros(len(t)), t, 'k')
plt.axis([xmin, xmax, ymin, ymax])


plt.title("-nu * log(x)")
plt.grid()
plt.legend()
plt.show()

# print(y1)
# print(x)