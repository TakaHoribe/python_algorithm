#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec

def problemFunc(x, y, Q, c):
    return 0.5 * (Q[0, 0] * x**2 + Q[1, 1] * y**2 + 2 * Q[0, 1] * x * y) + c[0, 0] * x + c[1, 0] * y

def ProblemConstr(x, A, b):
    return [-A[i, 0] / A[i, 1] * x + b[i, 0] / A[i, 1] for i in range(A.shape[0])]

def removeZeroFromMat(A):
    ep = 1.0e-12
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            if -ep < A[i, j] < ep:
                A[i, j] = ep
    return A

def plotReport(Q, c, A, b, reports):
    fig = plt.figure(1, figsize=(10.0, 7.0))
    plt.clf()
    gs = fig.add_gridspec(2, 10)
    ax1 = fig.add_subplot(gs[:, 0:7], xlabel='x1', ylabel='x2')
    ax2 = fig.add_subplot(gs[0, 8:10], title="object value")
    ax3 = fig.add_subplot(gs[1, 8:10], title="dual gap", yscale="log")

    ax2.grid()
    ax3.grid()
    ax3.grid(which='minor')

    A = removeZeroFromMat(A)

    # set color map
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
    s = ProblemConstr(t, A, b)
    ax1.contourf(X, Y, Z, 15, cmap="Oranges")

    # fill feasible area
    area1x = [-6, 1, 0, 0, 3, 6, 6, 12, 12, -6]
    area1y = [4, 4, 2, 0, 0, 3, 4, 4, -6, -6]
    ax1.fill(area1x,area1y,color="gray",alpha=0.4)
    area1x = [-6, 12, 12, -6]
    area1y = [4, 4, 12, 12]
    ax1.fill(area1x,area1y,color="gray",alpha=0.4)

    # plot constraint line
    ax1.plot(t, np.zeros(len(t)), 'k')
    ax1.plot(np.zeros(len(t)), t, 'k')
    ax1.axis([xmin, xmax, ymin, ymax])
    for i in range(A.shape[0]):
        ax1.plot(t, s[i], 'gray')

    # plot initial point
    ax1.plot(reports[0].x[0], reports[0].x[1], 'ko')

    # plot objective variables
    ax2.set_xlim(0, len(reports)-1)
    max_primal = max([reports[i].obj_primal for i in range(len(reports))])
    min_dual = min([reports[i].obj_dual for i in range(len(reports))])
    ax2.set_ylim(min_dual, max_primal)
    # ax2.set_ylim(reports[0].obj_dual, reports[0].obj_primal)
    ax2.plot(0, reports[0].obj_primal, 'bo', label='primal')
    ax2.plot(0, reports[0].obj_dual, 'ro', label='dual')
    ax2.legend()

    # plot dual gap
    ax3.set_xlim(0, len(reports)-1)
    ax3.set_ylim(reports[-1].mu, reports[0].mu)
    ax3.plot(0, reports[0].mu, 'ko')
    
    plt.pause(0.1)

    for i in range(1, len(reports)):
        ax1.plot(reports[i].x[0], reports[i].x[1], 'ko')
        yoko = [reports[i-1].x[0].item(), reports[i].x[0].item()]
        tate = [reports[i-1].x[1].item(), reports[i].x[1].item()]
        ax1.plot(yoko, tate, 'k-')

        ax2.plot(i, reports[i].obj_primal, 'bo')
        ax2.plot(i, reports[i].obj_dual, 'ro') 
        ax2.plot([i-1, i], [reports[i-1].obj_primal, reports[i].obj_primal], 'b-')
        ax2.plot([i-1, i], [reports[i-1].obj_dual, reports[i].obj_dual], 'r-')

        ax3.plot(i, reports[i].mu, 'ko')
        ax3.plot([i-1, i], [reports[i-1].mu, reports[i].mu], 'k-')

        plt.pause(0.1)

    ax1.plot(reports[-1].x[0], reports[-1].x[1], 'k*', markersize=15)

    # plt.show()
    plt.pause(1.0)
