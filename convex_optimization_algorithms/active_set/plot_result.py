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

def getLagrangeMultiplierArray(M, report):
    y_arr = np.zeros(M)
    k = 0
    for j in range(M):
        if j in report.W:
            y_arr[j] = report.y[k].item()
            k += 1
    return y_arr

def removeZeroFromMat(A):
    ep = 1.0e-12
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            if -ep < A[i, j] < ep:
                A[i, j] = ep
    return A

def plotReport(Q, c, A, b, reports):

    M = b.shape[0]
    fig = plt.figure(1, figsize=(10.0, 7.0))
    gs = fig.add_gridspec(1, 10)
    ax1 = fig.add_subplot(gs[0, 0:7], xlabel='x1', ylabel='x2')
    ax2 = fig.add_subplot(gs[0, 8:10], title="lagrange multplier")
    ax2.grid()


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
    area2x = [-6, 12, 12, -6]
    area2y = [4, 4, 12, 12]
    ax1.fill(area2x,area2y,color="gray",alpha=0.4)

    # plot constraint line
    ax1.plot(t, np.zeros(len(t)), 'k')
    ax1.plot(np.zeros(len(t)), t, 'k')
    ax1.axis([xmin, xmax, ymin, ymax])
    for i in range(A.shape[0]):
        ax1.plot(t, s[i], 'gray')

    ax1.plot(reports[0].x[0], reports[0].x[1], 'ko')


    ax2.set_xlim(0, M+0.5)
    ax2.set_ylim(-5, 5)
    ax2.plot(range(10), np.zeros(10), 'k')
    y_bar = getLagrangeMultiplierArray(M, reports[0])
    ax2b = ax2.bar(range(1,M+1), y_bar, width=0.5)
    
    plt.pause(1.0)

    constr_lines = []
    for i in range(1, len(reports)):
        for line in constr_lines:
                line.remove()
        constr_lines = []
        for j in range(A.shape[0]):
            if j in reports[i].W:
                l, = ax1.plot(t, s[j], "red")
                constr_lines.append(l)
        yoko = [reports[i-1].x[0].item(), reports[i].x[0].item()]
        tate = [reports[i-1].x[1].item(), reports[i].x[1].item()]
        ax1.plot(yoko, tate, 'k-')
        ax1.plot(reports[i].x[0], reports[i].x[1], 'ko')

        y_bar = getLagrangeMultiplierArray(M, reports[i])
        ax2b.remove()
        ax2b = ax2.bar(range(1,M+1), y_bar, width=0.5)
        print("i = ", i, ", y_bar = ", y_bar)

        plt.pause(0.5)

    plt.show()
