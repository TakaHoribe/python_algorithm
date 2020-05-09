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

def plotReport(Q, c, A, b, reports):

    plt.figure(1, figsize=(12.0, 6.0))
    gs = gridspec.GridSpec(1,3, width_ratios=[3,1,1]) 
    plt.subplot(gs[0])

    ep = 1.0e-12
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            if -ep < A[i, j] < ep:
                A[i, j] = ep


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
    # plt.pcolor(X, Y, Z, cmap="Oranges")
    plt.contourf(X, Y, Z, 15, cmap="Oranges")
    area1x = [-6, 1, 0, 0, 3, 6, 6, 12, 12, -6]
    area1y = [4, 4, 2, 0, 0, 3, 4, 4, -6, -6]
    plt.fill(area1x,area1y,color="gray",alpha=0.4)
    area1x = [-6, 12, 12, -6]
    area1y = [4, 4, 12, 12]
    plt.fill(area1x,area1y,color="gray",alpha=0.4)
    plt.xlabel("x1")
    plt.ylabel("x2")

    plt.plot(t, np.zeros(len(t)), 'k')
    plt.plot(np.zeros(len(t)), t, 'k')
    plt.axis([xmin, xmax, ymin, ymax])

    for i in range(A.shape[0]):
        plt.plot(t, s[i], 'gray')

    plt.subplot(gs[1])
    plt.xlim(0, len(reports)-1)
    plt.ylim(reports[0].obj_dual, reports[0].obj_primal)
    plt.title("object value")

    plt.subplot(gs[2])
    plt.xlim(0, len(reports)-1)
    plt.ylim(reports[-1].mu, reports[0].mu)
    plt.title("dual gap")
    
    plt.pause(0.1)

    for i in range(0, len(reports)):
        plt.subplot(gs[0])
        plt.plot(reports[i].x[0], reports[i].x[1], 'ko')
        if i is not 0:
            yoko = [reports[i-1].x[0].item(), reports[i].x[0].item()]
            tate = [reports[i-1].x[1].item(), reports[i].x[1].item()]
            plt.plot(yoko, tate, 'k-')

        plt.subplot(gs[1])
        if i is 0:
            plt.plot(i, reports[i].obj_primal, 'bo', label='primal')
            plt.plot(i, reports[i].obj_dual, 'ro', label='dual')
        else:
            plt.plot(i, reports[i].obj_primal, 'bo')
            plt.plot(i, reports[i].obj_dual, 'ro') 
        if i is not 0:
            plt.plot([i-1, i], [reports[i-1].obj_primal, reports[i].obj_primal], 'b-')
            plt.plot([i-1, i], [reports[i-1].obj_dual, reports[i].obj_dual], 'r-')
        plt.grid()
        plt.legend()

        plt.subplot(gs[2])
        plt.plot(i, reports[i].mu, 'ko')
        if i is not 0:
            plt.plot([i-1, i], [reports[i-1].mu, reports[i].mu], 'k-')

        plt.yscale('log')
        plt.grid(which='minor')
        plt.grid()

        plt.pause(0.5)
        plt.subplot(gs[2])
        plt.grid(which='minor')
        plt.grid()
        plt.subplot(gs[1])
        plt.grid()
    plt.subplot(gs[2])
    plt.grid(which='minor')
    plt.grid()
    plt.subplot(gs[1])
    plt.grid()


    # ---------------------------------




    plt.show()
