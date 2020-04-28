#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import time
import matplotlib.pyplot as plt
import numpy as np

class PrimalDualInterpoint():

    def __init__(self):
        self.initialized = False

    def setProblem(self, Q, c, A, b):
        self.Q = Q
        self.c = c
        self.A = A
        self.b = b
        self.N = len(c)
        self.M = len(b)


    def solve(self):
        if not self.initialized:
            return

    def setInitialGuess(self, x, y, z):
        x = np.matrix()
        self.initialized = True
        return

    
    def calcDualGap(self, x, z):
        n = len(x)
        mu = (x * z) / n
        return mu

def plotQP(Q, c, A, b):

    plt.figure(1)
    plt.subplot(111)

    x = np.arange(-0.5, 2.01, 0.1)
    y = np.arange(-0.5, 2.01, 0.1)
    X,Y = np.meshgrid(x, y)
    t = np.arange(-0.5, 2.01, 0.1)
    func = lambda x, y : 0.5 * (Q[0, 0] * x**2 + Q[1, 1] * y**2 + 2 * Q[0, 1] * x * y) + c[0, 0] * x + c[1, 0] * y
    const = [lambda x : -A[i, 0] / A[i, 1] * x + b[i, 0] / A[i, 1] for i in range(A.shape[0])]
    Z = func(X, Y)
    s = [const[i](t) for i in range(A.shape[0])]
    plt.pcolor(X, Y, Z)

    for i in range(A.shape[0]):
        plt.plot(t, s[i], 'gray')

    plt.plot(t, np.zeros(len(t)), 'k')
    plt.plot(np.zeros(len(t)), t, 'k')
    plt.axis([-0.5, 2, -0.5, 2])
    plt.show()

def main():

    Q = np.matrix([[2.0, 0.0],
                   [0.0, 2.0]])
    c = np.matrix([[-2.0],
                   [-4.0]])
    A = np.matrix([[2.0, -2.0],
                   [4.0,  2.0]])
    b = np.matrix([[-1.0],
                   [ 5.0]])

    plotQP(Q, c, A, b)

    obj = PrimalDualInterpoint()
    obj.setProblem(Q, c, A, b)
    obj.solve()


if __name__ == "__main__":
    main()