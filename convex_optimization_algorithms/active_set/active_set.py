#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg
from matplotlib import gridspec

import plot_result


epsilon = 1.0e-8

class ActiveSet():

    class Report():
        def __init__(self, i=-1, alpha=-1, x=np.matrix([]), y=np.matrix([]), x_ideal=np.matrix([]), dx=np.matrix([]), W=[]):
            self.i = i
            self.alpha = alpha
            self.x = x  # variable for primal problem
            self.y = y  # lagramge multiplier for g(x)=0
            self.dx = dx
            self.x_ideal = x_ideal
            # self.W = W
            self.W = W.copy()

    def __init__(self):
        self.initialized = False
        self.max_loop_num = 100
        self.reports = []


    def setProblem(self, Q, c, A, b):
        self.Q = Q
        self.c = c
        self.A = A
        self.b = b
        self.N = len(c)
        self.M = len(b)

        self.initialized = True


    def getInitialGuess(self):
        x = np.matrix(np.ones((self.N, 1))) * 0.0
        y = np.matrix([])
        # y = np.matrix(np.zeros((self.M, 1)))
        return (x, y)

    def solveConstrLinEq(self, W):
        if len(W) is 0:
            res = solveLinEq(self.Q, -self.c)
            return (res, np.matrix([]))

        (Ak, bk) = self.getCurrentConstraintMat(W)

        # generate lagrange matrix
        mk = len(W)
        A = np.block([[self.Q, Ak.transpose()], [Ak, np.zeros((mk, mk))]])
        b = np.block([[-self.c], [bk]])
        res = solveLinEq(A, b)
        return (res[0:self.N], res[self.N:])

    def getCurrentConstraintMat(self, W):
        Ak = np.matrix([])
        bk = np.matrix([])
        for i in W:
            if Ak.size == 0:
                Ak = self.A[i,:]
                bk = self.b[i,:]
            else:
                Ak = np.block([[Ak], [self.A[i,:]]])
                bk = np.block([[bk], [self.b[i,:]]])
        return (Ak, bk)

    def solve(self):
        if not self.initialized:
            return

        (x, y) = self.getInitialGuess()
        self.reports.append(ActiveSet.Report(i=0, alpha=0, x=x, y=y, dx=0.0))

        W = [0]  # initial constraint candidate
        for iter_num in range(1, self.max_loop_num):
            (x_next, y_next) = self.solveConstrLinEq(W)
            dx = x_next - x

            alpha = self.calcStepRatio(x, dx, W)
            x = x + alpha * dx
            y = y_next

            self.reports.append(ActiveSet.Report(i=iter_num, alpha=alpha, x=x, y=y, x_ideal=x_next, dx=dx, W=W))


            # no constraint while moving
            if alpha > 0.999999:
                # if all lagrange multipliers are positive, it is an optimal point.
                if  y.size == 0 or (y >= -epsilon).all():
                    print("optimization succeeded!")
                    return
                # remove lagrange multiplier if it is negative.
                else:
                    for i in range(len(y)):
                        if y[i] < -epsilon:
                            del W[i]
                            break
            else: # moving is constrainted (alpha < 1), reset active active set for next step
                W_next = []
                resi = self.A * x - self.b
                for i in range(len(resi)):
                    if abs(resi[i].item()) < epsilon:
                        W_next.append(i)
                W = W_next

        print("over max iteration num.")

    def calcStepRatio(self, x, dx, W):
        mu_k_min = 1.0
        for i in range(self.M):
            if i in W:
                continue
            den = (self.A[i,:] * dx).item()
            if den > 0:
                mu_k = (self.b[i] - self.A[i,:] * x).item() / den
                if mu_k < mu_k_min:
                    mu_k_min = mu_k
        return mu_k_min


def solveLinEq(A, b):
    LU = linalg.lu_factor(A)  # LU : much faster than pinv
    delta = linalg.lu_solve(LU, b)
    return np.matrix(delta)

def solveQP(Q, c, A, b):
    obj = ActiveSet()
    obj.setProblem(Q, c, A, b)
    obj.solve()

    print("| i |  alpha  |        x          |        y          | W |")
    print("---------------------------------------------------------------------------------------------------------------------------------")
    for r in obj.reports:
        print('|{0: 2d}'.format(r.i), '|{0: 8.2f}'.format(r.alpha), '|{0: 8.3f}'.format(r.x.item(0)), \
            ',{0: 8.3f}'.format(r.x.item(1)), "|  ", r.y.transpose(), "|  ", r.W)
    plot_result.plotReport(Q, c, A, b, obj.reports)

def main():

    # N = 2
    # M = 6

    A = np.matrix([[1.0, -1.0],
                   [1.0, 0.0],
                   [0.0,  1.0],
                   [-1.0, 0.0],
                   [0.0,  -1.0],
                   [-2.0, 1.0]])
    b = np.matrix([[3.0],
                   [6.0],
                   [4.0],
                   [0.0],
                   [0.0],
                   [2.0]])

    # No.1
    Q = np.matrix([[2.0, -0.],
                   [-0., 2.0]])
    c = np.matrix([[.0],
                   [-8.0]])
    solveQP(Q, c, A, b)

    # No.2
    Q = np.matrix([[2.0, -1.5],
                   [-1.5, 2.0]])
    c = np.matrix([[2.0],
                   [-6.0]])
    solveQP(Q, c, A, b)

    # No.3
    Q = np.matrix([[2.0, .0],
                   [.0, 2.0]])
    c = np.matrix([[-20.0],
                   [-2.0]])
    solveQP(Q, c, A, b)




if __name__ == "__main__":
    main()
