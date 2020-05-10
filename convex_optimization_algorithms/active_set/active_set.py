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
        def __init__(self, i=-1, alpha=-1, x=np.matrix([]), y=np.matrix([]), dx=np.matrix([]), W=[]):
            self.i = i
            self.alpha = alpha
            self.x = x  # variable for primal problem
            self.y = y  # lagramge multiplier for g(x)=0
            self.dx = dx
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
            print(" --------------------------------  ")
            print("AAAA = ", self.reports[0].W)
            print("i = ", iter_num)

            (x_next, y_next) = self.solveConstrLinEq(W)
            dx = x_next - x
            print("x_next = ", x_next.transpose())
            print("y_next = ", y_next.transpose())

            alpha = self.calcStepRatio(x, dx, W)
            print("alpha = ", alpha)
            x = x + alpha * dx
            y = y_next

            print("x = ", x.transpose())
            print("y = ", y.transpose())
            print("W = ", W)
            self.reports.append(ActiveSet.Report(i=iter_num, alpha=alpha, x=x, y=y, dx=dx, W=W))


            # 移動中に拘束条件に引っかからなかった場合
            if alpha > 0.999999:
                print("Judge : alpha == 1")
                # ラグランジュ乗数が全て正なら、そこが最適解
                if  y.size == 0 or (y >= -epsilon).all():
                    print("optimization succeeded!")
                    return
                # ラグランジュに負のものがあれば、それを取り除いて再計算
                else:
                    print("y < 0 is found")
                    for i in range(len(y)):
                        if y[i] < -epsilon:
                            print("y[", i, "]  = ", y[i], " < 0 is being deleted, and optimize again.")
                            del W[i]
                            break
            else: # 移動中に拘束条件に引っかかった場合（alpha < 1）、Wを拘束条件から再設定
                W_next = []
                resi = self.A * x - self.b
                for i in range(len(resi)):
                    if abs(resi[i].item()) < epsilon:
                        W_next.append(i)
                print("Judge : alpha < 1 : len(W_k) = ", len(W), ", len(W_k+1) = ", len(W))
                print("W_k = ", W)
                print("W_k+1 = ", W_next)
                W = W_next
                 
        print("over max iteration num.")

    def calcStepRatio(self, x, dx, W):
        mu_k_min = 1.0
        for i in range(self.M):
            if i in W:
                print("calcStepRatio() : i = ", i, ", is in W. continue.")
                continue
            den = (self.A[i,:] * dx).item()
            print("calcStepRatio() : i = ", i, ", den = ", den, ", A[i,:] = ", self.A[i,:], ", dx = ", dx.transpose())
            if den > 0:
                mu_k = (self.b[i] - self.A[i,:] * x).item() / den
                print("calcStepRatio() : i = ", i, ", is not in W. mu = ", mu_k, " (mu_k_min = ", mu_k_min, ")")
                if mu_k < mu_k_min:
                    print("mu_k < mu_k_min. updated")
                    mu_k_min = mu_k
        return mu_k_min


def solveLinEq(A, b):
    LU = linalg.lu_factor(A)  # LU : much faster than pinv
    delta = linalg.lu_solve(LU, b)
    return np.matrix(delta)

def main():

    N = 2
    M = 6

    Q = np.matrix([[2.0, 0.0],
                   [0.0, 2.0]])
    c = np.matrix([[0.0],
                   [-8.0]])
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


    obj = ActiveSet()
    obj.setProblem(Q, c, A, b)
    obj.solve()

    print("| i |  alpha  |        x          |        y          | W |")
    print("---------------------------------------------------------------------------------------------------------------------------------")
    for r in obj.reports:
        print('|{0: 2d}'.format(r.i), '|{0: 8.2f}'.format(r.alpha), '|{0: 8.3f}'.format(r.x.item(0)), \
            ',{0: 8.3f}'.format(r.x.item(1)), "|  ", r.y.transpose(), "|  ", r.W)

    plot_result.plotReport(Q, c, A, b, obj.reports)



if __name__ == "__main__":
    main()
