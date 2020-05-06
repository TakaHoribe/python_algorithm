#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg


class PrimalDualInterpoint():

    class Report():
        def __init__(self, i=-1, mu=0, alpha=0, x=np.matrix([]), y=np.matrix([]), \
                     z=np.matrix([]), dx=np.matrix([]), dy=np.matrix([]), dz=np.matrix([]), \
                     obj_primal=0, obj_dual=0, resi_primal=0, resi_dual=0):
            self.i = i
            self.mu = mu
            self.alpha = alpha
            self.x = x
            self.y = y
            self.z = z
            self.dx = dx
            self.dy = dy
            self.dz = dz
            self.obj_primal = obj_primal
            self.obj_dual = obj_dual
            self.resi_primal = resi_primal
            self.resi_dual = resi_dual

    def __init__(self):
        self.initialized = False
        self.mu0 = 10.0
        self.sigma = 0.01  # about 0.1 ~ 0.01
        self.tau = 0.95    # about 0.95 ~ 0.99
        self.max_loop_num = 100
        self.tol_mu = 0.001
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
        x = np.matrix(np.ones((self.N, 1))) * self.mu0
        y = np.matrix(np.zeros((self.M, 1)))
        z = np.matrix(np.ones((self.N, 1))) * self.mu0
        return (x, y, z)

    def calcPrimalObj(self, x):
        obj_primal = 0.5 * x.transpose() * self.Q * x + self.c.transpose() * x
        return obj_primal.item()

    def calcDualObj(self, x, y):
        obj_dual = - 0.5 * x.transpose() * self.Q * x + self.b.transpose() * y 
        return obj_dual.item()


    def solve(self):
        if not self.initialized:
            return

        (x, y, z) = self.getInitialGuess()
        mu = self.calcDualGap(x, z)
        self.reports.append(PrimalDualInterpoint.Report(i=0, mu=mu, x=x, y=y, z=z, \
                            obj_primal=self.calcPrimalObj(x), obj_dual=self.calcDualObj(x, y)))

        for iter_num in range(1, self.max_loop_num):
            (A_pd, B_pd) = self.calcPrimalDualMatrix(x, y, z, mu)
            deltas = solveLinEq(A_pd, B_pd)
            dx = deltas[0:self.N, 0]
            dy = deltas[self.N:self.N + self.M, 0]
            dz = deltas[self.N + self.M:, 0]
            alpha = self.calcStepRatio(x, z, dx, dz)

            x = x + alpha * dx
            y = y + alpha * dy
            z = z + alpha * dz

            mu = self.calcDualGap(x, z)
            self.reports.append(PrimalDualInterpoint.Report(i=iter_num, mu=mu, alpha=alpha, x=x, y=y, \
                                                            z=z, dx=dx, dy=dy, dz=dz, \
                                                            obj_primal=self.calcPrimalObj(x), \
                                                            obj_dual=self.calcDualObj(x, y),
                                                            resi_primal=B_pd.item(0), resi_dual=B_pd.item(1)))

            print("i = ", iter_num)
            print("alpha = ", alpha)
            print("x = ", x.transpose())
            print("y = ", y.transpose())
            print("z = ", z.transpose())
            print("dx = ", dx.transpose())
            print("dy = ", dy.transpose())
            print("dz = ", dz.transpose())
            print("mu = ", mu)

            if abs(mu) < self.tol_mu:
                print("optimization succeeded")
                break

    print("over max iteration num.")

    def calcStepRatio(self, x, z, dx, dz):
        some_larger_num_than_1 = 100.0
        ax = np.ones(self.N) * some_larger_num_than_1
        az = np.ones(self.N) * some_larger_num_than_1
        for i in range(self.N):
            if dx[i] < 0.0:
                ax[i] = -x[i] / dx[i]
            if dz[i] < 0.0:
                az[i] = -z[i] / dz[i]
        a = min(self.tau * np.amin(ax), self.tau * np.amin(az), 1.0)
        if a < -1.0E-5:
            print("!!!!!!!! ERROR !!!!!!!!!!!!!!!, a = ", a)
        # print("np.array(x) = ", np.array(x))
        # print("np.array(dx) = ", np.array(dx))
        # print("ax = ", ax)
        # print("ax.min = ", np.amin(ax))
        return a


    def calcPrimalDualMatrix(self, x, y, z, mu):
        N = self.N
        M = self.M
        Z = np.diag(np.array(z).reshape(N,))
        X = np.diag(np.array(x).reshape(N,))
        Aex = np.block([[self.A, np.zeros((M, M)), np.zeros((M, N))],
                        [-self.Q, self.A.transpose(), np.eye(N)],
                        [Z, np.zeros((N, N)), X]])
        Bex = np.block([[self.b - self.A * x],
                        [self.c + self.Q * x - self.A.transpose() * y - z],
                        [self.sigma * mu * np.ones((N, 1)) - X * z]])
        # print("Aex = ", Aex)
        # print("Bex = ", Bex)
        return (Aex, Bex)

    def calcDualGap(self, x, z):
        mu = (x.transpose() * z) / self.N
        return mu.item()

def solveLinEq(A, b):
    LU = linalg.lu_factor(A)  # LU : much faster than pinv
    delta = linalg.lu_solve(LU, b)
    # print("delta = ", delta)
    return np.matrix(delta)

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

def plotReport(Q, c, A, b, reports):
    plt.figure(1, figsize=(30.0, 10.0))
    plt.subplot(131)

    x_min = 0.1
    x_max = 5.31
    dx = 0.2
    x = np.arange(x_min, x_max, dx)
    y = np.arange(x_min, x_max, dx)
    X,Y = np.meshgrid(x, y)
    t = np.arange(x_min, x_max, dx)
    func = lambda x, y : 0.5 * (Q[0, 0] * x**2 + Q[1, 1] * y**2 + 2 * Q[0, 1] * x * y) + c[0, 0] * x + c[1, 0] * y
    const = [lambda x : -A[i, 0] / A[i, 1] * x + b[i, 0] / A[i, 1] for i in range(A.shape[0])]
    Z = func(X, Y)
    s = [const[i](t) for i in range(A.shape[0])]
    plt.pcolor(X, Y, Z)

    plt.plot(t, np.zeros(len(t)), 'k')
    plt.plot(np.zeros(len(t)), t, 'k')
    plt.axis([x_min, x_max, x_min, x_max])

    # plt.xscale('log')
    # plt.yscale('log')

    for i in range(A.shape[0]):
        plt.plot(t, s[i], 'gray')

    for i in range(0, len(reports)):
        plt.subplot(131)
        plt.plot(reports[i].x[0], reports[i].x[1], 'ko')
        if i is not 0:
            yoko = [reports[i-1].x[0].item(), reports[i].x[0].item()]
            tate = [reports[i-1].x[1].item(), reports[i].x[1].item()]
            plt.plot(yoko, tate, 'k-')

        plt.subplot(132)
        # plt.plot(i, reports[i].obj_primal, 'bo', legend='primal')
        # plt.plot(i, reports[i].obj_dual, 'ro', legend='dual')
        if i is not 0:
            plt.plot([i-1, i], [reports[i-1].obj_primal, reports[i].obj_primal], 'b-')
            plt.plot([i-1, i], [reports[i-1].obj_dual, reports[i].obj_dual], 'r-')

        plt.subplot(133)
        plt.plot(i, reports[i].mu, 'ko')
        if i is not 0:
            plt.plot([i-1, i], [reports[i-1].mu, reports[i].mu], 'k-')

        plt.yscale('log')

        plt.pause(1.0)


    # ---------------------------------




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

    # Q = np.matrix([[2.0, 0.0],
    #                [0.0, 2.0]])
    # c = np.matrix([[-4.0],
    #                [-2.0]])
    # A = np.matrix([[1.0, -1.0]])
    # b = np.matrix([[-1.0]])

    obj = PrimalDualInterpoint()
    obj.setProblem(Q, c, A, b)
    obj.solve()

    print("|  i   |       mu       | alpha  |           x            |           y           |          z          | obj prime |  obj dual |   Ax-b   | Qx+x-Ay-z |")
    print("---------------------------------------------------------------------------------------------------------------------------------")
    for r in obj.reports:
        print('{0: 2d}'.format(r.i), '{0: 13.5f}'.format(r.mu), '{0: 6.3f}'.format(r.alpha), '{0: 8.3f}'.format(r.x.item(0)), \
            '{0: 8.3f}'.format(r.x.item(1)), '{0: 8.3f}'.format(r.y.item(0)), '{0: 8.3f}'.format(r.y.item(1)), \
            '{0: 8.3f}'.format(r.z.item(0)), '{0: 8.3f}'.format(r.z.item(1)), '{0: 8.3f}'.format(r.obj_primal), '{0: 8.3f}'.format(r.obj_dual), \
            '{0: 7.3f}'.format(r.resi_primal), '{0: 7.3f}'.format(r.resi_dual))

    # plotQP(Q, c, A, b)
    plotReport(Q, c, A, b, obj.reports)



if __name__ == "__main__":
    main()