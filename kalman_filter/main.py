#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from matplotlib import pyplot as plt


class KalmanFilter():

    def __init__(self, A, C, x0, P0):
        self.A = A
        self.C = C
        self.x = x0
        self.P = P0
        self.R = None
        self.Q = None
        self.N = len()

    def __init__(self, A, C, x0, P0, R, Q):
        self.__init__(A, C, x0, P0)
        self.Q = Q
        self.R = R

    def update(self, y):
        e = y - self.C * self.x
        S = self.R + self.C * self.P * np.transpose(self.C)
        K = self.P * np.transpose(self.C) * np.linalg.inv(S)
        self.x = self.x + K * e
        self.P = (np.identity(len(self.x)) - K * self.C) * self.P

    def predict(self):
        self.x = self.A  * self.x

    def print(self):
        print("x = ", self.x)
        print("P = ", self.P)


def init():
    dt = 0.1
    A = np.matrix([[1, dt],[0, 1]])
    C = np.matrix([[1, 0],[0, 1]])
    Q = np.matrix([[0.1, 0],[0, 0.1]])
    R = np.matrix([[2, 0],[0, 2]])
    x0 = np.matrix([[100],[10]])
    P0 = np.matrix([[4, 0],[0, 4]])
    y = np.matrix([[120],[20]])
    return (A, C, Q, R, x0, P0, y)

if __name__ == '__main__':

    (A, C, Q, R, x0, P0, y) = init()

    kf = KalmanFilter(A, C, x0, P0)

    kf.update(y)
