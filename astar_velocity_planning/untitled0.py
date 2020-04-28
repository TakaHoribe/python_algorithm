#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 16 13:51:45 2020

@author: horibe
"""

sigma_x0 = 10
sigma_z = 2
x0 = 0
z = 5

k = sigma_x0**2 / (sigma_x0**2 + sigma_z**2)
x_ekf = x0 + k * (z - x0)
sigma_ekf = (1 - k) * (sigma_x0**2)
print("x = ", x_ekf, ", ")


sigma_x = sigma_x0
x = x0
sigma_z = sigma_z * 10
for i in range(10):
    k = sigma_x**2 / (sigma_x**2 + sigma_z**2)
    x = x + k * (z - x)
    sigma_x = (1 - k) * sigma_x
x_ekf_for = x
sigma_ekf_for = sigma_x