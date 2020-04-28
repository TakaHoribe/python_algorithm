#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 15:27:22 2020

@author: horibe
"""


import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


wheelbase = 2.79

estimate_twist = pd.read_csv('estimate_twist.csv')
vehicle_status = pd.read_csv('vehicle_status.csv')
steer_rpt = pd.read_csv('steer_rpt.csv')

steer = steer_rpt["field.output"].values
t_steer = steer_rpt["field.header.stamp"].values

vel = vehicle_status["field.status.velocity"].values
t_vel = vehicle_status["field.header.stamp"].values

wzndt = estimate_twist["field.twist.angular.z"].values
t_wzndt = estimate_twist["field.header.stamp"].values

ts = max(t_steer[0], t_wzndt[0], t_vel[0])

t_steer_s = (t_steer - ts) / 1e9
t_wzndt_s = (t_wzndt - ts) / 1e9
t_vel_s = (t_vel - ts) / 1e9
ts_s = ts / 1e9
te_s = min(t_steer_s[-1], t_wzndt_s[-1], t_vel_s[-1])

dt_s = 0.03
tp = np.arange(9.75, 191.0, dt_s)



# interpolation for time align
steer_p = np.interp(tp, t_steer_s, steer)
vel_p = np.interp(tp, t_vel_s, vel)
wzndt_p = np.interp(tp, t_wzndt_s, wzndt)

tire_agle = np.arctan(wzndt_p * wheelbase / vel_p)

steer_p_x = steer_p[abs(tire_agle) > 0.01]
tp_x = tp[abs(tire_agle) > 0.01]
vel_p_x = vel_p[abs(tire_agle) > 0.01]
tire_agle_x = tire_agle[abs(tire_agle) > 0.01]



# steer_p_xx = steer_p_x[tire_angle ]

N_arr = steer_p_x / tire_agle_x

X_arr = np.vstack((np.ones(len(vel_p_x)), vel_p_x, vel_p_x**2, steer_p_x))

"""
plt.plot(tp, steer_p, '.-')
plt.plot(t_steer_s, steer, 'x-', color="red")
"""
plt.plot(tp_x, N_arr)

A_hat = np.linalg.lstsq(X, y)[0]


A_hat = np.linalg.lstsq(X, y)[0]