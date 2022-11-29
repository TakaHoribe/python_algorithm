import numpy as np
import math
import matplotlib.pyplot as plt
import copy




if __name__ == '__main__':

    v_ego = 10.0     # [m/s]
    t_predict = 10.0 # [s]
    t_react = 2.0
    t_margin = 2.0
    a_rear = -1.0
    a_front = -1.0

    v_npc_array = np.arange(0, 20, 2)
    initial_distance_array = np.arange(-20, 20, 4)

    result = []

    for d0 in initial_distance_array:
        if d0 > 0:
            d_npc = v_npc_array * t_predict + v_npc_array**2/(2.0 * a_front) + d0
            d_ego = v_ego * (t_predict +t_react + t_margin) + v_ego**2 / (2.0 * a_rear)
        else:
            d_npc = v_npc_array * (t_predict + t_react + t_margin) + v_npc_array**2 / (2.0 * a_rear) + d0
            d_ego = v_ego * t_predict + v_ego**2 / (2.0 * a_front)
        d_result = d_npc - d_ego
        has_collision = (d_result * d0) < 0
        result.append(d_result)

    print(result)

    plt.imshow(result)
    plt.show()
