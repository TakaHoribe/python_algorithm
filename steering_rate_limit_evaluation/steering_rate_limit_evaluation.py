# -- steering rate limit evaluation --
# This code calculates the permissible steering rates for each vehicle speed based on the given
# vehicle shape and the lateral distance to obstacles. The results are provided in the form of a
#  2D plot, showing the time until collision with an obstacle for each vehicle speed and steering 
# rate value.

import sys
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Update the kinematic model to calculate the front wheel edge position
def kinematic_model_with_front_wheel(t, state, vehicle_speed, steering_angle_rate, wheelbase, rear_to_edge):
    _, _, theta, steering_angle = state
    dxdt_rear = vehicle_speed * np.cos(theta)
    dydt_rear = vehicle_speed * np.sin(theta)
    dthetadt = vehicle_speed * np.tan(steering_angle) / wheelbase
    dsteering_angledt = steering_angle_rate

    return [dxdt_rear, dydt_rear, dthetadt, dsteering_angledt]

def calc_margin_time(vehicle_speed, steering_angle_rate):

    # Adjust initial state to include front wheel position
    initial_state_with_front_wheel = [0, 0, 0, 0] 

    # Time span for the simulation
    t_span = [0, 5]  # Arbitrary end time, can be adjuste
    t_eval = np.arange(0, 5, 0.01)

    # Solve the system of differential equations using Runge-Kutta method
    solution_with_front_wheel = solve_ivp(
        kinematic_model_with_front_wheel, t_span, initial_state_with_front_wheel,
        args=(vehicle_speed, steering_angle_rate, wheelbase, rear_to_edge), t_eval=t_eval, dense_output=True)

    # Extract the solution
    t_with_front_wheel = solution_with_front_wheel.t
    x_rear, y_rear, theta, _ = solution_with_front_wheel.y

    # Calculate the front wheel edge position
    x_front = x_rear + rear_to_edge * np.cos(theta + psi)
    y_front = y_rear + rear_to_edge * np.sin(theta + psi) - vehicle_width*0.5

    # Determine the time when the front wheel has moved laterally by distance D
    for i in range(len(t_with_front_wheel)):
        if abs(y_front[i]) >= margin_distance:
            break

    return t_with_front_wheel[i]

def generate_2d_plot(result, vehicle_speed_array_kmph, steering_angle_rate_array, margin_time_threshold):
    result_flipud = np.flipud(result)
    plt.figure(figsize=(22, 8))
    for (i, j), val in np.ndenumerate(result_flipud):
        color = 'red' if val < margin_time_threshold else 'black'
        plt.text(j, i, f'{val:.2f}', va='center', ha='center', color=color)

    # Make a grid plot
    plt.imshow(result_flipud, cmap='Wistia', interpolation='nearest')
    plt.colorbar()
    plt.xticks(ticks=np.arange(len(steering_angle_rate_array)), labels=steering_angle_rate_array)
    plt.yticks(ticks=np.arange(len(vehicle_speed_array)), labels=np.around(vehicle_speed_array_kmph, 1)[::-1])
    plt.xlabel('Steering Rate (rad/s)')
    plt.ylabel('Vehicle Speed (km/h)')
    plt.title('Margin Time [s]')
    plt.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1)
    plt.show()

# Parameters (can be adjusted as needed)
wheelbase = 4.0  
vehicle_length = 5.0  # length from rear wheel to the front of the vehicle (m)
vehicle_width = 2.0
margin_distance = 1.0
margin_time_threshold = 1.5  # Cell is colorized with red for the margin time less than this threshold
rear_to_edge = np.sqrt((0.5 * vehicle_width) ** 2 + vehicle_length ** 2)
psi = np.arctan(0.5 * vehicle_width / vehicle_length)  # Angle from rear center to front left edge

MPS_TO_KMPH = 3.6
# constant speed of the vehicle (m/s)
vehicle_speed_array_kmph = np.array([1.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40, 45, 50, 55, 60])
vehicle_speed_array = vehicle_speed_array_kmph / MPS_TO_KMPH

# rate at which the steering angle changes (rad/s)
steering_angle_rate_array = np.array([0.002, 0.004, 0.006, 0.008, 0.01, 0.012, 0.015, 0.02, 0.03, 0.04, 
                                      0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 
                                      0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.4, 0.45, 0.5])  

result = np.zeros((len(vehicle_speed_array), len(steering_angle_rate_array)))

# Main process to calculate margin time
for i in range(len(vehicle_speed_array)):
    for j in range(len(steering_angle_rate_array)):
        time = calc_margin_time(vehicle_speed_array[i], steering_angle_rate_array[j])
        result[i, j] = time

np.set_printoptions(precision=2, floatmode='fixed', suppress=True, linewidth=200)
print(result)
np.savetxt(sys.stdout, result, fmt='%.2f', delimiter=', ')

# Generate 2D plot
generate_2d_plot(result, vehicle_speed_array_kmph, steering_angle_rate_array, margin_time_threshold)