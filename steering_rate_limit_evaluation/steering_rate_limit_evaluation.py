# -- steering rate limit evaluation --
# This code calculates the permissible steering rates for each vehicle speed based on the given
# vehicle shape and the lateral distance to obstacles. The results are provided in the form of a
#  2D plot, showing the time until collision with an obstacle for each vehicle speed and steering 
# rate value.

import sys
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parameters (can be adjusted as needed)
wheelbase = 4.0  # length of the vehicle (m)
vehicle_length = 5.0  # length of the vehicle (m)
vehicle_width = 2.0  # width of the vehicle (m)
margin_distance = 1.0  # lateral distance to move before stopping the simulation (m)

rear_to_edge = np.sqrt((0.5 * vehicle_width) ** 2 + vehicle_length ** 2)
psi = np.arctan(0.5 * vehicle_width / vehicle_length)


# Update the kinematic model to calculate the front wheel edge position
def kinematic_model_with_front_wheel(t, state, vehicle_speed, steering_angle_rate, wheelbase, rear_to_edge):
    x_rear, y_rear, theta, steering_angle= state
    dxdt_rear = vehicle_speed * np.cos(theta)
    dydt_rear = vehicle_speed * np.sin(theta)
    dthetadt = vehicle_speed * np.tan(steering_angle) / wheelbase
    dsteering_angledt = steering_angle_rate

    return [dxdt_rear, dydt_rear, dthetadt, dsteering_angledt]


def calc_required_time(vehicle_speed, steering_angle_rate):

    # Adjust initial state to include front wheel position
    initial_state_with_front_wheel = [0, 0, 0, 0]  # Includes initial x_front and y_front

    # Time span for the simulation
    t_span = [0, 5]  # Arbitrary end time, can be adjuste
    t_eval = np.arange(0, 5, 0.01)

    # Solve the system of differential equations using Runge-Kutta method
    solution_with_front_wheel = solve_ivp(
        kinematic_model_with_front_wheel, t_span, initial_state_with_front_wheel,
        args=(vehicle_speed, steering_angle_rate, wheelbase, rear_to_edge), t_eval=t_eval, dense_output=True)

    # Extract the solution
    t_with_front_wheel = solution_with_front_wheel.t
    x_rear, y_rear, theta, steering_angle = solution_with_front_wheel.y


    # Calculate the front wheel edge position
    x_front = x_rear + rear_to_edge * np.cos(theta + psi)
    y_front = y_rear + rear_to_edge * np.sin(theta + psi) - vehicle_width*0.5

    # Determine the time when the front wheel has moved laterally by distance D
    for i in range(len(t_with_front_wheel)):
        if abs(y_front[i]) >= margin_distance:
            break

    return t_with_front_wheel[i]

to_kmph = 3.6
vehicle_speed_array = np.array([1.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40, 45, 50, 55, 60]) / to_kmph  # constant speed of the vehicle (m/s)
steering_angle_rate_array = np.array([0.002, 0.005, 0.01, 0.015, 0.02, 0.04, 0.06, 0.08, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4])  # rate at which the steering angle changes (rad/s)


result = np.zeros((len(vehicle_speed_array), len(steering_angle_rate_array)))

for i in range(len(vehicle_speed_array)):
    for j in range(len(steering_angle_rate_array)):
        time = calc_required_time(vehicle_speed_array[i], steering_angle_rate_array[j])
        result[i, j] = time

np.set_printoptions(precision=2, floatmode='fixed', suppress=True, linewidth=200)
print(result)
np.savetxt(sys.stdout, result, fmt='%.2f', delimiter=', ')


# Generate 2D plot
plt.figure(figsize=(12, 8))
threshold = 1.5  # Threshold
for (i, j), val in np.ndenumerate(result):
    color = 'red' if val < threshold else 'black'
    plt.text(j, i, f'{val:.2f}', va='center', ha='center', color=color)

# Make a grid plot
plt.imshow(np.flipud(result), cmap='Wistia', interpolation='nearest')
plt.colorbar()
plt.xticks(ticks=np.arange(len(steering_angle_rate_array)), labels=steering_angle_rate_array)
plt.yticks(ticks=np.arange(len(vehicle_speed_array)), labels=np.around(vehicle_speed_array * to_kmph, 1)[::-1])
plt.xlabel('Steering Rate (rad/s)')
plt.ylabel('Vehicle Speed (km/h)')
plt.title('Margin Time [s]')
plt.show()

