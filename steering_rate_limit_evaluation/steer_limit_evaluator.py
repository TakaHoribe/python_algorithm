
import sys
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parameters (can be adjusted as needed)
wheelbase = 4.0  
margin_latacc_threshold = 1.0

# constant speed of the vehicle (m/s)
MPS_TO_KMPH = 3.6
vehicle_speed_array_kmph = np.array([1.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40, 45, 50, 55, 60])
vehicle_speed_array = vehicle_speed_array_kmph / MPS_TO_KMPH

# evaluated steering angle (rad)
steering_angle_array = np.array([0.002, 0.004, 0.006, 0.008, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 
                                 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.3,
                                 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65])  

# Update the kinematic model to calculate the front wheel edge position
def calc_lateral_acceleration(speed, steering, wheelbase):
    lat_acc = speed * speed * np.tan(steering) / wheelbase
    return lat_acc

def generate_2d_plot(result, vehicle_speed_array_kmph, steering_angle_array, margin_latacc_threshold):
    result_flipud = np.flipud(result)
    plt.figure(figsize=(22, 8))
    for (i, j), val in np.ndenumerate(result_flipud):
        color = 'red' if val > margin_latacc_threshold else 'black'
        plt.text(j, i, f'{val:.2f}', va='center', ha='center', color=color)

    # Make a grid plot
    plt.imshow(result_flipud, cmap='Wistia', interpolation='nearest')
    plt.colorbar()
    plt.xticks(ticks=np.arange(len(steering_angle_array)), labels=steering_angle_array)
    plt.yticks(ticks=np.arange(len(vehicle_speed_array)), labels=np.around(vehicle_speed_array_kmph, 1)[::-1])
    plt.xlabel('Steering (rad)')
    plt.ylabel('Vehicle Speed (km/h)')
    plt.title('Lateral Acceleration [m/ss]')
    plt.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1)
    plt.show()

result = np.zeros((len(vehicle_speed_array), len(steering_angle_array)))

# Main process to calculate margin time
for i in range(len(vehicle_speed_array)):
    for j in range(len(steering_angle_array)):
        result[i, j] = calc_lateral_acceleration(vehicle_speed_array[i], steering_angle_array[j], wheelbase)
        print(vehicle_speed_array[i], steering_angle_array[j], wheelbase, result[i, j])

np.set_printoptions(precision=2, floatmode='fixed', suppress=True, linewidth=200)
print(result)
np.savetxt(sys.stdout, result, fmt='%.2f', delimiter=', ')

# Generate 2D plot
generate_2d_plot(result, vehicle_speed_array_kmph, steering_angle_array, margin_latacc_threshold)