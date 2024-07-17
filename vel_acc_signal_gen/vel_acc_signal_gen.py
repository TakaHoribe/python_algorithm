import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

class SystemModel:
    def __init__(self, t_max, dt, v_min, v_max, a_min, a_max, num_bins, dead_zone_threshold):
        self.t_max = t_max  # Maximum simulation time
        self.dt = dt  # Time step size
        self.v_min = v_min  # Minimum velocity
        self.v_max = v_max  # Maximum velocity
        self.a_min = a_min  # Minimum acceleration
        self.a_max = a_max  # Maximum acceleration
        self.num_bins = num_bins  # Number of bins for velocity and acceleration grid
        self.dead_zone_threshold = dead_zone_threshold  # Threshold below which acceleration is considered zero

        self.last_observation_time = 0.0  # Time of the last observation

        self.t = np.arange(0, t_max, dt)  # Time array
        self.v = np.zeros_like(self.t)  # Velocity array
        self.a = np.zeros_like(self.t)  # Acceleration array
        self.counts = np.zeros((num_bins, num_bins))  # Count of observations in each velocity-acceleration grid cell
        self.counts_history = np.zeros((len(self.t), num_bins, num_bins))  # History of counts for each time step

        self.v_bins = np.linspace(v_min, v_max, num_bins + 1)  # Velocity bin edges
        self.least_observed_v_bin = 0  # Index of the velocity bin with the fewest observations
        self.least_observed_v_bin_history = np.zeros(len(self.t))  # History of the least observed velocity bin

        self.a_bins = np.linspace(a_min, a_max, num_bins + 1)  # Acceleration bin edges
        self.least_observed_a_bin = 0  # Index of the acceleration bin with the fewest observations
        self.least_observed_a_bin_history = np.zeros(len(self.t))  # History of the least observed acceleration bin

        # Parameters for generating input acceleration waveforms
        self.frequencies = np.linspace(0.1, 2.0, 5)
        self.amplitudes = np.linspace(0.5, 3.0, 5)
        self.phases = np.linspace(0, 2 * np.pi, 5)

        # Initialize the input acceleration waveform
        self._initialize_acceleration()

    def _initialize_acceleration(self):
        """Generate the initial acceleration waveform."""
        for f in self.frequencies:
            for A in self.amplitudes:
                for phi in self.phases:
                    self.a += A * np.sin(2 * np.pi * f * self.t + phi)
        self.a += 0.05 * np.random.randn(len(self.t))  # Add random noise to the acceleration
        self.a = np.clip(self.a, self.a_min, self.a_max)  # Clip the acceleration to within the specified limits

    def get_v(self, i):
        """Get the velocity at the previous time step."""
        return self.v[i - 1]

    def get_a(self, i):
        """Get the acceleration at the previous time step."""
        return self.a[i - 1]

    def count_observations(self, v, a):
        """Count the number of observations in the velocity-acceleration grid."""
        v_bin = np.digitize(v, self.v_bins) - 1
        a_bin = np.digitize(a, self.a_bins) - 1
        if 0 <= v_bin < self.num_bins and 0 <= a_bin < self.num_bins:
            self.counts[v_bin, a_bin] += 1
        return v_bin, a_bin

    def find_least_observed_region(self):
        """Find the grid cell with the fewest observations."""
        return np.unravel_index(np.argmin(self.counts), self.counts.shape)

    def generate_target_input(self, v, a, v_target, a_target):
        """Generate the feedback component of the input acceleration."""
        Kp_v = 0.2  # Proportional gain for velocity
        Kp_a = 0.0  # Proportional gain for acceleration
        Kp_v_over = 1.0  # Proportional gain for velocity overshoot
        v_error = v_target - v  # Velocity error
        a_error = a_target - a  # Acceleration error
        v_over = min(self.v_max - v, 0.0)  # Velocity overshoot
        return Kp_v * v_error + Kp_a * a_error + Kp_v_over * v_over

    def update_dynamics(self, i):
        """Update the velocity and apply the dead zone to the acceleration."""
        if np.abs(self.a[i]) < self.dead_zone_threshold:
            self.a[i] = 0
        dv = self.a[i] * self.dt  # Change in velocity
        self.v[i] = self.v[i-1] + dv  # Update velocity
        return self.v[i]

    def simulate(self):
        """Run the simulation."""
        for i in range(1, len(self.t)):
            v = self.get_v(i)
            a = self.get_a(i)
            v_bin, a_bin = self.count_observations(v, a)
            self.counts_history[i] = self.counts

            current_time = self.t[i]
            if current_time - self.last_observation_time >= 10:
                self.least_observed_v_bin, self.least_observed_a_bin = self.find_least_observed_region()
                self.last_observation_time = current_time
                print(f"Update least region: {i}")

            self.least_observed_v_bin_history[i] = self.least_observed_v_bin
            self.least_observed_a_bin_history[i] = self.least_observed_a_bin
            v_target = (self.v_bins[self.least_observed_v_bin] + self.v_bins[self.least_observed_v_bin + 1]) / 2
            a_target = (self.a_bins[self.least_observed_a_bin] + self.a_bins[self.least_observed_a_bin + 1]) / 2
            feedback = self.generate_target_input(v, a, v_target, a_target)
            self.a[i] += feedback
            self.a[i] = np.clip(self.a[i], self.a_min, self.a_max)
            self.v[i] = self.update_dynamics(i)

    def plot_results(self):
        """Visualize the results of the simulation."""
        fig, axs = plt.subplots(3, 2, figsize=(20, 24))

        # Time vs velocity
        axs[0, 0].plot(self.t, self.v, label='Velocity (v) over Time (t)')
        axs[0, 0].set_xlabel('Time (t)')
        axs[0, 0].set_ylabel('Velocity (v)')
        axs[0, 0].set_title('Time vs Velocity')
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        # Time vs acceleration
        axs[1, 0].plot(self.t, self.a, label='Acceleration (a) over Time (t)')
        axs[1, 0].set_xlabel('Time (t)')
        axs[1, 0].set_ylabel('Acceleration (a)')
        axs[1, 0].set_title('Time vs Acceleration')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        # Time vs sub-area count (64 lines)
        for j in range(self.num_bins):
            for k in range(self.num_bins):
                axs[2, 0].plot(self.t, self.counts_history[:, j, k], label=f'Area ({j}, {k})')
        axs[2, 0].set_xlabel('Time (t)')
        axs[2, 0].set_ylabel('Observation Count')
        axs[2, 0].set_title('Time vs Measurement Count')
        axs[2, 0].grid(True)
        # axs[2, 0].legend(ncol=4, loc='upper left')

        # Time vs v_bin and a_bin history
        axs[0, 1].plot(self.t, self.least_observed_v_bin_history, label='Least Observed v_bin History')
        axs[0, 1].plot(self.t, self.least_observed_a_bin_history, label='Least Observed a_bin History')
        axs[0, 1].set_xlabel('Time (t)')
        axs[0, 1].set_ylabel('Bin Index')
        axs[0, 1].set_title('Time vs v_bin and a_bin History')
        axs[0, 1].grid(True)
        axs[0, 1].legend()

        # Velocity vs acceleration (scatter plot)
        axs[1, 1].scatter(self.a, self.v, label='Trajectory in (v, a) Space')
        axs[1, 1].set_xlabel('Acceleration (a)')
        axs[1, 1].set_ylabel('Velocity (v)')
        axs[1, 1].set_title('Velocity vs Acceleration (Scatter)')
        axs[1, 1].grid(True)
        axs[1, 1].legend()
        axs[1, 1].set_xlim([self.a_min, self.a_max])
        axs[1, 1].set_ylim([self.v_min, self.v_max])

        # Show the counts of observations in each grid cell
        sns.heatmap(self.counts, annot=True, cmap='coolwarm', xticklabels=np.round(self.a_bins, 2), yticklabels=np.round(self.v_bins, 2))
        axs[2, 1].set_xlabel('Acceleration bins')
        axs[2, 1].set_ylabel('Velocity bins')
        axs[2, 1].set_title('Counts of Observations in Each Grid Cell')

        # plt.tight_layout()
        plt.show()

# Parameters
t_max = 500  # Maximum simulation time
dt = 0.01  # Time step size
v_min, v_max = 0, 20  # Velocity range
a_min, a_max = -3, 3  # Acceleration range
num_bins = 7  # Number of bins for velocity and acceleration
dead_zone_threshold = 0.1  # Threshold for dead zone in acceleration

# Create a system model
system = SystemModel(t_max, dt, v_min, v_max, a_min, a_max, num_bins, dead_zone_threshold)

# Run the simulation
system.simulate()

# Plot the results
system.plot_results()
