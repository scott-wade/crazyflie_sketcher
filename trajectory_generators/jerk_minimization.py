import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize

data = pd.read_csv("heart_curve.csv", delimiter=",", header=1)
x_points = data.values[:,0]
x_points = np.append(x_points, data.values[0,0])
y_points = data.values[:,1]
y_points = np.append(y_points, data.values[0,1])

y_rotated = -y_points

num_spline_pts = 200

tck, u = splprep([x_points, y_rotated], s=0)  # tck is the spline representation
x_dense, y_dense = splev(np.linspace(0, 1, num_spline_pts), tck)  # Dense spline evaluation

# jerk minimization
params = {
            "mass": 0.027,          # Mass of the Crazyflie in kilograms
            "J": np.diag([1.43e-5, 1.43e-5, 2.89e-5]),  # Moment of inertia (kg·m²) for x, y, z axes
            "gravity": 9.81,        # Gravitational acceleration (m/s²)
            "L": 0.046,             # Arm length (meters)
            "kf": 1.28192e-8,       # Thrust coefficient (N·s²/rad²)
            "km": 5.964552e-3,      # Torque coefficient (N·m·s²/rad²)
        }

# Define initial and final states [p_x, p_y, p_z, v_x, v_y, v_z, q0, q1, q2, q3, omega_x, omega_y, omega_z]
initial_state = np.array([x_points[0], y_points[0], 1, 1, 3, 1, 0, 0, 0, 0, 0, 0])
final_state = np.array([x_points[-1], y_points[-1], 1, 1, 3, 1, 0, 0, 0, 0, 0, 0])


traj_pos = []
traj_vel = []

dt = 1
ddt = dt / 10

# Let's assume equal timesteps of 0.1s between all points 
# Calculate incrememts of 0.01 between waypoints as straight line segments
for i in range(len(x_points)-1):
    for tau in np.arange(0, dt + ddt, ddt):
        pos_poly = 6 * tau ** 5 - 15 * tau ** 4 + 10 * tau ** 3
        vel_poly = 30 * tau ** 4 - 60 * tau ** 3 + 30 * tau ** 2 # only if we wanted to go to zero

        pos = [x_points[i] + (x_points[i + 1] - x_points[i]) * pos_poly, 
               y_points[i] + (y_points[i + 1] - y_points[i]) * pos_poly]

        vel = [(x_points[i + 1] - x_points[i]) * vel_poly / 0.1,
               (y_points[i + 1] - y_points[i]) * vel_poly / 0.1]

        traj_pos.append(pos.copy())
        traj_vel.append(vel.copy())

traj_pos = np.array(traj_pos)
traj_vel = np.array(traj_vel)

t = np.arange(0, 0.01 * (traj_pos.shape[0]), 0.01)

fig, axs = plt.subplots(4)
fig.tight_layout()
axs[0].plot(x_points, y_points, 'ro')
axs[0].set_title('Input Points')
axs[1].plot(traj_pos[:,0], traj_pos[:,1], '-o')
axs[1].plot(x_points, y_points, 'ro')
axs[1].set_title('Jerk Minimized Points')
axs[1].set_xlabel('World Coordinates (m)')

axs[2].plot(t, traj_vel[:, 0], label = 'Jerk Vel X')
axs[2].set_title('Jerk Minimized X Velocity')
axs[3].plot(t, traj_vel[:, 1], label = 'Jerk Vel Y')
axs[3].set_title('Jerk Minimized Y Velocity')
axs[3].set_xlabel('Velocity (m/s)')

plt.show()