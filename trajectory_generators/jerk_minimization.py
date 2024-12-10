import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import splprep, splev
from MPC import rk4, quadrotor_dynamics
from scipy.optimize import minimize

data = pd.read_csv("circle_traj_20pts_xyz.csv", delimiter=",", header=1)

def find_init_point(array):
    array = np.asarray(array)
    idx = np.abs(array).argmin()
    return array[idx], idx

def normalize_pts(x_points, y_points, max_x = .2, max_y = .2):
    x_min = np.min(x_points)
    x_max = np.max(x_points)
    y_min = np.min(y_points)
    y_max = np.max(y_points)

    x_range = x_max - x_min
    y_range = y_max - y_min
    scale = max(x_range, y_range)

    x_points = (x_points - x_min) / scale * max_x
    y_points = (y_points - y_min) / scale * max_y

    return x_points, y_points

def create_ref(x_points, y_points, z_points, vx, vy, vz, filename):
    Xref = np.zeros((len(x_points),12))
    Xref[:,0] = x_points
    Xref[:,1] = y_points
    Xref[:,2] = z_points
    Xref[:,3] = vx
    Xref[:,4] = vy
    Xref[:,5] = vz

    df = pd.DataFrame({"x": Xref[:,0], "y": Xref[:,1], "z": Xref[:,2], 
                       "u": Xref[:,3], "v": Xref[:,4], "w": Xref[:,5], 
                       "phi": Xref[:,6], "theta": Xref[:,7], "psi": Xref[:,8],
                       "p": Xref[:,9], "q": Xref[:,10], "r": Xref[:,11]})
    df.to_csv(filename, index=False)
    print(f"Coordinates saved to {filename}")

    return Xref

x_points_norm, y_points_norm = normalize_pts(data.values[:, 0], data.values[:, 1])

smallest_x, x_ind = find_init_point(x_points_norm)
smallest_y, y_ind = find_init_point(y_points_norm)

if smallest_x < smallest_y:
    x_points = np.concatenate(([0], [.1], x_points_norm[x_ind:], x_points_norm[:x_ind], [x_points_norm[x_ind]]))
    y_points = np.concatenate(([0], [y_points_norm[y_ind] / 2], y_points_norm[x_ind:], y_points_norm[:x_ind], [y_points_norm[x_ind]]))
else:
    x_points = np.concatenate(([0], [.1], x_points_norm[y_ind:], x_points_norm[:y_ind], [x_points_norm[y_ind]]))
    y_points = np.concatenate(([0], [y_points_norm[y_ind] / 2], y_points_norm[y_ind:], y_points_norm[:y_ind], [y_points_norm[y_ind]]))

z_points = -0.175 * np.ones_like(x_points)
z_points[0] = 0
z_points[1] = -0.16

y_rotated = -y_points

num_spline_pts = 200

tck, u = splprep([x_points, y_rotated], s=0)  # tck is the spline representation
x_dense, y_dense = splev(np.linspace(0, .1, num_spline_pts), tck)  # Dense spline evaluation

traj_pos = []
traj_vel = []

dt = 1
ddt = dt / 10  

traj_pos = []  
traj_vel = []  
total_time = 0  

for i in range(len(x_points) - 1):
    x1 = np.array([x_points[i], y_points[i], z_points[i]])  # Start point
    x2 = np.array([x_points[i+1], y_points[i+1], z_points[i+1]])  # End point

    t1 = 0
    t2 = dt

    # Coefficient matrix for quintic polynomial
    A = np.array([
        [1, t1, t1**2, t1**3, t1**4, t1**5],
        [0, 1, 2*t1, 3*t1**2, 4*t1**3, 5*t1**4],
        [0, 0, 2, 6*t1, 12*t1**2, 20*t1**3],
        [1, t2, t2**2, t2**3, t2**4, t2**5],
        [0, 1, 2*t2, 3*t2**2, 4*t2**3, 5*t2**4],
        [0, 0, 2, 6*t2, 12*t2**2, 20*t2**3],
    ])

    # Boundary conditions for position, velocity, and acceleration
    b_x = [x1[0], 0, 0, x2[0], 0, 0]  
    b_y = [x1[1], 0, 0, x2[1], 0, 0]  
    b_z = [x1[2], 0, 0, x2[2], 0, 0]  

    # Solve for coefficients
    c_x = np.linalg.solve(A, b_x)
    c_y = np.linalg.solve(A, b_y)
    c_z = np.linalg.solve(A, b_z)

    # Generate trajectory points for this segment
    time_segment = np.arange(t1, t2, ddt)
    for t in time_segment:
        # Evaluate position
        pos = [
            np.polyval(c_x[::-1], t),
            np.polyval(c_y[::-1], t),
            np.polyval(c_z[::-1], t),
        ]

        # Evaluate velocity (first derivative of position)
        vel = [
            np.polyval(np.polyder(c_x[::-1]), t),
            np.polyval(np.polyder(c_y[::-1]), t),
            np.polyval(np.polyder(c_z[::-1]), t),
        ]

        traj_pos.append(pos)
        traj_vel.append(vel)

    total_time += dt  # Update total time

traj_pos = np.array(traj_pos)
traj_vel = np.array(traj_vel)

# Correctly define time array
t = np.linspace(0, total_time, len(traj_pos))

# Save the trajectory to a CSV file
create_ref(traj_pos[:, 0], traj_pos[:, 1], traj_pos[:, 2], traj_vel[:, 0], traj_vel[:, 1], traj_vel[:, 2], "f8_curve_20_jerkmin.csv")

# Figure 1: 3D plot of jerk-minimized points
fig1 = plt.figure(figsize=(10, 10))
ax2 = fig1.add_subplot(111, projection='3d')
ax2.plot(traj_pos[:, 0], traj_pos[:, 1], traj_pos[:, 2], '-o', label='Jerk Minimized Trajectory')
ax2.scatter(x_points, y_points, z_points, c='r', label='Input Points')
ax2.set_title('Jerk Minimized Points (3D)')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_zlabel('Z (m)')
ax2.legend()
fig1.tight_layout()

fig1.savefig(r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots_jerkmin\f8_curve_20_jerkmin_3d.png", format='png', dpi=300, bbox_inches='tight')
plt.show()

# Figure 2: 2D plots for velocities
fig2 = plt.figure(figsize=(10, 15))

# X velocity
ax3 = fig2.add_subplot(311)
ax3.plot(t, traj_vel[:, 0], label='Jerk Vel X')
ax3.set_title('Jerk Minimized X Velocity')
ax3.set_ylabel('Velocity (m/s)')

# Y velocity
ax4 = fig2.add_subplot(312)
ax4.plot(t, traj_vel[:, 1], label='Jerk Vel Y')
ax4.set_title('Jerk Minimized Y Velocity')
ax4.set_ylabel('Velocity (m/s)')

# Z velocity
ax5 = fig2.add_subplot(313)
ax5.plot(t, traj_vel[:, 2], label='Jerk Vel Z')
ax5.set_title('Jerk Minimized Z Velocity')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Velocity (m/s)')

fig2.tight_layout()

fig2.savefig(r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots_jerkmin\f8_curve_20_jerkmin_2d.png", format='png', dpi=300, bbox_inches='tight')
plt.show()