import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

filename = "pure_xy/cloud_input.csv"

def normalize_pts(x_points, y_points, z_points, max_x = .25, max_y = .25):
    x_min = np.min(x_points)
    x_max = np.max(x_points)
    y_min = np.min(y_points)
    y_max = np.max(y_points)
    z_min = np.min(z_points)
    z_max = np.max(z_points)

    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min
    scale = max(x_range, y_range, z_range)

    x_points = (x_points - x_min) / scale * max_x
    y_points = (y_points - y_min) / scale * max_y
    z_points = (z_points - z_min) / (scale * 10)

    return x_points, y_points, z_points

def plot_3D_trajectory(x_points, y_points, z_points, velocities, title, save_path=None):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(x_points, y_points, z_points, c=velocities, cmap='viridis', s=50)
    fig.colorbar(scatter, ax=ax, label='Velocity Magnitude')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    if save_path:
        plt.savefig(save_path, format='png', dpi=300, bbox_inches='tight')
    plt.show()

def generate_velocity_FD(x_points, y_points, z_points, dt):
    vx, vy, vz = [], [], []
    for i in range(len(x_points) - 1):
        vx.append((x_points[i + 1] - x_points[i]) / dt)
        vy.append((y_points[i + 1] - y_points[i]) / dt)
        vz.append((z_points[i + 1] - z_points[i]) / dt)
    vx.append(vx[-1])
    vy.append(vy[-1])
    vz.append(vz[-1])
    return vx, vy, vz

def compute_curvature_three_points_3D(x1, y1, z1, x2, y2, z2, x3, y3, z3):
    a = np.linalg.norm([x2 - x1, y2 - y1, z2 - z1])
    b = np.linalg.norm([x3 - x2, y3 - y2, z3 - z2])
    c = np.linalg.norm([x3 - x1, y3 - y1, z3 - z1])

    s = (a + b + c) / 2
    area = np.sqrt(s * (s - a) * (s - b) * (s - c))

    if area <= 1e-10:
        return 0.0

    R = (a * b * c) / (4 * area + 1e-6)
    curvature = 1 / R 
    return curvature

def find_longest_zero_section(curvature_list):
    longest_start = -1
    longest_end = -1
    max_length = 0

    current_start = -1

    for i, value in enumerate(curvature_list):
        if value == 0:
            if current_start == -1: 
                current_start = i
        else:
            if current_start != -1: 
                current_length = i - current_start
                if current_length > max_length:
                    max_length = current_length
                    longest_start = current_start
                    longest_end = i - 1
                current_start = -1

    if current_start != -1:
        current_length = len(curvature_list) - current_start
        if current_length > max_length:
            longest_start = current_start
            longest_end = len(curvature_list) - 1

    return longest_start, longest_end


def generate_velocity_curve_3D(x_points, y_points, z_points, dt):
    vx, vy, vz = [], [], []
    vel_init_list = []
    curvature_list = []

    for i in range(1, len(x_points) - 1):
        curvature = compute_curvature_three_points_3D(
            x_points[i - 1], y_points[i - 1], z_points[i - 1],
            x_points[i], y_points[i], z_points[i],
            x_points[i + 1], y_points[i + 1], z_points[i + 1]
        )
        curvature_list.append(curvature)   
        vel_init = 1 / (curvature + 1e-6)
        vel_init_list.append(vel_init)   
    
    longest_start, longest_end = find_longest_zero_section(curvature_list)
    longest_length = longest_end - longest_start
    t = np.linspace(0, 2 * np.pi, longest_length)
    vel_init_list[longest_start:longest_end] = np.zeros(longest_length,)

    mean_vel = np.mean(vel_init_list)
    threshold = 3 * mean_vel

    for idx in range(1, len(vel_init_list)):  # Start from 1 to ensure idx-1 exists
        if vel_init_list[idx] > threshold:
            print(f"Point index: {i}, x: {x_points[i]}, y: {y_points[i]}, z: {z_points[i]}, curvature: {curvature}")
            vel_init_list[idx] = vel_init_list[idx - 1]
    
    vel_min = min(vel_init_list)
    vel_max = max(vel_init_list)
    vel_init_list = [(0.5 + 0.5 * (vel - vel_min)) / (vel_max - vel_min + 1e-6) for vel in vel_init_list]

    vel_init_list[longest_start:longest_end] = vel_max * np.sin(t / 2) + vel_init_list[longest_start - 1]

    vel_min = min(vel_init_list)
    vel_max = max(vel_init_list)
    vel_init_list = [0.5 * (vel - vel_min) / (vel_max - vel_min + 1e-6) for vel in vel_init_list]

    for i in range(1, len(x_points) - 1):
       
        vel = vel_init_list[i - 1]
        direction = np.array([
            (x_points[i + 1] - x_points[i - 1]) / (2 * dt),
            (y_points[i + 1] - y_points[i - 1]) / (2 * dt),
            (z_points[i + 1] - z_points[i - 1]) / (2 * dt)
        ])
        norm = np.linalg.norm(direction)
        direction /= (norm + 1e-6)

        vx.append(vel * direction[0])
        vy.append(vel * direction[1])
        vz.append(vel * direction[2])

    vx = [vx[0]] + vx + [vx[-1]]
    vy = [vy[0]] + vy + [vy[-1]]
    vz = [vz[0]] + vz + [vz[-1]]

    return vx, vy, vz

def moving_average(data, window_size=10):
    half_window = window_size // 2
    padded_data = np.pad(data, pad_width=half_window, mode='edge')
    smoothed_data = np.convolve(padded_data, np.ones(window_size)/window_size, mode='valid')
    return smoothed_data

def smooth_trajectory(x_points, y_points, z_points, vx, vy, vz, window_size=5):
    x_points_smooth = moving_average(x_points, window_size)
    y_points_smooth = moving_average(y_points, window_size)
    z_points_smooth = moving_average(z_points, window_size)
    vx_smooth = moving_average(vx, window_size)
    vy_smooth = moving_average(vy, window_size)
    vz_smooth = moving_average(vz, window_size)
    return x_points_smooth, y_points_smooth, z_points_smooth, vx_smooth, vy_smooth, vz_smooth

def create_ref(x_points, y_points, z_points, vx, vy, vz, filename):
    Xref = np.zeros((len(x_points),12))
    Xref[:, 0] = x_points
    Xref[:, 1] = y_points
    Xref[:, 2] = z_points
    Xref[:, 3] = vx
    Xref[:, 4] = vy
    Xref[:, 5] = vz

    df = pd.DataFrame({"x": Xref[:,0], "y": Xref[:,1], "z": Xref[:,2], 
                       "u": Xref[:,3], "v": Xref[:,4], "w": Xref[:,5], 
                       "phi": Xref[:,6], "theta": Xref[:,7], "psi": Xref[:,8],
                       "p": Xref[:,9], "q": Xref[:,10], "r": Xref[:,11]})
    df.to_csv(filename, index=False)
    print(f"Coordinates saved to {filename}")

    return Xref


data = pd.read_csv(filename, delimiter=",", header=0)
x = data.values[:,0]
# x = np.append(x, data.values[0,0])
y = data.values[:,1]
# y = np.append(y, data.values[0,1])
y *= -1

t = np.linspace(0, 2 * np.pi, len(x))
z = np.zeros_like(x)

x_norm, y_norm, z_norm = normalize_pts(x, y, z)

vx, vy, vz = generate_velocity_FD(x_norm, y_norm, z_norm, 0.1)
x_avg, y_avg, z_avg, vx, vy, vz = smooth_trajectory(x_norm, y_norm, z_norm, vx, vy, vz, window_size=7)
vel_mag = np.sqrt(np.array(vx)**2 + np.array(vy)**2 + np.array(vz)**2)
plot_3D_trajectory(x_avg, y_avg, z_avg, vel_mag, "Trajectory with Forward Difference Velocity", "cloud_FD_3D.png")
create_ref(x_avg, y_avg, z_avg, vx, vy, vz, "cloud_FD.csv")

vx, vy, vz = generate_velocity_curve_3D(x_avg, y_avg, z_avg, 0.1)
# x_avg, y_avg, z_avg, vx, vy, vz = smooth_trajectory(x_norm, y_norm, z_norm, vx, vy, vz, window_size=5)
vel_mag = np.sqrt(np.array(vx)**2 + np.array(vy)**2 + np.array(vz)**2)
plot_3D_trajectory(x_avg, y_avg, z_avg, vel_mag, "Trajectory with Inverse Curvature Velocity", "cloud_curve_3D.png")
create_ref(x_avg, y_avg, z_avg, vx, vy, vz, "cloud_curve.csv")
