import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def generate_figure_8_coordinates(a=1, num_points=1000):
    t = np.linspace(-np.pi, np.pi, num_points)
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    return x, y

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

def plot_XYV_points(x_points, y_points, vx, vy, title, save_path=None):
    vel = np.sqrt(np.array(vx) ** 2 + np.array(vy) ** 2)

    plt.figure(figsize=(6, 6))
    scatter = plt.scatter(x_points, y_points, c=vel, cmap='viridis', s=50)    
    plt.colorbar(scatter, label='Velocity Magnitude')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title(title)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    if save_path:
        plt.savefig(save_path, format='png', dpi=300, bbox_inches='tight')
    
    plt.show()


def generate_velocity_FD(x_points, y_points, dt):
    vx = np.diff(x_points) / dt
    vy = np.diff(y_points) / dt

    vx = np.append(vx, vx[-1])
    vy = np.append(vy, vy[-1])

    # num_ramp = 100
    # ramp = np.linspace(0, 1, num_ramp)

    # vx[:num_ramp] = ramp * vx[num_ramp - 1]
    # vy[:num_ramp] = ramp * vy[num_ramp - 1]

    return vx, vy

def compute_curvature_three_points(x1, y1, x2, y2, x3, y3):
    a = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    b = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
    c = np.sqrt((x3 - x1)**2 + (y3 - y1)**2)

    # Heron's formula
    s = (a + b + c) / 2
    area = np.sqrt(max(s * (s - a) * (s - b) * (s - c), 0))

    # Circumradius of the triangle
    if area < 1e-10 or a < 1e-10 or b < 1e-10 or c < 1e-10:
        return 0.0
        
    R = (a * b * c) / (4 * area + 1e-6)
    curvature = 1 / R 
    return curvature

def generate_velocity_curve(x_points, y_points, dt):
    vx, vy, curvatures = [], [], [0, 0]

    for i in range(2, len(x_points) - 2):
        curvature = compute_curvature_three_points(
            x_points[i - 2], y_points[i - 2],
            x_points[i], y_points[i],
            x_points[i + 2], y_points[i + 2]
        )
        curvatures.append(curvature)

    for idx in range(1, len(curvatures)):
        if curvatures[idx] < 1e-2:
            curvatures[idx] = curvatures[idx - 1]

    for idx in range(2, -1, -1):
        curvatures[idx] = curvatures[idx + 2] 

    curvatures = np.array(curvatures)
    velocities = 1 / (curvatures + 1e-6)
    velocities = (velocities - np.min(velocities)) / (np.max(velocities) - np.min(velocities) + 1e-6) * 0.01

    plt.figure(figsize=(8, 4))
    plt.plot(curvatures, label="Curvature", color="blue")
    plt.title("Curvature Values Along the Trajectory")
    plt.xlabel("Point Index")
    plt.ylabel("Curvature")
    plt.grid(True)
    plt.legend()
    

    plt.savefig(r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots_xref\f8_curve.png", format='png', dpi=300, bbox_inches='tight')

    plt.show()

    for i in range(1, len(x_points) - 1):
        vel = velocities[i - 1]
        direction = np.array([
            (x_points[i + 1] - x_points[i - 1]) / (2 * dt),
            (y_points[i + 1] - y_points[i - 1]) / (2 * dt)
        ])
        direction /= np.linalg.norm(direction) + 1e-6

        vx.append(vel * direction[0])
        vy.append(vel * direction[1])

    vx = [vx[0]] + vx + [vx[-1]]
    vy = [vy[0]] + vy + [vy[-1]]
    return vx, vy

def moving_average(data, window_size=10):
    half_window = window_size // 2
    padded_data = np.pad(data, pad_width=half_window, mode='edge')
    smoothed_data = np.convolve(padded_data, np.ones(window_size)/window_size, mode='valid')
    return smoothed_data

def smooth_trajectory(x_points, y_points, vx, vy, window_size=5):
    x_points_smooth = moving_average(x_points, window_size)
    y_points_smooth = moving_average(y_points, window_size)
    vx_smooth = moving_average(vx, window_size)
    vy_smooth = moving_average(vy, window_size)
    return x_points_smooth, y_points_smooth, vx_smooth, vy_smooth

def create_ref(x_points, y_points, vx, vy, filename):
    Xref = np.zeros((len(x_points),12))
    Xref[:,0] = x_points
    Xref[:,1] = y_points
    Xref[:,2] = np.zeros((1, len(x_points)))
    Xref[:,3] = vx
    Xref[:,4] = vy

    df = pd.DataFrame({"x": Xref[:,0], "y": Xref[:,1], "z": Xref[:,2], 
                       "u": Xref[:,3], "v": Xref[:,4], "w": Xref[:,5], 
                       "phi": Xref[:,6], "theta": Xref[:,7], "psi": Xref[:,8],
                       "p": Xref[:,9], "q": Xref[:,10], "r": Xref[:,11]})
    df.to_csv(filename, index=False)
    print(f"Coordinates saved to {filename}")

    return Xref

x, y = generate_figure_8_coordinates()

x_norm, y_norm = normalize_pts(x, y)
vx_fd, vy_fd = generate_velocity_FD(x_norm, y_norm, 0.1)
plot_XYV_points(x_norm, y_norm, vx_fd, vy_fd, "Trajectory with Forward Difference Velocity", 
                r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots_xref\f8_FD_newstart.png")
create_ref(x_norm, y_norm, vx_fd, vy_fd, "f8_FD_newstart.csv")

vx_curve, vy_curve = generate_velocity_curve(x_norm, y_norm, 0.1)
x_smooth, y_smooth, vx_smooth, vy_smooth = smooth_trajectory(x_norm, y_norm, vx_curve, vy_curve)
plot_XYV_points(x_smooth, y_smooth, vx_smooth, vy_smooth, "Trajectory with Inverse Curvature Velocity", 
                r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots_xref\f8_curve_newstart.png")
# create_ref(x_smooth, y_smooth, vx_smooth, vy_smooth, "f8_newstart.csv")