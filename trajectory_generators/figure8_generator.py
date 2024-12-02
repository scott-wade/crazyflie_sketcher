import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def generate_figure_8_coordinates(a=1, num_points=1000):
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    return x, y

def normalize_pts(x_points, y_points):
    x_min = np.min(x_points)
    x_max = np.max(x_points)
    y_min = np.min(y_points)
    y_max = np.max(y_points)

    x_range = x_max - x_min
    y_range = y_max - y_min
    scale = max(x_range, y_range)

    x_points = (x_points - x_min) / scale
    y_points = (y_points - y_min) / scale

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
    vx = []
    vy = []
    for i in range(len(x_points)-1):

        vx.append((x_points[i + 1] - x_points[i]) / dt)
        vy.append((y_points[i + 1] - y_points[i]) / dt)

    vx.append(vx[-1])
    vy.append(vy[-1])
    
    return vx, vy

def compute_curvature_three_points(x1, y1, x2, y2, x3, y3):
    a = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    b = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
    c = np.sqrt((x3 - x1)**2 + (y3 - y1)**2)

    # Heron's formula
    s = (a + b + c) / 2
    area = np.sqrt(s * (s - a) * (s - b) * (s - c))

    # Circumradius of the triangle
    if area == 0:
        return np.inf
        
    R = (a * b * c) / (4 * area)

    curvature = 1 / R
    return curvature

def generate_velocity_curve(x_points, y_points, dt):
    vx = []
    vy = []
    
    for i in range(1, len(x_points) - 1):
        curvature = compute_curvature_three_points(
            x_points[i - 1], y_points[i - 1],
            x_points[i], y_points[i],
            x_points[i + 1], y_points[i + 1]
        )

        vel = 1 / (curvature + 1e-6) 

        vel = np.clip(vel, 0.5, 2.0)

        direction_x = (x_points[i + 1] - x_points[i - 1]) / (2 * dt)
        direction_y = (y_points[i + 1] - y_points[i - 1]) / (2 * dt)
        norm = np.sqrt(direction_x**2 + direction_y**2)
        dir_x = direction_x / (norm + 1e-6)
        dir_y = direction_y / (norm + 1e-6)

        vx.append(vel * dir_x)
        vy.append(vel * dir_y)
    
    vx = [vx[0]] + vx + [vx[-1]]
    vy = [vy[0]] + vy + [vy[-1]]

    return vx, vy

def create_ref(x_points, y_points, vx, vy, filename):
    Xref = np.zeros((len(x_points),12))
    Xref[:,0] = x_points
    Xref[:,1] = y_points
    Xref[:,2] = 0.5 * np.ones((1, len(x_points)))
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
vx, vy = generate_velocity_FD(x_norm, y_norm, 0.1)
plot_XYV_points(x_norm, y_norm, vx, vy, "Trajectory with Forward Difference Velocity", 
                r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots\f8_FD.png")
create_ref(x_norm, y_norm, vx, vy, "f8_FD.csv")

vx, vy = generate_velocity_curve(x_norm, y_norm, 0.1)
plot_XYV_points(x_norm, y_norm, vx, vy, "Trajectory with Inverse Curvature Velocity", 
                r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\output_plots\f8_curve.png")
create_ref(x_norm, y_norm, vx, vy, "f8_curve.csv")