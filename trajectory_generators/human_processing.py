import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

filename = "pure_xy/human.csv"

def normalize_pts(x_points, y_points, z_points, scale_x = 0.15, scale_y = 0.15):
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

    x_points = (x_points - x_min) / scale * scale_x
    y_points = (y_points - y_min) / scale * scale_y

    return x_points, y_points, z_points

def plot_XY_points(x_points, y_points, title, save_path=None):
    plt.figure(figsize=(6, 6))
    plt.plot(x_points, y_points, 'o-', markersize=5)
    plt.plot(x_points[0], y_points[0], 'go', markersize=5)
    plt.plot(x_points[-2], y_points[-2], 'ro', markersize=5)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title(title)
    plt.legend(['Path', 'Start', 'End'])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)

    if save_path:
        plt.savefig(save_path, format='png', dpi=300, bbox_inches='tight')
    
    plt.show()

def circle_gen(x_points, y_points, z_points):
    n_straight = 200  # Number of points for the straight path
    n_circle = 400    # Number of points for the circular path

    # Generate straight-line path
    x_straight = np.linspace(x_points[-1], 0.025, n_straight)
    y_straight = y_points[-1] * np.ones_like(x_straight)
    t = np.linspace(0, np.pi, n_straight)
    z_straight = np.sin(t)

    x_points = np.hstack((x_points, x_straight))
    y_points = np.hstack((y_points, y_straight))
    z_points = np.hstack((z_points, z_straight))

    # Generate circular path
    theta = np.linspace(0, 2 * np.pi, n_circle)  
    radius = 0.015

    x_circle = radius * np.cos(theta) + 0.04
    y_circle = radius * np.sin(theta) + y_points[-1]
    z_circle = np.zeros_like(theta) 

    x_points = np.hstack((x_points, x_circle))
    y_points = np.hstack((y_points, y_circle))
    z_points = np.hstack((z_points, z_circle))

    # x_points = x_points.reshape(-1, 1)
    # y_points = y_points.reshape(-1, 1)
    # z_points = z_points.reshape(-1, 1)

    fig = plt.figure(figsize=(6, 6))
    ax = plt.axes(projection='3d')

    ax.scatter3D(x_points, y_points, z_points, c=np.arange(len(x_points)), cmap='Greens')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory Positions")

    plt.savefig(r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\pure_xy\human_plot_3D.png", format='png', dpi=300, bbox_inches='tight')

    plt.show()

    return x_points, y_points, z_points

data = pd.read_csv(filename, delimiter=",", header=0)
x = data.values[:,0]
# x = np.append(x, data.values[0,0])
y = data.values[:,1]
# y = np.append(y, data.values[0,1])
y *= -1

t = np.linspace(0, 2 * np.pi, len(x))
z = np.zeros_like(x)

x_norm, y_norm, z_norm = normalize_pts(x, y, z)

plot_XY_points(x_norm, y_norm, "Input Points", r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\pure_xy\human_plot.png")

x_norm, y_norm, z_norm = circle_gen(x_norm, y_norm, z_norm)

x_norm, y_norm, z_norm = normalize_pts(x_norm, y_norm, z_norm, 1, 1)

plot_XY_points(x_norm, y_norm, "Input Points", r"C:\Users\daesc\OneDrive\Desktop\F24\ACSI\Project\crazyflie_sketcher\trajectory_generators\pure_xy\human_plot.png")

Xref = np.zeros((len(x_norm),12))
Xref[:, 0] = x_norm
Xref[:, 1] = y_norm
Xref[:, 2] = z_norm

df = pd.DataFrame({"x": Xref[:,0], "y": Xref[:,1], "z": Xref[:,2], 
                       "u": Xref[:,3], "v": Xref[:,4], "w": Xref[:,5], 
                       "phi": Xref[:,6], "theta": Xref[:,7], "psi": Xref[:,8],
                       "p": Xref[:,9], "q": Xref[:,10], "r": Xref[:,11]})

df.to_csv("pure_xy/human_head.csv", index=False)
print(f"Coordinates saved to {filename}")