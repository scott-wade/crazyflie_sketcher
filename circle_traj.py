import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def create_reference(N, R, dt):
    # Generate position and zero placeholders for velocity and other states
    Xref = [np.concatenate(([R * np.cos(t), R * np.sin(t), 1], np.zeros(9))) 
            for t in np.linspace(-np.pi/2, 3 * np.pi/2, N)]
    
    # Convert list of arrays to a 2D numpy array for easier manipulation
    Xref = np.array(Xref)
    
    # Calculate velocity using finite differences
    for i in range(N - 1):
        Xref[i, 3:6] = (Xref[i + 1, 0:3] - Xref[i, 0:3]) / dt
    Xref[N - 1, 3:6] = Xref[N - 2, 3:6]
    
    # Generate control input reference
    # Uref = np.full((N - 1, 4), 9.81 * 0.5 / 4)
    
    return Xref

def plot_points(xdata, ydata, zdata):
    fig = plt.figure(figsize=(6,6))
    ax = plt.axes(projection='3d')

    ax.scatter3D(xdata, ydata, zdata, c=np.arange(0, len(xdata)), cmap='Greens')

    # Corrected labeling methods
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory Positions")

    ax.set_aspect('auto', adjustable='box')
    ax.grid(True)
    plt.show()


def save_xref_to_csv(data, filename="circle_traj.csv"):
    # Create a DataFrame and save to CSV
    df = pd.DataFrame({"x": data[:,0], "y": data[:,1], "z": data[:,2], 
                       "u": data[:,3], "v": data[:,4], "w": data[:,5], 
                       "phi": data[:,6], "theta": data[:,7], "psi": data[:,8],
                       "p": data[:,9], "q": data[:,10], "r": data[:,11]})
    df.to_csv(filename, index=False)
    print(f"Coordinates saved to {filename}")

def save_xyz_to_csv(data, filename="circle_traj_xyz.csv"):
    # Create a DataFrame and save to CSV
    df = pd.DataFrame({"x": data[:,0], "y": data[:,1], "z": data[:,2]})
    df.to_csv(filename, index=False)
    print(f"Coordinates saved to {filename}")

Xref = create_reference(20, .5, 0.1) # assuming this is in meters

plot_points(Xref[:,0], Xref[:,1], Xref[:,2])
save_xyz_to_csv(Xref, "circle_traj_30pts_xyz.csv")

# save_xref_to_csv(Xref)
