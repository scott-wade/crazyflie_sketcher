from julia import Main
import numpy as np



# Include the Julia script
Main.include("/Users/ashleykline/CMU/24-774 Advanced Control Systems Integration/project/traj_gen_v2/main.jl")

current_state = [
    0.0, 0.0, 0.0,   # Position: x, y, z
    1.0, 0.0, 0.0, 0.0,  # Orientation: quaternion (w, qx, qy, qz)
    0.0, 0.0, 0.0,   # Velocity: vx, vy, vz
    0.0, 0.0, 0.0    # Angular Velocity: wx, wy, wz
]

goal_state = [
    5.0, 5.0, 3.0,   # Target position: x, y, z
    1.0, 0.0, 0.0, 0.0,  # Target orientation: quaternion (no rotation)
    0.0, 0.0, 0.0,   # Target velocity: vx, vy, vz
    0.0, 0.0, 0.0    # Target angular velocity: wx, wy, wz
]

# Function to generate trajectory
def generate_trajectory(current, goal, steps):
    """
    Generate a trajectory of states between the current and goal states.
    Args:
        current: numpy array of the current state
        goal: numpy array of the goal state
        steps: number of states in the trajectory
    Returns:
        numpy array of shape (steps, state_size)
    """
    trajectory = np.linspace(current, goal, steps)
    return trajectory



# Generate a trajectory with 5 intermediate steps
xref = generate_trajectory(current_state, goal_state, 30)
#xref = [xref_matrix[:, i].tolist() for i in range(xref_matrix.shape[1])]
#xref = [list(xref_matrix[:, i]) for i in range(xref_matrix.shape[1])]
# print(type(xref))

# Pass the Python objects to Julia
Main.current_state = current_state
Main.goal_state = goal_state
Main.xref = xref.T

# Access the `run_mpc` function from the `MPCSim` module
run_mpc = Main.run_mpc

# Call the Julia function
optimized_state_ref_traj = run_mpc(Main.current_state, Main.xref, Main.goal_state)

print("Optimized State:", optimized_state_ref_traj)


# for python- need Julia library