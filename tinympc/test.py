import tinympc
import numpy as np
import csv
file_name = "f8_curve.csv"
x = []
y = []
z = []
x_dot = []
y_dot = []
z_dot =[]
scale = 2
offset = 0.1
# Read the CSV file and populate the lists
with open(file_name, 'r') as file:
    next(file)  # Skip the header row
    for line in file:
        values = line.strip().split(',')
        
        x.append(float(values[0]))
        
        y.append(float(values[1]))
        z.append(float(values[2])-offset)
        x_dot.append(float(values[3]))
        y_dot.append(float(values[4]))
        z_dot.append(float(values[5]))
# Define necessary data
A = np.array([ # 


    [1.0, 0.0, 0.0, 0.0, 0.0245250, 0.0, 0.050, 0.0, 0.0, 0.0, 0.02044, 0.0],
    [0.0, 1.0, 0.0, -0.0245250, 0.0, 0.0, 0.0, 0.050, 0.0, -0.02044, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0250000, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0250000, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
    [0.0, 0.0, 0.0, 0.0, 0.9810000, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0122625, 0.0],
    [0.0, 0.0, 0.0, -0.9810000, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0122625, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

B = np.array([ # 


    [-0.07069, 0.07773, 0.07091, -0.07795],
    [0.07034, 0.07747, -0.07042, -0.07739],
    [0.0052554, 0.0052554, 0.0052554, 0.0052554],
    [-0.1720966, -0.1895213, 0.1722891, 0.1893288],
    [-0.1729419, 0.1901740, 0.1734809, -0.1907131],
    [0.0123423, -0.0045148, -0.0174024, 0.0095748],
    [-0.0565520, 0.0621869, 0.0567283, -0.0623632],
    [0.0562756, 0.0619735, -0.0563386, -0.0619105],
    [0.2102143, 0.2102143, 0.2102143, 0.2102143],
    [-13.7677303, -15.1617018, 13.7831318, 15.1463003],
    [-13.8353509, 15.2139209, 13.8784751, -15.2570451],
    [0.9873856, -0.3611820, -1.3921880, 0.7659845]])

Q = np.diag([1.0, 1.0, 1.0, 0.4, 0.4, 0.4, # 


            0.4, 0.4, 0.4, 0.2, 0.2, 0.2]);
R = np.diag([1000.0]*4); # 



N = 20 # 



# Set up the problem
prob = tinympc.TinyMPC()
# Assuming x, y, z velocities are at indices 3, 4, 5 in your state vector
v_max = 0.5  # Maximum allowed velocity
v_min = -v_max  # Minimum allowed velocity

# Create state constraint vectors
x_min = np.array([-np.inf] * 12)  # Assuming 12 states
x_max = np.array([np.inf] * 12)

# Set velocity constraints
x_min[3:5] = v_min
x_max[3:5] = v_max

# Set up the problem with constraints
prob.setup(A, B, Q, R, N, x_min=x_min, x_max=x_max)


# Define initial condition
x0 = np.array([0.5, 1.3, -0.7, 0, 0, 0, 0, 0, 0, 0, 0, 0])
# Solve the problem
solution = prob.solve()

# Solve the problem
solution = prob.solve()

# Print the controls at the first time step
print(solution["controls"])


states = np.zeros((len(x),12))

# Simulate for an arbitrary number of time steps

for i,(x0,x1,x2,x3,x4,x5) in enumerate(zip(x,y,z,x_dot,y_dot,z_dot)):
    
    states[i] = np.array([x0,x1,x2,0,0,0,x3,x4,x5,0,0,0])
Nsim = np.shape(states)[0]
xs = np.zeros((Nsim, Q.shape[0])) # History of states for plotting
us = np.zeros((Nsim, R.shape[0])) # History of controls for plotting

print(np.shape(states)[0])
for i, state in enumerate(states):
    print(f"Iteration {i}, Initial state:", state)
    prob.set_x0(state)
    try:
        solution = prob.solve()
        if "controls" not in solution or len(solution["controls"]) == 0:
            raise ValueError("Solver did not return valid controls")
        print(f"Controls for iteration {i}:", solution["controls"])
        x0 = A @ state + B @ solution["controls"]
        xs[i] = x0
        us[i] = solution["controls"]
    except Exception as e:
        print(f"Error in iteration {i}: {e}")
        continue
with open('figure8_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z', 'p', 'q', 'r', 'x_dot', 'y_dot', 'z_dot', 'p_dot', 'q_dot', 'r_dot'])
        for state in (xs):
            writer.writerow(list(state))

print("Data has been written to figure8_data.csv")

    
import matplotlib.pyplot as plt

# Plot trajectory
fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(xs[:,:3], label=["x", "y", "z"])
axs[1].plot(us, label=["u1", "u2", "u3", "u4"])
axs[0].set_title("quadrotor trajectory over time")
axs[1].set_xlabel("time steps (100Hz)")
axs[0].legend()
axs[1].legend()
plt.show()