import numpy as np

x_dot = [1,2,3]
y_dot= [3,4,5]
z_dot=[7,8,9]
x_dot = np.array([x_dot])
y_dot = np.array([y_dot])
z_dot = np.array([z_dot])

norm_vel = (x_dot**2 + y_dot**2 +z_dot**2)**(0.5)
norm_vel = norm_vel.tolist()

print(norm_vel[0])