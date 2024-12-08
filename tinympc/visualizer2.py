import matplotlib.pyplot as plt
import pandas as pd

# Load the data
file_path = 'dooblyy.csv'
data = pd.read_csv(file_path)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111)

# Plot data with purple color
ax.scatter(data['u'], data['v'], color='red', label='Ashley Generated Velocities')

# # # Error bars for x, y, and z with different colors
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             xerr=data['x_error'], fmt='none', ecolor='red', alpha=0.5, label="x error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             yerr=data['y_error'], fmt='none', ecolor='green', alpha=0.5, label="y error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             zerr=data['z_error'], fmt='none', ecolor='blue', alpha=0.5, label="z error")

# Labels
ax.set_xlabel('u')
ax.set_ylabel('v')
# ax.set_zlabel('w')
ax.set_title('3D Scatter plot with Error Bars')
ax.legend()


file_path = 'figure8_data.csv'
data = pd.read_csv(file_path)



# # Plot data with purple color
# ax.scatter(data['x_dot'], data['y_dot'], color='purple', label='TinyMPC Generated velocities')
# ax.set_zlim(-0.5,0.5)
# ax.set_ylim(0,0.5)
# ax.set_xlim(0,0.5)
# # Error bars for x, y, and z with different colors
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             xerr=data['x_error'], fmt='none', ecolor='red', alpha=0.5, label="x error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             yerr=data['y_error'], fmt='none', ecolor='green', alpha=0.5, label="y error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             zerr=data['z_error'], fmt='none', ecolor='blue', alpha=0.5, label="z error")

# Labels
ax.set_xlabel('x')
ax.set_ylabel('y')
#ax.set_zlabel('z')
ax.set_title('TinyMPC Trajectory generation')
ax.legend()
plt.axis('equal')

plt.show()
