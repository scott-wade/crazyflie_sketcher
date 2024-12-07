import matplotlib.pyplot as plt
import pandas as pd

# Load the data
file_path = 'figure8_data.csv'
data = pd.read_csv(file_path)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot data with purple color
ax.scatter(data['x_estimate'], data['y_estimate'], data['z_estimate'], color='purple', label='estimates')

# # # Error bars for x, y, and z with different colors
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             xerr=data['x_error'], fmt='none', ecolor='red', alpha=0.5, label="x error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             yerr=data['y_error'], fmt='none', ecolor='green', alpha=0.5, label="y error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             zerr=data['z_error'], fmt='none', ecolor='blue', alpha=0.5, label="z error")

# Labels
ax.set_xlabel('X Estimate')
ax.set_ylabel('Y Estimate')
ax.set_zlabel('Z Estimate')
ax.set_title('3D Scatter plot with Error Bars')
ax.legend()

file_path = 'dooblyy.csv'
data = pd.read_csv(file_path)/4



# # Plot data with purple color
ax.scatter(data['x_estimate'], data['y_estimate'], data['z_estimate'], color='purple', label='estimates')
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
ax.set_xlabel('X Estimate')
ax.set_ylabel('Y Estimate')
ax.set_zlabel('Z Estimate')
ax.set_title('3D Scatter plot with Error Bars')
ax.legend()


plt.show()
