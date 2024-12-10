import matplotlib.pyplot as plt
import pandas as pd
x_offset = 0.3
y_offset = -0.15
z_offset = -0.23
# Load the data
file_path = 'cloud_no_magnet.csv'
data = pd.read_csv(file_path)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot data with purple color
ax.scatter(data['x_estimate'], data['y_estimate'], data['z_estimate'], color='purple', label='Achieved')

# # Error bars for x, y, and z with different colors
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             xerr=data['x_error'], fmt='none', ecolor='red', alpha=0.5, label="x error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             yerr=data['y_error'], fmt='none', ecolor='green', alpha=0.5, label="y error")
# ax.errorbar(data['x_estimate'], data['y_estimate'], data['z_estimate'],
#             zerr=data['z_error'], fmt='none', ecolor='blue', alpha=0.5, label="z error")

file_path = 'output_cloudFD_4board.csv'
data = pd.read_csv(file_path)
ax.scatter(data['x']+x_offset, data['y']+y_offset, data['z']+z_offset, color='red', label='Desired')
# Labels
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_title('MPC, No Magnet Dynamics: CloudFD')
ax.legend()
plt.axis("equal")
plt.show()
