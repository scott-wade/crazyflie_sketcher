import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Load the data
file_path = 'output_fig8FD_small_Sunday.csv'
data = pd.read_csv(file_path)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111)
print(np.size(data['z']))
counter = 0.0
t = []
for i in range(np.size(data['z'])):
    t.append(counter)
    counter += 0.01

x_offset = 0.0
y_offset = 0.0
z_offset = -0.10
# Plot data with purple color
ax.scatter(t, data['z']+z_offset, color='red', label='Desired Z')

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
#ax.set_zlabel('Z Estimate')
ax.set_title('3D Scatter plot with Error Bars')
ax.legend()


file_path = 'results_pki0_5.csv'
data = pd.read_csv(file_path)
file_path = 'results_pki0_65.csv'
data2 = pd.read_csv(file_path)



# # Plot data with purple color
#ax.scatter(t, data['z_estimate'], color='purple', label='Measured Z pzKi=0.5')
ax.scatter(t, data2['z_estimate'], color='blue', label='Measured Z pzKi=0.65')
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
#ax.set_zlabel('Z Estimate')
ax.set_title('3D Scatter plot with Error Bars')
ax.legend()
#plt.axis('equal')


plt.show()
