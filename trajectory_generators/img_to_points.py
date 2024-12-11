import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
import pandas as pd

''' Drawing to Points '''
# Read the image
original = cv2.imread('hello2.png')

gray_img = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
_, binary_img = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY)
inverted_img = cv2.bitwise_not(binary_img)
skeleton = cv2.ximgproc.thinning(inverted_img, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
skeleton_inverted = cv2.bitwise_not(skeleton)

# cv2.imshow('Original Image', original)
# cv2.imshow('Thinned Skeleton', skeleton_inverted)

points = []

height, width = skeleton.shape 
for row in range(height):
    for col in range(width):
        if skeleton[row, col] == 255:  # Check for black pixel in skeleton
            points.append([col, row])

points = np.array(points)

# Plot the points
plt.figure(figsize=(8, 8))
plt.scatter(points[:, 0], points[:, 1], s=1, c='blue')
plt.gca().invert_yaxis()
plt.title('Extracted Points from Skeleton')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid()
plt.show()

# Identify the upper-rightmost point
start_index = np.lexsort((-points[:, 1], -points[:, 0]))[0]  # Sort by x (descending), then y (descending)
start_point = points[start_index]


distance_matrix = cdist(points, points, metric='euclidean')

# Solve TSP using a greedy approach
def solve_tsp_with_start(distance_matrix, start_index):
    n = len(distance_matrix)
    visited = [False] * n
    path = [start_index]  # Start at the specified point
    visited[start_index] = True

    for _ in range(n - 1):
        last = path[-1]
        # Find nearest unvisited point
        nearest = np.argmin([distance_matrix[last, j] if not visited[j] else np.inf for j in range(n)])
        path.append(nearest)
        visited[nearest] = True

    # path.append(start_index)  # Return to the starting point
    return path

# Solve TSP
tsp_path = solve_tsp_with_start(distance_matrix, start_index)
ordered_points = points[tsp_path]

plt.figure(figsize=(8, 8))
plt.scatter(points[:, 0], points[:, 1], c='blue', label='Original Points', s=10)
plt.plot(ordered_points[:, 0], ordered_points[:, 1], c='red', label='TSP Path')
plt.scatter(start_point[0], start_point[1], c='green', s=50, label='Start/End (Lower Right)')
plt.gca().invert_yaxis()
plt.title('Traveling Salesman Problem - Waypoints')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()
plt.grid()
plt.show()

output_filename = "acsi_text_2.csv"
pd.DataFrame(ordered_points.astype(float), columns=["x", "y"]).to_csv(output_filename, index=False)
print(f"Points exported to {output_filename}")

# Wait and close windows
cv2.waitKey(10000)
cv2.destroyAllWindows()
