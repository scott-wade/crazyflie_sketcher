import numpy as np
import matplotlib.pyplot as plt

def generate_figure_8_coordinates(a=1, num_points=1000):
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    return x, y

# Generate coordinates
x, y = generate_figure_8_coordinates()

# Create the plot
plt.figure(figsize=(8, 6))
plt.plot(x, y)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.title('Figure-8 Curve')
plt.xlabel('x')
plt.ylabel('y')
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True, linestyle='--', alpha=0.7)
plt.show()
