import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.figure(figsize=(10, 5)), plt.gca()
ax.set_xlim(-6, 116)
ax.set_ylim(-6.8, 6.8)
ax.set_aspect('auto')

def get_y_squish(ax):
    x_bounds = ax.get_xlim()
    y_bounds = ax.get_ylim()
    bbox = ax.get_window_extent()
    if bbox.width == 0 or bbox.height == 0:
        return 1.0
    x_ppu = bbox.width / (x_bounds[1] - x_bounds[0])
    y_ppu = bbox.height / (y_bounds[1] - y_bounds[0])
    return x_ppu / y_ppu

# Force a draw so bbox is populated
plt.draw()
y_squish = get_y_squish(ax)
print("Squish:", y_squish)

corners = np.array([
    [-2, -1],
    [ 2, -1],
    [ 2,  1],
    [-2,  1]
])
# Rotated 90 degrees
yaw = np.pi/2
R = np.array([
    [np.cos(yaw), -np.sin(yaw)],
    [np.sin(yaw),  np.cos(yaw)]
])
rot = (R @ corners.T).T
rot[:, 1] *= y_squish
rot += np.array([50, 0])

poly = plt.Polygon(rot, facecolor='red')
ax.add_patch(poly)

plt.savefig('test_squish.png')
