import numpy as np
from scipy.ndimage import gaussian_filter
from mayavi import mlab
from numba import jit

# ===================================
# Step 1: Create Smooth Terrain
# ===================================
grid_size = 1
resolution = 0.05  # Slightly coarser resolution for better performance
x = np.arange(0, grid_size, resolution)
y = np.arange(0, grid_size, resolution)
X, Y = np.meshgrid(x, y)

amplitude_variation = 0.3
frequency_variation_x = 2 * np.pi * (0.2 * np.random.rand(*X.shape))
frequency_variation_y = 2 * np.pi * (0.2 * np.random.rand(*Y.shape))
Z = amplitude_variation * np.sin(frequency_variation_x * X) * np.cos(frequency_variation_y * Y)

num_bumps = 2000
for _ in range(num_bumps):
    x_center = np.random.uniform(0, grid_size)
    y_center = np.random.uniform(0, grid_size)
    height = 0.025 * np.random.rand()
    width = 0.05 + 0.1 * np.random.rand()
    Z += height * np.exp(-((X - x_center)**2 + (Y - y_center)**2) / (2 * width**2))

Z = np.abs(Z)
Z = gaussian_filter(Z, sigma=10)
np.max(Z)
# ===================================
# Step 2: Particle Initialization
# ===================================
particle_radius = 0.02
particle_diameter = 2 * particle_radius

positions = []
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        # Terrain height at this grid point
        terrain_height = Z[i, j]
        # Number of particles to stack at this grid point
        num_particles = int(terrain_height / particle_diameter)
        # Stack particles vertically
        for k in range(num_particles):
            px, py = X[i, j], Y[i, j]
            pz = k * particle_diameter + particle_radius  # Center of the particle
            positions.append([px, py, pz])

positions = np.array(positions)

velocities = np.zeros_like(positions)
masses = np.ones(len(positions))
radii = np.full(len(positions), particle_radius)

# ===================================
# Step 3: Visualization with Mayavi
# ===================================
mlab.figure(size=(800, 600), bgcolor=(0.9, 0.9, 1))

# Plot terrain as a surface
mlab.surf(X, Y, Z.T, colormap='terrain', warp_scale=1)

# Plot particles
particle_plot = mlab.points3d(
    positions[:, 0], positions[:, 1], positions[:, 2],
    scale_factor=particle_diameter,
    color=(0.8, 0.7, 0.6),
)

mlab.axes(extent=[0, grid_size, 0, grid_size, 0, Z.max()])
mlab.view(azimuth=45, elevation=60, distance=8)

mlab.show()
