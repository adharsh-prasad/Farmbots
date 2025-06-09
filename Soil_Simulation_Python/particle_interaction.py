import numpy as np
from scipy.ndimage import gaussian_filter
from mayavi import mlab
from numba import jit

# ===================================
# Step 1: Create Smooth Terrain
# ===================================
grid_size = 5
resolution = 0.005
x = np.arange(0, grid_size, resolution)
y = np.arange(0, grid_size, resolution)
X, Y = np.meshgrid(x, y)

amplitude_variation = 0.3
frequency_variation_x = 2 * np.pi * (1 * np.random.rand(*X.shape))
frequency_variation_y = 2 * np.pi * (1 * np.random.rand(*Y.shape))
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

# ===================================
# Step 2: Particle Simulation
# ===================================
class Rover:
    def __init__(self, center, size, velocity):
        self.center = np.array(center, dtype=float)
        self.size = np.array(size, dtype=float)
        self.velocity = np.array(velocity, dtype=float)

    @property
    def bounds(self):
        half = 0.5 * self.size
        return self.center - half, self.center + half

    def move(self, dt):
        self.center += self.velocity * dt

@jit(nopython=True)
def rover_particle_interact(rover_center, rover_size, rover_velocity, particle_positions, particle_velocities):
    half_size = 0.5 * rover_size
    rmin = rover_center - half_size
    rmax = rover_center + half_size
    
    for i in range(len(particle_positions)):
        px, py, pz = particle_positions[i]
        bx, by, bz = rmin
        tx, ty, tz = rmax
        
        if (bx <= px <= tx and by <= py <= ty and bz <= pz <= tz):
            particle_velocities[i] += rover_velocity * 0.8

@jit(nopython=True)
def particle_collision(positions, velocities, masses, radii, restitution):
    for i in range(len(positions)):
        for j in range(i+1, len(positions)):
            diff = positions[j] - positions[i]
            dist_sq = np.sum(diff**2)
            min_dist = radii[i] + radii[j]
            if dist_sq < (min_dist ** 2) and dist_sq > 1e-12:
                dist = np.sqrt(dist_sq)
                normal = diff / dist
                rel_vel = velocities[j] - velocities[i]
                vel_along_normal = np.dot(rel_vel, normal)
                if vel_along_normal < 0:
                    m1, m2 = masses[i], masses[j]
                    j_impulse = -(1 + restitution) * vel_along_normal
                    j_impulse /= (1/m1 + 1/m2)
                    impulse = j_impulse * normal
                    velocities[int(i)] -= (impulse / m1)
                    velocities[int(j)] += (impulse / m2)
        return velocities

domain_bounds = [grid_size, grid_size, Z.max() + 0.5]
dt = 0.02
sim_steps = 200
num_particles_per_layer = 500
num_layers = 4

rng = np.random.default_rng(42)
positions = []
for layer in range(num_layers):
    for _ in range(num_particles_per_layer):
        px = rng.uniform(0.5, grid_size - 0.5)
        py = rng.uniform(0.5, grid_size - 0.5)
        pz_base_idx_x = int(px / resolution)
        pz_base_idx_y = int(py / resolution)
        pz_base_height = Z[pz_base_idx_y % Z.shape[0], pz_base_idx_x % Z.shape[1]]
        pz_layer_offset = layer * 0.05
        positions.append([px, py, pz_base_height + pz_layer_offset])
positions = np.array(positions)

velocities = np.zeros_like(positions)
masses = np.ones(len(positions))
radii = np.full(len(positions), 0.02)

rover = Rover(center=[1.5, 1.5, Z.max() + 0.25], size=[0.4, 0.4, 0.1], velocity=[-0.3, 0.3, 0])

# ===================================
# Step 3: Visualization with Mayavi
# ===================================
mlab.figure(size=(800, 600), bgcolor=(0.9, 0.9, 1))

# Plot terrain as a surface
mlab.surf(X, Y, Z.T, colormap='terrain', warp_scale=1)

# Plot particles
particle_plot = mlab.points3d(
    positions[:, 0], positions[:, 1], positions[:, 2],
    scale_factor=0.04,
    color=(0.8, 0.7, 0.6),
)

# Plot rover as a cube
rover_min_bounds, rover_max_bounds = rover.bounds
rx = [rover_min_bounds[0], rover_max_bounds[0]]
ry = [rover_min_bounds[1], rover_max_bounds[1]]
rz = [rover_min_bounds[2], rover_max_bounds[2]]
rover_points = mlab.points3d(rx, ry, rz, mode='cube', color=(1, 0, 0), scale_factor=0.3)
rover_src = mlab.pipeline.scalar_scatter(rx, ry, rz)
mlab.pipeline.surface(rover_src, opacity=0.3, color=(1, 0, 0))

# Add axes and set view
mlab.axes(extent=[0, domain_bounds[0], 0, domain_bounds[1], 0, domain_bounds[2]])
mlab.view(azimuth=45, elevation=60, distance=8)

# ===================================
# Step 4: Animation
# ===================================
@mlab.animate(delay=10)
def anim():
    global positions, velocities
    for _ in range(sim_steps):
        # Move the rover
        rover.move(dt)

        # Check collisions of rover with particles
        rover_particle_interact(rover.center, rover.size, rover.velocity, positions, velocities)

        # Handle particle-particle collisions
        velocities = particle_collision(positions, velocities, masses, radii, restitution=0.6)

        # Update particle positions
        positions += velocities * dt

        # Keep particles within bounds
        np.clip(positions[:, 0], 0.5, grid_size - 0.5, out=positions[:, 0])
        np.clip(positions[:, 1], 0.5, grid_size - 0.5, out=positions[:, 1])
        np.clip(positions[:, 2], Z.min(), domain_bounds[2], out=positions[:, 2])

        # Reflect velocities when particles hit boundaries
        mask_x = (positions[:, 0] <= 0.5) | (positions[:, 0] >= grid_size - 0.5)
        mask_y = (positions[:, 1] <= 0.5) | (positions[:, 1] >= grid_size - 0.5)
        mask_z = (positions[:, 2] <= Z.min()) | (positions[:, 2] >= domain_bounds[2])
        velocities[mask_x, 0] *= -1
        velocities[mask_y, 1] *= -1
        velocities[mask_z, 2] *= -1

        # Update the particle plot
        particle_plot.mlab_source.set(x=positions[:, 0], y=positions[:, 1], z=positions[:, 2])

        # Update the rover bounding box
        rover_min_bounds, rover_max_bounds = rover.bounds
        rx = [rover_min_bounds[0], rover_max_bounds[0]]
        ry = [rover_min_bounds[1], rover_max_bounds[1]]
        rz = [rover_min_bounds[2], rover_max_bounds[2]]
        rover_points.mlab_source.set(x=rx, y=ry, z=rz)

        yield

# Start the animation
anim()
mlab.show()

