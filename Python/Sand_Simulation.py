import pygame
import numpy as np
from scipy.ndimage import gaussian_filter

pygame.init()
WIDTH, HEIGHT = 1920, 1080
GRID_SIZE = 10  # 10m x 10m
PARTICLE_SIZE = 2
FPS = 60

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("3D Farmland Simulation")
clock = pygame.time.Clock()

class Rover:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.width = 0.3
        self.length = 0.3
        self.height = 0.15
        self.color = (255, 0, 0)  # Red color for visibility

    def draw(self, surface):
        # Convert rover dimensions to screen coordinates
        screen_x = int(self.x * WIDTH / GRID_SIZE)
        screen_y = int(HEIGHT - (self.y * HEIGHT / GRID_SIZE) - (self.z * 200))
        screen_width = int(self.width * WIDTH / GRID_SIZE)
        screen_length = int(self.length * HEIGHT / GRID_SIZE)
        
        # Draw the rover as a rectangle
        pygame.draw.rect(surface, self.color, (screen_x, screen_y, screen_width, screen_length))

class SoilParticle:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.color = self.get_color(z)

    def get_color(self, height):
        temp = 0.28/11
        if height < temp:
            return (0, 0, 255)  # Deep water
        elif temp<height < 2*temp:
            return (65, 105, 225)  # Water
        elif 2*temp < height < 3*temp:
            return (210, 180, 140)  # Light sand
        elif 3*temp < height < 4*temp:
            return (194, 178, 128)  # Sand
        elif 4*temp < height < 5*temp:
            return (124, 252, 0)  # Light green grass
        elif 5*temp < height < 6*temp:
            return (34, 139, 34)  # Green grass
        elif 6*temp < height < 7*temp:
            return (0, 100, 0)  # Dark green grass
        elif 7*temp < height < 8*temp:
            return (46, 139, 87)  # Light forest
        elif 8*temp < height < 9*temp:
            return (0, 100, 0)  # Forest
        elif 9*temp < height < 10*temp:
            return (105, 105, 105)  # Light rock
        else:
            return (139, 137, 137)  # Rock

        

    def draw(self, surface):
        screen_x = int(self.x * WIDTH / GRID_SIZE)
        screen_y = int(HEIGHT - (self.y * HEIGHT / GRID_SIZE) - (self.z * 200))
        pygame.draw.circle(surface, self.color, (screen_x, screen_y), PARTICLE_SIZE)

def generate_terrain():
    x = np.linspace(0, GRID_SIZE, 100)
    y = np.linspace(0, GRID_SIZE, 100)
    X, Y = np.meshgrid(x, y)

    amplitude_variation = 0.1 * np.random.rand(*X.shape)
    frequency_variation_x = 2 * np.pi * (0.1 * np.random.rand(*X.shape))
    frequency_variation_y = 2 * np.pi * (0.1 * np.random.rand(*Y.shape))
    Z = amplitude_variation * np.sin(frequency_variation_x * X) * np.cos(frequency_variation_y * Y)

    num_bumps = 2000
    for _ in range(num_bumps):
        x_center = np.random.rand() * GRID_SIZE
        y_center = np.random.rand() * GRID_SIZE
        height = 1 * (0.005 + 0.025 * np.random.rand()) * np.random.randn()
        width = 0.1 + 0.2 * np.random.rand()
        Z += height * np.exp(-((X - x_center)**2 + (Y - y_center)**2) / (2 * width**2))

    Z = np.abs(Z)
    Z = gaussian_filter(Z, sigma=1)
    return Z

def create_particles(terrain):
    particles = []
    max = 0
    for _ in range(2000000):
        x = np.random.uniform(0, GRID_SIZE)
        y = np.random.uniform(0, GRID_SIZE)
        z = terrain[int(y * 10)][int(x * 10)] * 2
        if z>max:
            max = z
        particles.append(SoilParticle(x, y, z))
    print(max)
    return particles

terrain = generate_terrain()
particles = create_particles(terrain)

rover = Rover(5, 5, 0.15)  # Place the rover at (5m, 5m) with z = 0.15m

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((135, 206, 235))  # Sky blue background

    for particle in particles:
        particle.draw(screen)

    rover.draw(screen)  # Draw the rover

    pygame.display.flip()
    clock.tick(FPS)

