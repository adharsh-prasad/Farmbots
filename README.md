# Particle-Based Soil Simulation for Agricultural Robotics (Python)

This project simulates interactive soil behavior using a particle system in Python. A rover moves through a granular terrain composed of stacked particles, which respond dynamically to motion and collision. The simulation uses Mayavi for 3D visualization, NumPy for particle logic, and Numba for performance optimization.

## 📌 Project Goals
- Simulate granular terrain deformation caused by rover movement
- Visualize real-time rover-soil interaction in 3D
- Implement a particle system capable of stacking, collapsing, and shifting
- Build a modular pipeline in pure Python without external physics engines

## 📁 File Structure
| File | Description |
|------|-------------|
| `particle_simulator.py` | Main simulation loop and rendering logic |
| `rover.py` | Defines rover structure, position update, and visualization |
| `terrain_generator.py` | Generates terrain elevation from procedural functions |
| `mayavi_visualizer.py` | Handles 3D point cloud rendering of soil particles |
| `physics_utils.py` | Contains Numba-accelerated soil behavior functions |

## ▶️ How to Run
1. Install dependencies:
   ```bash
   pip install numpy mayavi pygame numba
2. Run the simulation:
   ```bash
   python particle_simulator.py

🧠 Key Features
Real-time terrain deformation as the rover pushes through particles

Stack-based particle simulation with toppling and compression behavior

Dynamic rover height adjustment based on local elevation

Mayavi-powered 3D visualization with color-coded soil layers

Optimized performance with Numba to handle >10k particles interactively

🔧 Requirements
Python 3.10+

NumPy, Mayavi, Pygame, Numba (see requirements.txt if available)

📸 Output
3D soil terrain updated in real-time as rover moves

Visual feedback for terrain compression, deformation, and rover height

Optional screenshot/GIFs folder (media/) if shared in zipped folder
