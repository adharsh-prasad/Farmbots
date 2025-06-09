# MATLAB-Based Terrain Simulation and Plowing Path Planning

This project simulates an agricultural rover operating in a 3D terrain environment. It focuses on terrain modeling, autonomous path planning using A* and spiral patterns, and simulating soil-leveling behavior via multiple passes. The simulation visualizes the rover's motion, terrain deformation, and plow coverage in real-time.

## 📌 Project Goals
- Model realistic farmland terrain using sinusoidal waves and random noise
- Simulate rover movement and plowing action
- Optimize terrain leveling using intelligent path planning
- Support multi-agent rover deployment with modular design

## 📁 File Structure
| File | Description |
|------|-------------|
| `main_simulation.m` | Main script to run the full simulation |
| `terrain_generator.m` | Generates farm terrain with bumps and elevation noise |
| `rover_class.m` | Defines rover properties and terrain-following motion |
| `path_planning.m` | Contains A*, spiral, and rectangular spiral plowing logic |
| `plow_tracker.m` | Tracks terrain deformation using a plow count matrix |
| `visualization_tools.m` | Handles real-time terrain and rover visualization |

## ▶️ How to Run
1. Open `main_simulation.m` in MATLAB (R2021b or later recommended).
2. Run the script to launch the simulation.
3. The visualization will include:
   - 3D terrain elevation map
   - Real-time rover movement
   - Plow count heatmap indicating leveling intensity

## 🧠 Key Features
- A* and spiral path planning optimized for plow coverage
- Multi-pass terrain leveling with diminishing returns logic
- Dynamic rover orientation based on local terrain gradient
- Real-time visualization using `surf`, `patch`, and color mapping
- Scalable to multiple rovers via object-oriented design

## 🔧 Requirements
- MATLAB R2021b or later
- Image Processing Toolbox (for Gaussian smoothing and visualization)

## 📸 Output
- Terrain surface with rover path
- Plow heatmaps over multiple passes
- Animation of rover motion and leveling progression

