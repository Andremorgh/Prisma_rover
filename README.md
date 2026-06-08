# Prisma Rover ROS 2 Workspace

Welcome to the official repository of **Prisma Rover**, an advanced software stack based on **ROS 2 Humble** designed for autonomous navigation, reactive fuzzy obstacle avoidance, SLAM mapping (2D and 3D), and intelligent object search using Reinforcement Learning (RL).

The entire system is Dockerized with modular build profiles to ensure maximum reproducibility on real hardware and in simulation.

---

## 🧭 Documentation Map

The project contains three main documentation files inside the `docs/` folder to guide you through development and usage:

1. 📖 **[GUIDE.md](file:///home/andrea/Desktop/Quantum_obj_rover/docs/GUIDE.md)**: **Technical User Manual**. Contains detailed instructions on launch files, kinematics and EKF fusion details, 2D/3D sensor configurations, mapping modes (SLAM Toolbox / RTAB-Map), and testing/validation checklists.
2. 🗂️ **[PROJECT_STRUCTURE.md](file:///home/andrea/Desktop/Quantum_obj_rover/docs/PROJECT_STRUCTURE.md)**: **Workspace and Node Directory**. Shows the complete file tree, the Dockerfile multi-stage build hierarchy (with ASCII and Mermaid diagrams), and a detailed list of all proprietary ROS 2 nodes with their available parameters and flags.
3. 📝 **[changes.md](file:///home/andrea/Desktop/Quantum_obj_rover/docs/changes.md)**: **Refactoring Changelog**. Tracks the refactoring history and porting from the legacy monolithic architecture to the current modular design.

---

## ⚡ Quick Start

### 1. Prerequisites
Ensure you have Docker installed, and optionally Nvidia Container Toolkit (if you want hardware graphics acceleration on Nvidia GPUs):
```bash
# Initialize local permissions for X11 forwarding
./docker_init.sh
```

### 2. Building the Docker Image
The Dockerfile supports 4 execution profiles. Build the desired profile by specifying it as an argument (defaults to `base`):
```bash
# Build the base profile (default)
./docker_build.sh base

# Available profiles:
# - base    : Simulation and basic Nav2 only (lightweight)
# - yolo    : Adds YOLOv11 and the RL agent for object search
# - quantum : Adds the fuzzy obstacle avoidance controller
# - full    : All packages enabled simultaneously
```

### 3. Running the Container
Start the container associated with your compiled profile. The system automatically detects your GPU (Nvidia, AMD, Intel) and configures the X11 graphics server for RViz2 and Gazebo:
```bash
# Run the container (defaults to base profile)
./docker_run.sh base
```
*(Add `software` or `cpu` as an additional argument if you want to force Mesa software rendering on systems without a dedicated GPU).*

### 4. Compilation and Running ROS 2 (inside the container)
Once inside the container shell, compile the ROS 2 workspace:
```bash
# Compile active ROS 2 packages
colcon build --symlink-install

# Execute the complete simulation + EKF + SLAM + Nav2 launch
ros2 launch prisma_rover_navigation sim_navigation.launch.py headless:=false rviz:=true
```

---

## 🛠️ Development & Support
This stack is developed and maintained by the Prisma Lab team. For code contributions or bug reports, please refer to the guidelines described in [GUIDE.md](file:///home/andrea/Desktop/Quantum_obj_rover/docs/GUIDE.md).
