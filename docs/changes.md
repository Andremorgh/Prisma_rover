# Refactoring Log & Workspace Status (Prisma Rover)

This document tracks all changes, refactoring steps, additions, and differences compared to the original legacy `Quantum_obj_rover` repository configuration up to the current development state.

---

## 1. Comparative Analysis of Docker & Workspace Architectures

The migration from the legacy `Quantum_obj_rover` setup to the new dockerized workspace involved a total overhaul of the runtime and development environments. Below is a detailed comparison of the two architectures:

### A. Docker Setup: Monolithic vs. Multi-Stage

| Feature | Original Configuration (Monolithic) | New Configuration (Multi-Stage) |
| :--- | :--- | :--- |
| **Image Architecture** | Single monolithic stage based on `ros:humble`. All dependencies (simulation, vision, AI, hardware drivers) were baked into one large image. | Structured into 5 sequential dependency stages (`base` $\to$ `sim` $\to$ `yolo` $\to$ `quantum` $\to$ `full`) via multi-stage `Dockerfile` directives. |
| **Source Code Mounting** | Code was statically copied into the image at build time via `COPY ./src ${HOME}/ros2_ws/src` and compiled during `docker build`. | Source workspace is mounted dynamically at runtime as a host volume (`-v $(pwd)/ros2_ws:/home/user/ros2_ws`). No source code is copied. |
| **Development Cycle** | Any source code modification required running a slow `docker build`, or modifying code live inside the running container (losing persistence). | Development is done on the host machine (via local IDEs) and compiled dynamically inside the container using `colcon build`. Build files persist on the host disk. |
| **Size & Modularity** | Large, bloated image containing heavy AI frameworks, UI libraries, and drivers, even for basic headless simulation runs. | Space-optimized build caching. Allows running lightweight containers (e.g., matching the `base` or `quantum` profile targets) depending on available resources. |

---

### B. Project Directory Layout

#### Legacy Layout (Monolithic & Unorganized)
* ROS 2 packages were scattered in loose subfolders (e.g., `src/pkg/rover_gazebo`, `src/pkg/andrea/`).
* The main `Dockerfile` and several test variations (`Dockerfile_drone`, `Dockerfile_temp`) were cluttering the repository root.
* No clear separation existed between the development workspace (`ros2_ws`) and the environment virtualization files.

#### New Layout (Standard ROS 2 & Clean)
```text
Quantum_obj_rover/
├── docker/                             # Docker configuration files
│   ├── Dockerfile                      # Multi-stage recipe (base, sim, yolo, quantum, full)
│   ├── entrypoint.sh                   # Startup script for automatic ROS 2 and workspace sourcing
│   └── cyclonedds_config.xml           # CycloneDDS network settings
├── ros2_ws/                            # Standard ROS 2 workspace (mounted as a volume in Docker)
│   ├── src/                            # Sorted ROS 2 packages
│   │   ├── prisma_rover_interfaces/    # Unified custom message and service interfaces
│   │   ├── prisma_rover_description/   # Robot URDF, meshes, and kinematics models
│   │   ├── prisma_rover_sim/           # Gazebo simulation worlds and configurations
│   │   ├── prisma_rover_perception/    # Vision nodes (ArUco detector and pose estimator)
│   │   ├── prisma_rover_quantum_controller/ # Fuzzy (Quantum) controller and NumPy Lidar listener
│   │   ├── prisma_rover_explorer/      # Systematic frontier and sweep coverage
│   │   └── ...                         # Other migrated packages (teb_local_planner, roboclaw_ros2)
│   ├── build/                          # Dynamically generated build cache folder (host side)
│   ├── install/                        # Symbolically linked installation folder (host side)
│   └── log/                            # Compilation and run logs (host side)
├── docker_build.sh                     # Host script to compile the Docker image
├── docker_run.sh                       # Host script to run the container (GPU/DRI detection & X11)
├── .dockerignore                       # Prevents copying temporary caches to the Docker daemon
├── docs/                               # Project documentation folder
│   ├── GUIDE.md                        # Technical operations manual and testing checklists
│   ├── PROJECT_STRUCTURE.md            # Directory index and node parameters documentation
│   └── changes.md                      # Refactoring logs (this file)
└── README.md                           # Main repository entry point
```

---

### C. ROS 2, Middleware (RMW) and Networking

1. **RMW (ROS Middleware)**:
   * Both setups configure **CycloneDDS** (`rmw_cyclonedds_cpp`) instead of the default FastRTPS (`rmw_fastrtps_cpp`), providing significantly higher stability when transferring high-frequency PointCloud2 and Image streams.
2. **Network Isolation**:
   * In the new Docker setup, `docker/cyclonedds_config.xml` enables localhost isolation (`ROS_LOCALHOST_ONLY=1`) by default. This prevents multicast traffic from flooding local physical Wi-Fi/Ethernet networks during simulation runs.
3. **Entrypoint Orchestration**:
   * Added `entrypoint.sh` as the main container entrypoint. The script:
     1. Sources base ROS 2 Humble (`/opt/ros/humble/setup.bash`).
     2. Exports CycloneDDS environment variables.
     3. Automatically checks for and sources `/home/user/ros2_ws/install/setup.bash` if it exists, removing the need for manual sourcing.
     4. Dynamically handles workspace package filtering using `COLCON_IGNORE` to match the selected `$ROVER_PROFILE` at runtime.

---

### D. Dependency Management (Apt, Pip & Workspace Sources)

#### 1. System Dependencies (APT)
In the new Docker setup, APT packages are installed logically across specific stages:
* **Stage `base-deps`**: Minimum utilities (`curl`, `git`, `build-essential`, `python3-pip`, `tmux`).
* **Stage `sim-deps`**: Simulation and navigation suites (`ignition-fortress`, `ros-humble-ros-gz`, `ros-humble-navigation2`, `ros-humble-rtabmap-ros`, `ros-humble-robot-localization`, `ros-humble-pointcloud-to-laserscan`).
* **Stage `yolo-deps`**: Vision and transform libraries (`ros-humble-vision-opencv`, `ros-humble-image-transport`, `ros-humble-tf-transformations`).

#### 2. Python Dependencies (Pip)
Unlike the legacy setup which installed all Python requirements in one sequence (risking version mismatches), the new Dockerfile:
* Installs AI and computation libraries (`ultralytics` for YOLO, `open3d`, `shapely`, `scikit-learn`, `ros2-numpy`) cleanly inside the `yolo-deps` stage, disabling pip caches to reduce image sizes (`--no-cache-dir`).

#### 3. Workspace Packages Compiled from Source
* In the legacy repository, third-party source packages (like `teb_local_planner` and `costmap_converter`) were cloned directly into the Docker image during `docker build`.
* In the new repository, all third-party sources are placed directly inside `ros2_ws/src/` on the host. This allows tracking external dependencies via git on the host and building them alongside the custom packages with a single `colcon build` run.

### E. Workspace Mounting Rationale: Build-Time (.dockerignore) vs. Run-Time (Volumes)

To optimize development, there is a clear separation between how files are treated when compiling the Docker image versus when running the containers:

1. **Build-Time Behavior (`.dockerignore`)**:
   * When building the image (`docker build`), Docker requires a "build context" (which is the repository root `.`). The client packages all local files and sends them to the Docker daemon.
   * To prevent transferring gigabytes of local compilation caches (`ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`) and repository history (`.git`), a `.dockerignore` file is placed in the root.
   * This keeps the build process fast and keeps the resulting Docker images lightweight, containing only the static environment and pre-installed dependencies.

2. **Run-Time Behavior (Volumes)**:
   * When running the container (`docker run`), we mount the entire host directory `ros2_ws/` to `/home/user/ros2_ws` in the container.
   * **Important**: Volumes completely bypass `.dockerignore`. All local host folders (including `build` and `install` directories) are mounted inside the running container.
   * This is a deliberate choice: by mounting the entire `ros2_ws/` instead of just `ros2_ws/src/`, all compilation binaries are stored persistently on the host's disk. This allows for fast, incremental compilation (taking 2 seconds instead of rebuilding all C++ and Python nodes from scratch every time the container is restarted).


---

## 2. Package Migration Status

| Legacy Package | Migrated Package (in `ros2_ws/src/`) | Migration Details & Status |
| :--- | :--- | :--- |
| `rover_description_pkg` | `prisma_rover_description` | **Completed**. Replaced original `.dae` mesh files (which caused Gazebo render freezes due to missing UV coordinates) with primitives. Parametrizzato TF frames using namespaced prefixes (`prisma_rover/`). |
| `rover_gazebo` | `prisma_rover_sim` | **Completed**. Cleaned up launch files. Fixed Gazebo texture cache bugs by renaming ArUco markers uniquely. Added 10x10m enclosure walls in `aruco_world.sdf` and `yolo_world.sdf` to confine the robot. Imported worlds `quantum_test.sdf` and `leonardo_race_field.sdf` for testing the fuzzy controller. |
| `rover_bringup` | `prisma_rover_bringup`<br>`prisma_rover_navigation`<br>`prisma_rover_localization` | **Completed**. Reduced 19 redundant launch configurations. Separated starts into localization (EKF), navigation (Nav2 & SLAM Toolbox with sim_time updates), and physical hardware bringup. |
| `roboclaw_ros2` | `roboclaw_ros2` | **Completed**. Ported the C++ controller driver and configured proper wheel kinematics parameters. |
| `obj_msgs` & `aruco_interfaces` | `prisma_rover_interfaces` | **Completed**. Unified custom object detection and ArUco marker messages into a single interfaces package. |
| `obj_detection` | `obj_detection` | **Completed**. Refactored Python nodes to use the new message interfaces. Added real-time JSON database writes to `stored_objects.json` for persistent object tracking. |
| `aruco_detector_ocv_ros2`<br>`aruco_pose_estimation` | `prisma_rover_perception/aruco_detector`<br>`prisma_rover_perception/aruco_pose_estimation` | **Completed**. Refactored vision nodes to propagate `use_sim_time` to auxiliary TF listeners, resolving EKF synchronization drops. |
| `yolov11_ros2` | `yolov11_ros2` | **Completed**. Added support for YOLOv11 segmentation tracking in simulation. |
| `custom_explorer` | `prisma_rover_explorer` | **Completed**. Migrated and refactored. Converted coordinate subscribers to use `PoseStamped` (matching the publisher outputs), and mapped topics under `/prisma_rover/` namespace. |
| `rover_manager` | `prisma_rover_manager` | **Completed**. Ported and refactored C++ coordination code. Configured frames and TF checks to use `prisma_rover/map` and relativized topic names. |
| `obj_agent` | `prisma_rover_obj_det_agent` | **Completed**. Migrated and refactored. Replaced legacy `obj_msgs` imports with `prisma_rover_interfaces`. Updated static models and vocab paths to resolve dynamically at runtime. |
| `quantum_controller` | `prisma_rover_quantum_controller` | **Completed & Optimized**. Consolidated real and simulated nodes. Vectorized the Lidar range listener in NumPy to parse both 2D (`LaserScan`) and 3D (`PointCloud2`) data. Streamlined launch arguments (added `launch_sim`) and removed legacy Nav2 parameters. |

---

## 3. Legacy Packages Not Yet Migrated

All legacy robot packages have been successfully migrated to the active ROS 2 workspace in `ros2_ws/src/`. No legacy packages remain unmigrated.

---

## 4. Recent Maintenance & Repository Optimization

The following updates were applied to prepare the repository for GitHub and optimize local development:

### A. Default Profile Set to Base
* **`docker_run.sh` / `docker_build.sh`**: Changed the default profile from `full` to `base` to allow lightweight execution by default.
* **`docker_build.sh`**: Removed the automatic tagging of the `latest` image to prevent multi-tag conflicts when managing local Docker images.
* **Documentation**: Updated `README.md` and `docs/GUIDE.md` examples to reference the `base` profile as the baseline.

### B. Startup and Package Dependencies Optimizations
* **`Dockerfile`**: Pre-installed the `lap` library in the `yolo-deps` stage to support ByteTrack tracking in YOLOv11, avoiding dynamic pip downloads inside the container.
* **`docker_run.sh`**: Removed the inline runtime `pip install` check for `lap`, resulting in instant container startup times.
* **`entrypoint.sh`**: Replaced the basic profile logs with a detailed bulleted list of features included in the active profile during container startup.

### C. Gitignore Configuration
* Created a clean and comprehensive `.gitignore` in the repository root.
* **`**/COLCON_IGNORE`**: Explicitly ignored all colcon filter markers dynamically created by the entrypoint. This prevents local profile selections from dirtying git status or locking packages for other developers.
* **Backups**: Excluded `meshes_backup/` directories and OS/IDE metadata (`.DS_Store`, `.vscode/`).

