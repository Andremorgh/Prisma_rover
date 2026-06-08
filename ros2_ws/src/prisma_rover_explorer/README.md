# prisma_rover_explorer

`prisma_rover_explorer` is the proprietary ROS 2 package (previously named `custom_explorer`) responsible for autonomous path planning and environment exploration.

## Main Features

* **Frontier-Based Exploration (`explorer.py`)**: Analyzes the occupancy grid map produced by the SLAM module (`/prisma_rover/global_costmap/costmap`) to identify "frontiers" (the boundary zones between known free space and unmapped areas). It selects the optimal frontier goal and sends navigation targets to Nav2 to explore the environment.
* **Grid Sweep Coverage (`coverage_node.py`)**: Coordinates a *Boustrophedon* sweep pattern to systematically cover a specified boundary area, useful for search-and-rescue or systematic search missions.
* **Nav2 Integration**: Acts as an action client to the Nav2 navigation server `/prisma_rover/navigate_to_pose`.

## Package Structure

* **`prisma_rover_explorer/`**:
  * `explorer.py`: Main Python node for frontier detection and navigation.
  * `coverage_node.py`: Python node executing the sweep coverage path planner.
* **`setup.py` / `setup.cfg`**: Python packaging configuration files.

## Key Dependencies

* `rclpy`, `nav_msgs`, `geometry_msgs`
* Action client for `/navigate_to_pose` (`nav2_msgs`)
* `numpy` (for matrix operations on the costmaps)

## How to Use

To compile and launch the exploration nodes (ensure navigation and SLAM are active):

```bash
# Compilation
colcon build --packages-select prisma_rover_explorer

# Example: Run the Frontier Exploration node (with namespace)
ros2 run prisma_rover_explorer explorer --ros-args -r __ns:=/prisma_rover

# Example: Run the Sweep Coverage node
ros2 run prisma_rover_explorer coverage_node --ros-args -r __ns:=/prisma_rover
```
