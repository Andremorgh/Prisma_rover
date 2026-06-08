# prisma_rover_obj_det_agent

`prisma_rover_obj_det_agent` is the proprietary ROS 2 package (previously named `obj_agent`) that implements an **intelligent navigation agent based on Reinforcement Learning (RL)** with LSTM memory, trained to find and reach specific target objects in the scene.

## Main Features

* **RL Agent (PyTorch LSTM)**: Runs an Actor-Critic Recurrent Neural Network (LSTM) loading the pre-trained weights `.cleanrl_model`. The model processes the robot state, map grid data, and text description of targets to output movement commands.
* **YOLO Integration**: Uses YOLO vision detection to track the presence of target objects in the camera frame.
* **Nav2 Action Interface**: Sends action goals to the Nav2 action server `/prisma_rover/navigate_to_pose` to steer the rover towards targets chosen by the RL agent.

## Package Structure

* **`launch/`**:
  * `agent.launch.py`: Co-launches the YOLO detection node and the RL agent control loop under the correct rover namespace.
* **`scripts/`**:
  * `agent.py`: Main Python control loop for the RL agent. Loads network weights, processes state features, and communicates with Nav2.
  * `detect_objects_agent.py`: YOLOv11-based 3D object detector and coordinate projector.
* **`prisma_rover_obj_det_agent/` (Neural Network & Model weights)**:
  * `agent_1747644302.cleanrl_model`: Active PyTorch neural network weights.
  * `vocab.json`: Vocabulary words list used for embedding text-based inputs.
  * `two_conv_nets.py`: Architecture definition of the neural network (CNN + LSTM).
  * `textutils.py`, `minigrid_obss_utils.py`, `dictlist.py`: Input pre-processing utilities.

## Key Dependencies

* `rclpy`, `sensor_msgs`, `geometry_msgs`
* `torch` (PyTorch)
* `ultralytics` (YOLO)
* `cv_bridge`, `numpy`

## How to Use

The package is compiled in both the `yolo` and `full` Docker profiles. To build and run:

```bash
# Compilation
colcon build --packages-select prisma_rover_obj_det_agent

# Co-launch the RL Agent and the object detector
ros2 launch prisma_rover_obj_det_agent agent.launch.py visualization:=false
```
