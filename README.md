# Multi-Robot Navigation Simulator

A custom, lightweight 2D simulator for multi-robot navigation using ROS (Robot Operating System). This project replaces heavy physics simulators (like Gazebo) with a kinematic simulation node capable of handling multiple robots, laser scan simulation, and full integration with the ROS Navigation Stack (`move_base`, `amcl`).

## Features

- **Custom Kinematic Simulator**: Written in C++ (`multi_robot_simulator_node`), lighter and faster than Gazebo for 2D navigation testing.
- **Multi-Robot Support**: Namespace-based architecture allowing multiple agents (`robot1`, `robot2`) to operate simultaneously.
- **Full Navigation Stack**: Integrated with `move_base` for path planning and `amcl` for probabilistic localization.
- **Realistic Sensor Simulation**: Simulates LaserScan data based on a static occupancy grid map.
- **RViz Integration**: Ready-to-use visualization configuration.

## Project Structure

```text
multi_robot_simulator/
├── config/                  # Simulator physics and RViz settings
│   ├── sim_config.yaml      # Robot physics (speed, size) and sensors
│   └── MRS.rviz             # Visualization config
├── launch/                  # Startup scripts
│   ├── start_simulation.launch  # Main entry point
├── maps/                    # Environment data
│   ├── my_map.pgm           # Occupancy grid image
│   └── my_map.yaml          # Map metadata
├── param/                   # Navigation Stack configuration (The "Brain")
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params_robotX.yaml
│   ├── global/local_costmap_params.yaml
│   └── amcl_robotX.yaml
├── src/                     # Source code
│   └── simulator_node.cpp   # The core simulator engine
└── include/                 # Headers
    └── multi_robot_simulator/
│       ├── robot.h      # Define what is a robot
│       └── simulator.h      # Define the simulator class
```

## Prerequisites

- **OS**: Ubuntu 20.04 (recommended)
- **ROS Distro**: Noetic (or Melodic)
- **Dependencies**:
  ```bash
  sudo apt-get install ros-noetic-navigation ros-noetic-map-server ros-noetic-tf2-ros
  ```

## Installation

1. **Clone the repository** into your workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url> multi_robot_simulator
   ```

2. **Build the package**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. **Source the environment**:
   ```bash
   source devel/setup.bash
   ```

## Usage

### 1. Start the Simulation
Open a terminal and launch the simulator node, map server, and RViz:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch multi_robot_simulator start_simulation.launch
```

### 3. Control
- Open the **RViz** window.
- Select **"2D Nav Goal"** in the top toolbar.
- Click and drag on the map to send a target destination to the robot.

## Configuration Guide

| Goal | File to Modify | Parameter |
|------|----------------|-----------|
| **Max Speed (Physics)** | `config/sim_config.yaml` | `max_velocities/linear_x` |
| **Max Speed (Planner)** | `param/base_local_planner_params.yaml` | `max_vel_x` |
| **Robot Size/Radius** | `param/costmap_common_params_robotX.yaml` | `robot_radius` or `footprint` |
| **Initial Position** | `config/sim_config.yaml` | `initial_pose` |
| **Laser Range/FOV** | `config/sim_config.yaml` | `sensors` section |

## System Architecture
![](https://github.com/SimoneSangiorgio/RobProg/blob/5cc8eb206404aed41bf46fc164372a59d167d425/rosgraph_simplified.png)
1. **`move_base`**: In the first iteration, giving a goal, it suscribes to `/move_base/goal` and using the static map `/map` it publish the first `/cmd_vel`. Then, subscribing also to `/tf`, `/odom`, `/scan` and `/tf_static`, it evaluates the i-th `/cmd_vel` 
2. **`simulator_node`**: Subscribes to `/cmd_vel`, updates robot pose `/tf` based on kinematics, and publishes `/odom` and `/scan`.
3. **`amcl`**: Corrects odometry drift by comparing laser scans `/scan` and `/tf` to the known map `/map` and `/tf_static`.

