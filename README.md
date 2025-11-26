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
│   └── ...
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
