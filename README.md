# Wall-Following and Marker Navigation with TurtleBot3 Waffle Pi

This project is designed for **TurtleBot3 Waffle Pi** using **ROS 2 Foxy** and includes features such as **wall-following**, **marker detection**, **navigation using Nav2**, and **visual detection with OpenCV**. It is a comprehensive exploration of autonomous navigation within a maze environment, enabling the robot to detect colored markers, map its environment, and follow walls with adaptive control.

---

## Table of Contents

- [Overview](#overview)
- [Setup](#setup)
- [Project Structure](#project-structure)
- [Features](#features)
  - [1. Wall-Following](#1-wall-following)
  - [2. Marker Detection](#2-marker-detection)
  - [3. Navigation with Nav2](#3-navigation-with-nav2)
  - [4. Initial Pose and AMCL Configuration](#4-initial-pose-and-amcl-configuration)
  - [5. ROS Parameters and Configurations](#5-ros-parameters-and-configurations)
- [Technical Concepts](#technical-concepts)
  - [Inflation Radius](#inflation-radius)
  - [Tolerance](#tolerance)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project is aimed at providing an autonomous solution for navigating within a maze using TurtleBot3 Waffle Pi. Key functionalities include wall-following for structured path traversal, color-based marker detection for localization, and navigating to specific goal points using the Nav2 stack. OpenCV is leveraged to enable real-time color-based landmark detection.

## Setup

### Prerequisites

- **ROS 2 Foxy**
- **TurtleBot3 Waffle Pi model**
- **OpenCV** for Python
- **Nav2** and related packages
- **Gazebo** for simulation
- **Rviz2** for visualization

Install dependencies:

```bash
sudo apt update
sudo apt install ros-foxy-turtlebot3* ros-foxy-nav2* ros-foxy-gazebo-ros-pkgs python3-opencv
```

### Setting Up ROS Domain ID

For multi-robot or multi-user scenarios, setting a unique ROS domain ID is recommended:

```bash
export ROS_DOMAIN_ID=2  # Example domain ID
```

### Building the Workspace

Clone the repository, navigate to your workspace, and build it with `colcon`:

```bash
cd ~/colcon_ws/src
git clone https://github.com/your-repo/wall-follower-marker-navigation.git
cd ..
colcon build
source install/setup.bash
```

## Project Structure

```
.
├── src/
│   ├── wall_follower/
│   ├── marker_navigator/
│   ├── scripts/
│   │   ├── see_marker.py
│   │   ├── point_transformer.py
│   ├── config/
│   │   ├── nav2_params.yaml
│   │   ├── amcl_params.yaml
│   ├── launch/
│   │   ├── wall_follower.launch.py
│   │   ├── marker_navigator.launch.py
│   └── CMakeLists.txt
├── README.md
└── LICENSE
```

## Features

### 1. Wall-Following

- **Description**: The robot follows the maze walls using adaptive control, moving forward, turning left or right, and making corrections based on distance data.
- **Logic**: The laser scanner detects nearby walls, and the robot adjusts its movement by maintaining a set distance from the wall.
- **Code**: Located in `src/wall_follower/`, primarily in `wall_follower.cpp`.

### 2. Marker Detection

- **Description**: Using OpenCV, the robot detects colored markers in real-time via a camera.
- **Logic**: Markers are detected based on color segmentation (HSV color space), identifying specific color combinations to recognize landmarks.
- **Code**: Defined in `scripts/see_marker.py` and `landmark.py`.
- **Configuration**: HSV values for colors are set in `landmark.py`. Markers include unique color pairs, e.g., “pink on blue.”

### 3. Navigation with Nav2

- **Description**: The robot navigates to detected markers using the Nav2 stack, setting up goals for each marker.
- **Logic**: Using `NavigateToPose` from Nav2, the robot can autonomously move to a specified marker position.
- **Code**: `src/marker_navigator/marker_navigator.py`
- **Parameters**: Parameters for Nav2 are configured in `config/nav2_params.yaml`, including inflation radius, robot footprint, and controller parameters.

### 4. Initial Pose and AMCL Configuration

- **Description**: To initialize localization, an initial pose is set either programmatically or manually in Rviz.
- **Code**: Initial pose handling is managed in `marker_navigator.py`.
- **AMCL**: Adaptive Monte Carlo Localization (AMCL) is configured to ensure accurate localization in the maze.

### 5. ROS Parameters and Configurations

- **Inflation Radius**: Defines the minimum distance between the robot and obstacles, enhancing obstacle avoidance. Set in `nav2_params.yaml`.
- **Tolerance**: Determines how close the robot should reach a goal before considering it achieved, allowing more flexibility around goal points. Also set in `nav2_params.yaml`.

## Technical Concepts

### Inflation Radius

The **inflation radius** is a buffer around obstacles in the costmap. A higher inflation radius increases the buffer, which is useful in tight spaces. This value is particularly important for wall-following, as it prevents the robot from getting too close to walls.

### Tolerance

Tolerance is the allowable range around a goal position that the robot can achieve to consider the goal reached. A larger tolerance might be required for loose accuracy, while smaller tolerances require higher precision.

## Usage

### Launch Wall-Following

```bash
ros2 launch wall_follower wall_follower.launch.py
```

### Launch Marker Detection and Navigation

```bash
ros2 launch marker_navigator marker_navigator.launch.py
```

### Setting Initial Pose in Rviz

Open Rviz, enable the “2D Pose Estimate” tool, and click the robot’s approximate location on the map to set the initial pose manually.

## Troubleshooting

- **AMCL Errors**: Check initial pose configuration if the robot's localization is inconsistent.
- **Sensor Origin Errors**: If errors mention the sensor origin out of bounds, verify costmap configurations.
- **Nav2 Timeout**: If Nav2 navigation goals fail, ensure the Nav2 server is fully initialized before sending goals.

## Contributing

Contributions are welcome! Feel free to submit a pull request with improvements, fixes, or additional features.

## License

This project is licensed under the [MIT License](LICENSE).

