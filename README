# ROS2 Wall Follower

This project implements a fully autonomous wall-following and mapping solution for the TurtleBot3 Waffle Pi using ROS2. The robot explores an unknown maze environment, builds a map as it navigates, and returns to its starting position, without requiring any human intervention after initiation. This project is ideal for understanding basic autonomous navigation concepts using ROS2, integrating Simultaneous Localization and Mapping (SLAM), and visualizing results in RViz.

## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Technologies Used](#technologies-used)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Deliverables](#deliverables)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Project Overview
The `ROS2_Wall_Follower` project is designed to enable a TurtleBot3 Waffle Pi to autonomously explore a maze using wall-following behavior. It uses ROS2 nodes to manage the robot’s movements, detects walls using laser sensors, and builds a map of the environment in real time. The robot is able to:
- Explore an unknown maze while following walls.
- Build an accurate map of the environment using SLAM techniques.
- Track and visualize the path taken by the robot during exploration.
- Return to its starting point once it has fully explored the maze.

The project is developed as part of the COMP3431: Robot Software Architecture course at UNSW, focusing on ROS2 application development and robotic control.

## Features
- **Autonomous Wall-Following**: Uses a custom C++ or Python node that enables the TurtleBot3 to follow the left or right wall of a maze.
- **SLAM with Cartographer**: Integrates the Cartographer ROS2 package for simultaneous localization and mapping, providing real-time map generation as the robot navigates.
- **Path Visualization**: Uses RViz2 to visualize the robot's path and the map it generates, allowing users to see the robot's exploration progress in real-time.
- **Return to Start**: After completing its exploration, the robot can autonomously return to its starting position.
- **ROS2 Integration**: Takes advantage of ROS2's robust communication system for sensor data processing, control commands, and visualization.

## Technologies Used
- **Robot Operating System 2 (ROS2)**: Middleware for communication between nodes and managing sensor data and control messages.
- **TurtleBot3 Waffle Pi**: A small, affordable robot that provides a platform for implementing SLAM and autonomous navigation.
- **Cartographer**: A ROS2 package used for 2D SLAM to build a map of the environment.
- **C++ and Python**: Programming languages for developing the wall-following behavior and processing sensor data.
- **RViz2**: A 3D visualization tool for viewing sensor data, robot states, and the generated map.

## Requirements
### Hardware
- TurtleBot3 Waffle Pi (with LDS sensor)
- Computer with ROS2 installed (Foxy)
- Wi-Fi connection for communication between the robot and computer

### Software
- **ROS2 Foxy**: Required for running ROS2 nodes and managing robot communication.
- **Cartographer**: For SLAM and generating maps of the environment.
- **RViz2**: For visualizing the robot's movements and map.
- **Python 3.x** or **C++**: Depending on your choice of implementation for the control nodes.


