# 🤖 Wall-Following Robot Navigation System

This project uses the **ROS2 Framework**, **OpenCV**, **Nav2**, **Gazebo**, and **Cartographer** to develop an autonomous navigation system for a TurtleBot3 robot. The robot autonomously follows walls, detects and localizes color-coded markers, and navigates all while visualizing its path and detected markers in RViz. The project demonstrates the integration of advanced robotic navigation concepts, computer vision techniques, and simulation tools. TurtleBot3 was chosen as the robotic platform due to its compatibility with ROS2 and its well-documented resources. **Reference**: [TurtleBot3 Overview and Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).

---

## 🌟 Features

### 1. 🚧 Autonomous Wall-Following
   - Implements a **left-wall following** algorithm to guide the robot through a maze.
   - Uses TurtleBot3’s sensor data to maintain a stable distance from walls, ensuring smooth navigation.
   - **Reference**: [TurtleBot3 GitHub Repository](https://github.com/ROBOTIS-GIT/turtlebot3)

### 2. 🎯 Marker Detection with OpenCV
   - Uses **OpenCV** and **linear regression** to estimate the distance and shape of color-coded markers.
   - Differentiates markers based on unique color combinations (e.g., pink on green, blue on pink).
   - Publishes marker positions to the `/marker_position` topic for further processing.

### 3. 🌍 Marker Position Transformation
   - Converts marker positions from the **camera frame** to the **map frame** using **tf2**, ensuring accurate localization.
   - Publishes transformed positions as a `MarkerArray`, enabling real-time visualization in RViz.
   - **Reference**: Learn more about [tf2 Transformations](https://docs.ros.org/en/foxy/Tutorials/Tf2.html).

### 4. 🧭 Path Planning with Nav2
   - Uses **Nav2** for advanced path planning and obstacle avoidance.
   - Allows the robot to navigate from one marker to another in a predefined sequence.
   - Users can interactively set goals in RViz by clicking on markers, enabling flexible and dynamic navigation.
   - **Reference**: Explore the [Nav2 Documentation](https://navigation.ros.org/).

### 5. 🖥️ Simulation in Gazebo
   - Simulates the robot and its environment in **Gazebo**, allowing realistic testing of navigation algorithms in a virtual maze.
   - Provides an efficient way to test and refine algorithms before deployment on physical robots.
   - **Reference**: [Gazebo Documentation](http://gazebosim.org/tutorials)

### 6. 🗺️ Mapping with Cartographer
   - Integrates **Cartographer** for 2D SLAM, allowing the robot to create a real-time map of its environment while navigating.
   - Displays the generated map in RViz alongside the robot’s path and detected markers.
   - **Reference**: [Cartographer Documentation](https://google-cartographer.readthedocs.io/en/latest/)

---

## 🛠️ Project Structure

### 1. Wall-Following Node
   - Implements left-wall following behavior using sensor data, enabling autonomous maze navigation.

### 2. Marker Detection (`see_marker` Node)
   - Processes images from the TurtleBot3 Pi camera to detect and identify color-coded markers.
   - Publishes marker positions to the `/marker_position` topic.

### 3. Coordinate Transformation (`point_transformer` Node)
   - Subscribes to `/marker_position` to retrieve detected marker positions.
   - Transforms marker coordinates to the `map` frame using tf2 for accurate display in RViz.
   - Publishes transformed marker positions as a `MarkerArray`.

### 4. Nav2 Path Planning
   - Reads marker positions and their sequence from a configuration file and generates a path for the robot to follow.
   - Integrates ROS2’s Nav2 stack to autonomously navigate between markers and avoid obstacles.

---

## 📋 Notes

### Usage Instructions
- The detailed usage instructions for this project, including ROS2 CLI commands and launch files, are located in the `src/wall_follower/README.md` file in the project directory.

### Prerequisites
- ROS2 (Foxy or later) installed and configured.
- Familiarity with ROS2 CLI commands is required.
- TurtleBot3 and its dependencies installed. **Reference**: [TurtleBot3 Overview and Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).

---

## 🔧 Key ROS2 Concepts Demonstrated

### 🚀 Core Features
1. **Publishers and Subscribers**:
   - Demonstrates real-time data exchange between nodes using ROS2 publishers and subscribers.
   - **Reference**: [Writing ROS2 Publishers and Subscribers](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

2. **Nodes and Topics**:
   - Each feature is implemented as a separate ROS2 node, communicating over well-defined topics (e.g., `/marker_position`, `/odom`).

3. **tf2 Transformations**:
   - Converts marker coordinates from the camera frame to the map frame for accurate localization.

4. **Nav2 Integration**:
   - Implements advanced path planning and obstacle avoidance using Nav2’s robust navigation stack.

5. **Interactive Visualization with RViz**:
   - Real-time visualization of the robot’s path and marker positions in RViz for debugging and analysis.

---

## 🔧 Potential Improvements

- **Dynamic Obstacle Avoidance**: Enhance Nav2 integration to handle dynamic obstacles in real-time.
- **Lighting Adaptation**: Improve marker detection under variable lighting conditions.
- **Customizable Marker Sequences**: Allow users to specify custom paths or navigation goals dynamically in RViz.

---

## 📝 Academic and Project Context

This project was developed as part of the **COMP3431 Robotic Software Architecture** course at UNSW under the guidance of **Prof. Claude Summit**, Head of Artificial Intelligence at the UNSW School of Computer Science. It provided hands-on experience with:

- ROS2 concepts like publishers, subscribers, nodes, and topics.
- tf2 transformations for coordinate management across frames.
- Nav2 for autonomous navigation and path planning.
- OpenCV for computer vision applications in robotics.
- Gazebo for realistic simulation and testing.
- Cartographer for real-time mapping and SLAM integration.

--- 
