Terminal 1: ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
Terminal 2: ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
Terminal 3: ros2 launch wall_follower wall_follower.launch.py

Terminal: ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/src/wall_follower/src/map.yaml
Terminal: ros2 launch wall_follower marker_navigator.launch.py
