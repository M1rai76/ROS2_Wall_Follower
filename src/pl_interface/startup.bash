ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py &
sleep 10
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
sleep 10
cd $HOME/colcon_ws/src/pl_interface/iprolog_src;ros2 launch pl_interface pl_interface.launch.py
