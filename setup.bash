#! /bin/bash

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ./install/setup.bash

. /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/dev_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch tb3_autonomy turtlebot3_world.launch.py &
ros2 launch tb3_autonomy nav2.launch.py
