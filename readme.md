# TurtleBot Navigation Robot

This project involves setting up and running a navigation robot using TurtleBot. The robot can be controlled and navigated using the provided setup script.

## Prerequisites

- ROS (Robot Operating System) installed
- TurtleBot packages installed
- A workspace set up for ROS

## Setup

1. Clone the repository to your workspace:
    ```sh
    git clone https://github.com/vishvanathan-k/turtlebot_autonomy.git turtlebot_ws/
    cd turtlebot_ws/
    ```

2. Source the ROS setup script:
    ```sh
    source /opt/ros/humble/setup.bash
    ```

3. Build the workspace:
    ```sh
    colcon build --symlink-install
    ```

4. Source the workspace setup script:
    ```sh
    source ./install/setup.bash
    ```

5. Source the Gazebo setup script:
    ```sh
    . /usr/share/gazebo/setup.sh
    ```

6. Set the Gazebo model path:
    ```sh
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/dev_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
    ```

7. Set the TurtleBot model:
    ```sh
    export TURTLEBOT3_MODEL=waffle_pi
    ```

## Running the Navigation Robot

To run the navigation robot, execute the following commands:

1. Launch the TurtleBot3 world:
    ```sh
    ros2 launch tb3_autonomy turtlebot3_world.launch.py &
    ```

2. Launch the navigation stack:
    ```sh
    ros2 launch tb3_autonomy nav2.launch.py
    ```

This will launch the necessary ROS nodes and bring up the TurtleBot for navigation. You can give goals in RViz2 to make the robot go there.

## Files

- `setup.sh`: Script to set up and run the TurtleBot navigation.
- `src/turtlebot3_navigation/tb3_autonomy/launch/nav2.launch.py`: Launch file for the navigation stack.
- `src/turtlebot3_navigation/tb3_autonomy/launch/turtlebot3_world.launch.py`: Launch file for the TurtleBot3 world.
- `src/turtlebot3_navigation/tb3_autonomy/config/sim_house_locations.yaml`: Configuration file for simulation house locations.
- `src/turtlebot3_navigation/tb3_autonomy/maps/map.yaml`: Map file for the navigation.

