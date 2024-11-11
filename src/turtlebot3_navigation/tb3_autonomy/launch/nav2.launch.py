import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_tb3 = get_package_share_directory('tb3_autonomy')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_tb3, 'maps', 'map.yaml')
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )]
    )

    autonomy_node_cmd = Node(
      package="tb3_autonomy",
      executable="autonomy_node",
      name="autonomy_node",
      parameters=[{
          "location_file": os.path.join(pkg_tb3, "config", "sim_house_locations.yaml")
      }]
    )

    set_init_amcl_pose_cmd = Node(
        package="tb3_autonomy",
        executable="set_init_amcl_pose",
        name="set_init_amcl_pose",
        output='screen',
        parameters=[{
            "x": -2.0,
            "y": -0.5,
        }]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(nav2_launch_cmd)
    ld.add_action(set_init_amcl_pose_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(autonomy_node_cmd)

    return ld