import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'diff_drive_robot'

    ros2_control_params = "/config/my_controllers.yaml"
    controller_manager_timeout = ['--timeout', '5.0']

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_drive_controller",
                "--param-file", ros2_control_params] + controller_manager_timeout,
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_controller_spawner,
    ])
