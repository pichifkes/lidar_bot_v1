import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='lidar_bot_v1' 

    # 1. Robot State Publisher
    # (Kept mostly the same, but fixed 'ture' to 'true')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # 2. Launch Gazebo (ros_gz_sim)
    # We use gz_sim.launch.py instead of gazebo.launch.py
    # gz_args="-r" runs the simulation immediately (no need to press play)
    # gz_args="empty.sdf" loads the default empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Spawn Entity (create)
    # Replaced spawn_entity.py with the 'create' node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.5'],
        output='screen'
    )

    # 4. ROS-GZ Bridge (NEW REQUIREMENT)
    # This is necessary for ROS to get the simulation time from Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
    ])