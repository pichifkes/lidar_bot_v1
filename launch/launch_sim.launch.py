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
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )


    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'tugbot_warehouse.sdf'
    )

    # 2. Launch Gazebo (ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        # Pass the world_path to Gazebo
        launch_arguments={'gz_args': f'-r {world_path}'}.items() 
    )

    # 3. Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.5'], 
        output='screen'
    )

    # 4. ROS-GZ Bridge (ALL IN ONE)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # A. Clock (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # B. Command Velocity (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # C. Laser Scan (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            
            # D. Transforms / TF (Gazebo -> ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            
            # E. Odometry (Gazebo -> ROS)
            '/model/my_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # F. JOINT STATES (Gazebo -> ROS) *** VITAL FOR WHEELS IN RVIZ ***
            #'/world/empty/model/my_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model', ### this is potentialy simpler
        ],
        output='screen'
    )

    # 5. Joint State Relay (Optional but helps rename the topic)
    # The topic coming from Gazebo is usually weirdly named, this fixes it
    # But for now, let's stick to the bridge above.

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
    ])