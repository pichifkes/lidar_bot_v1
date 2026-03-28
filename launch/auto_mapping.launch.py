import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'lidar_bot_v1'

    # 1. Launch the Gazebo Simulation (Your existing launch file)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'launch_sim.launch.py'
        )])
    )

    # 2. Launch SLAM Toolbox (Map Builder)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Launch Nav2 (Path Planner, specifically the navigation portion)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Launch RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Launch your custom Python Explorer Brain!
    # We use ExecuteProcess with your exact file path.
    explorer_cmd = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/dev_ws/src/lidar_bot_v1/frontier_explorer.py')],
        output='screen'
    )
    
    # Wait 8 seconds before starting the Python script so Nav2 and SLAM can load
    delayed_explorer = TimerAction(
        period=8.0, 
        actions=[explorer_cmd]
    )

    return LaunchDescription([
        sim_launch,
        slam_launch,
        nav2_launch,
        rviz_node,
        delayed_explorer
    ])
