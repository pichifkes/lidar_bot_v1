import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

def get_quaternion_from_euler(yaw):
    """A helper function to convert a 2D angle (yaw) into a 3D quaternion"""
    q_z = math.sin(yaw / 2.0)
    q_w = math.cos(yaw / 2.0)
    return 0.0, 0.0, q_z, q_w

def create_waypoint(navigator, x, y, yaw_angle):
    """Helper function to format coordinates into a Nav2 PoseStamped message"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    
    qx, qy, qz, qw = get_quaternion_from_euler(yaw_angle)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def main():
    rclpy.init()

    # Initialize the Nav2 Commander
    navigator = BasicNavigator()

    # 1. Set the initial starting position of the robot (0,0)
    initial_pose = create_waypoint(navigator, 0.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to fully load
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    # 2. Define the "Vacuum" Zig-Zag Path (Adjust X/Y to fit your warehouse)
    # This creates a back-and-forth mowing pattern
    print("Sending vacuum waypoints...")
    waypoints =[
        create_waypoint(navigator, 2.0, 0.0, 1.57),  # Move forward 2m, turn left
        create_waypoint(navigator, 2.0, 1.0, 3.14),  # Move up 1m, turn left again
        create_waypoint(navigator, 0.0, 1.0, -1.57), # Move back to X=0, turn right
        create_waypoint(navigator, 0.0, 2.0, 0.0),   # Move up 1m, turn right again
        create_waypoint(navigator, 2.0, 2.0, 0.0)    # Move forward 2m
    ]

    # 3. Send the waypoints to Nav2
    navigator.goThroughPoses(waypoints)

    # 4. Monitor the progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Executing waypoint... {feedback.current_waypoint}/{len(waypoints)}")

    # 5. Check the final result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Vacuuming complete! Room is clean.")
    elif result == TaskResult.CANCELED:
        print("Task was canceled.")
    elif result == TaskResult.FAILED:
        print("Robot got stuck and failed to clean the room.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()