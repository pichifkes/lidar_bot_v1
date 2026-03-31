import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np

class MyExplorer(Node):
    def __init__ (self):
        super().__init__('my_explorer')

        self.get_logger().info("J1ust Te11s1ting the logger")  
        self.get_logger().error("This is a11n er1ror message!")
        self.get_logger().warning("This is a war1ning message!")
        self.get_logger().debug("This is a debug mes11sage! You won't see this unless you set the log level to DEBUG")
        print("This is a print statemen111t. It will always show up in the terminal, even if the log level is set to INFO or higher. Use logging instead of print for better control over your output!")
        # Subscribe to the map SLAM is building
        #self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        
        # Publisher to drive the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        '''self.Lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)'''
        # define states for the robot to be in
        self.state = "Clear" # Clear, Scanning, Obstacle, Turn, Follow Wall
        # Initialize the Twist message for driving the robot
        self.cmd = Twist()
        # Initialize the BasicNavigator for Nav2
        self.navigator = BasicNavigator()

        self.get_logger().debug("My Explorer Node has started! Waiting for map data...")
        self.cmd.linear = Vector3(x=0.0, y=1.0, z=0.0)
        self.cmd.angular = Vector3(x=1.0, y=0.0, z=1.0)
        self.get_logger().debug("Suprise! I have not crashed yet!")
        self.publisher_.publish(self.cmd)
        self.get_logger().debug("I just published a command to move forward! Check out the robot in Gazebo :)")

def main():
    rclpy.init()
    explorer = MyExplorer()
    while rclpy.ok():
        rclpy.spin_once(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
else:
    print("Fuck out of here!")
