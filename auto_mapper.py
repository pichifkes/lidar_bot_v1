import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Publisher to drive the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to read the Lidar
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        # 0 = Find Wall, 1 = Turn Left, 2 = Follow Wall
        self.state = 0 
        self.cmd = Twist()
        
        self.get_logger().info("Autonomous Mapping Node Started! Looking for a wall...")

    def scan_callback(self, msg):
        # Helper function to find the closest object in a slice of the Lidar array
        def get_min_range(ranges, start_idx, end_idx):
            slice_ranges = []
            for r in ranges[start_idx:end_idx]:
                # Ignore Inf and NaN values (Gazebo outputs these if no object is hit)
                if not math.isinf(r) and not math.isnan(r) and r > 0.1:
                    slice_ranges.append(r)
            return min(slice_ranges) if slice_ranges else 12.0 # 12.0 is max range of your Lidar

        # Slice the 360 Lidar rays into directions
        # Index 180 is straight ahead, 90 is right, 270 is left
        front_dist = get_min_range(msg.ranges, 160, 200) # +/- 20 degrees from front
        right_dist = get_min_range(msg.ranges, 70, 110)  # +/- 20 degrees from right

        d = 0.8 # Target distance from the wall (in meters)

        if self.state == 0:
            # STATE 0: Drive straight until we hit a wall
            if front_dist < d:
                self.state = 1
                self.get_logger().info(f"Wall detected in front at {front_dist:.2f}m. Turning left...")
            else:
                self.cmd.linear.x = 0.3
                self.cmd.angular.z = 0.0

        elif self.state == 1:
            # STATE 1: Turn Left until the front is clear
            if front_dist > d + 0.2:
                self.state = 2
                self.get_logger().info("Front clear. Aligning wall to the right side...")
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.5 # Rotate left

        elif self.state == 2:
            # STATE 2: Follow the Wall (keeping it on the right)
            if front_dist < d:
                # We hit a corner, go back to turning left
                self.state = 1
                self.get_logger().info("Corner detected! Turning left...")
            else:
                # Wall Following logic
                if right_dist < (d - 0.2):
                    # Too close to the right wall! Veer left.
                    self.cmd.linear.x = 0.2
                    self.cmd.angular.z = 0.4
                elif right_dist > (d + 0.2):
                    # Too far from the right wall! Veer right to find it.
                    self.cmd.linear.x = 0.2
                    self.cmd.angular.z = -0.4
                else:
                    # Perfect distance. Drive straight.
                    self.cmd.linear.x = 0.3
                    self.cmd.angular.z = 0.0

        # Publish the movement command
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Stop the robot when shutting down
    node.publisher_.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()