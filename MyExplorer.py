import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import math


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

        self.Lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # define states for the robot to be in
        self.state = "Clear" # Clear, Scanning, Obstacle, Turn, Follow Wall

        # Initialize the Twist message for driving the robot
        self.cmd = Twist()

        # Initialize the BasicNavigator for Nav2
        self.navigator = BasicNavigator()
    
    def is_valid_distance(self, distance):
        """
        Helper function to check if a distance reading from the Lidar is valid (not NaN or Inf).

        :param distance: The distance reading to check.
        :return: True if the distance is valid, False otherwise.
        """
        return not np.isinf(distance) and not np.isnan(distance) and distance > 0.1

    def pol2cart(self,dist, angle_deg):
        """
        converts polar coordinates (from lidar) to Cartesian coordinates.
        
        :param rho: The distance reading from the Lidar.
        :param phi: The angle (in radians) corresponding to the Lidar reading.
        :return: A tuple (x, y) representing the Cartesian coordinates of the point detected by the Lidar.
        """
        radian_angle = np.radians(angle_deg)
        x = dist * np.cos(radian_angle)
        y = dist * np.sin(radian_angle)
        return(round(x, 3), round(y, 3))
    
    def pol_array_to_cartesian(self, ranges, center_angle=0, fov=90):
        """
        Converts an array of polar coordinates (Lidar ranges) to Cartesian coordinates.

        :param ranges: An array of distance readings from the Lidar, where each index corresponds to an angle (0-359 degrees).
        :return: A list of tuples [(x1, y1), (x2, y2), ...] representing the Cartesian coordinates of the points detected by the Lidar.
        """
        points = []
        half_fov = fov // 2
        
        # Check angles from (center - half_fov) to (center + half_fov)
        for i in range(-half_fov, half_fov + 1):
            angle = (center_angle + i) % 360 # Wrap around (e.g., -10 becomes 350)
            dist = ranges[angle]
            
            if self.is_valid_distance(dist):
                points.append(self.pol2cart(dist, angle))
                
        return np.array(points) # Return as numpy array for easy math

    def is_wall(self, angle, msg, distance_threshold=0.15):
            """
            Detects a wall, removes outliers, and calculates confidence.

            :param angle: The center angle to look for a wall.
            :param msg: The LaserScan message.
            :param distance_threshold: How close a point needs to be to the line (in meters) to be considered an "inlier".
            :return: (wall_distance, wall_angle_deg, confidence) or None if no wall.
            """
            distance_at_angle = msg.ranges[angle]
            
            # Check if the specific angle given is actually valid
            if np.isinf(distance_at_angle) or np.isnan(distance_at_angle):
                self.get_logger().warning(f"Given angle: {angle} is NaN/Inf, but checking surroundings anyway.")

            # 1. Get all valid points in a 90 degree slice around the target angle
            # Note: 180 degrees is usually too large, as it grabs corners and other walls. 90 is safer.
            points = self.get_cartesian_points_in_slice(msg.ranges, angle, fov=90)
            
            # We need at least a few points to make a line
            if len(points) < 5:
                return None 

            # 2. Fit an initial line using SVD (A math trick that handles vertical lines perfectly)
            centroid = np.mean(points, axis=0)
            centered_points = points - centroid
            _, _, vh = np.linalg.svd(centered_points)
            direction_vector = vh[0] # This represents the slope of our line [vx, vy]
            
            vx, vy = direction_vector
            xc, yc = centroid

            # 3. Calculate how far EVERY point is from this initial line
            # Math formula for distance from point to line defined by a vector and a point
            c = vx * yc - vy * xc
            distances = np.abs(vy * points[:, 0] - vx * points[:, 1] + c)

            # 4. Filter out the oddities (Calculate "Sureness" / Confidence)
            inliers_mask = distances < distance_threshold
            inliers = points[inliers_mask]
            
            confidence = len(inliers) / len(points) # Example: 80 points align out of 100 = 0.80 (80%)

            # If sureness is too low, we don't have a reliable wall
            if confidence < 0.50: 
                return None

            # 5. Refit the line using ONLY the good points (inliers) for a perfect reading
            centroid = np.mean(inliers, axis=0)
            centered_points = inliers - centroid
            _, _, vh = np.linalg.svd(centered_points)
            direction_vector = vh[0]
            
            vx, vy = direction_vector
            xc, yc = centroid
            c = vx * yc - vy * xc

            # 6. Calculate Final Wall Distance (Shortest distance from robot at 0,0 to the line)
            wall_distance = abs(c)

            # 7. Calculate Final Wall Angle
            wall_angle_rad = math.atan2(vy, vx)
            wall_angle_deg = np.degrees(wall_angle_rad)
            
            # Normalize angle to be between 0 and 180 
            # (Because a wall extending North-South is the same as a wall extending South-North)
            wall_angle_deg = wall_angle_deg % 180

            return round(wall_distance, 3), round(wall_angle_deg, 2), round(confidence, 2)   

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not np.isinf(r) and not np.isnan(r)]
        min_dist = min(valid_ranges) if valid_ranges else float('inf')
        
        
        self.is_wall(0, msg) # Check front

        self.get_logger().info(str(np.shape(msg.ranges)))
        
        
            
            
def main():
    rclpy.init()
    explorer = MyExplorer()
    while rclpy.ok():
        print("Spinning the explorer node...")
        rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
else:
    print("Fuck out of here!")
