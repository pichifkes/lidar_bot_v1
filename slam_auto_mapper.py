import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
from scipy.spatial import KDTree


class PointMemory:
    """ Spatial Hashing Grid to store global map points """
    def __init__(self, cell_size=0.1):
        self.cell_size = cell_size
        self.grid = {}
        
    def add_global_points(self, points):
        if points is None or len(points) == 0:
            return
        for p in points:
            x_idx, y_idx = int(p[0] // self.cell_size), int(p[1] // self.cell_size)
            self.grid[(x_idx, y_idx)] = self.grid.get((x_idx, y_idx), 0) + 1
                
    def get_confirmed_map_points(self, strength_threshold=5):
        """ Returns only points that have appeared multiple times """
        confirmed =[]
        for coords, strength in self.grid.items():
            if strength >= strength_threshold:
                real_x = (coords[0] * self.cell_size) + (self.cell_size / 2)
                real_y = (coords[1] * self.cell_size) + (self.cell_size / 2)
                confirmed.append([real_x, real_y])
        return np.array(confirmed) if confirmed else np.empty((0, 2))


class CustomSLAM(Node):
    def __init__(self):
        super().__init__('custom_slam')
        self.get_logger().info("Starting Custom SLAM & Exploration Node...")

        # Memory and SLAM states
        self.memory = PointMemory(cell_size=0.1)
        self.last_odom_matrix = None
        self.corrected_pose_matrix = self.get_transform_matrix(0.0, 0.0, 0.0)

        # Autonomous Exploration State
        self.state = "FORWARD"
        self.front_clearance = 10.0

        # Subscriptions & Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to run exploration logic continuously
        self.timer = self.create_timer(0.1, self.exploration_loop)

    # --- MATH & KINEMATICS UTILITIES ---
    def euler_from_quaternion(self, q):
        t0 = +2.0 * (q.w * q.z + q.x * q.y)
        t1 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t0, t1)

    def get_transform_matrix(self, x, y, theta):
        """ Creates a 2D 3x3 transformation matrix """
        return np.array([[np.cos(theta), -np.sin(theta), x],[np.sin(theta),  np.cos(theta), y],
            [0, 0, 1]
        ])

    def extract_pose(self, matrix):
        """ Extracts x, y, theta from a 3x3 transformation matrix """
        x, y = matrix[0, 2], matrix[1, 2]
        theta = math.atan2(matrix[1, 0], matrix[0, 0])
        return x, y, theta

    def transform_points(self, points, matrix):
        """ Transforms an array of [x, y] points using a 3x3 matrix """
        if len(points) == 0: return points
        ones = np.ones((points.shape[0], 1))
        points_3d = np.hstack([points, ones])
        transformed = np.dot(matrix, points_3d.T).T
        return transformed[:, :2]

    # --- SENSORS & SLAM ---
    def odom_callback(self, msg):
        """ Calculates exactly how much the wheels have moved since last tick """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        current_odom_matrix = self.get_transform_matrix(x, y, theta)

        if self.last_odom_matrix is None:
            self.last_odom_matrix = current_odom_matrix
            return

        # Calculate the delta (movement) from wheel odometry
        # delta = inverse(old_odom) * new_odom
        odom_delta = np.dot(np.linalg.inv(self.last_odom_matrix), current_odom_matrix)
        
        # Apply this delta to our internally tracked, ICP-corrected pose
        self.corrected_pose_matrix = np.dot(self.corrected_pose_matrix, odom_delta)
        
        # Save current odom for next time
        self.last_odom_matrix = current_odom_matrix

    def scan_callback(self, msg):
        """ SLAM Pipeline: Read LiDAR -> RANSAC -> ICP Correction -> Map Update """
        if self.last_odom_matrix is None: return

        # 1. Look ahead for Obstacle Avoidance
        front_ranges =[]
        # Get ranges from -30 deg to +30 deg (assuming 0 is straight ahead)
        center_idx = len(msg.ranges) // 2
        span = int((30.0 * math.pi / 180.0) / msg.angle_increment)
        for i in range(center_idx - span, center_idx + span):
            if 0.1 < msg.ranges[i] < 10.0:
                front_ranges.append(msg.ranges[i])
        self.front_clearance = min(front_ranges) if front_ranges else 10.0

        # 2. Extract valid Cartesian points
        local_points =[]
        for i, dist in enumerate(msg.ranges):
            if not np.isinf(dist) and not np.isnan(dist) and dist > 0.1:
                angle = msg.angle_min + (i * msg.angle_increment)
                local_points.append([dist * math.cos(angle), dist * math.sin(angle)])
        local_points = np.array(local_points)
        if len(local_points) < 10: return

        # 3. Denoise with RANSAC (Keep only walls)
        clean_local_points = self.extract_walls_ransac(local_points)
        if len(clean_local_points) < 10: return

        # 4. ICP Localization (Fix wheel slip by aligning with known map)
        map_points = self.memory.get_confirmed_map_points(strength_threshold=3)
        if len(map_points) > 50:
            # We have a map! Let's align our clean LiDAR scan to it.
            # Initial guess is our Odometry-predicted pose
            corrected_matrix = self.run_icp(clean_local_points, map_points, self.corrected_pose_matrix)
            self.corrected_pose_matrix = corrected_matrix

        # 5. Transform points to global frame using the CORRECTED pose
        global_points = self.transform_points(clean_local_points, self.corrected_pose_matrix)
        
        # 6. Add to memory
        self.memory.add_global_points(global_points)

    def extract_walls_ransac(self, points, distance_threshold=0.1, iterations=20, min_inliers=10):
        """ Keeps only points that form geometric lines """
        wall_points =[]
        unassigned = points.copy()
        
        while len(unassigned) >= min_inliers:
            best_mask = None
            best_inlier_count = 0
            
            for _ in range(iterations):
                idx = np.random.choice(len(unassigned), 2, replace=False)
                p1, p2 = unassigned[idx]
                v = p2 - p1
                length = np.linalg.norm(v)
                if length < 1e-6: continue
                
                nx, ny = -v[1]/length, v[0]/length
                distances = np.abs((unassigned[:, 0] - p1[0]) * nx + (unassigned[:, 1] - p1[1]) * ny)
                
                inlier_mask = distances < distance_threshold
                count = np.sum(inlier_mask)
                
                if count > best_inlier_count:
                    best_inlier_count = count
                    best_mask = inlier_mask
            
            if best_inlier_count < min_inliers: break
            
            wall_points.extend(unassigned[best_mask])
            unassigned = unassigned[~best_mask]
            
        return np.array(wall_points)

    # --- ITERATIVE CLOSEST POINT (ICP) ---
    def run_icp(self, source_points, target_points, initial_guess_matrix, max_iterations=10, tolerance=0.001):
        """ 
        Snaps current LiDAR scan (source) to the Global Map (target).
        This corrects Odometry drift using spatial landmarks!
        """
        target_tree = KDTree(target_points)
        current_matrix = initial_guess_matrix.copy()

        for i in range(max_iterations):
            # 1. Transform source points using our current best guess
            transformed_source = self.transform_points(source_points, current_matrix)

            # 2. Find the closest map point for every LiDAR point
            distances, indices = target_tree.query(transformed_source)
            
            # 3. Reject points that are too far away (they might be new unexplored areas)
            valid_mask = distances < 0.5 
            if np.sum(valid_mask) < 15:
                break # Not enough overlapping points to fix localization, trust Odometry instead

            matched_source = transformed_source[valid_mask]
            matched_target = target_points[indices[valid_mask]]

            # 4. Calculate Centers of Mass
            centroid_source = np.mean(matched_source, axis=0)
            centroid_target = np.mean(matched_target, axis=0)

            # 5. Center the points
            centered_source = matched_source - centroid_source
            centered_target = matched_target - centroid_target

            # 6. SVD to find the best rotation to align them
            W = np.dot(centered_source.T, centered_target)
            U, _, Vh = np.linalg.svd(W)
            R = np.dot(U, Vh).T

            # 7. Calculate translation
            t = centroid_target - np.dot(R, centroid_source)

            # 8. Update our transform matrix
            delta_matrix = np.eye(3)
            delta_matrix[0:2, 0:2] = R
            delta_matrix[0:2, 2] = t
            current_matrix = np.dot(delta_matrix, current_matrix)

            # 9. Check if it converged (stopped moving)
            if np.mean(distances[valid_mask]) < tolerance:
                break

        return current_matrix

    # --- AUTONOMOUS EXPLORATION ---
    def exploration_loop(self):
        """ Simple Finite State Machine to drive around the room safely """
        cmd = Twist()

        # Decide State
        if self.front_clearance < 0.8:  # Wall is closer than 80 cm
            self.state = "TURN"
        else:
            self.state = "FORWARD"

        # Execute State
        if self.state == "FORWARD":
            cmd.linear.x = 0.25      # Drive forward 0.25 m/s
            cmd.angular.z = 0.0
        elif self.state == "TURN":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5      # Spin to find an open path

        self.cmd_pub.publish(cmd)

        # Print Status
        x, y, theta = self.extract_pose(self.corrected_pose_matrix)
        map_size = len(self.memory.get_confirmed_map_points())
        self.get_logger().info(f"State: {self.state} | Pos: ({x:.2f}, {y:.2f}) | Confirmed Map Points: {map_size}")


def main(args=None):
    rclpy.init(args=args)
    slam_node = CustomSLAM()
    
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
        
    stop_cmd = Twist()
    slam_node.cmd_pub.publish(stop_cmd)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()