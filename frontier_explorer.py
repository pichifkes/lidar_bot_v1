import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Subscribe to the map SLAM is building
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        
        self.navigator = BasicNavigator()
        #self.get_logger().info("Waiting for Nav2 to be ready...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active! Starting autonomous exploration.")
        
        self.map_data = None
        self.map_info = None
        
        # Check the map every 3 seconds to decide where to go next
        self.timer = self.create_timer(3.0, self.exploration_loop)

    def map_callback(self, msg):
        # Convert the 1D map array into a 2D numpy grid
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def exploration_loop(self):
        if self.map_data is None:
            return
            
        # If the robot is currently driving to a frontier, let it finish
        if not self.navigator.isTaskComplete():
            return
            
        # 0 = Free space, -1 = Unknown space, 100 = Wall/Obstacle
        free_cells = (self.map_data == 0)
        unknown_cells = (self.map_data == -1)
        
        # Shift the unknown cells up, down, left, and right to find edges
        up = np.roll(unknown_cells, 1, axis=0)
        down = np.roll(unknown_cells, -1, axis=0)
        left = np.roll(unknown_cells, 1, axis=1)
        right = np.roll(unknown_cells, -1, axis=1)
        
        # A cell is a "Frontier" if it is free space AND touches an unknown space
        is_frontier = free_cells & (up | down | left | right)
        
        # Get the X and Y coordinates of all frontier pixels
        frontier_y, frontier_x = np.where(is_frontier)
        
        if len(frontier_x) == 0:
            self.get_logger().info("🎉 NO FRONTIERS LEFT! MAPPING 100% COMPLETE! 🎉")
            self.timer.cancel() # Stop the loop
            return
            
        # Pick a random frontier pixel to drive to
        idx = np.random.randint(0, len(frontier_x))
        target_x = frontier_x[idx]
        target_y = frontier_y[idx]
        
        # Convert the grid pixel back to real-world meters
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        world_x = (target_x * res) + origin_x + (res / 2.0)
        world_y = (target_y * res) + origin_y + (res / 2.0)
        
        self.get_logger().info(f"Targeting new frontier at X: {world_x:.2f}, Y: {world_y:.2f}")
        
        # Send the destination to Nav2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = world_x
        goal_pose.pose.position.y = world_y
        goal_pose.pose.orientation.w = 1.0 # We don't care about rotation
        
        self.navigator.goToPose(goal_pose)

def main():
    rclpy.init()
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()