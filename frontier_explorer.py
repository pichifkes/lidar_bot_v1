import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Subscribe to the map SLAM is building
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        
        self.navigator = BasicNavigator()
        self.get_logger().info("Nav2 initialized! Starting autonomous exploration.")
        
        self.map_data = None
        self.map_info = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def exploration_loop(self):
        if self.map_data is None:
            return
            
        # If the robot is currently driving to a frontier, let it finish
        if not self.navigator.isTaskComplete():
            return
            
        free_cells = (self.map_data == 0)
        unknown_cells = (self.map_data == -1)
        
        up = np.roll(unknown_cells, 1, axis=0)
        down = np.roll(unknown_cells, -1, axis=0)
        left = np.roll(unknown_cells, 1, axis=1)
        right = np.roll(unknown_cells, -1, axis=1)
        
        is_frontier = free_cells & (up | down | left | right)
        frontier_y, frontier_x = np.where(is_frontier)
        
        if len(frontier_x) == 0:
            self.get_logger().info("🎉 NO FRONTIERS LEFT! MAPPING 100% COMPLETE! 🎉")
            return
            
        idx = np.random.randint(0, len(frontier_x))
        target_x = frontier_x[idx]
        target_y = frontier_y[idx]
        
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        world_x = (target_x * res) + origin_x + (res / 2.0)
        world_y = (target_y * res) + origin_y + (res / 2.0)
        
        self.get_logger().info(f"Targeting new frontier at X: {world_x:.2f}, Y: {world_y:.2f}")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = world_x
        goal_pose.pose.position.y = world_y
        goal_pose.pose.orientation.w = 1.0 
        
        self.navigator.goToPose(goal_pose)

def main():
    rclpy.init()
    explorer = FrontierExplorer()
    
    # NEW LOGIC: Use a standard while loop instead of a Timer
    # This prevents the "Executor is already spinning" crash
    while rclpy.ok():
        rclpy.spin_once(explorer, timeout_sec=0.5)
        explorer.exploration_loop()

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()