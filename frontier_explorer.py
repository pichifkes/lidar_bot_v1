import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import threading
import time

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.map_data = None
        self.map_info = None

    def map_callback(self, msg):
        # Convert the map into a 2D numpy array
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

def main():
    rclpy.init()

    # 1. Run the map listener in a background thread
    map_node = MapSubscriber()
    spin_thread = threading.Thread(target=rclpy.spin, args=(map_node,))
    spin_thread.daemon = True
    spin_thread.start()

    # 2. Start the Nav2 Commander in the main thread
    navigator = BasicNavigator()
    print("Waiting for Nav2 to activate...")
    
    # 3. Main Brain Loop
    while rclpy.ok():
        # Wait until SLAM sends us the first map
        if map_node.map_data is None:
            time.sleep(1.0)
            continue
            
        # If the robot is currently driving, wait for it to finish
        if not navigator.isTaskComplete():
            time.sleep(1.0)
            continue
            
        # --- FIND FRONTIERS ---
        free_cells = (map_node.map_data == 0)
        unknown_cells = (map_node.map_data == -1)
        
        up = np.roll(unknown_cells, 1, axis=0)
        down = np.roll(unknown_cells, -1, axis=0)
        left = np.roll(unknown_cells, 1, axis=1)
        right = np.roll(unknown_cells, -1, axis=1)
        
        # A frontier is a free cell touching an unknown cell
        is_frontier = free_cells & (up | down | left | right)
        frontier_y, frontier_x = np.where(is_frontier)
        
        if len(frontier_x) == 0:
            print("🎉 NO FRONTIERS LEFT! MAPPING 100% COMPLETE! 🎉")
            break
            
        # Pick a random frontier and calculate real-world coordinates
        idx = np.random.randint(0, len(frontier_x))
        res = map_node.map_info.resolution
        origin_x = map_node.map_info.origin.position.x
        origin_y = map_node.map_info.origin.position.y
        
        world_x = (frontier_x[idx] * res) + origin_x + (res / 2.0)
        world_y = (frontier_y[idx] * res) + origin_y + (res / 2.0)
        
        print(f"Targeting new frontier at X: {world_x:.2f}, Y: {world_y:.2f}")
        
        # Send goal to Nav2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(world_x)
        goal_pose.pose.position.y = float(world_y)
        goal_pose.pose.orientation.w = 1.0
        
        navigator.goToPose(goal_pose)
        time.sleep(2.0) # Give it 2 seconds to start moving

    # Cleanup
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()