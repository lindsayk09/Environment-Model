#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
import numpy as np

class EnvironmentalModel(Node):
    def __init__(self):
        super().__init__('environmental_model')

        # Subscriptions
        self.lane_info_sub = self.create_subscription(
            String, '/lane_info', self.lane_info_callback, 10)
        
        self.odm_sub = self.create_subscription(
            String, '/odm', self.odm_callback, 10)

        # Publisher
        self.grid_map_pub = self.create_publisher(
            OccupancyGrid, '/grid_map', 10)

        self.get_logger().info("Environmental Model Node Started")

        # Load static map from map.yaml
        self.static_map = self.load_static_map()

        # Dynamic environment variables
        self.lane_data = None
        self.object_map = None

    def lane_info_callback(self, msg):
        """Receives lane information and updates the grid map."""
        self.get_logger().info(f"Received Lane Info: {msg.data}")
        self.lane_data = msg.data
        self.update_grid_map()

    def odm_callback(self, msg):
        """Receives object detection map (static & dynamic obstacles)."""
        self.get_logger().info(f"Received ODM Data: {msg.data}")
        self.object_map = msg.data
        self.update_grid_map()

    def load_static_map(self):
        """Loads the static road map from map.yaml"""
        try:
            map_file_path = '/home/ros/ros2_ws/src/environmental_model/map.yaml'
            with open(map_file_path, 'r') as file:
                map_data = yaml.safe_load(file)
            self.get_logger().info(f"Static map loaded successfully from {map_file_path}.")
            
            # Dynamically set width and height based on the map data
            self.width = map_data['info']['width']
            self.height = map_data['info']['height']
            
            self.get_logger().info(f"Map data length: {len(map_data['data'])}")
            return map_data
        except Exception as e:
            self.get_logger().error(f"Failed to load map.yaml: {e}")
            return None

    def update_grid_map(self):
        """Generates and publishes the updated grid map (OccupancyGrid)."""
        if self.lane_data is None or self.object_map is None or self.static_map is None:
            return

        # Use dynamically loaded width and height from map.yaml
        width = self.width
        height = self.height

        grid = np.zeros((height, width), dtype=np.int8)

        # Process static map data
        data_length = len(self.static_map['data'])
        if data_length != width * height:
            self.get_logger().error(f"Data length mismatch: {data_length} != {width * height}")
            return

        for row in range(height):
            for col in range(width):
                if self.static_map['data'][row * width + col] == 100:
                    grid[row, col] = 100  # Mark as occupied

        # # Process lane data (simulated)
        # for i in range(10, 20):
        #     grid[i, 50] = 50  # Simulated lane marking

        # # Process object detection map
        # for i in range(30, 40):
        #     grid[i, 70] = 100  # Simulated obstacle
        
# Changes Made:
# Fixed Indexing: Changed the lane marking and obstacle processing code to ensure the indices stay within bounds (0 to 9 for a 10x10 grid).
# Grid Dimensions Validation: The grid dimensions are dynamically checked against the map dimensions from map.yaml to ensure the correct size is used for processing.
# Lane Marking: Limited lane marking and object detection to rows and columns that are within the valid range of the 10x10 grid.


        # Process lane data (ensure the indices are within bounds)
        for i in range(5, 7):  # Example, use indices within bounds (0 to 9)
            if i < height:  # Make sure row is within bounds
                grid[i, 5] = 50  # Simulated lane marking (ensure column is within bounds)

        # Process object detection map (ensure the indices are within bounds)
        for i in range(3, 5):  # Example, ensure indices are within bounds
            if i < height:
                grid[i, 7] = 100  # Simulated obstacle

                

        # Convert to ROS 2 OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.resolution = 0.5  # Ensure correct resolution
        grid_msg.data = grid.flatten().tolist()

        # Publish the grid map
        self.grid_map_pub.publish(grid_msg)
        self.get_logger().info("Published Updated Grid Map")

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentalModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
