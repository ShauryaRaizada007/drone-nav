import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class TestCostmap(Node):
    def __init__(self):
        super().__init__('test_costmap')
        self.pub = self.create_publisher(OccupancyGrid, '/costmap/occupancy', 10)
        self.timer = self.create_timer(1.0, self.publish_costmap)
        self.get_logger().info('Test costmap publisher ready.')

    def publish_costmap(self):
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info.resolution = 0.1
        og.info.width = 100
        og.info.height = 100
        og.info.origin.position.x = -5.0
        og.info.origin.position.y = 0.0
        grid = np.zeros((100, 100), dtype=np.int8)
        grid[20:30, 60:70] = 100
        og.data = grid.flatten().tolist()
        self.pub.publish(og)

def main(args=None):
    rclpy.init(args=args)
    node = TestCostmap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
