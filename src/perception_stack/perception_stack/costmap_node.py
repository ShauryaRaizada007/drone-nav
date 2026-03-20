import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np

# Camera intrinsics (matches our 640x480 SDF camera)
FX = 554.25
FY = 554.25
CX = 320.0
CY = 240.0

RESOLUTION = 0.1        # meters per cell
GRID_SIZE  = 100        # 100x100 cells = 10m x 10m
DRONE_HEIGHT = 1.5      # assumed flight height in metres

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/depth/estimated', self.depth_callback, 10)
        self.pub = self.create_publisher(
            OccupancyGrid, '/costmap/occupancy', 10)
        self.get_logger().info('Costmap node ready.')

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        depth_m = depth.astype(np.float32) / 255.0 * 10.0  # scale to 0-10m

        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))

        # Back-project to 3D
        Z = depth_m
        X = (u - CX) * Z / FX
        Y = (v - CY) * Z / FY

        # Filter: keep points near drone flight height
        mask = (Z > 0.3) & (Z < 9.0) & (np.abs(Y - DRONE_HEIGHT) < 1.0)
        X_f = X[mask]
        Z_f = Z[mask]

        # Build occupancy grid
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int8)
        origin = GRID_SIZE // 2

        ix = (X_f / RESOLUTION + origin).astype(int)
        iz = (Z_f / RESOLUTION).astype(int)

        valid = (ix >= 0) & (ix < GRID_SIZE) & (iz >= 0) & (iz < GRID_SIZE)
        grid[iz[valid], ix[valid]] = 100

        # Publish
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info.resolution = RESOLUTION
        og.info.width  = GRID_SIZE
        og.info.height = GRID_SIZE
        og.info.origin.position.x = -origin * RESOLUTION
        og.info.origin.position.y = 0.0
        og.data = grid.flatten().tolist()
        self.pub.publish(og)
        self.get_logger().info('Costmap published.')

def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()