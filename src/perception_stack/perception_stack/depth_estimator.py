import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from transformers import pipeline
import time
from PIL import Image as PILImage

class DepthEstimator(Node):
    def __init__(self):
        super().__init__("depth_estimator")
        self.get_logger().info("Loading DepthAnything-v2-small...")
        self.pipe = pipeline(
            task="depth-estimation",
            model="depth-anything/Depth-Anything-V2-Small-hf"
        )
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)
        self.pub = self.create_publisher(Image, "/depth/estimated", 10)
        self.frame_count = 0
        self.get_logger().info("Depth estimator ready.")

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 5 != 0:
            return
        t0 = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        pil_image = PILImage.fromarray(cv_image)
        result = self.pipe(pil_image)
        depth = np.array(result["depth"])
        depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_uint8 = depth_norm.astype(np.uint8)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding="mono8")
        depth_msg.header = msg.header
        self.pub.publish(depth_msg)
        latency = (time.time() - t0) * 1000
        self.get_logger().info(f"Depth inference: {latency:.1f}ms")

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
