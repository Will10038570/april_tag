import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 30


class RealsenseRgbPublisher(Node):

    def __init__(self):
        super().__init__('realsense_rgb_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FRAME_RATE
        )
        self.pipeline.start(config)
        self.get_logger().info('RealSense pipeline started, publishing camera/color/image_raw')

        self.timer = self.create_timer(1.0 / FRAME_RATE, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseRgbPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
