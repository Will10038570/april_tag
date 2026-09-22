import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# key -> (default topic published by realsense2_camera, encoding, window title)
STREAM_SPECS = {
    'rgb': ('/camera/camera/color/image_raw', 'bgr8', 'RGB'),
    'depth': ('/camera/camera/depth/image_rect_raw', '16UC1', 'Depth'),
    'ir1': ('/camera/camera/infra1/image_rect_raw', 'mono8', 'Infrared 1'),
    'ir2': ('/camera/camera/infra2/image_rect_raw', 'mono8', 'Infrared 2'),
}


class CameraViewer(Node):

    def __init__(self):
        super().__init__('realsense_camera_viewer')

        self.declare_parameter('enable_rgb', True)
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('enable_ir1', False)
        self.declare_parameter('enable_ir2', False)
        for key, (default_topic, _, _) in STREAM_SPECS.items():
            self.declare_parameter(f'{key}_topic', default_topic)

        enabled_keys = [
            key for key in STREAM_SPECS
            if self.get_parameter(f'enable_{key}').value
        ]
        if not enabled_keys:
            raise RuntimeError('At least one stream must be enabled (rgb/depth/ir1/ir2)')

        self.bridge = CvBridge()
        self.quit_requested = False

        for key in enabled_keys:
            _, encoding, window_name = STREAM_SPECS[key]
            topic = self.get_parameter(f'{key}_topic').value
            self.create_subscription(
                Image, topic, self._make_callback(key, encoding, window_name), 10
            )
            self.get_logger().info(f'Subscribed to {topic} -> window "{window_name}"')

        self.get_logger().info('Press q in any window to quit')

    def _make_callback(self, key, encoding, window_name):
        def callback(msg):
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            if key == 'depth':
                image = cv2.convertScaleAbs(image, alpha=0.03)
                image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
            cv2.imshow(window_name, image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.quit_requested = True
        return callback

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
