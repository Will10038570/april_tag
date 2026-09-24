import cv2
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool

from apriltag.perception.tag_perception import (
    build_detector,
    draw_detections_and_collect_targets,
)
from apriltag.ros.ros_io import make_camera_intrinsics, msg_to_cv2, publish_image, publish_pose_and_tf
from apriltag.runtime.target_flow import choose_best_target


class AprilTagDetectionNode(Node):
    """Detect AprilTags in camera images and publish the closest tag's pose.

    Starts disabled. apriltag_control enables it through the `~/enable`
    (std_srvs/SetBool) service only while a start_tracking goal is running;
    while disabled the camera subscriptions are destroyed so no image is processed.
    """

    def __init__(self):
        super().__init__('apriltag_detection')

        # Low-latency image QoS: keep last 1.
        self.image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # initialize AprilTag detector
        self.detector = build_detector(tag_family="tag36h11")

        # meters, adjust to your tag's real size
        self.tag_size = 0.0635

        # camera intrinsics (filled by camera_info)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.cam_width = None
        self.cam_height = None
        self.camera_frame = 'camera_link'

        # camera subscriptions exist only while enabled
        self.image_sub = None
        self.info_sub = None

        self.pose_pub = self.create_publisher(PoseStamped, 'apriltag_pose', 1)
        self.image_pub = self.create_publisher(Image, 'apriltag/marked_image', self.image_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.enable_srv = self.create_service(SetBool, '~/enable', self._on_enable)
        self.get_logger().info('Detection is disabled. Waiting for apriltag_control to enable it.')

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self._enable()
            response.message = 'Detection enabled.'
        else:
            self._disable()
            response.message = 'Detection disabled.'
        response.success = True
        self.get_logger().info(response.message)
        return response

    def _enable(self) -> None:
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image,
                                                      'camera/camera/color/image_raw',
                                                      self.image_callback,
                                                      self.image_qos)
        if self.fx is None and self.info_sub is None:
            self.info_sub = self.create_subscription(CameraInfo,
                                                     'camera/camera/color/camera_info',
                                                     self.info_callback,
                                                     10)

    def _disable(self) -> None:
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.info_sub is not None:
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    # callback to receive camera intrinsics
    def info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.cam_width = msg.width
        self.cam_height = msg.height
        self.camera_frame = msg.header.frame_id if msg.header and msg.header.frame_id else self.camera_frame
        self.get_logger().info(f'Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
        # Unsubscribe after getting intrinsics
        if self.info_sub is not None:
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    # callback to process incoming images and publish the best tag pose
    def image_callback(self, msg: Image):
        intrinsics = make_camera_intrinsics(
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            self.cam_width,
            self.cam_height,
        )
        # pose estimation needs intrinsics
        if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
            return

        frame = msg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy),
            tag_size=self.tag_size,
        )

        vis, targets = draw_detections_and_collect_targets(
            frame.copy(),
            detections,
            intrinsics,
            self.detector,
            self.tag_size,
            self.get_logger(),
        )

        best = choose_best_target(targets)
        if best is not None:
            publish_pose_and_tf(
                stamp=msg.header.stamp,
                camera_frame=self.camera_frame,
                pose_pub=self.pose_pub,
                tf_broadcaster=self.tf_broadcaster,
                tag_id=best["id"],
                t_vec=best["t"],
                r_mat=best.get("R", None),
            )

        publish_image(self.image_pub, vis, header=msg.header, resize_to=(640, 360))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
