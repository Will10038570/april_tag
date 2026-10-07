import cv2
import rclpy
import tf2_ros
from apriltag_interfaces.msg import TagPoseArray
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image

from apriltag.domain.math_utils import parse_tag_sizes, tag_size_for
from apriltag.perception.tag_perception import (
    build_detector,
    draw_detections_and_collect_targets,
)
from apriltag.ros.ros_io import make_camera_intrinsics, msg_to_cv2, publish_image, publish_tag_poses


class AprilTagDetectionNode(Node):
    """Detect AprilTags in camera images and publish all of them.

    Runs continuously from startup. Every image with camera intrinsics known
    gives one TagPoseArray on `apriltag_poses` with every detected tag of the
    configured families (empty when none is visible) and a TF per tag. It does
    not choose a tag: apriltag_control picks the one it tracks.
    """

    def __init__(self):
        super().__init__('apriltag_detection')

        # Low-latency image QoS: keep last 1.
        self.image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # AprilTag families to detect, space separated. pupil_apriltags loads
        # only one family per Detector, so each family gets its own detector.
        families_param = str(self.declare_parameter('tag_families', 'tag36h11').value)
        self.tag_families = list(dict.fromkeys(families_param.split()))
        if not self.tag_families:
            raise ValueError('Parameter tag_families is empty; give at least one family, e.g. tag36h11.')
        self.detectors = [build_detector(tag_family=family) for family in self.tag_families]

        # real black-border side of the tags (m); tag_sizes 'id:size ...'
        # overrides it per id (any family), e.g. '3:0.095'
        self.tag_size = float(self.declare_parameter('tag_size', 0.0635).value)
        if self.tag_size <= 0.0:
            raise ValueError('Parameter tag_size must be positive.')
        self.tag_sizes = parse_tag_sizes(self.declare_parameter('tag_sizes', '').value)
        # only for the debug image: tags apriltag_control would track by its
        # tag_family / tag_id parameters are outlined in yellow ('-1': any id;
        # a target_id given in a goal is not known here)
        self.tag_family = str(self.declare_parameter('tag_family', 'tag36h11').value)
        self.tag_id = str(self.declare_parameter('tag_id', '0').value)

        # camera intrinsics (filled by camera_info)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.cam_width = None
        self.cam_height = None
        self.camera_frame = 'camera_link'

        self.poses_pub = self.create_publisher(TagPoseArray, 'apriltag_poses', 1)
        self.image_pub = self.create_publisher(Image, 'apriltag/marked_image', self.image_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.image_sub = self.create_subscription(Image,
                                                  'camera/camera/color/image_raw',
                                                  self.image_callback,
                                                  self.image_qos)
        self.info_sub = self.create_subscription(CameraInfo,
                                                 'camera/camera/color/camera_info',
                                                 self.info_callback,
                                                 10)
        self.get_logger().info(
            f'tag_families={" ".join(self.tag_families)} tag_size={self.tag_size} '
            f'tag_sizes={self.tag_sizes}. Detecting continuously; publishing all tags on apriltag_poses.')

    def _is_tracked(self, family: str, tag_id: int) -> bool:
        return family == self.tag_family and self.tag_id in ('-1', str(tag_id))

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

    # callback to process incoming images and publish all tag poses
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
        detections = []
        for detector in self.detectors:
            detections.extend(detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy),
                tag_size=self.tag_size,
            ))

        vis, targets = draw_detections_and_collect_targets(
            frame.copy(),
            detections,
            intrinsics,
            None,  # detector: unused, pose comes from detect(estimate_tag_pose=True)
            self.tag_size,
            self.get_logger(),
            size_of=lambda tag_id: tag_size_for(tag_id, self.tag_sizes, self.tag_size),
            is_tracked=self._is_tracked,
        )

        publish_tag_poses(
            stamp=msg.header.stamp,
            camera_frame=self.camera_frame,
            poses_pub=self.poses_pub,
            tf_broadcaster=self.tf_broadcaster,
            targets=targets,
            image_width=frame.shape[1],
            image_height=frame.shape[0],
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
