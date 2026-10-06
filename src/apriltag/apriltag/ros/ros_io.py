"""ROS I/O helpers for message conversion and publishing."""

import array
from typing import List, Optional, Tuple

import cv2
import numpy as np
from apriltag_interfaces.msg import TagPose, TagPoseArray
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image

from apriltag.domain.app_types import CameraIntrinsics
from apriltag.domain.math_utils import rotation_matrix_to_quaternion


def cv2_to_img_msg(img_bgr: np.ndarray, header=None, encoding: str = "bgr8") -> Image:
    """Convert OpenCV BGR image to ROS Image message."""
    msg = Image()
    if header is not None:
        msg.header = header

    msg.height = int(img_bgr.shape[0])
    msg.width = int(img_bgr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(img_bgr.shape[1] * img_bgr.shape[2])
    # array.array is ~1000x faster than bytes here: rclpy validates bytes element by element
    msg.data = array.array("B", img_bgr.tobytes())
    return msg


def publish_image(
    image_pub,
    img_bgr: np.ndarray,
    header=None,
    resize_to: Optional[Tuple[int, int]] = None,
) -> None:
    """Publish OpenCV image to ROS topic as sensor_msgs/Image.

    Args:
        image_pub: ROS image publisher.
        img_bgr: Input OpenCV image in BGR format.
        header: Optional ROS header to copy.
        resize_to: Optional target size as (width, height).
    """
    if image_pub.get_subscription_count() == 0:
        return

    out_img = img_bgr
    if resize_to is not None:
        out_img = cv2.resize(img_bgr, resize_to, interpolation=cv2.INTER_LINEAR)

    image_pub.publish(cv2_to_img_msg(out_img, header=header))


def msg_to_cv2(img_msg) -> np.ndarray:
    """Convert ROS Image to OpenCV BGR image."""
    img_cv2 = np.ndarray(
        shape=(img_msg.height, img_msg.width, 3),
        dtype=np.uint8,
        buffer=img_msg.data,
    )
    return cv2.cvtColor(img_cv2, cv2.COLOR_RGB2BGR)


def make_camera_intrinsics(
    fx,
    fy,
    cx,
    cy,
    width,
    height,
) -> CameraIntrinsics:
    """Build CameraIntrinsics from node fields."""
    return CameraIntrinsics(
        fx=float(fx) if fx is not None else 0.0,
        fy=float(fy) if fy is not None else 0.0,
        cx=float(cx) if cx is not None else 0.0,
        cy=float(cy) if cy is not None else 0.0,
        width=int(width) if width is not None else 0,
        height=int(height) if height is not None else 0,
    )


def publish_twist(cmd_pub, vx: float, vy: float, vw: float) -> None:
    """Publish cmd_vel Twist."""
    twist = Twist()
    twist.linear.x = float(vx)
    twist.linear.y = float(vy)
    twist.angular.z = float(vw)
    cmd_pub.publish(twist)


def _quaternion_from_rotation(r_mat: Optional[np.ndarray]) -> Tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) for a rotation matrix; identity if missing or invalid."""
    if r_mat is None:
        return 0.0, 0.0, 0.0, 1.0
    try:
        return rotation_matrix_to_quaternion(np.asarray(r_mat))
    except Exception:
        return 0.0, 0.0, 0.0, 1.0


def publish_tag_poses(
    stamp,
    camera_frame: str,
    poses_pub,
    tf_broadcaster,
    targets: List[dict],
) -> None:
    """Publish one TagPoseArray with all targets and a TF per target.

    `targets` are dictionaries with family, id (str), t and R from
    draw_detections_and_collect_targets; an empty list publishes an empty
    array. `stamp` should be the source image's header stamp so consumers can
    compute dt from capture time. TF child frames are `<family>_<id>`.
    """
    array_msg = TagPoseArray()
    array_msg.header.stamp = stamp
    array_msg.header.frame_id = camera_frame
    transforms = []

    for target in targets:
        t_vec = target["t"]
        qx, qy, qz, qw = _quaternion_from_rotation(target.get("R", None))

        tag = TagPose()
        tag.family = target["family"]
        tag.id = target["id"]
        tag.pose.position.x = float(t_vec[0])
        tag.pose.position.y = float(t_vec[1])
        tag.pose.position.z = float(t_vec[2])
        tag.pose.orientation.x = qx
        tag.pose.orientation.y = qy
        tag.pose.orientation.z = qz
        tag.pose.orientation.w = qw
        array_msg.tags.append(tag)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = camera_frame
        tf_msg.child_frame_id = f"{target['family']}_{target['id']}"
        tf_msg.transform.translation.x = float(t_vec[0])
        tf_msg.transform.translation.y = float(t_vec[1])
        tf_msg.transform.translation.z = float(t_vec[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        transforms.append(tf_msg)

    poses_pub.publish(array_msg)
    if transforms:
        tf_broadcaster.sendTransform(transforms)
