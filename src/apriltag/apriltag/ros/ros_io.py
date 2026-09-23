"""ROS I/O helpers for message conversion and publishing."""

from typing import Optional, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
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
    msg.data = bytes(img_bgr.data)
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


def publish_trajectory(traj_pub, pose_msg: PoseStamped) -> None:
    """Publish a short two-point trajectory (origin -> tag pose)."""
    point = PoseStamped()
    point.header = pose_msg.header
    point.pose = pose_msg.pose

    origin = PoseStamped()
    origin.header = pose_msg.header
    origin.pose.orientation.w = 1.0

    path_msg = Path()
    path_msg.header = pose_msg.header
    path_msg.poses = [origin, point]
    traj_pub.publish(path_msg)


def publish_pose_and_tf(
    clock,
    camera_frame: str,
    pose_pub,
    traj_pub,
    tf_broadcaster,
    tag_id,
    t_vec,
    r_mat: Optional[np.ndarray] = None,
) -> None:
    """Publish PoseStamped, Path and TF for one AprilTag target."""
    pose_msg = PoseStamped()
    pose_msg.header.stamp = clock.now().to_msg()
    pose_msg.header.frame_id = camera_frame
    pose_msg.pose.position.x = float(t_vec[0])
    pose_msg.pose.position.y = float(t_vec[1])
    pose_msg.pose.position.z = float(t_vec[2])

    if r_mat is not None:
        try:
            qx, qy, qz, qw = rotation_matrix_to_quaternion(np.asarray(r_mat))
        except Exception:
            qx = qy = qz = 0.0
            qw = 1.0
    else:
        qx = qy = qz = 0.0
        qw = 1.0

    pose_msg.pose.orientation.x = qx
    pose_msg.pose.orientation.y = qy
    pose_msg.pose.orientation.z = qz
    pose_msg.pose.orientation.w = qw
    pose_pub.publish(pose_msg)

    publish_trajectory(traj_pub, pose_msg)

    tf_msg = TransformStamped()
    tf_msg.header.stamp = clock.now().to_msg()
    tf_msg.header.frame_id = camera_frame
    tf_msg.child_frame_id = f"apriltag_{tag_id}"
    tf_msg.transform.translation.x = float(t_vec[0])
    tf_msg.transform.translation.y = float(t_vec[1])
    tf_msg.transform.translation.z = float(t_vec[2])
    tf_msg.transform.rotation.x = qx
    tf_msg.transform.rotation.y = qy
    tf_msg.transform.rotation.z = qz
    tf_msg.transform.rotation.w = qw
    tf_broadcaster.sendTransform(tf_msg)
