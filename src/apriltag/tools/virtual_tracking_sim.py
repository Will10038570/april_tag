#!/usr/bin/env python3
"""Virtual end-to-end test for the AprilTag tracking pipeline.

Replaces the RealSense camera and the AMR base with a simulation, so the real
apriltag_detection and apriltag_control nodes run unmodified:

- a virtual AprilTag (tag36h11) stands at the world origin, facing -x;
- a virtual holonomic AMR carries a camera; its pose is integrated from
  /cmd_vel_nav;
- a synthetic camera image of the tag is rendered from the AMR/tag relative
  pose and published with camera_info, so apriltag_detection really detects it;
- Start sends the start_tracking goal and the run lasts until the action returns.

The OpenCV dashboard shows a top view (drag the AMR to set the initial pose),
the camera image, pose errors (ground truth vs. controller feedback) and the
cmd_vel commands. Each finished run is saved as CSV + PNG.

Controls:
    top view   drag AMR body = move, drag the round handle in front = rotate,
               mouse wheel or a / d = rotate 2 deg (only while not running)
    keys       s = start, x = stop (cancel goal), r = reset to initial pose,
               + / - = zoom top view, q or Esc = quit
"""

import array
import csv
import math
import os
import re
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from apriltag_interfaces.action import StartTracking
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from apriltag.domain.math_utils import optical_to_control_error

WINDOW_NAME = 'AprilTag virtual tracking'
FEEDBACK_RE = re.compile(r'x_err=(\S+)\s+y_err=(\S+)\s+yaw_err=(\S+)')
ACTIVE_PHASES = ('STARTING', 'RUNNING')


# --------------------------------------------------------------------------
# Geometry and rendering (pure functions)
# --------------------------------------------------------------------------

# tag axes in world (columns): x = right (seen from the front), y = down,
# z = into the tag. The tag faces -x, so into-tag is +x.
TAG_AXES_WORLD = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def camera_world_pose(robot_pose: np.ndarray, camera_offset: np.ndarray) -> Tuple[float, float, float]:
    """Camera (x, y, yaw) in world; the camera looks along the robot's +x."""
    x, y, th = robot_pose
    cam_xy = np.array([x, y]) + rot2(th) @ camera_offset
    return float(cam_xy[0]), float(cam_xy[1]), float(th)


def tag_in_optical(robot_pose: np.ndarray, camera_offset: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Tag pose (t, R) in the camera optical frame, same convention as pupil_apriltags.

    Camera and tag centre are at the same height. R == I when the camera
    faces the tag squarely.
    """
    cam_x, cam_y, th = camera_world_pose(robot_pose, camera_offset)
    c, s = math.cos(th), math.sin(th)
    # optical axes in world (columns): x = right, y = down, z = forward
    cam_axes = np.array([
        [s, 0.0, c],
        [-c, 0.0, s],
        [0.0, -1.0, 0.0],
    ])
    t = cam_axes.T @ (np.zeros(3) - np.array([cam_x, cam_y, 0.0]))
    r_mat = cam_axes.T @ TAG_AXES_WORLD
    return t, r_mat


def make_tag_board(tag_id: int, cell_px: int = 16) -> np.ndarray:
    """tag36h11 image with its 1-cell white quiet zone (10 x 10 cells).

    The black square (8 x 8 cells) is what tag_size refers to.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    marker = draw(dictionary, tag_id, 8 * cell_px)
    board = np.full((10 * cell_px, 10 * cell_px), 255, dtype=np.uint8)
    board[cell_px:9 * cell_px, cell_px:9 * cell_px] = marker
    return board


def project(points_optical: np.ndarray, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    z = points_optical[:, 2:3]
    return points_optical[:, :2] / z * np.array([fx, fy]) + np.array([cx, cy])


def render_camera_image(board: np.ndarray, t: np.ndarray, r_mat: np.ndarray, tag_size: float,
                        fx: float, fy: float, cx: float, cy: float,
                        width: int, height: int, background: int = 110) -> np.ndarray:
    """Render the tag board into a grayscale camera image."""
    img = np.full((height, width), background, dtype=np.uint8)

    half = tag_size * 10.0 / 8.0 / 2.0
    corners_tag = np.array([
        [-half, -half, 0.0],
        [half, -half, 0.0],
        [half, half, 0.0],
        [-half, half, 0.0],
    ])
    corners_cam = (r_mat @ corners_tag.T).T + t
    if np.any(corners_cam[:, 2] < 0.05):
        return img

    dst = project(corners_cam, fx, fy, cx, cy).astype(np.float32)
    n = board.shape[0]
    # pixel-centre convention on both sides
    src = np.array([[-0.5, -0.5], [n - 0.5, -0.5], [n - 0.5, n - 0.5], [-0.5, n - 0.5]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(board, homography, (width, height), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(board, 255), homography, (width, height),
                               flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    alpha = mask.astype(np.float32) / 255.0
    return (warped * alpha + img * (1.0 - alpha)).astype(np.uint8)


# --------------------------------------------------------------------------
# Simulation node
# --------------------------------------------------------------------------

@dataclass
class RunLog:
    # physics samples: (t, x, y, yaw, gt_x_err, gt_y_err, gt_yaw_err, vx, vy, wz, stage)
    samples: List[tuple] = field(default_factory=list)
    # controller feedback: (t, x_err, y_err, yaw_err)
    feedback: List[tuple] = field(default_factory=list)
    stage2_t: Optional[float] = None
    # every /cmd_vel_nav message as received: (t, vx, vy, wz)
    cmd_raw: List[tuple] = field(default_factory=list)


class VirtualTrackingSim(Node):

    def __init__(self):
        super().__init__('virtual_tracking_sim')

        def param(name, default):
            return self.declare_parameter(name, default).value

        # must match apriltag_detection / apriltag_control
        self.tag_size = float(param('tag_size', 0.0635))
        self.stage_distances = (float(param('stage1_distance', 0.50)), float(param('stage2_distance', 0.28)))
        self.tag_id = int(param('tag_id', 0))

        # virtual camera (RealSense D435 colour at 640x480 is roughly fx = fy = 615)
        self.width = int(param('image_width', 640))
        self.height = int(param('image_height', 480))
        self.fx = float(param('fx', 615.0))
        self.fy = float(param('fy', 615.0))
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.hfov = 2.0 * math.atan(self.cx / self.fx)
        self.camera_frame = str(param('camera_frame', 'camera_color_optical_frame'))
        # camera mount in the robot frame (x forward, y left)
        self.camera_offset = np.array([float(param('camera_x_offset', 0.0)),
                                       float(param('camera_y_offset', 0.026))])

        # virtual AMR
        self.robot_length = float(param('robot_length', 0.50))
        self.robot_width = float(param('robot_width', 0.40))
        self.cmd_timeout = float(param('cmd_timeout', 0.5))
        init_pose = np.array([float(param('init_x', -1.0)),
                              float(param('init_y', 0.15)),
                              math.radians(float(param('init_yaw_deg', 10.0)))])

        self.headless = bool(param('headless', False))
        self.auto_start = bool(param('auto_start', False))
        self.log_dir = os.path.abspath(os.path.expanduser(str(param('log_dir', 'virtual_tracking_logs'))))

        self.board = make_tag_board(self.tag_id)

        self.lock = threading.Lock()
        self.pose = init_pose.copy()
        self.initial_pose = init_pose.copy()
        self.cmd = np.zeros(3)
        self.cmd_time = 0.0
        self.trail: List[Tuple[float, float]] = []
        self.run = RunLog()
        self.phase = 'IDLE'
        self.stage = 1
        self.run_start: Optional[float] = None
        self.run_start_ros: Optional[Time] = None
        self.run_end: Optional[float] = None
        self.result_message = ''
        self.control_log: List[str] = []
        self.goal_handle = None
        self.raw_image: Optional[np.ndarray] = None
        self.marked_image: Optional[np.ndarray] = None
        self.marked_time = 0.0
        self.tag_visible = False
        self.png_request: Optional[str] = None
        self.finished = threading.Event()

        image_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.image_pub = self.create_publisher(Image, 'camera/camera/color/image_raw', image_qos)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera/color/camera_info', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_cmd_vel, 10)
        self.create_subscription(Image, 'apriltag/marked_image', self._on_marked_image, image_qos)
        self.create_subscription(Log, '/rosout', self._on_rosout, 100)
        self.action_client = ActionClient(self, StartTracking, 'start_tracking')

        self._last_physics = time.monotonic()
        self.create_timer(1.0 / float(param('physics_rate', 50.0)), self._physics_step)
        self.create_timer(1.0 / float(param('camera_rate', 30.0)), self._camera_step)
        if self.auto_start:
            self._auto_start_timer = self.create_timer(1.0, self._try_auto_start)

        self.get_logger().info(
            f'Virtual tracking sim ready. AMR at x={init_pose[0]:.2f} y={init_pose[1]:.2f} '
            f'yaw={math.degrees(init_pose[2]):.1f}deg, tag at origin facing -x.'
        )

    # ---- ground truth -----------------------------------------------------

    def ground_truth_error(self, pose: np.ndarray, stage: int):
        t, r_mat = tag_in_optical(pose, self.camera_offset)
        desired = self.stage_distances[stage - 1]
        return optical_to_control_error(t, desired, r_mat, float(self.camera_offset[1]))

    def goal_pose(self, stage: int) -> Tuple[float, float]:
        """Robot position where the controller's errors are all zero."""
        return (-self.stage_distances[stage - 1] - float(self.camera_offset[0]), 0.0)

    # ---- ROS callbacks ----------------------------------------------------

    def _on_cmd_vel(self, msg: Twist):
        now = time.monotonic()
        with self.lock:
            self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
            self.cmd_time = now
            if self.phase in ACTIVE_PHASES:
                self.run.cmd_raw.append((now - self.run_start, *self.cmd))

    def _on_marked_image(self, msg: Image):
        img = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data).copy()
        with self.lock:
            self.marked_image = img  # bgr8
            self.marked_time = time.monotonic()

    def _on_rosout(self, msg: Log):
        if not msg.name.endswith('apriltag_control'):
            return
        with self.lock:
            if self.run_start_ros is None or Time.from_msg(msg.stamp) < self.run_start_ros:
                return
            self.control_log = (self.control_log + [msg.msg])[-3:]
            if self.phase in ACTIVE_PHASES and msg.msg.startswith('Stage 1 aligned') and self.stage == 1:
                self.stage = 2
                self.run.stage2_t = time.monotonic() - self.run_start

    def _physics_step(self):
        now = time.monotonic()
        dt = min(now - self._last_physics, 0.1)
        self._last_physics = now
        with self.lock:
            cmd = self.cmd if now - self.cmd_time <= self.cmd_timeout else np.zeros(3)
            x, y, th = self.pose
            vx, vy, wz = cmd
            c, s = math.cos(th), math.sin(th)
            self.pose = np.array([x + (c * vx - s * vy) * dt,
                                  y + (s * vx + c * vy) * dt,
                                  math.atan2(math.sin(th + wz * dt), math.cos(th + wz * dt))])
            if self.phase in ACTIVE_PHASES:
                gt = self.ground_truth_error(self.pose, self.stage)
                t_run = now - self.run_start
                self.run.samples.append((t_run, *self.pose, gt.x_error, gt.y_error, gt.yaw_error,
                                         *cmd, self.stage))
                self.trail.append((self.pose[0], self.pose[1]))

    def _camera_step(self):
        with self.lock:
            pose = self.pose.copy()
        t, r_mat = tag_in_optical(pose, self.camera_offset)
        gray = render_camera_image(self.board, t, r_mat, self.tag_size,
                                   self.fx, self.fy, self.cx, self.cy, self.width, self.height)
        center_px = project(t.reshape(1, 3), self.fx, self.fy, self.cx, self.cy)[0] if t[2] > 0.05 else None
        visible = center_px is not None and 0 <= center_px[0] < self.width and 0 <= center_px[1] < self.height

        rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        stamp = self.get_clock().now().to_msg()

        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.camera_frame
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = 'rgb8'
        img_msg.step = self.width * 3
        # array.array is ~1000x faster than bytes here: rclpy validates bytes element by element
        img_msg.data = array.array('B', rgb.tobytes())

        info = CameraInfo()
        info.header = img_msg.header
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.info_pub.publish(info)
        self.image_pub.publish(img_msg)
        with self.lock:
            self.raw_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            self.tag_visible = visible

    # ---- action -----------------------------------------------------------

    def _try_auto_start(self):
        if self.action_client.server_is_ready():
            self._auto_start_timer.cancel()
            self.start()

    def start(self):
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                return
            if not self.action_client.server_is_ready():
                self.result_message = 'start_tracking action server not available.'
                return
            self.initial_pose = self.pose.copy()
            self.run = RunLog()
            self.trail = [(self.pose[0], self.pose[1])]
            self.stage = 1
            self.control_log = []
            self.result_message = ''
            self.run_start = time.monotonic()
            self.run_start_ros = self.get_clock().now()
            self.run_end = None
            self.phase = 'STARTING'
            self.finished.clear()

        goal = StartTracking.Goal()
        goal.start = True
        future = self.action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info('start_tracking goal sent.')

    def stop(self):
        with self.lock:
            handle = self.goal_handle if self.phase in ACTIVE_PHASES else None
        if handle is not None:
            handle.cancel_goal_async()

    def reset(self):
        self.stop()
        with self.lock:
            self.pose = self.initial_pose.copy()
            self.cmd = np.zeros(3)
            self.trail = []
            self.run = RunLog()
            self.stage = 1
            self.phase = 'IDLE'
            self.result_message = ''
            self.control_log = []
            self.run_start = None
            self.run_end = None

    def _on_goal_response(self, future):
        handle = future.result()
        with self.lock:
            if not handle.accepted:
                self.phase = 'REJECTED'
                self.result_message = 'Goal rejected.'
                self.finished.set()
                return
            self.goal_handle = handle
            if self.phase == 'STARTING':
                self.phase = 'RUNNING'
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        match = FEEDBACK_RE.search(feedback_msg.feedback.status)
        if match is None:
            return
        values = [float(v) for v in match.groups()]
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                self.run.feedback.append((time.monotonic() - self.run_start, *values))

    def _on_result(self, future):
        response = future.result()
        phase = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(response.status, f'STATUS_{response.status}')
        with self.lock:
            if self.phase not in ACTIVE_PHASES:
                return  # reset while running
            self.phase = phase
            self.result_message = response.result.message
            self.run_end = time.monotonic()
            self.goal_handle = None
        self.get_logger().info(f'start_tracking finished: {phase} - {response.result.message}')
        self._save_run(phase)
        self.finished.set()

    # ---- logging ----------------------------------------------------------

    def _save_run(self, phase: str):
        os.makedirs(self.log_dir, exist_ok=True)
        base = os.path.join(self.log_dir, time.strftime('%Y%m%d_%H%M%S') + f'_{phase.lower()}')
        with self.lock:
            run = RunLog(list(self.run.samples), list(self.run.feedback), self.run.stage2_t,
                         list(self.run.cmd_raw))
            init = self.initial_pose.copy()
            message = self.result_message

        with open(base + '_sim.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f'# result={phase}', f'message={message}',
                             f'init_x={init[0]:.4f}', f'init_y={init[1]:.4f}',
                             f'init_yaw_deg={math.degrees(init[2]):.2f}'])
            writer.writerow(['t', 'x', 'y', 'yaw', 'gt_x_err', 'gt_y_err', 'gt_yaw_err',
                             'cmd_vx', 'cmd_vy', 'cmd_wz', 'stage'])
            writer.writerows(run.samples)
        with open(base + '_feedback.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'x_err', 'y_err', 'yaw_err'])
            writer.writerows(run.feedback)
        with open(base + '_cmd_raw.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'vx', 'vy', 'wz'])
            writer.writerows(run.cmd_raw)

        final = run.samples[-1] if run.samples else None
        summary = f'[{phase}] {message}'
        if final is not None:
            max_cmd = np.max(np.abs(np.array([s[7:10] for s in run.samples])), axis=0)
            summary += (f' | final GT err x={final[4]:+.4f} y={final[5]:+.4f} yaw={final[6]:+.4f}'
                        f' | max |cmd| vx={max_cmd[0]:.3f} vy={max_cmd[1]:.3f} wz={max_cmd[2]:.3f}')
        self.get_logger().info(summary)
        self.get_logger().info(f'Run saved to {base}_*.csv')
        with self.lock:
            self.png_request = base + '.png'

    # ---- UI access --------------------------------------------------------

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            if self.run_start is None:
                elapsed = 0.0
            else:
                elapsed = (self.run_end or now) - self.run_start
            marked_fresh = self.marked_image is not None and now - self.marked_time < 0.5
            return {
                'pose': self.pose.copy(),
                'initial_pose': self.initial_pose.copy(),
                'cmd': self.cmd.copy() if now - self.cmd_time <= self.cmd_timeout else np.zeros(3),
                'trail': list(self.trail),
                'samples': list(self.run.samples),
                'feedback': list(self.run.feedback),
                'cmd_raw': list(self.run.cmd_raw),
                'stage2_t': self.run.stage2_t,
                'phase': self.phase,
                'stage': self.stage,
                'elapsed': elapsed,
                'result': self.result_message,
                'control_log': list(self.control_log),
                'image': self.marked_image if marked_fresh else self.raw_image,
                'image_is_marked': marked_fresh,
                'tag_visible': self.tag_visible,
            }

    def is_running(self) -> bool:
        with self.lock:
            return self.phase in ACTIVE_PHASES

    def set_pose(self, pose: np.ndarray):
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                return
            self.pose = pose.copy()
            self.initial_pose = pose.copy()
            self.trail = []

    def take_png_request(self) -> Optional[str]:
        with self.lock:
            path, self.png_request = self.png_request, None
            return path


# --------------------------------------------------------------------------
# OpenCV dashboard
# --------------------------------------------------------------------------

BG = (30, 30, 30)
PANEL = (42, 42, 42)
GRID = (70, 70, 70)
TEXT = (225, 225, 225)
DIM = (140, 140, 140)
C_X = (230, 150, 40)     # blue
C_Y = (40, 150, 255)     # orange
C_YAW = (80, 200, 80)    # green


def shade(color, k):
    """k < 1 darkens, k > 1 lightens towards white."""
    if k <= 1:
        return tuple(int(c * k) for c in color)
    return tuple(int(c + (255 - c) * (k - 1)) for c in color)
C_ROBOT = (255, 200, 120)
C_CAM = (0, 220, 255)
C_TOL = (55, 75, 55)
C_LIMIT = (90, 90, 230)
FONT = cv2.FONT_HERSHEY_SIMPLEX

CANVAS_W, CANVAS_H = 1400, 900
TOP_RECT = (0, 0, 700, 450)
IMG_RECT = (700, 0, 700, 450)
ERR_RECT = (0, 450, 700, 320)
CMD_RECT = (700, 450, 700, 320)
BAR_RECT = (0, 770, 1400, 130)


def put(img, text, org, color=TEXT, scale=0.45, thickness=1):
    cv2.putText(img, text, (int(org[0]), int(org[1])), FONT, scale, color, thickness, cv2.LINE_AA)


def draw_plot(img, rect, title, t_max, y_lim, lines=(), dots=(), band=None, hlines=(), vlines=(), legend=()):
    """Minimal time-series plot.

    lines: (t_array, v_array, color[, thickness=2])
    dots:  (t_array, v_array, color[, radius=3, filled=False])
    """
    x0, y0, w, h = rect
    cv2.rectangle(img, (x0 + 4, y0 + 4), (x0 + w - 5, y0 + h - 5), PANEL, -1)
    put(img, title, (x0 + 12, y0 + 22), scale=0.5)
    left, top = x0 + 58, y0 + 32
    right, bottom = x0 + w - 16, y0 + h - 28
    y_min, y_max = y_lim

    def to_px(t, v):
        u = left + np.asarray(t) / t_max * (right - left)
        vv = top + (y_max - np.clip(np.asarray(v), y_min, y_max)) / (y_max - y_min) * (bottom - top)
        return u, vv

    if band is not None:
        _, b0 = to_px(0.0, band[1])
        _, b1 = to_px(0.0, band[0])
        cv2.rectangle(img, (left, int(b0)), (right, int(b1)), C_TOL, -1)

    for i in range(5):
        v = y_min + (y_max - y_min) * i / 4
        _, py = to_px(0.0, v)
        cv2.line(img, (left, int(py)), (right, int(py)), GRID, 1)
        put(img, f'{v:+.2f}', (x0 + 10, py + 4), DIM, 0.38)
    step = next(s for s in (0.5, 1, 2, 5, 10, 20, 30, 60, 120) if t_max / s <= 8)
    for k in range(int(t_max / step) + 1):
        px, _ = to_px(k * step, 0.0)
        cv2.line(img, (int(px), top), (int(px), bottom), GRID, 1)
        put(img, f'{k * step:g}s', (px - 8, bottom + 16), DIM, 0.38)

    for v, color in hlines:
        _, py = to_px(0.0, v)
        cv2.line(img, (left, int(py)), (right, int(py)), color, 1)
    for t, label in vlines:
        px, _ = to_px(t, 0.0)
        cv2.line(img, (int(px), top), (int(px), bottom), TEXT, 1)
        put(img, label, (px + 4, top + 12), TEXT, 0.4)

    for t, v, color, *style in lines:
        if len(t) < 2:
            continue
        thickness = style[0] if style else 2
        u, vv = to_px(t, v)
        pts = np.column_stack([u, vv]).astype(np.int32)
        cv2.polylines(img, [pts], False, color, thickness, cv2.LINE_AA)
    for t, v, color, *style in dots:
        radius = style[0] if style else 3
        fill = -1 if len(style) > 1 and style[1] else 1
        u, vv = to_px(t, v)
        for pu, pv, val in zip(u, vv, v):
            if np.isfinite(val):
                cv2.circle(img, (int(pu), int(pv)), radius, color, fill, cv2.LINE_AA)

    cv2.rectangle(img, (left, top), (right, bottom), DIM, 1)

    # legend inside the plot, top-right
    widths = [cv2.getTextSize(label, FONT, 0.4, 1)[0][0] + 26 for label, _ in legend]
    if widths:
        lx = right - 8 - sum(widths)
        cv2.rectangle(img, (lx - 6, top + 4), (right - 4, top + 24), PANEL, -1)
        for (label, color), lw in zip(legend, widths):
            cv2.line(img, (lx, top + 14), (lx + 16, top + 14), color, 2)
            put(img, label, (lx + 20, top + 18), TEXT, 0.4)
            lx += lw


class Dashboard:

    def __init__(self, sim: VirtualTrackingSim):
        self.sim = sim
        self.scale = 180.0  # px per metre in the top view
        self.drag_mode: Optional[str] = None
        self.drag_offset = np.zeros(2)
        self.buttons = {}

    # ---- top view transforms ----------------------------------------------

    def _origin(self):
        x0, y0, w, h = TOP_RECT
        return np.array([x0 + w - 90, y0 + h / 2 + 12])

    def w2p(self, xy) -> Tuple[int, int]:
        o = self._origin()
        return int(o[0] + xy[0] * self.scale), int(o[1] - xy[1] * self.scale)

    def p2w(self, u, v) -> np.ndarray:
        o = self._origin()
        return np.array([(u - o[0]) / self.scale, (o[1] - v) / self.scale])

    def _handle_pos(self, pose) -> np.ndarray:
        return pose[:2] + rot2(pose[2]) @ np.array([self.sim.robot_length / 2 + 0.12, 0.0])

    # ---- drawing ----------------------------------------------------------

    def _draw_robot(self, img, pose, color, thickness=2, handle=True):
        hl, hw = self.sim.robot_length / 2, self.sim.robot_width / 2
        body = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        pts = np.array([self.w2p(pose[:2] + rot2(pose[2]) @ p) for p in body], dtype=np.int32)
        cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA)
        nose = pose[:2] + rot2(pose[2]) @ np.array([hl, 0.0])
        cv2.arrowedLine(img, self.w2p(pose[:2]), self.w2p(nose), color, thickness, cv2.LINE_AA, tipLength=0.25)
        if handle:
            cv2.line(img, self.w2p(nose), self.w2p(self._handle_pos(pose)), color, 1, cv2.LINE_AA)
            cv2.circle(img, self.w2p(self._handle_pos(pose)), 7, color, -1, cv2.LINE_AA)

    def _draw_top_view(self, img, snap):
        sim = self.sim
        x0, y0, w, h = TOP_RECT
        # draw in canvas coordinates on a scratch canvas, then copy only the
        # panel area so nothing spills into neighbouring panels
        target = np.full_like(img, PANEL)

        # grid
        for gx in np.arange(-6.0, 1.01, 0.25):
            u, _ = self.w2p((gx, 0.0))
            cv2.line(target, (u, 0), (u, CANVAS_H), GRID if abs(gx % 1.0) < 1e-6 else (52, 52, 52), 1)
            if abs(gx * 2 - round(gx * 2)) < 1e-6:
                put(target, f'{gx:g}', (u + 2, y0 + h - 12), DIM, 0.35)
        for gy in np.arange(-3.0, 3.01, 0.25):
            _, v = self.w2p((0.0, gy))
            cv2.line(target, (0, v), (CANVAS_W, v), GRID if abs(gy % 1.0) < 1e-6 else (52, 52, 52), 1)
            if abs(gy * 2 - round(gy * 2)) < 1e-6:
                put(target, f'{gy:g}', (x0 + 10, v - 3), DIM, 0.35)

        # stage goals
        for stage in (1, 2):
            gp = sim.goal_pose(stage)
            u, v = self.w2p(gp)
            cv2.drawMarker(target, (u, v), (160, 160, 255), cv2.MARKER_CROSS, 12, 1)
            put(target, f'S{stage} {sim.stage_distances[stage - 1]:.2f}m', (u - 30, v - 10 - 14 * (stage - 1)),
                (160, 160, 255), 0.38)

        # tag
        half = sim.tag_size * 10 / 8 / 2
        cv2.line(target, self.w2p((0.0, -half)), self.w2p((0.0, half)), (255, 255, 255), 6)
        cv2.arrowedLine(target, self.w2p((0.0, 0.0)), self.w2p((-0.15, 0.0)), (255, 255, 255), 1,
                        cv2.LINE_AA, tipLength=0.3)
        put(target, f'tag {sim.tag_id}', (self.w2p((0.0, 0.0))[0] + 8, self.w2p((0.0, 0.0))[1] + 4))

        # trail and initial pose ghost
        if len(snap['trail']) > 1:
            pts = np.array([self.w2p(p) for p in snap['trail']], dtype=np.int32)
            cv2.polylines(target, [pts], False, (180, 120, 255), 1, cv2.LINE_AA)
        if snap['phase'] != 'IDLE':
            self._draw_robot(target, snap['initial_pose'], (90, 90, 90), 1, handle=False)

        # camera FOV
        pose = snap['pose']
        cam_x, cam_y, cam_th = camera_world_pose(pose, sim.camera_offset)
        fov_color = C_CAM if snap['tag_visible'] else DIM
        for side in (-1, 1):
            ang = cam_th + side * sim.hfov / 2
            end = (cam_x + 3.0 * math.cos(ang), cam_y + 3.0 * math.sin(ang))
            cv2.line(target, self.w2p((cam_x, cam_y)), self.w2p(end), fov_color, 1, cv2.LINE_AA)

        running = snap['phase'] in ACTIVE_PHASES
        self._draw_robot(target, pose, C_ROBOT, 2, handle=not running)
        cv2.circle(target, self.w2p((cam_x, cam_y)), 4, C_CAM, -1, cv2.LINE_AA)

        img[y0 + 4:y0 + h - 5, x0 + 4:x0 + w - 5] = target[y0 + 4:y0 + h - 5, x0 + 4:x0 + w - 5]
        put(img, 'Top view (m)  drag AMR = move, drag handle / wheel / a,d = rotate', (x0 + 12, y0 + 22), scale=0.45)
        put(img, f'AMR x={pose[0]:+.3f} y={pose[1]:+.3f} yaw={math.degrees(pose[2]):+.1f}deg',
            (x0 + 12, y0 + 42), C_ROBOT, 0.45)

    def _draw_camera(self, img, snap):
        x0, y0, w, h = IMG_RECT
        cv2.rectangle(img, (x0 + 4, y0 + 4), (x0 + w - 5, y0 + h - 5), PANEL, -1)
        frame = snap['image']
        title = ('marked_image from apriltag_detection' if snap['image_is_marked']
                 else 'synthetic camera image (detection disabled)')
        put(img, title, (x0 + 12, y0 + 22), scale=0.5)
        if frame is None:
            return
        area_w, area_h = w - 24, h - 44
        s = min(area_w / frame.shape[1], area_h / frame.shape[0])
        fw, fh = int(frame.shape[1] * s), int(frame.shape[0] * s)
        resized = cv2.resize(frame, (fw, fh), interpolation=cv2.INTER_AREA)
        ox, oy = x0 + 12 + (area_w - fw) // 2, y0 + 32 + (area_h - fh) // 2
        img[oy:oy + fh, ox:ox + fw] = resized

    def _draw_plots(self, img, snap):
        samples = np.array(snap['samples'], dtype=float).reshape(-1, 11)
        feedback = np.array(snap['feedback'], dtype=float).reshape(-1, 4)
        t_max = max(5.0, snap['elapsed'] * 1.05)
        vlines = [(snap['stage2_t'], 'stage 2')] if snap['stage2_t'] is not None else []

        t = samples[:, 0]
        err_values = np.concatenate([samples[:, 4:7].ravel(), feedback[:, 1:4].ravel()])
        err_values = err_values[np.isfinite(err_values)]
        lim = max(0.1, float(np.max(np.abs(err_values))) * 1.1) if err_values.size else 0.2
        draw_plot(
            img, ERR_RECT, 'Pose error   line = ground truth,  o = controller (action feedback)', t_max, (-lim, lim),
            lines=[(t, samples[:, 4], C_X), (t, samples[:, 5], C_Y), (t, samples[:, 6], C_YAW)],
            dots=[(feedback[:, 0], feedback[:, 1], C_X), (feedback[:, 0], feedback[:, 2], C_Y),
                  (feedback[:, 0], feedback[:, 3], C_YAW)],
            band=(-0.05, 0.05), vlines=vlines,
            legend=[('x_err m', C_X), ('y_err m', C_Y), ('yaw_err rad', C_YAW)],
        )
        cmd_raw = np.array(snap['cmd_raw'], dtype=float).reshape(-1, 4)
        applied = [(t, samples[:, 7 + i], shade(c, 0.55), 4) for i, c in enumerate((C_X, C_Y, C_YAW))]
        raw_lines = [(cmd_raw[:, 0], cmd_raw[:, 1 + i], shade(c, 1.35), 1) for i, c in enumerate((C_X, C_Y, C_YAW))]
        raw_dots = [(cmd_raw[:, 0], cmd_raw[:, 1 + i], shade(c, 1.35), 2, True)
                    for i, c in enumerate((C_X, C_Y, C_YAW))]
        draw_plot(
            img, CMD_RECT, 'cmd_vel_nav   thick = applied by sim (50Hz),  thin+dot = raw from control',
            t_max, (-0.6, 0.6),
            lines=applied + raw_lines, dots=raw_dots,
            hlines=[(0.5, C_LIMIT), (-0.5, C_LIMIT)], vlines=vlines,
            legend=[('vx m/s', C_X), ('vy m/s', C_Y), ('wz rad/s', C_YAW), ('limit', C_LIMIT)],
        )

    def _draw_bar(self, img, snap):
        x0, y0, w, h = BAR_RECT
        cv2.rectangle(img, (x0 + 4, y0 + 4), (x0 + w - 5, y0 + h - 5), PANEL, -1)
        self.buttons = {}
        for i, (name, label, color) in enumerate([('start', 'Start (s)', (60, 140, 60)),
                                                  ('stop', 'Stop (x)', (60, 60, 170)),
                                                  ('reset', 'Reset (r)', (100, 100, 100))]):
            bx, by = x0 + 16 + i * 130, y0 + 16
            cv2.rectangle(img, (bx, by), (bx + 118, by + 40), color, -1)
            put(img, label, (bx + 14, by + 26), TEXT, 0.55)
            self.buttons[name] = (bx, by, bx + 118, by + 40)

        phase = snap['phase']
        phase_color = {'SUCCEEDED': (80, 220, 80), 'ABORTED': (80, 80, 255), 'CANCELED': (80, 180, 255),
                       'REJECTED': (80, 80, 255)}.get(phase, TEXT)
        sx = x0 + 420
        put(img, f'{phase}', (sx, y0 + 34), phase_color, 0.8, 2)
        put(img, f'stage {snap["stage"]}   elapsed {snap["elapsed"]:.2f}s', (sx + 190, y0 + 32), TEXT, 0.55)

        cmd = snap['cmd']
        gt = self.sim.ground_truth_error(snap['pose'], snap['stage'])
        fb = snap['feedback'][-1] if snap['feedback'] else None
        put(img, f'cmd  vx={cmd[0]:+.3f}  vy={cmd[1]:+.3f}  wz={cmd[2]:+.3f}', (x0 + 16, y0 + 80), TEXT, 0.48)
        put(img, f'GT   x={gt.x_error:+.3f}  y={gt.y_error:+.3f}  yaw={gt.yaw_error:+.3f}', (x0 + 16, y0 + 102),
            TEXT, 0.48)
        if fb is not None:
            put(img, f'ctrl x={fb[1]:+.3f}  y={fb[2]:+.3f}  yaw={fb[3]:+.3f}', (x0 + 16, y0 + 122), DIM, 0.48)
        if snap['result']:
            put(img, snap['result'][:150], (sx, y0 + 64), phase_color, 0.42)
        for i, line in enumerate(snap['control_log'][-2:]):
            put(img, f'control: {line[:140]}', (sx, y0 + 88 + 18 * i), DIM, 0.4)

    def render(self) -> np.ndarray:
        snap = self.sim.snapshot()
        img = np.full((CANVAS_H, CANVAS_W, 3), BG, dtype=np.uint8)
        self._draw_top_view(img, snap)
        self._draw_camera(img, snap)
        self._draw_plots(img, snap)
        self._draw_bar(img, snap)
        return img

    # ---- input ------------------------------------------------------------

    def _rotate(self, delta_rad: float):
        pose = self.sim.snapshot()['pose']
        pose[2] = math.atan2(math.sin(pose[2] + delta_rad), math.cos(pose[2] + delta_rad))
        self.sim.set_pose(pose)

    def on_mouse(self, event, u, v, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for name, (bx0, by0, bx1, by1) in self.buttons.items():
                if bx0 <= u <= bx1 and by0 <= v <= by1:
                    getattr(self.sim, name)()
                    return

        x0, y0, w, h = TOP_RECT
        in_top = x0 <= u < x0 + w and y0 <= v < y0 + h
        if self.sim.is_running():
            self.drag_mode = None
            return

        pose = self.sim.snapshot()['pose']
        mouse = self.p2w(u, v)
        if event == cv2.EVENT_LBUTTONDOWN and in_top:
            if np.linalg.norm(mouse - self._handle_pos(pose)) * self.scale < 12:
                self.drag_mode = 'rotate'
            elif np.linalg.norm(rot2(-pose[2]) @ (mouse - pose[:2]) / [self.sim.robot_length / 2,
                                                                       self.sim.robot_width / 2], np.inf) <= 1.0:
                self.drag_mode = 'move'
                self.drag_offset = mouse - pose[:2]
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_mode is not None:
            if self.drag_mode == 'move':
                pose[:2] = mouse - self.drag_offset
            else:
                pose[2] = math.atan2(mouse[1] - pose[1], mouse[0] - pose[0])
            self.sim.set_pose(pose)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag_mode = None
        elif event == cv2.EVENT_MOUSEWHEEL and in_top:
            self._rotate(math.radians(2.0) * (1 if cv2.getMouseWheelDelta(flags) > 0 else -1))

    def on_key(self, key) -> bool:
        """Return False to quit."""
        if key in (ord('q'), 27):
            return False
        if key == ord('s'):
            self.sim.start()
        elif key == ord('x'):
            self.sim.stop()
        elif key == ord('r'):
            self.sim.reset()
        elif key in (ord('a'), ord('d')) and not self.sim.is_running():
            self._rotate(math.radians(2.0 if key == ord('a') else -2.0))
        elif key in (ord('+'), ord('=')):
            self.scale = min(self.scale * 1.2, 1200.0)
        elif key == ord('-'):
            self.scale = max(self.scale / 1.2, 60.0)
        return True


def main(args=None):
    rclpy.init(args=args)
    sim = VirtualTrackingSim()
    executor = MultiThreadedExecutor()
    executor.add_node(sim)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    dashboard = Dashboard(sim)
    try:
        if sim.headless:
            # run one goal (auto_start) and save the dashboard PNG at the end
            while rclpy.ok() and not sim.finished.wait(timeout=0.2):
                pass
            path = sim.take_png_request()
            if path:
                cv2.imwrite(path, dashboard.render())
        else:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, CANVAS_W, CANVAS_H)
            cv2.setMouseCallback(WINDOW_NAME, dashboard.on_mouse)
            while rclpy.ok():
                frame = dashboard.render()
                cv2.imshow(WINDOW_NAME, frame)
                path = sim.take_png_request()
                if path:
                    cv2.imwrite(path, frame)
                key = cv2.waitKey(33) & 0xFF
                if key != 0xFF and not dashboard.on_key(key):
                    break
                # closed with the window's X button; the GTK build in our Docker
                # image does not support this property and always returns -1
                visible = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE)
                if 0 <= visible < 1:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
        cv2.destroyAllWindows()
        executor.shutdown()
        sim.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
