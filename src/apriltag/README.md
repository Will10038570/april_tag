# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The package runs as two nodes under the `up` namespace:

- **`apriltag_detection`** — detects tags and publishes the closest tag's pose. It starts disabled and only subscribes to the camera while enabled.
- **`apriltag_control`** — orchestrator. It owns the `start_tracking` action, enables detection when a goal starts, runs the planner + LQR, publishes `/cmd_vel_nav`, and disables detection again when the goal succeeds, fails, is cancelled or is stopped.

```
 client ──start_tracking──► apriltag_control ──SetBool /up/apriltag_detection/enable──► apriltag_detection
                                  ▲                                                        │
                                  │                     camera/camera/color/image_raw ────►│ detect + pose
                                  └──────────── /up/apriltag_pose (PoseStamped) ◄──────────┘ (+ TF, marked_image)
                                  │
   quaternion → R → control error → TrajectoryPlanner → LQR → clamp
                                  │
                                  ▼
                   /cmd_vel_nav  /up/apriltag_trajectory
```

Target loss is checked by a timer in `apriltag_control` (detection publishes nothing when no tag is visible).

## Features

- Real-time AprilTag detection via `pupil_apriltags`
- Two-stage trajectory planning: yaw alignment → forward translation
- Discrete-time LQR controller with DARE-based gain computation
- Safety watchdog: safe-stop when the target is lost for too long
- Publishes pose, trajectory path, annotated image, and TF transform
- Clear separation between control, perception, domain types, ROS I/O, and runtime orchestration

## Dependencies

ROS 2 dependencies declared in `package.xml`:

- `rclpy`
- `tf2_ros`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `std_srvs`

Python dependencies used by the package:

- `pupil_apriltags`
- `opencv-python`
- `numpy`
- `scipy`

Install Python prerequisites into the same environment used by ROS 2:

```bash
pip install pupil-apriltags opencv-python numpy scipy
```

## Build And Run

Requires ROS 2 to be sourced and a camera publishing on the default topics.

Build the workspace:

```bash
colcon build --symlink-install 
```

Source the workspace overlay:

```bash
source install/setup.bash
```

Launch the camera, both AprilTag nodes, and the pose printer under the `up` namespace:

```bash
ros2 launch apriltag test_april_tag_pose.launch.py
```

To run the nodes alone in the same namespace:

```bash
ros2 run apriltag apriltag_detection --ros-args -r __ns:=/up
ros2 run apriltag apriltag_control --ros-args -r __ns:=/up
```

Without a namespace, the relative names below lose the `/up` prefix (e.g. `/start_tracking`, `/apriltag_pose`).

Start tracking from another terminal with the action server:

```bash
ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: true}"
```

Stop tracking and publish a zero velocity command:

```bash
ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: false}"
```

Run the controller demo module without ROS:

```bash
python -m apriltag.control
```

## Virtual End-to-End Test

`tools/virtual_tracking_sim.py` replaces the RealSense camera and the AMR with a simulation, so the real `apriltag_detection` and `apriltag_control` run unmodified: a virtual tag36h11 tag stands at the world origin, a virtual holonomic AMR integrates `/cmd_vel_nav`, and a synthetic camera image of the tag is rendered from their relative pose and published on `up/camera/camera/color/image_raw` + `camera_info`.

Run it inside the Docker container (needs `DISPLAY`):

```bash
ros2 launch apriltag test_virtual_tracking.launch.py
```

The whole test runs in `ROS_DOMAIN_ID=99` (launch argument `domain_id`) so a real AMR never receives the simulated `/cmd_vel_nav`. Use the same domain for CLI debugging, e.g. `ROS_DOMAIN_ID=99 ros2 topic echo /cmd_vel_nav`.

Dashboard:

| Panel | Content |
|---|---|
| Top view | Tag, AMR, camera FOV (yellow = tag in view), trail, Stage 1 / 2 goal positions. Drag the AMR body to move it, drag the round handle (or mouse wheel, `a` / `d`) to rotate — only while not running |
| Camera | `marked_image` from detection while enabled, otherwise the raw synthetic image |
| Pose error | Ground-truth error (lines) vs. the controller's error parsed from the action feedback (circles), ±0.05 tolerance band, Stage 2 switch time |
| cmd_vel_nav | Thick dark line: the cmd the sim actually applies to the AMR (sampled at 50 Hz, zero after 0.5 s without a message). Thin line + dots: every raw `/cmd_vel_nav` message from `apriltag_control`. ±0.5 limit |

Keys: `s` start (sends the `start_tracking` goal), `x` stop (cancels the goal), `r` reset to the initial pose, `+` / `-` zoom, `q` quit.

Every finished run is saved to `virtual_tracking_logs/` (relative to the working directory): `<time>_<result>_sim.csv` (50 Hz pose, ground-truth error, cmd), `<time>_<result>_feedback.csv` (controller feedback), `<time>_<result>_cmd_raw.csv` (every raw `/cmd_vel_nav` message) and a PNG of the dashboard.

Headless (no window, one goal, then exit after saving):

```bash
ros2 launch apriltag test_virtual_tracking.launch.py headless:=true auto_start:=true \
    init_x:=-1.0 init_y:=0.15 init_yaw_deg:=10.0
```

The sim's camera intrinsics, camera mount offset, tag size and stage distances are ROS parameters of `virtual_tracking_sim`; `tag_size`, `camera_y_offset` and the stage distances must match the values in `detection_node.py` / `control_node.py`.

## Topics

| Topic | Type | Node | Direction | Description |
|---|---|---|---|---|
| `/up/camera/camera/color/image_raw` | `sensor_msgs/Image` | detection | Subscribe (only while enabled) | Raw camera frames |
| `/up/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | detection | Subscribe (until received) | Camera intrinsics |
| `/up/apriltag_pose` | `geometry_msgs/PoseStamped` | detection → control | Publish / Subscribe | Closest tag pose in camera optical frame, stamped with the image time |
| `/up/apriltag/marked_image` | `sensor_msgs/Image` | detection | Publish | Annotated image with detections |
| `/up/apriltag_trajectory` | `nav_msgs/Path` | control | Publish | Two-point path (origin → tag) |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | control | Publish | Velocity commands |

## Services

| Service | Type | Node | Description |
|---|---|---|---|
| `/up/apriltag_detection/enable` | `std_srvs/SetBool` | detection | Called by `apriltag_control`; `true` subscribes to the camera, `false` unsubscribes |

## Actions

| Action | Type | Description |
|---|---|---|
| `/up/start_tracking` | `apriltag_interfaces/action/StartTracking` | Served by `apriltag_control`. `start: true` enables detection, resets controller state and tracks until aligned (succeed) or the tag is lost (abort); `start: false` pauses and publishes a zero velocity command. Detection is disabled whenever a goal ends |

### TF Transforms

Broadcasts `camera_frame → apriltag_<tag_id>` for each detected tag.

## Configuration

Parameters are set in `AprilTagDetectionNode.__init__()` (`apriltag/detection_node.py`: `tag_size`, tag family) and `AprilTagControlNode.__init__()` (`apriltag/control_node.py`: everything else):

| Parameter | Default | Description |
|---|---|---|
| `tag_size` | `0.019` m | **Must match the physical tag size** |
| `desired_distance` | `0.02` m | Target standoff distance from the tag |
| `max_vx`, `max_vy` | `1.0` m/s | Linear velocity saturation |
| `max_vw` | `2.0` rad/s | Angular velocity saturation |
| `max_dt` | `0.2` s | Timestep cap (handles frame drops) |
| `lost_target_timeout` | `0.6` s | Time before safe-stop on tag loss |
| Tag family | `tag36h11` | AprilTag family |
| LQR Q weights | `(8.0, 8.0, 5.0)` | State cost: x, y, yaw |
| LQR R weights | `(1.0, 1.0, 0.6)` | Control cost: vx, vy, vw |
| `yaw_align_threshold` | `0.12` rad (~7°) | Threshold to switch from align to translate stage |
| `smooth_tau` | `0.25` s | Reference trajectory smoothing time constant |

## Project Structure

```
apriltag_ws/
├── src/
│   ├── apriltag_interfaces/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── action/
│   │       └── StartTracking.action
│   └── apriltag/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── apriltag
│       ├── apriltag/
│       │   ├── __init__.py
│       │   ├── detection_node.py        # apriltag_detection node
│       │   ├── control_node.py          # apriltag_control node (orchestrator)
│       │   ├── control.py               # PID, LQR, and TrajectoryPlanner
│       │   ├── domain/
│       │   │   ├── app_types.py         # Shared dataclasses
│       │   │   └── math_utils.py        # Math utilities and frame conversions
│       │   ├── perception/
│       │   │   └── tag_perception.py    # AprilTag detection and target extraction
│       │   ├── ros/
│       │   │   └── ros_io.py            # ROS message conversion and publish helpers
│       │   └── runtime/
│       │       ├── control_flow.py      # Control pipeline steps
│       │       ├── safety_guard.py      # Target-loss watchdog logic
│       │       └── target_flow.py       # Target selection
│       └── test/
└── README.md
```

## Control Architecture

### Trajectory Planner (two stages)

1. **`align`** — Holds position reference fixed and drives yaw error to zero. Transitions to `translate` when `|yaw_error| ≤ 0.12 rad`.
2. **`translate`** — Drives x/y/yaw errors to zero. Reverts to `align` if yaw drifts beyond `1.5 × threshold`.

First-order smoothing (`smooth_tau = 0.25 s`) is applied to reference transitions.

### LQR Controller

State: $e = [x_{err},\ y_{err},\ \psi_{err}]^\top$, Control: $u = [v_x,\ v_y,\ v_\omega]^\top$

Error dynamics model:

$$e[k+1] = e[k] - \Delta t \cdot u[k] \quad \Rightarrow \quad A = I,\quad B = -\Delta t \cdot I$$

The optimal gain $K$ is computed once by solving the **Discrete Algebraic Riccati Equation (DARE)**:

$$u[k] = -K\,(e[k] - e_{ref}[k])$$

### Coordinate Frame Convention

- **Optical frame**: x → right, y → down, z → forward (camera)
- **Control frame** (`base_link`): x → forward, y → left

The conversion is:

$$x_{err} = z_{opt} - d_{desired}, \quad y_{err} = -x_{opt} + y_{cam}$$

Here $y_{cam}$ is the camera lateral offset in the robot control frame. Positive values mean the camera is mounted on the robot's left side; negative values mean it is mounted on the right side.

Yaw error is derived from the detected pose rotation matrix instead of the translation vector. Using the tag pose $R$, the controller extracts the heading misalignment around the robot yaw axis as:

$$\psi_{err} = \mathrm{atan2}\left(-R_{0,2},\ R_{2,2}\right)$$
