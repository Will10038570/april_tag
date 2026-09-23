# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The codebase is now organized as a standard ROS 2 `ament_python` package. Most application source lives under `src/apriltag/apriltag/`, with `apriltag.main` registered as the executable entry point.

```
Camera frame arrives
       │
       ▼
 Image conversion              [apriltag/ros/ros_io.py]
       │
       ▼
 AprilTag Detection            [apriltag/perception/tag_perception.py]
       │  → publishes annotated image to /apriltag/marked_image
       ▼
 Target Selection & Control    [apriltag/runtime/target_flow.py]
   ├── TrajectoryPlanner       → align yaw first, then translate
   ├── LQRTracker              → optimal velocity commands (vx, vy, vw)
   └── Safety Watchdog         → safe-stop on tag loss
       │
       ▼
 /cmd_vel  /apriltag_pose  /apriltag_trajectory  TF
```

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

Run the node:

```bash
ros2 run apriltag apriltag_node
```

Start tracking from another terminal with the action server:

```bash
ros2 action send_goal /start_tracking apriltag_interfaces/action/StartTracking "{start: true}"
```

Stop tracking and publish a zero velocity command:

```bash
ros2 action send_goal /start_tracking apriltag_interfaces/action/StartTracking "{start: false}"
```

Run the controller demo module without ROS:

```bash
python -m apriltag.control
```

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Subscribe | Raw camera frames |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Subscribe | Camera intrinsics |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Velocity commands |
| `/apriltag_pose` | `geometry_msgs/PoseStamped` | Publish | Tag pose in camera frame |
| `/apriltag_trajectory` | `nav_msgs/Path` | Publish | Two-point path (origin → tag) |
| `/apriltag/marked_image` | `sensor_msgs/Image` | Publish | Annotated image with detections |

## Actions

| Action | Type | Description |
|---|---|---|
| `/start_tracking` | `apriltag_interfaces/action/StartTracking` | Starts tracking and resets controller state, equivalent to pressing Enter in the node terminal |

### TF Transforms

Broadcasts `camera_frame → apriltag_<tag_id>` for each detected tag.

## Configuration

All parameters are set in `AprilTagRosNode.__init__()` in `src/apriltag/apriltag/main.py`:

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
│       │   ├── main.py                  # ROS 2 node entry point
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
│       │       └── target_flow.py       # Target dispatch and orchestration
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
