# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The package runs as two nodes under the `up` namespace:

- **`apriltag_detection`** — detects tags and publishes the closest tag's pose. It starts disabled and only subscribes to the camera while enabled.
- **`apriltag_control`** — orchestrator. It owns the `start_tracking` action, closes AMCL and enables detection when a goal starts, runs the planner + LQR, publishes `/cmd_vel` in Stage 1 and `/pre_cmd_vel` (G7+ precision mode) in Stage 2, disables the PLC lidar safety field before Stage 2, and when the goal succeeds, fails, is cancelled or is stopped it disables detection, enables lidar safety and restores AMCL.

```
 client ──start_tracking──► apriltag_control ──SetBool /up/apriltag_detection/enable──► apriltag_detection
                                  ▲                                                        │
                                  │                     camera/camera/color/image_raw ────►│ detect + pose
                                  └──────────── /up/apriltag_pose (PoseStamped) ◄──────────┘ (+ TF, marked_image)
                                  │
   quaternion → R → control error → TrajectoryPlanner → LQR → clamp
                                  │
                                  ▼
      /cmd_vel (Stage 1) / /pre_cmd_vel (Stage 2)
```

Target loss is checked by a timer in `apriltag_control` (detection publishes nothing when no tag is visible).

## G7+ Integration: AMCL And Lidar Safety

`apriltag_control` uses two external G7+ providers at fixed points of a `start_tracking` goal: AMCL through services (checked), lidar safety through a topic, the same way as G7+ AutoCharging (fire-and-forget, no read-back):

```
start=True goal
 ├─ 1. close AMCL: check → close (only if on) → check again      any failure → abort
 ├─ 2. enable detection → Stage 1 (50 cm)
 ├─ 3. Stage 1 converged: stop, publish Bool(true) on /g7_plc/disable_lidar_safety
 │     → Stage 2 (28 cm)
 └─ 4. goal end (succeeded / tag lost / cancelled / start=false / failure / node shutdown):
       stop → disable detection → publish Bool(false) (always) → open AMCL → check
```

- AMCL is only opened again if this node closed it (AMCL that was already off stays off). A failed AMCL restore is logged, retried at the next goal end or at node shutdown, and noted in the result message. It does not change the goal outcome.
- Lidar safety is enabled (`false` published) at every goal end and at node shutdown, whether or not Stage 2 was reached. The PLC bridge gives no reply on this topic, so this node cannot confirm the write; the `lidar_safety=` feedback is the last value published.
- All service calls run in the action execute thread, so `pose_callback` and the target-loss watchdog keep running. Between the stages the robot stays stopped until lidar safety is published.
- Only one `start=True` goal runs at a time. A second one is rejected.
- SIGINT / SIGTERM are handled by the node itself, so the restore calls are still made when it is shut down. SIGKILL cannot be handled.

**Responsibility boundary:** this package only calls the AMCL services (and checks their responses) and publishes the lidar safety flag at the right time. Whether AMCL or the PLC actually behave as requested belongs to the G7+ providers (`dev_amcl`, ros1_bridge, `ads_bridge_node`).

## Features

- Real-time AprilTag detection via `pupil_apriltags`
- Two-stage docking: align at 50 cm (Stage 1), then at 28 cm (Stage 2)
- Discrete-time LQR controller with DARE-based gain computation
- Safety watchdog: safe-stop when the target is lost for too long
- G7+ integration: closes AMCL during the goal and disables the PLC lidar safety field for Stage 2, restoring both on every exit path
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
ros2 launch apriltag april_tag.launch.py
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

`tools/virtual_tracking_sim.py` replaces the RealSense camera and the AMR with a simulation, so the real `apriltag_detection` and `apriltag_control` run unmodified: a virtual tag36h11 tag stands at the world origin, a virtual holonomic AMR integrates `/cmd_vel` and `/pre_cmd_vel`, and a synthetic camera image of the tag is rendered from their relative pose and published on `up/camera/camera/color/image_raw` + `camera_info`.

Run it inside the Docker container (needs `DISPLAY`):

```bash
ros2 launch apriltag test_virtual_tracking.launch.py
```

The stage distances are not set on the sim: it reads `stage1_distance` / `stage2_distance` from `apriltag_control` at startup (`apriltag_control/get_parameters`) and starts no goal before that. Set them with `stage1_distance:=… stage2_distance:=…` on the test launch.

A fake node `fake_g7_services` stands in for AMCL and the PLC lidar safety. It serves the same three AMCL services and subscribes to the lidar safety topic, but only keeps two bool flags (AMCL on/off, lidar safety enabled/disabled), shown in the dashboard. After every goal the sim logs `after goal: amcl=… lidar_safety=… (restored)`, or an error if `apriltag_control` did not restore them.

The whole test runs in `ROS_DOMAIN_ID=65` (launch argument `domain_id`), the same domain as the robot, so **unplug the robot's network cable** before running it. Use the same domain for CLI debugging, e.g. `ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel`.

Dashboard:

| Panel | Content |
|---|---|
| Top view | Tag, AMR, camera FOV (yellow = tag in view), trail, Stage 1 / 2 goal positions. Drag the AMR body to move it, drag the round handle (or mouse wheel, `a` / `d`) to rotate — only while not running |
| Camera | `marked_image` from detection while enabled, otherwise the raw synthetic image |
| Pose error | Ground-truth error (lines) vs. the controller's error parsed from the action feedback (circles), ±0.05 tolerance band, Stage 2 switch time |
| cmd_vel | Thick dark line: the cmd the sim actually applies to the AMR (sampled at 50 Hz, zero after 0.5 s without a message). Thin line + dots: every raw `/cmd_vel` / `/pre_cmd_vel` message from `apriltag_control`. ±0.5 limit |

Keys: `s` start (sends the `start_tracking` goal), `x` stop (cancels the goal), `r` reset to the initial pose, `+` / `-` zoom, `q` quit.

Every finished run is saved to `virtual_tracking_logs/` (relative to the working directory): `<time>_<result>_sim.csv` (50 Hz pose, ground-truth error, cmd), `<time>_<result>_feedback.csv` (controller feedback), `<time>_<result>_cmd_raw.csv` (every raw `/cmd_vel` / `/pre_cmd_vel` message) and a PNG of the dashboard.

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
| `/cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 1 velocity commands (parameter `stage1_cmd_vel_topic`); stops are sent on both topics |
| `/pre_cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 2 velocity commands, G7+ precision mode: `motor_control` holds the wheels until every steering angle is within 5° (parameter `stage2_cmd_vel_topic`) |
| `/g7_plc/disable_lidar_safety` | `std_msgs/Bool` | control | Publish | G7+ PLC lidar safety flag: `true` (disable) before Stage 2, `false` (enable) at every goal end. Subscribed by `ads_bridge_node`; fire-and-forget, no read-back |

## Services

| Service | Type | Node | Description |
|---|---|---|---|
| `/up/apriltag_detection/enable` | `std_srvs/SetBool` | detection | Called by `apriltag_control`; `true` subscribes to the camera, `false` unsubscribes |
| `/check_mcl_if_trigger` | `std_srvs/Trigger` | G7+ AMCL | Called by `apriltag_control`; `success=true` means AMCL is running |
| `/close_amcl`, `/open_amcl` | `std_srvs/Empty` | G7+ AMCL | Called by `apriltag_control` at goal start / end |

## Actions

| Action | Type | Description |
|---|---|---|
| `/up/start_tracking` | `apriltag_interfaces/action/StartTracking` | Served by `apriltag_control`. `start: true` closes AMCL, enables detection, resets controller state and tracks until aligned (succeed) or the tag is lost / a service fails (abort); `start: false` stops the running goal and publishes a zero velocity command. Whenever a goal ends, detection is disabled and lidar safety / AMCL are restored. Feedback: `tracking: x_err=… y_err=… yaw_err=… stage=… amcl=… lidar_safety=…` |

### TF Transforms

Broadcasts `camera_frame → apriltag_<tag_id>` for each detected tag.

## Logging

`apriltag_control` prefixes its logs so they can be filtered, e.g. `grep '\[request\]'`:

| Prefix | When |
|---|---|
| `[action] goal received / accepted`, `[action] cancel requested` | Goal and cancel requests |
| `[request]` / `[response]` | Every service call (content, and response time) |
| `[publish]` | `/cmd_vel` / `/pre_cmd_vel` commands: stops always, control commands at most once per second; every `/g7_plc/disable_lidar_safety` message |
| `[action] feedback` | At most once per second |
| `[action] result` | Goal end with status, success and message |

The once-per-second limit is `LOG_THROTTLE_SEC` in `control_node.py`.

## Configuration

Parameters are hard-coded in `AprilTagDetectionNode.__init__()` (`apriltag/detection_node.py`) and `AprilTagControlNode.__init__()` (`apriltag/control_node.py`). Check the first two against the real robot before running on it:

| Parameter | Value | Where | Description |
|---|---|---|---|
| `tag_size` | `0.0635` m | `detection_node.py` | **Edge of the tag's black square (8 × 8 cells, without the white border). Must match the printed tag** |
| `camera_y_offset` | `0.026` m (= `0.036` mount + `y_offset` `-0.01` trim) | `control_node.py` | Camera lateral offset from the robot centre line, + = left |
| Tag family | `tag36h11` | `detection_node.py` | AprilTag family |
| `stop_x/y/yaw_error_tolerance` | `0.05` m / `0.05` m / `0.05` rad | `control_node.py` | A stage is aligned when all errors stay within these |
| `stop_hold_seconds` | `1.0` s | `control_node.py` | ... for this long |
| `max_vx`, `max_vy` | `0.5` m/s | `control_node.py` | Linear velocity saturation |
| `max_vw` | `0.5` rad/s | `control_node.py` | Angular velocity saturation |
| `max_dt` | `0.2` s | `control_node.py` | Timestep cap (handles frame drops) |
| `lost_target_timeout` | `1.0` s | `control_node.py` | No tag pose for this long → safe stop, goal aborted |
| `smooth_tau` | `0.5` s | `control_node.py` (`TrajectoryPlanner`) | Reference smoothing time constant |
| LQR Q weights | `(1.0, 1.0, 0.5)` | `control.py` default | State cost: x, y, yaw |
| LQR R weights | `(3.0, 1.0, 3.0)` | `control_node.py` | Control cost: vx, vy, vw (vy is penalised least, so lateral correction is fastest) |
| `detection_service_timeout` | `2.0` s | `control_node.py` | Wait for `apriltag_detection/enable` |

ROS parameters of `apriltag_control`:

| Parameter | Default |
|---|---|
| `stage1_distance` | `0.50` m (Stage 1 target distance to the tag) |
| `stage2_distance` | `0.28` m (Stage 2, final target distance to the tag) |
| `manage_amcl_and_lidar_safety` | `true` (`false` skips every AMCL and lidar safety call; tracking is never blocked by them) |
| `amcl_check_service` | `/check_mcl_if_trigger` |
| `amcl_close_service` | `/close_amcl` |
| `amcl_open_service` | `/open_amcl` |
| `lidar_safety_topic` | `/g7_plc/disable_lidar_safety` |
| `stage1_cmd_vel_topic` | `/cmd_vel` (Stage 1 velocity commands) |
| `stage2_cmd_vel_topic` | `/pre_cmd_vel` (Stage 2 velocity commands, G7+ precision mode) |
| `external_service_timeout` | `5.0` s (wait for service + wait for response, each) |

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
│       │   ├── control.py               # TrajectoryPlanner and LQR trackers
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
│       ├── launch/
│       │   ├── april_tag.launch.py                # camera + detection + control (real robot)
│       │   └── test_virtual_tracking.launch.py    # virtual end-to-end test
│       ├── tools/
│       │   ├── print_tag_pose.py
│       │   └── virtual_tracking_sim.py  # virtual camera / AMR / fake G7+ AMCL + lidar safety + dashboard
│       └── test/
└── README.md
```

## Control Architecture

### Docking Stages

1. **Stage 1** — track to `stage1_distance` (0.50 m). When all errors stay within tolerance for `stop_hold_seconds`, the robot stops and the lidar safety field is disabled.
2. **Stage 2** — track to `stage2_distance` (0.28 m). When converged the robot stops and the goal succeeds.

### Trajectory Planner

The reference for x, y and yaw starts at the current error and slides toward zero on all three axes at once, with first-order smoothing (`smooth_tau = 0.5 s`). It is reset at the start of each stage.

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
