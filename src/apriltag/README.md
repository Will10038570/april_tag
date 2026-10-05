# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The package runs as two nodes under the `up` namespace:

- **`apriltag_detection`** — detects tags and publishes the closest tag's pose. It starts disabled and only subscribes to the camera while enabled.
- **`apriltag_control`** — orchestrator, run as a state machine. It owns the `start_tracking` and `leave_cs` actions. `start_tracking` closes AMCL and enables detection, runs the planner + LQR, publishes `/cmd_vel` in Stage 1 and `/pre_cmd_vel` (G7+ precision mode) in Stage 2 and disables the PLC lidar safety field before Stage 2. When Stage 2 is aligned the goal succeeds and the node holds in `IN_POSITION` with AMCL closed, lidar safety disabled and detection enabled. `leave_cs` then backs straight up at `max_vx` on `/pre_cmd_vel` (no planner / LQR) until the tag is `leave_distance` ahead of the camera; when it ends, or whenever a stage fails, is cancelled or is stopped, the node disables detection, enables lidar safety and restores AMCL.

```
 client ──start_tracking──► apriltag_control ──SetBool /up/apriltag_detection/enable──► apriltag_detection
                                  ▲                                                        │
                                  │                     camera/camera/color/image_raw ────►│ detect + pose
                                  └──────────── /up/apriltag_pose (PoseStamped) ◄──────────┘ (+ TF, marked_image)
                                  │
   quaternion → R → control error → TrajectoryPlanner → LQR → clamp
                                  │
                                  ▼
      /cmd_vel (Stage 1) / /pre_cmd_vel (Stage 2, leaving)
```

Target loss is checked by a timer in `apriltag_control` (detection publishes nothing when no tag is visible).

## G7+ Integration: AMCL And Lidar Safety

`apriltag_control` uses two external G7+ providers at fixed points of the procedure: AMCL through services (checked), lidar safety through a topic, the same way as G7+ AutoCharging (fire-and-forget, no read-back):

```
start_tracking start=True
 ├─ 1. close AMCL: check → close (only if on) → check again      any failure → abort
 ├─ 2. enable detection → Stage 1 (50 cm)
 ├─ 3. Stage 1 converged: stop, publish Bool(true) on /g7_plc/disable_lidar_safety
 │     → Stage 2 (28 cm)
 └─ 4. Stage 2 converged: stop, goal succeeds → IN_POSITION
       (AMCL stays closed, lidar safety stays disabled, detection stays enabled)
leave_cs start=True (only in IN_POSITION)
 └─ 5. back straight at -max_vx on /pre_cmd_vel until tag forward distance >= leave_distance (1.0 m)

procedure end (leave ended / tag lost / cancelled / start=false / failure / node shutdown):
       stop → disable detection → publish Bool(false) (always) → open AMCL → check
       (node shutdown: stop → publish Bool(false) → open AMCL → check → disable detection)
```

- AMCL is only opened again if this node closed it (AMCL that was already off stays off). A failed AMCL restore is logged, retried at the next procedure end or at node shutdown, and noted in the result message. It does not change the goal outcome.
- Lidar safety is enabled (`false` published) at every procedure end and at node shutdown, whether or not Stage 2 was reached. A successful Stage 2 is **not** a procedure end: AMCL and lidar safety are restored only after `leave_cs`, after `start_tracking` `start: false` in `IN_POSITION`, or at node shutdown. `IN_POSITION` has no timeout. The PLC bridge gives no reply on this topic, so this node cannot confirm the write; the `lidar_safety=` feedback is the last value published.
- All service calls run in the action execute thread, so `pose_callback` and the target-loss watchdog keep running. Between the stages the robot stays stopped until lidar safety is published.
- Which goals are accepted depends on the state (see [State Machine](#state-machine)). Every rejected goal is logged.
- SIGINT / SIGTERM are handled by the node itself, so the restore calls are still made when it is shut down, also from `IN_POSITION`. At shutdown AMCL / lidar safety are restored before detection is disabled, since detection may already be shutting down. SIGKILL cannot be handled.

## State Machine

`apriltag_control` keeps its procedure in `TrackingState` (`control_node.py`); every transition is logged as `[state] OLD -> NEW (reason)`.

```
IDLE ──start_tracking(true)──► STARTING ──AMCL closed + detection enabled──► STAGE1 ──► STAGE2 ──aligned──► IN_POSITION
 ▲                                │                                            │          │                │  │
 │                                └──────────── failure / stop / cancel ───────┴──────────┘                │  │ leave_cs(true)
 │                                                      │                                                  │  ▼
 │                                                      ▼                        start_tracking(false)     │ LEAVING
 └───────────── cleanup ◄─────────────────────── FINISHING ◄──────────────────── / shutdown ──────────────┘  │
                                                        ◄─────── aligned / failure / cancel / stop ─────────────┘
```

| State | Meaning |
|---|---|
| `IDLE` | Nothing running; AMCL / lidar safety restored, detection disabled |
| `STARTING` | `start_tracking` accepted: closing AMCL, enabling detection |
| `STAGE1` | Tracking to `stage1_distance` on `/cmd_vel`. After it converges the robot stays stopped (still `STAGE1`) until lidar safety is disabled |
| `STAGE2` | Tracking to `stage2_distance` on `/pre_cmd_vel` |
| `IN_POSITION` | Stage 2 aligned, stopped and holding; no velocity is published |
| `LEAVING` | `leave_cs`: backing straight at `-max_vx` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (no planner / LQR) |
| `FINISHING` | Procedure ended: stop, disable detection, restore AMCL / lidar safety, then `IDLE` |

Velocity commands are only published in `STAGE1`, `STAGE2` and `LEAVING`, and the target-loss watchdog only runs there.

| State | `start_tracking` true | `start_tracking` false | `leave_cs` true | `leave_cs` false |
|---|---|---|---|---|
| `IDLE` | accept → `STARTING` | accept, publish stop | reject | reject |
| `STARTING` / `STAGE1` / `STAGE2` | reject | accept, stop → `FINISHING` → `IDLE` | reject | reject |
| `IN_POSITION` | reject | accept, abandon → `FINISHING` → `IDLE` | accept → `LEAVING` | reject |
| `LEAVING` | reject | accept, stop leave → `FINISHING` → `IDLE` | reject | accept, stop leave → `FINISHING` → `IDLE` |
| `FINISHING` | reject | accept, wait for the cleanup | reject | reject |

A goal stopped by `start: false` ends ABORTED (`Tracking stopped.` / `Leave stopped.`); the `start: false` goal itself succeeds. A rejected goal is logged as `[action] <name> goal rejected: start=… state=… (<rule>)`.

**Responsibility boundary:** this package only calls the AMCL services (and checks their responses) and publishes the lidar safety flag at the right time. Whether AMCL or the PLC actually behave as requested belongs to the G7+ providers (`dev_amcl`, ros1_bridge, `ads_bridge_node`).

## Features

- Real-time AprilTag detection via `pupil_apriltags`
- Two-stage docking: align at 50 cm (Stage 1), then at 28 cm (Stage 2), then hold in `IN_POSITION`
- Leaving: `leave_cs` backs straight up until the tag is 1.0 m away, then restores AMCL and lidar safety
- Discrete-time LQR controller with DARE-based gain computation
- Safety watchdog: safe-stop when the target is lost for too long
- G7+ integration: closes AMCL during the procedure and disables the PLC lidar safety field from Stage 2 until leaving ends, restoring both on every exit path
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

When it succeeds the robot holds in `IN_POSITION`. Leave the position (back to `leave_distance`, then restore AMCL / lidar safety):

```bash
ros2 action send_goal /up/leave_cs apriltag_interfaces/action/StartTracking "{start: true}"
```

Stop whatever is running (tracking or leaving) and publish a zero velocity command; in `IN_POSITION` this abandons the alignment and restores AMCL / lidar safety:

```bash
ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: false}"
```

Stop leaving only (accepted only in `LEAVING`):

```bash
ros2 action send_goal /up/leave_cs apriltag_interfaces/action/StartTracking "{start: false}"
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

The stage distances are not set on the sim: it reads `stage1_distance` / `stage2_distance` / `leave_distance` from `apriltag_control` at startup (`apriltag_control/get_parameters`) and starts no goal before that. Set them with `stage1_distance:=… stage2_distance:=… leave_distance:=…` on the test launch.

A run is `start_tracking` followed by `leave_cs`. When `start_tracking` succeeds the sim logs `in position: amcl=off lidar_safety=disabled (held)` (or an error if they were restored too early) and waits in `IN_POSITION`; Leave (`l`) sends `leave_cs` (headless: sent automatically). The run ends with the `leave_cs` result, or as `ABANDONED` when Stop is pressed in `IN_POSITION`.

A fake node `fake_g7_services` stands in for AMCL and the PLC lidar safety. It serves the same three AMCL services and subscribes to the lidar safety topic, but only keeps two bool flags (AMCL on/off, lidar safety enabled/disabled), shown in the dashboard. At the end of every run the sim logs `after goal: amcl=… lidar_safety=… (restored)`, or an error if `apriltag_control` did not restore them.

The whole test runs in `ROS_DOMAIN_ID=65` (launch argument `domain_id`), the same domain as the robot, so **unplug the robot's network cable** before running it. Use the same domain for CLI debugging, e.g. `ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel`.

Dashboard:

| Panel | Content |
|---|---|
| Top view | Tag, AMR, camera FOV (yellow = tag in view), trail, Stage 1 / 2 and leave (`L`) goal positions. Drag the AMR body to move it, drag the round handle (or mouse wheel, `a` / `d`) to rotate — only while not running |
| Camera | `marked_image` from detection while enabled, otherwise the raw synthetic image |
| Pose error | Ground-truth error (lines) vs. the controller's error parsed from the action feedback (circles), ±0.05 tolerance band, Stage 2 and leave start times |
| cmd_vel | Thick dark line: the cmd the sim actually applies to the AMR (sampled at 50 Hz, zero after 0.5 s without a message). Thin line + dots: every raw `/cmd_vel` / `/pre_cmd_vel` message from `apriltag_control`. ±0.5 limit |

Keys: `s` start (sends the `start_tracking` goal), `l` leave (sends `leave_cs`, only in `IN_POSITION`), `x` stop (cancels the goal; in `IN_POSITION` sends `start_tracking` `start: false`), `r` reset to the initial pose (abandons `IN_POSITION` first), `+` / `-` zoom, `q` quit.

Every finished run is saved to `virtual_tracking_logs/` (relative to the working directory): `<time>_<result>_sim.csv` (50 Hz pose, ground-truth error, cmd), `<time>_<result>_feedback.csv` (controller feedback), `<time>_<result>_cmd_raw.csv` (every raw `/cmd_vel` / `/pre_cmd_vel` message) and a PNG of the dashboard.

Headless (no window, `start_tracking` then `leave_cs`, then exit after saving):

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
| `/pre_cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 2 and leaving velocity commands, G7+ precision mode: `motor_control` holds the wheels until every steering angle is within 5° (parameter `stage2_cmd_vel_topic`) |
| `/g7_plc/disable_lidar_safety` | `std_msgs/Bool` | control | Publish | G7+ PLC lidar safety flag: `true` (disable) before Stage 2, `false` (enable) at every procedure end (not when Stage 2 succeeds). Subscribed by `ads_bridge_node`; fire-and-forget, no read-back |

## Services

| Service | Type | Node | Description |
|---|---|---|---|
| `/up/apriltag_detection/enable` | `std_srvs/SetBool` | detection | Called by `apriltag_control`; `true` subscribes to the camera, `false` unsubscribes |
| `/check_mcl_if_trigger` | `std_srvs/Trigger` | G7+ AMCL | Called by `apriltag_control`; `success=true` means AMCL is running |
| `/close_amcl`, `/open_amcl` | `std_srvs/Empty` | G7+ AMCL | Called by `apriltag_control` at procedure start / end |

## Actions

| Action | Type | Description |
|---|---|---|
| `/up/start_tracking` | `apriltag_interfaces/action/StartTracking` | Served by `apriltag_control`. `start: true` (only in `IDLE`) closes AMCL, enables detection, resets controller state and tracks Stage 1 + 2 until aligned (succeed → `IN_POSITION`, nothing restored) or the tag is lost / a service fails (abort, restored). `start: false` stops tracking or leaving, abandons `IN_POSITION`, and publishes a zero velocity command. Feedback: `tracking: x_err=… y_err=… yaw_err=… state=… amcl=… lidar_safety=…` |
| `/up/leave_cs` | `apriltag_interfaces/action/StartTracking` | Served by `apriltag_control`. `start: true` (only in `IN_POSITION`) backs straight up at `-max_vx` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (succeed) or the tag is lost (abort); either way detection is disabled and lidar safety / AMCL are restored, then `IDLE`. `start: false` (only in `LEAVING`) stops leaving. Same feedback as `start_tracking` |

### TF Transforms

Broadcasts `camera_frame → apriltag_<tag_id>` for each detected tag.

## Logging

Both launch files print log lines without time and without ros2 launch's `[<process>-N]` prefix (`output_format='{line}'`). Every `apriltag_control` message starts with its current state:

```
[INFO] [up.apriltag_control][stage LEAVING] : [publish] /pre_cmd_vel vx=-0.100 vy=+0.000 wz=+0.000
[INFO] [up.apriltag_detection]: Detection is disabled. Waiting for apriltag_control to enable it.
```

The `[state] OLD -> NEW` line already shows the new state in its prefix.

`apriltag_control` also prefixes its logs so they can be filtered, e.g. `grep '\[request\]'`:

| Prefix | When |
|---|---|
| `[state]` | Every state transition: `[state] OLD -> NEW (reason)` |
| `[action] <name> goal received / accepted / rejected`, `[action] cancel requested` | Goal and cancel requests; a rejection names the state and the rule |
| `[request]` / `[response]` | Every service call (content, and response time) |
| `[publish]` | `/cmd_vel` / `/pre_cmd_vel` commands: stops always, control commands at most once per second; every `/g7_plc/disable_lidar_safety` message |
| `[action] feedback` | At most once per second |
| `[action] <name> result` | Goal end with status, success and message |

The once-per-second limit is `LOG_THROTTLE_SEC` in `control_node.py`.

## Configuration

Parameters are hard-coded in `AprilTagDetectionNode.__init__()` (`apriltag/detection_node.py`) and `AprilTagControlNode.__init__()` (`apriltag/control_node.py`). Check the first two against the real robot before running on it:

| Parameter | Value | Where | Description |
|---|---|---|---|
| `tag_size` | `0.0635` m | `detection_node.py` | **Edge of the tag's black square (8 × 8 cells, without the white border). Must match the printed tag** |
| `camera_y_offset` | `0.026` m (= `0.036` mount + `y_offset` `-0.01` trim) | `control_node.py` | Camera lateral offset from the robot centre line, + = left |
| `tag_family` | `tag36h11` (ROS parameter, launch arg `tag_family:=` in `april_tag.launch.py`) | `detection_node.py` | AprilTag family of the printed tag. The virtual test sim always draws `tag36h11` |
| `tag_id` | `-1` (ROS parameter, not in the launch files) | `detection_node.py` | Only this tag ID is tracked; `-1` tracks the closest tag of any ID |
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
| `leave_distance` | `1.0` m (`leave_cs` stops once the camera-to-tag forward distance reaches it) |
| `manage_amcl_and_lidar_safety` | `true` (`false` skips every AMCL and lidar safety call; tracking is never blocked by them) |
| `amcl_check_service` | `/check_mcl_if_trigger` |
| `amcl_close_service` | `/close_amcl` |
| `amcl_open_service` | `/open_amcl` |
| `lidar_safety_topic` | `/g7_plc/disable_lidar_safety` |
| `stage1_cmd_vel_topic` | `/cmd_vel` (Stage 1 velocity commands) |
| `stage2_cmd_vel_topic` | `/pre_cmd_vel` (Stage 2 and leaving velocity commands, G7+ precision mode) |
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
│           └── test_state_machine.py    # TrackingState transition table
└── README.md
```

## Control Architecture

### Docking Stages

1. **Stage 1** — track to `stage1_distance` (0.50 m). When all errors stay within tolerance for `stop_hold_seconds`, the robot stops and the lidar safety field is disabled.
2. **Stage 2** — track to `stage2_distance` (0.28 m). When converged the robot stops, the goal succeeds and the node holds in `IN_POSITION` (AMCL closed, lidar safety disabled).
3. **Leave** (`leave_cs`) — no planner / LQR: every tag pose publishes `vx = -max_vx, vy = 0, wz = 0` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (1.0 m); y / yaw are not checked and there is no hold time. Then the robot stops (`Leave reached 1.00m (forward=…m).`), detection is disabled and AMCL / lidar safety are restored. Losing the tag for `lost_target_timeout` stops the robot and aborts the leave.

### Trajectory Planner

The reference for x, y and yaw starts at the current error and slides toward zero on all three axes at once, with first-order smoothing (`smooth_tau = 0.5 s`). It is reset at the start of each stage. Leaving does not use it.

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
