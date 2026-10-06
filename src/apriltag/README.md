# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The package runs as two nodes under the `up` namespace:

- **`apriltag_detection`** — runs continuously from startup: every camera image gives one `TagPoseArray` on `/up/apriltag_poses` with every detected tag (family, id, pose) of the families in `tag_families`, empty when none is visible, plus a TF per tag. It does not choose a tag and is not controlled by `apriltag_control`.
- **`apriltag_control`** — orchestrator, run as a state machine. It owns the `start_tracking` and `leave_cs` actions. `start_tracking` enables lidar safety, closes AMCL (Stage 1), runs the planner + LQR on `/cmd_vel`, then disables the PLC lidar safety field (Stage 2) and continues on `/pre_cmd_vel` (G7+ precision mode). When Stage 2 is aligned the node returns to `IDLE` with AMCL closed and lidar safety disabled; then the goal succeeds. `leave_cs` (accepted in `IDLE`, with or without a previous alignment) disables lidar safety and closes AMCL, then backs straight up at `max_vx` on `/pre_cmd_vel` (no planner / LQR) until the tag is `leave_distance` ahead of the camera; when it ends, or whenever a stage fails, is cancelled or is stopped, the node enables lidar safety and makes sure AMCL is open before `IDLE`. It tracks one tag picked from `/up/apriltag_poses`: family `tag_family`, id `tag_id` (`-1`: any id), the closest one (smallest positive forward z) if several match.

```
 camera/camera/color/image_raw ──► apriltag_detection ──► TF <family>_<id>, apriltag/marked_image
                                          │ detect all families, all tags
                                          ▼
                   /up/apriltag_poses (apriltag_interfaces/TagPoseArray, every image)
                                          │
 client ──start_tracking / leave_cs──► apriltag_control
                                          │ pick tag_family / tag_id, closest (smallest z > 0)
   quaternion → R → control error → TrajectoryPlanner → LQR → clamp
                                  │
                                  ▼
      /cmd_vel (Stage 1) / /pre_cmd_vel (Stage 2, leaving)
```

Target loss is checked by a timer in `apriltag_control`: arrays without the tracked tag are ignored, so the tag counts as lost after `lost_target_timeout`.

`TagPoseArray` (`apriltag_interfaces/msg`): `std_msgs/Header header` (image stamp, camera frame) and `TagPose[] tags`; `TagPose`: `string family` (e.g. `tag36h11`), `string id` (decimal, e.g. `"0"`), `geometry_msgs/Pose pose` (camera optical frame, m). Nothing is published before `camera_info` has been received.

## G7+ Integration: AMCL And Lidar Safety

`apriltag_control` uses two external G7+ providers at fixed points of the procedure: AMCL through services (checked), lidar safety through a topic, the same way as G7+ AutoCharging (fire-and-forget, no read-back):

```
start_tracking start=True (only in IDLE)
 ├─ STAGE1 preparing: publish Bool(false) on /g7_plc/disable_lidar_safety (enable),
 │                    close AMCL: check → close (only if on) → check again
 │                                                             any failure → abort, restore
 ├─ STAGE1 running:   track 50 cm on /cmd_vel; converged → stop
 ├─ STAGE2 preparing: publish Bool(true) (disable lidar safety)
 ├─ STAGE2 running:   track 28 cm on /pre_cmd_vel; converged → stop
 └─ STAGE2 ending:    stop → IDLE, goal succeeds
                      (AMCL stays closed, lidar safety stays disabled)
leave_cs start=True (only in IDLE)
 ├─ LEAVING preparing: publish Bool(true) (disable lidar safety), close AMCL (only if on)
 ├─ LEAVING running:   back straight at -max_vx on /pre_cmd_vel until tag forward distance >= leave_distance (1.0 m)
 └─ LEAVING ending:    stop → publish Bool(false) → open AMCL (only if off) → check → IDLE

any other end (tag lost / cancelled / start=false / failure), in any state:
       stop → publish Bool(false) → open AMCL (only if off) → check → IDLE
node shutdown (also in IDLE): stop → publish Bool(false) → open AMCL (only if off) → check
```

- There is no record of who closed AMCL: every restore checks AMCL and opens it if it is off. A failed AMCL restore is logged and noted in the result message; it does not change the goal outcome. The next restore (procedure end or node shutdown) tries again.
- `IDLE` does not tell whether the robot is aligned: after a successful `start_tracking` AMCL stays closed and lidar safety stays disabled until `leave_cs` (or a node shutdown). The `start_tracking` result message says so. A new `start_tracking` from there enables lidar safety again before Stage 1; if the PLC then blocks the motion near the station the goal waits (there is no stage timeout) until it is cancelled.
- Lidar safety is enabled (`false` published) at the start of Stage 1, at every procedure end except a successful Stage 2, and at node shutdown. The PLC bridge gives no reply on this topic, so this node cannot confirm the write; the `lidar_safety=` feedback is the last value published.
- All service calls run in the action execute thread, so `tags_callback` and the target-loss watchdog keep running. While a state is preparing (and between the stages) no velocity is published and the target-loss watchdog is off.
- Which goals are accepted depends on the state (see [State Machine](#state-machine)). Every rejected goal is logged.
- SIGINT / SIGTERM are handled by the node itself, so the restore calls are still made when it is shut down, also from `IDLE` after a successful Stage 2. SIGKILL cannot be handled.

## State Machine

`apriltag_control` keeps its procedure in `TrackingState` (`control_node.py`); every transition is logged as `[state] OLD -> NEW (reason)`.

```
IDLE ──start_tracking(true)──► STAGE1 ──Stage 1 aligned──► STAGE2 ──Stage 2 aligned──► IDLE   (AMCL off, lidar safety disabled)
IDLE ──leave_cs(true)────────► LEAVING ──leave reached──► IDLE                                (AMCL on, lidar safety enabled)

STAGE1 / STAGE2 / LEAVING ──failure / stop / cancel / tag lost / shutdown──► IDLE              (AMCL on, lidar safety enabled)
```

Each non-`IDLE` state runs three steps, logged as `[step] STATE OLD -> NEW (reason)`:

| Step | Meaning |
|---|---|
| `preparing` | Switching the external state this state needs; no velocity, no target-loss check |
| `running` | Control loop active; the target-loss watchdog runs |
| `ending` | Stopped; the goal's execute thread cleans up, then `IDLE`. The goal result is sent after `IDLE`, so the next goal can be sent right away |

| State | Meaning | AMCL | Lidar safety |
|---|---|---|---|
| `IDLE` | Nothing running | as the last procedure left it | as the last procedure left it |
| `STAGE1` | preparing: enable lidar safety, close AMCL; running: track `stage1_distance` on `/cmd_vel` | off | enabled |
| `STAGE2` | preparing: disable lidar safety; running: track `stage2_distance` on `/pre_cmd_vel`; ending (aligned): stop only | off | disabled |
| `LEAVING` | preparing: disable lidar safety, close AMCL; running: back straight at `-max_vx` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (no planner / LQR); ending: enable lidar safety, open AMCL | off → on | disabled → enabled |

`apriltag_detection` runs in every state; outside `running` its poses are ignored.

| State | `start_tracking` true | `start_tracking` false | `leave_cs` true | `leave_cs` false |
|---|---|---|---|---|
| `IDLE` | accept → `STAGE1` | reject | accept → `LEAVING` | reject |
| `STAGE1` / `STAGE2` | reject | accept, stop and restore → `IDLE` | reject | reject |
| `LEAVING` | reject | reject | reject | accept, stop leave and restore → `IDLE` |

An action cancel is always accepted and ends that goal's procedure (CANCELED, restored). A goal stopped by `start: false` ends ABORTED (`Tracking stopped.` / `Leave stopped.`); the `start: false` goal itself succeeds. A rejected goal is logged as `[action] <name> goal rejected: start=… state=… (<rule>)`.

**Responsibility boundary:** this package only calls the AMCL services (and checks their responses) and publishes the lidar safety flag at the right time. Whether AMCL or the PLC actually behave as requested belongs to the G7+ providers (`dev_amcl`, ros1_bridge, `ads_bridge_node`).

## Features

- Real-time AprilTag detection via `pupil_apriltags`, several families at once, all tags published with family and id
- Two-stage docking: align at 50 cm (Stage 1), then at 28 cm (Stage 2), then back to `IDLE` with AMCL closed and lidar safety disabled
- Leaving: `leave_cs` backs straight up until the tag is 1.0 m away, then restores AMCL and lidar safety
- Discrete-time LQR controller with DARE-based gain computation
- Safety watchdog: safe-stop when the target is lost for too long
- G7+ integration: each state switches AMCL / the PLC lidar safety field itself (Stage 1 closes AMCL, Stage 2 disables lidar safety, leaving makes sure both are off before it moves and restores both at the end); every failure path restores both
- Publishes all tag poses, annotated image, and a TF transform per tag
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

Launch the camera and both AprilTag nodes under the `up` namespace:

```bash
ros2 launch apriltag april_tag.launch.py
# other tag / more families:
ros2 launch apriltag april_tag.launch.py tag_family:=tag36h11 tag_id:=3 detect_tag_families:="tag36h11 tag25h9"
```

| Launch argument | Default | Node |
|---|---|---|
| `detect_tag_families` | `tag36h11` | detection (`tag_families`): families to detect, space separated; each family adds one detector run per image |
| `tag_family` / `tag_id` | `tag36h11` / `0` | control: the tag to track (`tag_id:=-1`: any id of `tag_family`) |
| `manage_amcl_and_lidar_safety` | `false` | control |

To run the nodes alone in the same namespace:

```bash
ros2 run apriltag apriltag_detection --ros-args -r __ns:=/up
ros2 run apriltag apriltag_control --ros-args -r __ns:=/up
```

Print `TagPoseArray` from detection at most 5 Hz (header, then family / id / position / orientation of each tag):

```bash
ros2 run apriltag detection_viewer --ros-args -r __ns:=/up
```

Without a namespace, the relative names below lose the `/up` prefix (e.g. `/start_tracking`, `/apriltag_poses`).

Start tracking from another terminal with the action server:

```bash
ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: true}"
```

When it succeeds the node is back in `IDLE` with AMCL closed and lidar safety disabled. Leave the position (back to `leave_distance`, then restore AMCL / lidar safety); also accepted without a previous alignment:

```bash
ros2 action send_goal /up/leave_cs apriltag_interfaces/action/StartTracking "{start: true}"
```

Stop tracking (accepted only in `STAGE1` / `STAGE2`; stops and restores AMCL / lidar safety):

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
ros2 launch apriltag virtual_tracking.launch.py
```

The stage distances are not set on the sim: it reads `stage1_distance` / `stage2_distance` / `leave_distance` from `apriltag_control` at startup (`apriltag_control/get_parameters`) and starts no goal before that. Set them with `stage1_distance:=… stage2_distance:=… leave_distance:=…` on the test launch.

A run is `start_tracking` followed by `leave_cs`. When `start_tracking` succeeds the sim logs `aligned: amcl=off lidar_safety=disabled (held)` (or an error if they were restored too early) and waits in its `ALIGNED` phase (`apriltag_control` is in `IDLE`); Leave (`l`) sends `leave_cs` (headless: sent automatically). The run ends with the `leave_cs` result. Leave without a previous Start runs a leave-only run.

A fake node `fake_g7_services` stands in for AMCL and the PLC lidar safety. It serves the same three AMCL services and subscribes to the lidar safety topic, but only keeps two bool flags (AMCL on/off, lidar safety enabled/disabled), shown in the dashboard. At the end of every run the sim logs `after goal: amcl=… lidar_safety=… (restored)`, or an error if `apriltag_control` did not restore them.

The whole test runs in `ROS_DOMAIN_ID=65` (launch argument `domain_id`), the same domain as the robot, so **unplug the robot's network cable** before running it. Use the same domain for CLI debugging, e.g. `ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel`.

Dashboard:

| Panel | Content |
|---|---|
| Top view | Tag, AMR, camera FOV (yellow = tag in view), trail, Stage 1 / 2 and leave (`L`) goal positions. Drag the AMR body to move it, drag the round handle (or mouse wheel, `a` / `d`) to rotate — only while not running |
| Camera | `marked_image` from detection, or the raw synthetic image while no `marked_image` has arrived yet |
| Pose error | Ground-truth error (lines) vs. the controller's error parsed from the action feedback (circles), ±0.05 tolerance band, Stage 2 and leave start times |
| cmd_vel | Thick dark line: the cmd the sim actually applies to the AMR (sampled at 50 Hz, zero after 0.5 s without a message). Thin line + dots: every raw `/cmd_vel` / `/pre_cmd_vel` message from `apriltag_control`. ±0.5 limit |

Keys: `s` start (sends the `start_tracking` goal), `l` leave (sends `leave_cs`: in `ALIGNED`, or a leave-only run when no run is active), `x` stop (cancels the running goal; nothing to stop in `ALIGNED`), `r` reset to the initial pose, `+` / `-` zoom, `q` quit.

Every finished run is saved to `virtual_tracking_logs/` (relative to the working directory): `<time>_<result>_sim.csv` (50 Hz pose, ground-truth error, cmd), `<time>_<result>_feedback.csv` (controller feedback), `<time>_<result>_cmd_raw.csv` (every raw `/cmd_vel` / `/pre_cmd_vel` message) and a PNG of the dashboard.

Headless (no window, `start_tracking` then `leave_cs`, then exit after saving):

```bash
ros2 launch apriltag virtual_tracking.launch.py headless:=true auto_start:=true \
    init_x:=-1.0 init_y:=0.15 init_yaw_deg:=10.0
```

The sim's camera intrinsics, camera mount offset, tag size and stage distances are ROS parameters of `virtual_tracking_sim`; the sim draws `tag36h11` id `0` (sim parameter `tag_id`), which is the default `tag_family` / `tag_id` of the test launch. `tag_size`, `camera_y_offset` and the stage distances must match the values in `detection_node.py` / `control_node.py`.

## Topics

| Topic | Type | Node | Direction | Description |
|---|---|---|---|---|
| `/up/camera/camera/color/image_raw` | `sensor_msgs/Image` | detection | Subscribe | Raw camera frames |
| `/up/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | detection | Subscribe (until received) | Camera intrinsics |
| `/up/apriltag_poses` | `apriltag_interfaces/TagPoseArray` | detection → control | Publish / Subscribe | All detected tags (family, id, pose in camera optical frame) of one image, stamped with the image time; empty when none is visible |
| `/up/apriltag/marked_image` | `sensor_msgs/Image` | detection | Publish | Annotated image with detections |
| `/cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 1 velocity commands (parameter `stage1_cmd_vel_topic`); stops are sent on both topics |
| `/pre_cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 2 and leaving velocity commands, G7+ precision mode: `motor_control` holds the wheels until every steering angle is within 5° (parameter `stage2_cmd_vel_topic`) |
| `/g7_plc/disable_lidar_safety` | `std_msgs/Bool` | control | Publish | G7+ PLC lidar safety flag: `true` (disable) before Stage 2 and before leaving, `false` (enable) before Stage 1 and at every procedure end (not when Stage 2 succeeds). Subscribed by `ads_bridge_node`; fire-and-forget, no read-back |

## Services

| Service | Type | Node | Description |
|---|---|---|---|
| `/check_mcl_if_trigger` | `std_srvs/Trigger` | G7+ AMCL | Called by `apriltag_control`; `success=true` means AMCL is running |
| `/close_amcl`, `/open_amcl` | `std_srvs/Empty` | G7+ AMCL | Called by `apriltag_control` before Stage 1 and before leaving (close, only if on) and at procedure end / shutdown (open, only if off) |

## Actions

| Action | Type | Description |
|---|---|---|
| `/up/start_tracking` | `apriltag_interfaces/action/StartTracking` | Served by `apriltag_control`. `start: true` (only in `IDLE`) enables lidar safety, closes AMCL and tracks Stage 1 + 2 until aligned (succeed → `IDLE` with AMCL closed, lidar safety disabled) or the tag is lost / a service fails (abort, restored). `start: false` (only in `STAGE1` / `STAGE2`) stops tracking and restores. Feedback: `tracking: x_err=… y_err=… yaw_err=… state=… step=… amcl=… lidar_safety=…` |
| `/up/leave_cs` | `apriltag_interfaces/action/StartTracking` | Served by `apriltag_control`. `start: true` (only in `IDLE`, with or without a previous alignment) disables lidar safety, closes AMCL and backs straight up at `-max_vx` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (succeed) or the tag is lost (abort); either way lidar safety / AMCL are restored, then `IDLE`. `start: false` (only in `LEAVING`) stops leaving. Same feedback as `start_tracking` |

### TF Transforms

`apriltag_detection` broadcasts `camera_frame → <family>_<id>` (e.g. `tag36h11_0`) for each detected tag.

## Logging

Both launch files print log lines without time and without ros2 launch's `[<process>-N]` prefix (`output_format='{line}'`). Every `apriltag_control` message starts with its current state:

```
[INFO] [up.apriltag_control][stage LEAVING] : [publish] /pre_cmd_vel vx=-0.100 vy=+0.000 wz=+0.000
[INFO] [up.apriltag_detection]: tag_families=tag36h11. Detecting continuously; publishing all tags on apriltag_poses.
```

The `[state] OLD -> NEW` line already shows the new state in its prefix.

`apriltag_control` also prefixes its logs so they can be filtered, e.g. `grep '\[request\]'`:

| Prefix | When |
|---|---|
| `[state]` | Every state transition: `[state] OLD -> NEW (reason)` |
| `[step]` | Every step change inside a state: `[step] STATE OLD -> NEW (reason)` |
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
| `tag_families` | `tag36h11` (ROS parameter, launch arg `detect_tag_families:=`) | `detection_node.py` | Families detected, space separated; one `pupil_apriltags` detector per family (one `Detector` only loads one family). `tag_size` is shared by all families |
| `stop_x/y/yaw_error_tolerance` | `0.05` m / `0.05` m / `0.05` rad | `control_node.py` | A stage is aligned when all errors stay within these |
| `stop_hold_seconds` | `1.0` s | `control_node.py` | ... for this long |
| `max_vx`, `max_vy` | `0.5` m/s | `control_node.py` | Linear velocity saturation |
| `max_vw` | `0.5` rad/s | `control_node.py` | Angular velocity saturation |
| `max_dt` | `0.2` s | `control_node.py` | Timestep cap (handles frame drops) |
| `lost_target_timeout` | `1.0` s | `control_node.py` | No tag pose for this long → safe stop, goal aborted |
| `smooth_tau` | `0.5` s | `control_node.py` (`TrajectoryPlanner`) | Reference smoothing time constant |
| LQR Q weights | `(1.0, 1.0, 0.5)` | `control.py` default | State cost: x, y, yaw |
| LQR R weights | `(3.0, 1.0, 3.0)` | `control_node.py` | Control cost: vx, vy, vw (vy is penalised least, so lateral correction is fastest) |

ROS parameters of `apriltag_control`:

| Parameter | Default |
|---|---|
| `tag_family` | `tag36h11` (family of the tracked tag) |
| `tag_id` | `"0"` (string; id of the tracked tag, `"-1"` = any id; the closest match is tracked) |
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
│   │   ├── action/
│   │   │   └── StartTracking.action
│   │   └── msg/
│   │       ├── TagPose.msg              # family, id, pose of one tag
│   │       └── TagPoseArray.msg         # all tags of one image
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
│       │       └── target_flow.py       # Tracked-tag selection (family / id / closest)
│       ├── launch/
│       │   ├── april_tag.launch.py                # camera + detection + control (real robot)
│       │   └── virtual_tracking.launch.py         # virtual end-to-end test
│       ├── tools/
│       │   ├── detection_viewer.py      # prints /up/apriltag_poses
│       │   └── virtual_tracking_sim.py  # virtual camera / AMR / fake G7+ AMCL + lidar safety + dashboard
│       └── test/
│           ├── test_state_machine.py    # TrackingState transition table and goal rules
│           ├── test_leave_step.py       # leave step
│           └── test_target_flow.py      # tracked-tag selection
└── README.md
```

## Control Architecture

### Docking Stages

1. **Stage 1** — track to `stage1_distance` (0.50 m). When all errors stay within tolerance for `stop_hold_seconds`, the robot stops and Stage 2 disables the lidar safety field before it moves.
2. **Stage 2** — track to `stage2_distance` (0.28 m). When converged the robot stops and the node returns to `IDLE` (AMCL closed, lidar safety disabled); then the goal succeeds.
3. **Leave** (`leave_cs`) — no planner / LQR: every tag pose publishes `vx = -max_vx, vy = 0, wz = 0` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (1.0 m); y / yaw are not checked and there is no hold time. Then the robot stops (`Leave reached 1.00m (forward=…m).`) and AMCL / lidar safety are restored. Losing the tag for `lost_target_timeout` stops the robot and aborts the leave.

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
