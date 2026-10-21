# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The package runs as two nodes under the `wheelchair` namespace (the RealSense camera too):

- **`apriltag_detection`** — runs continuously from startup: every camera image gives one `TagPoseArray` on `/wheelchair/apriltag_poses` with every detected tag (family, id, pose) of the families in `tag_families`, empty when none is visible, plus a TF per tag, and one `TagTracing` on `/wheelchair/tag_tracing` with the tracked tag: the closest one (smallest positive forward z) of `tracking_tag_family` / `tracking_tag_id` (`-1`: any id), `valid=false` when there is none. It is not controlled by `apriltag_control`.
- **`apriltag_control`** — orchestrator, run as a state machine, driven by the `std_srvs/Trigger` services `/wheelchair/attach` / `detach` / `cancel` and reporting on `/wheelchair/status` (std_msgs / std_srvs only, so ros1_bridge can carry them; see [Wheelchair Interface](#wheelchair-interface)). `attach` enables lidar safety, closes AMCL (Stage 1), runs the planner + LQR on `/cmd_vel`, then disables the PLC lidar safety field (Stage 2) and continues on `/pre_cmd_vel` (G7+ precision mode). When Stage 2 is aligned the node returns to `IDLE` with AMCL closed and lidar safety disabled; then the cmd succeeds. `detach` (accepted in `IDLE`, with or without a previous alignment) disables lidar safety and closes AMCL, then backs straight up at `max_vx` on `/pre_cmd_vel` (no planner / LQR) until the tag is `leave_distance` ahead of the camera; when it ends, or whenever a stage fails, times out or is cancelled, the node enables lidar safety and makes sure AMCL is open before `IDLE`. It subscribes only to `/wheelchair/tag_tracing` and checks the tag there against its own `tracking_tag_family` / `tracking_tag_id` (the launch files pass detection's values; `-1`: any id); `valid=false` or a tag that does not match counts as not seen (TAG_LOST after `lost_target_timeout`).

```
 camera/camera/color/image_raw ──► apriltag_detection ──► TF <family>_<id>, apriltag/marked_image
                                          │ detect all families, all tags
                                          ├──► /wheelchair/apriltag_poses (apriltag_interfaces/TagPoseArray, every image; reference)
                                          │ pick tracking_tag_family / tracking_tag_id, closest (smallest z > 0)
                                          ▼
                   /wheelchair/tag_tracing (apriltag_interfaces/TagTracing, every image)
                                          │
 /wheelchair/attach | detach | cancel (Trigger) ──► apriltag_control ──► /wheelchair/status (5 Hz), /wheelchair/attached
                                          │ valid and matches tracking_tag_family / tracking_tag_id? else not seen
   quaternion → R → control error → TrajectoryPlanner → LQR → clamp
                                  │
                                  ▼
      /cmd_vel (Stage 1) / /pre_cmd_vel (Stage 2, leaving)
```

Target loss is checked by a timer in `apriltag_control`: `tag_tracing` messages with `valid=false` or with a tag that does not match are ignored (a mismatch is warned once per tag: `[target] tag_tracing has … tracking …; treated as not seen`), so the tag counts as lost after `lost_target_timeout` (launch argument, default 1.0 s; checked only in the `running` step, against the time the poses were received).

`TagPoseArray` (`apriltag_interfaces/msg`): `std_msgs/Header header` (image stamp, camera frame) and `TagPose[] tags`; `TagPose`: `string family` (e.g. `tag36h11`), `string id` (decimal, e.g. `"0"`), `geometry_msgs/Pose pose` (camera optical frame, m). `TagTracing`: `std_msgs/Header header` (same as the image's `TagPoseArray`), `bool valid`, `TagPose tag` (empty when `valid` is false), `image_width` / `image_height`. Nothing is published before `camera_info` has been received.

Both nodes get the same `tracking_tag_family` / `tracking_tag_id` from the launch files. With `tracking_tag_id:=-1` the closest tag of the family is tracked, whatever its id.

`apriltag_control` resets its yaw estimator / probe when a cmd starts and when the tag id on `tag_tracing` changes (only possible with `-1`).

## G7+ Integration: AMCL And Lidar Safety

`apriltag_control` uses two external G7+ providers at fixed points of the procedure: AMCL through services (checked), lidar safety through a topic, the same way as G7+ AutoCharging (fire-and-forget, no read-back):

```
attach (only in IDLE)
 ├─ STAGE1 preparing: publish Bool(false) on /g7_plc/disable_lidar_safety (enable),
 │                    close AMCL: check → close (only if on) → check again
 │                                                             any failure → end, restore
 ├─ STAGE1 running:   track 50 cm on /cmd_vel; converged → stop
 ├─ STAGE2 preparing: publish Bool(true) (disable lidar safety)
 ├─ STAGE2 running:   track 28 cm on /pre_cmd_vel; converged → stop
 └─ STAGE2 ending:    stop → IDLE, cmd succeeds
                      (AMCL stays closed, lidar safety stays disabled)
detach (only in IDLE)
 ├─ LEAVING preparing: publish Bool(true) (disable lidar safety), close AMCL (only if on)
 ├─ LEAVING running:   back straight at -max_vx on /pre_cmd_vel until tag forward distance >= leave_distance
 └─ LEAVING ending:    stop → publish Bool(false) → open AMCL (only if off) → check → IDLE

any other end (tag lost / cancel / timeout / failure), in any state:
       stop → publish Bool(false) → open AMCL (only if off) → check → IDLE
node shutdown (also in IDLE): stop → publish Bool(false) → open AMCL (only if off) → check
```

- There is no record of who closed AMCL: every restore checks AMCL and opens it if it is off. A failed AMCL restore is logged and noted in the status `error_msg`, and the status reports `error_code=AMCL_SERVICE_FAIL` (also for a reached detach or a cancelled cmd). The next restore (procedure end or node shutdown) tries again.
- `IDLE` does not tell whether the robot is aligned: after a successful `attach` AMCL stays closed and lidar safety stays disabled until `detach` (or a node shutdown). The attach result log says so. A new `attach` from there enables lidar safety again before Stage 1; if the PLC then blocks the motion near the station the cmd ends after `dock_timeout` (TIMEOUT).
- Lidar safety is enabled (`false` published) at the start of Stage 1, at every procedure end except a successful Stage 2, and at node shutdown. The PLC bridge gives no reply on this topic, so this node cannot confirm the write; the `lidar_safety=` in the `[status]` log is the last value published.
- All service calls run in the cmd's worker thread, so `tracing_callback` and the target-loss watchdog keep running. While a state is preparing (and between the stages) no velocity is published and the target-loss watchdog is off.
- Which cmds are accepted depends on the state (see [State Machine](#state-machine)). Every rejected cmd is logged.
- SIGINT / SIGTERM are handled by the node itself, so the restore calls are still made when it is shut down, also from `IDLE` after a successful Stage 2. SIGKILL cannot be handled.

## State Machine

`apriltag_control` keeps its procedure in `TrackingState` (`control_node.py`); every transition is logged as `[state] OLD -> NEW (reason)`.

```
IDLE ──attach──────► STAGE1 ──Stage 1 aligned──► STAGE2 ──Stage 2 aligned──► IDLE   (AMCL off, lidar safety disabled)
IDLE ──detach──────► LEAVING ──leave reached──► IDLE                                (AMCL on, lidar safety enabled)

STAGE1 / STAGE2 / LEAVING ──failure / timeout / cancel / tag lost / shutdown──► IDLE              (AMCL on, lidar safety enabled)
```

Each non-`IDLE` state runs three steps, logged as `[step] STATE OLD -> NEW (reason)`:

| Step | Meaning |
|---|---|
| `preparing` | Switching the external state this state needs; no velocity, no target-loss check |
| `running` | Control loop active; the target-loss watchdog runs |
| `ending` | Stopped; the cmd's worker thread cleans up, then `IDLE` together with the status `error_code` / `error_msg`, so the next cmd can be sent right away |

| State | Meaning | AMCL | Lidar safety |
|---|---|---|---|
| `IDLE` | Nothing running | as the last procedure left it | as the last procedure left it |
| `STAGE1` | preparing: enable lidar safety, close AMCL; running: track `stage1_distance` on `/cmd_vel` | off | enabled |
| `STAGE2` | preparing: disable lidar safety; running: track `stage2_distance` on `/pre_cmd_vel`; ending (aligned): stop only | off | disabled |
| `LEAVING` | preparing: disable lidar safety, close AMCL; running: back straight at `-max_vx` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance` (no planner / LQR); ending: enable lidar safety, open AMCL | off → on | disabled → enabled |

`apriltag_detection` runs in every state; outside `running` its poses are ignored.

| State | `attach` | `detach` |
|---|---|---|
| `IDLE` | accept → `STAGE1` | accept → `LEAVING` |
| `STAGE1` / `STAGE2` / `LEAVING` | reject | reject |

A rejected cmd gets `success=false` with the reason and is logged (`[cmd] <cmd> rejected: …, status unchanged`); the status keeps showing the running cmd. A running cmd is stopped with `cancel`; see [Wheelchair Interface](#wheelchair-interface).

Every cmd also has a whole-cmd timeout (`dock_timeout` for attach / `undock_timeout` for detach, launch arguments, default 300 s), counted from cmd accepted until the procedure ends (the cleanup is not counted). A service call already in progress is not interrupted, so the cmd may end up to ~10 s later.

**Responsibility boundary:** this package only calls the AMCL services (and checks their responses) and publishes the lidar safety flag at the right time. Whether AMCL or the PLC actually behave as requested belongs to the G7+ providers (`dev_amcl`, ros1_bridge, `ads_bridge_node`).

## Features

- Real-time AprilTag detection via `pupil_apriltags`, several families at once, all tags published with family and id
- Two-stage docking: align at 50 cm (Stage 1), then at 28 cm (Stage 2), then back to `IDLE` with AMCL closed and lidar safety disabled
- Leaving: `detach` backs straight up until the tag is `leave_distance` away, then restores AMCL and lidar safety
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

Launch the camera and both AprilTag nodes under the `wheelchair` namespace:

```bash
ros2 launch apriltag april_tag.launch.py
# more families / other timeouts:
ros2 launch apriltag april_tag.launch.py detect_tag_families:="tag36h11 tag25h9" dock_timeout:=90.0
```

| Launch argument | Default | Node |
|---|---|---|
| `detect_tag_families` | `tag36h11` | detection (`tag_families`): families to detect, space separated; each family adds one detector run per image |
| `tracking_tag_family` / `tracking_tag_id` | `tag36h11` / `0` | detection: the tag published on `tag_tracing` (closest one; `-1` = any id); the family must be in `detect_tag_families`. control: the same values, to check `tag_tracing` |
| `leave_distance` | `1.0` m | control: `detach` backs up until the tag is this far ahead of the camera (> 0) |
| `dock_timeout` / `undock_timeout` | `300.0` / `300.0` s | control: attach / detach timeout (TIMEOUT) |
| `lost_target_timeout` | `1.0` s | control: tracked tag not seen for this long while running → TAG_LOST |
| `manage_amcl_and_lidar_safety` | `false` | control |

To run the nodes alone in the same namespace:

```bash
ros2 run apriltag apriltag_detection --ros-args -r __ns:=/wheelchair
ros2 run apriltag apriltag_control --ros-args -r __ns:=/wheelchair
```

Print what detection publishes at most 5 Hz. Parameter `topic` (launch argument of `detection_viewer.launch.py`): `tag_tracing` (default; header, `valid`, image size, then the tracked tag if valid) or `apriltag_poses` (header, then family / id / position / orientation of each tag):

```bash
ros2 run apriltag detection_viewer --ros-args -r __ns:=/wheelchair
ros2 run apriltag detection_viewer --ros-args -r __ns:=/wheelchair -p topic:=apriltag_poses
ros2 launch apriltag detection_viewer.launch.py topic:=apriltag_poses
```

Without a namespace, the relative names lose the `/wheelchair` prefix (e.g. `/attach`, `/apriltag_poses`).

Call the cmd services from another terminal (the response's `message` carries the new `cmd_id`) and watch the status:

```bash
ros2 service call /wheelchair/attach std_srvs/srv/Trigger    # success=True, message='{"cmd_id": 1, "msg": "attach accepted"}'
ros2 topic echo /wheelchair/status
# after a successful attach (IDLE, cmd_id 1, error_code 0): back up and restore AMCL / lidar safety
ros2 service call /wheelchair/detach std_srvs/srv/Trigger
# stop the running cmd
ros2 service call /wheelchair/cancel std_srvs/srv/Trigger
```

Or use the test tools, one per service without arguments (`tools/attach.py`, `tools/detach.py`, `tools/cancel.py`, shared code in `tools/wheelchair_cmd.py`; run them with `python3`, they have no entry point):

```bash
python3 src/apriltag/tools/attach.py
python3 src/apriltag/tools/detach.py
python3 src/apriltag/tools/cancel.py
```

Like `ros2 service call`, each prints the response (`<cmd>: success=… message=…`) and returns at once; follow the cmd on `/wheelchair/status`. Exit code: 0 accepted, 1 rejected or no response within 2 s, 2 service not found within 5 s (check `ROS_DOMAIN_ID` / `CYCLONEDDS_URI`) or an argparse error.

Run the controller demo module without ROS:

```bash
python -m apriltag.control
```

## Virtual End-to-End Test

`tools/virtual_tracking_sim.py` replaces the RealSense camera and the AMR with a simulation, so the real `apriltag_detection` and `apriltag_control` run unmodified: a virtual tag36h11 tag stands at the world origin, a virtual holonomic AMR integrates `/cmd_vel` and `/pre_cmd_vel`, and a synthetic camera image of the tag is rendered from their relative pose and published on `wheelchair/camera/camera/color/image_raw` + `camera_info`.

Run it inside the Docker container (needs `DISPLAY`):

```bash
ros2 launch apriltag virtual_tracking.launch.py
```

The stage / leave distances are not set on the sim: it reads `stage1_distance` / `stage2_distance` / `leave_distance` from `apriltag_control` at startup (`apriltag_control/get_parameters`) and sends no cmd before that. Set them with `stage1_distance:=… stage2_distance:=… leave_distance:=…` on the test launch.

A run is an `attach` call followed by a `detach` call; the `cmd_id` comes from the service response. A cmd has ended when `/wheelchair/status` shows `IDLE` with its `cmd_id` (`error_code` 0 SUCCEEDED, 5 CANCELED, else ABORTED); a rejected call, or one without a response after `cmd_reject_timeout` (sim parameter, 2.0 s), ends the run as REJECTED. When the attach succeeds the sim logs `aligned: amcl=off lidar_safety=disabled (held)` (or an error if they were restored too early) and waits in its `ALIGNED` phase (`apriltag_control` is in `IDLE`); Leave (`l`) sends `detach` (headless: sent automatically). The run ends with the detach. Leave without a previous Start runs a leave-only run.

A fake node `fake_g7_services` stands in for AMCL and the PLC lidar safety. It serves the same three AMCL services and subscribes to the lidar safety topic, but only keeps two bool flags (AMCL on/off, lidar safety enabled/disabled), shown in the dashboard. At the end of every run the sim logs `after cmd: amcl=… lidar_safety=… (restored)`, or an error if `apriltag_control` did not restore them.

The whole test runs in `ROS_DOMAIN_ID=65` (launch argument `domain_id`), the same domain as the robot, so **unplug the robot's network cable** before running it. Use the same domain for CLI debugging, e.g. `ROS_DOMAIN_ID=65 ros2 topic echo /cmd_vel`.

Dashboard:

| Panel | Content |
|---|---|
| Top view | Tag, AMR, camera FOV (yellow = tag in view), trail, Stage 1 / 2 and leave (`L`) goal positions. Drag the AMR body to move it, drag the round handle (or mouse wheel, `a` / `d`) to rotate — only while not running |
| Camera | `marked_image` from detection, or the raw synthetic image while no `marked_image` has arrived yet |
| Pose error | Ground-truth error (lines) vs. the controller's error from the status `x_error` / `y_error` / `yaw_error` (circles), ±0.05 tolerance band, Stage 2 and leave start times |
| cmd_vel | Thick dark line: the cmd the sim actually applies to the AMR (sampled at 50 Hz, zero after 0.5 s without a message). Thin line + dots: every raw `/cmd_vel` / `/pre_cmd_vel` message from `apriltag_control`. ±0.5 limit |

Keys: `s` start (calls `attach`), `l` leave (calls `detach`: in `ALIGNED`, or a leave-only run when no run is active), `x` stop (calls `cancel`; nothing to stop in `ALIGNED`), `r` reset to the initial pose, `+` / `-` zoom, `q` quit.

Every finished run is saved to `virtual_tracking_logs/` (relative to the working directory): `<time>_<result>_sim.csv` (50 Hz pose, ground-truth error, cmd), `<time>_<result>_feedback.csv` (status while the cmd runs: `t, x_err, y_err, yaw_err, cmd_id, state, last_frame_age`; state 1 STAGE1, 2 STAGE2, 3 LEAVING; NaN for `null`), `<time>_<result>_cmd_raw.csv` (every raw `/cmd_vel` / `/pre_cmd_vel` message) and a PNG of the dashboard.

Headless (no window, `attach` then `detach`, then exit after saving):

```bash
ros2 launch apriltag virtual_tracking.launch.py headless:=true auto_start:=true \
    init_x:=-1.0 init_y:=0.15 init_yaw_deg:=10.0
```

The sim's camera intrinsics, camera mount offset, tag size and stage distances are ROS parameters of `virtual_tracking_sim`; the sim draws `tag36h11` id `0` (sim parameter `tag_id`), the default `tracking_tag_family` / `tracking_tag_id` of the test launch; `tracking_tag_id:=5` makes `tag_tracing` always `valid=false` (TAG_LOST, `error_code` 1). `tag_size`, `camera_y_offset` and the stage distances must match the values in `detection_node.py` / `control_node.py`.

## Topics

| Topic | Type | Node | Direction | Description |
|---|---|---|---|---|
| `/wheelchair/camera/camera/color/image_raw` | `sensor_msgs/Image` | detection | Subscribe | Raw camera frames |
| `/wheelchair/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | detection | Subscribe (until received) | Camera intrinsics |
| `/wheelchair/apriltag_poses` | `apriltag_interfaces/TagPoseArray` | detection | Publish | All detected tags (family, id, pose in camera optical frame) of one image, stamped with the image time; empty when none is visible. Reference / debugging only |
| `/wheelchair/tag_tracing` | `apriltag_interfaces/TagTracing` | detection → control | Publish / Subscribe | The tracked tag of one image (closest of `tracking_tag_family` / `tracking_tag_id`), same header as `apriltag_poses`; `valid=false` when there is none |
| `/wheelchair/apriltag/marked_image` | `sensor_msgs/Image` | detection | Publish | Annotated image with detections |
| `/wheelchair/status` | `std_msgs/String` (JSON) | control | Publish | Status at 5 Hz (transient_local) |
| `/wheelchair/attached` | `std_msgs/Bool` | control | Publish | Always `false` at 5 Hz (transient_local): the gripper is not sensed |
| `/g7_body/simple_ui/body_status` | `std_msgs/Int32` | control | Publish | For the G7+ UI, only on a state change (transient_local, parameter `body_status_topic`): `1` entering STAGE1, `2` entering LEAVING, `0` back to IDLE (success or failure); nothing on STAGE2 or at startup |
| `/cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 1 velocity commands (parameter `stage1_cmd_vel_topic`); stops are sent on both topics |
| `/pre_cmd_vel` | `geometry_msgs/Twist` | control | Publish | Stage 2 and leaving velocity commands, G7+ precision mode: `motor_control` holds the wheels until every steering angle is within 5° (parameter `stage2_cmd_vel_topic`) |
| `/g7_plc/disable_lidar_safety` | `std_msgs/Bool` | control | Publish | G7+ PLC lidar safety flag: `true` (disable) before Stage 2 and before leaving, `false` (enable) before Stage 1 and at every procedure end (not when Stage 2 succeeds). Subscribed by `ads_bridge_node`; fire-and-forget, no read-back |

## Services

| Service | Type | Node | Description |
|---|---|---|---|
| `/check_mcl_if_trigger` | `std_srvs/Trigger` | G7+ AMCL | Called by `apriltag_control`; `success=true` means AMCL is running |
| `/close_amcl`, `/open_amcl` | `std_srvs/Empty` | G7+ AMCL | Called by `apriltag_control` before Stage 1 and before leaving (close, only if on) and at procedure end / shutdown (open, only if off) |
| `/wheelchair/attach`, `/wheelchair/detach`, `/wheelchair/cancel` | `std_srvs/Trigger` | control | Cmds, see [Wheelchair Interface](#wheelchair-interface) |

## Wheelchair Interface

Follows `[service_robot] wheelchair_interface.pdf` (v0.1) with these differences: `detach` is the leaving (no gripper, `attached` is always `false`), the states are this node's `IDLE` / `STAGE1` / `STAGE2` / `LEAVING` (no `CANCELLED` / `ERROR`: a cmd that failed or was cancelled ends in `IDLE` with `error_code` ≠ 0), the error codes are the ones below (not the PDF's yet), there is no `/wheelchair/get_status` service, and the cmds are three `std_srvs/Trigger` services instead of the `/wheelchair/cmd` topic (no `reset`; `apriltag_control` gives the `cmd_id`, not the caller).

`/wheelchair/attach`, `/wheelchair/detach`, `/wheelchair/cancel` (`std_srvs/Trigger`, empty request) answer at once whether the cmd is taken; the result is in the status. `success` = accepted, `message` = JSON `{"cmd_id": n | null, "msg": "<reason>"}`. Only one cmd runs at a time.

| Service | Effect |
|---|---|
| `attach` | Only in `IDLE`: Stage 1 + 2 (dock). Gets a new `cmd_id` n (1, 2, … since the node started; status `cmd_id` = n), error cleared. Succeeds in `IDLE` with AMCL closed, lidar safety disabled. Else `success=false`, `cmd_id` null |
| `detach` | Only in `IDLE`, with or without a previous attach (new `cmd_id` like attach): back straight up until the tag is `leave_distance` ahead, then restore AMCL / lidar safety |
| `cancel` | Not in `IDLE`: `success=true` with the running `cmd_id`; stop at once, restore, `IDLE` with `error_code` 5. If the cmd is already ending (e.g. just aligned) `msg` is `already ending, result unchanged` and the status keeps its own result, so always check the status `error_code`. In `IDLE`: `success=false` (`nothing running`) |

`/wheelchair/status` (`std_msgs/String`, reliable, transient_local, depth 1, 5 Hz):

```json
{"state": "STAGE1", "cmd_id": 17, "attached": false, "error_code": 0, "error_msg": "", "stamp": 1792389633.11,
 "x_error": 0.12, "y_error": -0.02, "yaw_error": 0.01, "last_frame_age": 0.03}
```

`state` is the `TrackingState`; `cmd_id` the current / last cmd (0 after startup); `stamp` the publish time (epoch s); `x_error` / `y_error` (m), `yaw_error` (rad) and `last_frame_age` (s, node clock − `header.stamp` of the newest frame with the tracked tag) are debug fields, `null` when unknown. A cmd has ended when the state is `IDLE` with its `cmd_id`; `error_code` 0 means it succeeded. `error_code` / `error_msg` are written together with the change to `IDLE`.

| `error_code` | Meaning |
|---|---|
| `OK` 0 | attach aligned / detach reached |
| `TAG_LOST` 1 | tracked tag not seen for `lost_target_timeout` while running |
| `AMCL_SERVICE_FAIL` 2 | AMCL service missing / no response / wrong state, or the restore failed (whatever ended the cmd) |
| `LIDAR_SERVICE_FAIL` 3 | reserved, not reported yet (lidar safety is fire-and-forget) |
| `TIMEOUT` 4 | `dock_timeout` / `undock_timeout` |
| `ACTION_CANCELED` 5 | `cancel` |
| `OTHER` 6 | anything else (e.g. an exception) |
| `NODE_SHUTDOWN` 7 | `apriltag_control` shut down |

### TF Transforms

`apriltag_detection` broadcasts `camera_frame → <family>_<id>` (e.g. `tag36h11_0`) for each detected tag.

## Logging

Both launch files print log lines without time and without ros2 launch's `[<process>-N]` prefix (`output_format='{line}'`). Every `apriltag_control` message starts with its current state:

```
[INFO] [wheelchair.apriltag_control][stage LEAVING] : [publish] /pre_cmd_vel vx=-0.100 vy=+0.000 wz=+0.000
[INFO] [wheelchair.apriltag_detection]: tag_families=tag36h11 tag_size=0.0635 tag_sizes={} tracking=tag36h11 id 0. Detecting continuously; publishing all tags on apriltag_poses, the tracked one on tag_tracing.
```

The `[state] OLD -> NEW` line already shows the new state in its prefix.

`apriltag_control` also prefixes its logs so they can be filtered, e.g. `grep '\[request\]'`:

| Prefix | When |
|---|---|
| `[state]` | Every state transition: `[state] OLD -> NEW (reason)` |
| `[step]` | Every step change inside a state: `[step] STATE OLD -> NEW (reason)` |
| `[cmd] accepted / rejected` | Every cmd service call; a rejection names the reason (state, nothing running) |
| `[request]` / `[response]` | Every service call (content, and response time) |
| `[publish]` | `/cmd_vel` / `/pre_cmd_vel` commands: stops always, control commands at most once per second; every `/g7_plc/disable_lidar_safety` message |
| `[status]` | The status JSON, at most once per second while a cmd runs |
| `[cmd] <cmd> cmd_id=<n> result` | Cmd end with SUCCEEDED / CANCELED / ABORTED, error_code, message and final errors |

The once-per-second limit is `LOG_THROTTLE_SEC` in `control_node.py`.

## Configuration

Parameters are hard-coded in `AprilTagDetectionNode.__init__()` (`apriltag/detection_node.py`) and `AprilTagControlNode.__init__()` (`apriltag/control_node.py`). Check the first two against the real robot before running on it:

| Parameter | Value | Where | Description |
|---|---|---|---|
| `tag_size` | `0.0635` m | `detection_node.py` | **Edge of the tag's black square (8 × 8 cells, without the white border). Must match the printed tag** |
| `camera_y_offset` | `0.026` m (= `0.036` mount + `y_offset` `-0.01` trim) | `control_node.py` | Camera lateral offset from the robot centre line, + = left |
| `tag_families` | `tag36h11` (ROS parameter, launch arg `detect_tag_families:=`) | `detection_node.py` | Families detected, space separated; one `pupil_apriltags` detector per family (one `Detector` only loads one family). `tag_size` is shared by all families |
| `tracking_tag_family` / `tracking_tag_id` | `tag36h11` / `0` (ROS parameters, same launch args) | `detection_node.py` | Tag published on `tag_tracing`: the closest one of this family / id (`-1`: any id). The family must be in `tag_families` and the id ≥ -1, else the node exits at startup |
| `stop_x/y/yaw_error_tolerance` | `0.05` m / `0.05` m / `0.05` rad | `control_node.py` | A stage is aligned when all errors stay within these |
| `stop_hold_seconds` | `1.0` s | `control_node.py` | ... for this long |
| `max_vx`, `max_vy` | `0.5` m/s | `control_node.py` | Linear velocity saturation |
| `max_vw` | `0.5` rad/s | `control_node.py` | Angular velocity saturation |
| `max_dt` | `0.2` s | `control_node.py` | Timestep cap (handles frame drops) |
| `smooth_tau` | `0.5` s | `control_node.py` (`TrajectoryPlanner`) | Reference smoothing time constant |
| LQR Q weights | `(1.0, 1.0, 0.5)` | `control.py` default | State cost: x, y, yaw |
| LQR R weights | `(3.0, 1.0, 3.0)` | `control_node.py` | Control cost: vx, vy, vw (vy is penalised least, so lateral correction is fastest) |

ROS parameters of `apriltag_control`:

| Parameter | Default |
|---|---|
| `stage1_distance` | `0.50` m (Stage 1 target distance to the tag) |
| `stage2_distance` | `0.28` m (Stage 2, final target distance to the tag) |
| `tracking_tag_family` / `tracking_tag_id` | `tag36h11` / `0` (tag checked on `tag_tracing`; must match apriltag_detection's) |
| `leave_distance` | `1.0` m (detach back-up distance; must be > 0) |
| `dock_timeout` / `undock_timeout` | `300.0` s (attach / detach timeout; must be > 0) |
| `lost_target_timeout` | `1.0` s (tracked tag not seen for this long while running → TAG_LOST; must be > 0) |
| `manage_amcl_and_lidar_safety` | `true` (`false` skips every AMCL and lidar safety call; tracking is never blocked by them) |
| `amcl_check_service` | `/check_mcl_if_trigger` |
| `amcl_close_service` | `/close_amcl` |
| `amcl_open_service` | `/open_amcl` |
| `lidar_safety_topic` | `/g7_plc/disable_lidar_safety` |
| `body_status_topic` | `/g7_body/simple_ui/body_status` |
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
│   │   └── msg/
│   │       ├── TagPose.msg              # family, id, pose of one tag
│   │       ├── TagPoseArray.msg         # all tags of one image
│   │       └── TagTracing.msg           # the tracked tag of one image (valid flag)
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
│       │       ├── cmd_flow.py          # service reply JSON, status JSON, error codes, parameter checks, outcome
│       │       ├── control_flow.py      # Control pipeline steps
│       │       ├── safety_guard.py      # Target-loss watchdog logic
│       │       └── target_flow.py       # Tracked-tag selection (family / id / closest)
│       ├── launch/
│       │   ├── april_tag.launch.py                # camera + detection + control (real robot)
│       │   └── virtual_tracking.launch.py         # virtual end-to-end test
│       ├── tools/
│       │   ├── attach.py / detach.py / cancel.py  # test tools: call one service each, print the response
│       │   ├── wheelchair_cmd.py        # shared code of attach.py / detach.py / cancel.py
│       │   ├── detection_viewer.py      # prints /wheelchair/tag_tracing (default) or /wheelchair/apriltag_poses
│       │   └── virtual_tracking_sim.py  # virtual camera / AMR / fake G7+ AMCL + lidar safety + dashboard
│       └── test/
│           ├── test_state_machine.py    # TrackingState transition table and cmd rules
│           ├── test_cmd_flow.py         # service reply JSON, status JSON, parameter checks, outcome
│           ├── test_leave_step.py       # leave step
│           └── test_target_flow.py      # tracked-tag selection
└── README.md
```

## Control Architecture

### Docking Stages

1. **Stage 1** — track to `stage1_distance` (0.50 m). When all errors stay within tolerance for `stop_hold_seconds`, the robot stops and Stage 2 disables the lidar safety field before it moves.
2. **Stage 2** — track to `stage2_distance` (0.28 m). When converged the robot stops and the node returns to `IDLE` (AMCL closed, lidar safety disabled); then the cmd succeeds.
3. **Leave** (`detach`) — no planner / LQR: every tag pose publishes `vx = -max_vx, vy = 0, wz = 0` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `leave_distance`; y / yaw are not checked and there is no hold time. Then the robot stops (`Leave reached 1.00m (forward=…m).`) and AMCL / lidar safety are restored. Losing the tag for `lost_target_timeout` stops the robot and aborts the leave.

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
