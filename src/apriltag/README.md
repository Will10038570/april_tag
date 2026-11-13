# AprilTag Visual Tracker — ROS 2

A ROS 2 Python package that detects [AprilTag](https://april.eecs.umich.edu/software/apriltag) fiducial markers, estimates their 3D pose, and drives a robot toward the selected tag with a staged reference planner plus discrete-time LQR control.

## Overview

The package runs as two nodes under the `wheelchair` namespace (the RealSense camera too):

- **`apriltag_detection`** — runs continuously from startup: every camera image gives one `TagPoseArray` on `/wheelchair/apriltag_poses` with every detected tag (family, id, pose) of the families in `tag_families`, empty when none is visible, plus a TF per tag, and one `TagTracing` on `/wheelchair/tag_tracing` with the tracked tag: the closest one (smallest positive forward z) of `tracking_tag_family` / `tracking_tag_id` (`-1`: any id), `valid=false` when there is none. It is not controlled by `apriltag_control`.
- **`apriltag_control`** — orchestrator, run as a state machine, driven by the `std_srvs/SetBool` services `/wheelchair/attach` / `detach` (`data: true` starts, `false` cancels) and reporting on `/wheelchair/status` (std_msgs / std_srvs only, so ros1_bridge can carry them; see [Wheelchair Interface](#wheelchair-interface)). `attach` enables lidar safety, closes AMCL (Stage 1), runs the planner + LQR on `/cmd_vel`, then disables the PLC lidar safety field (Stage 2) and continues on `/pre_cmd_vel` (G7+ precision mode). When Stage 2 is aligned the node returns to `IDLE` with AMCL closed and lidar safety disabled; then the cmd succeeds. `detach` (accepted in `IDLE`, with or without a previous alignment) disables lidar safety and closes AMCL, then backs straight up at `max_vx` on `/pre_cmd_vel` (no planner / LQR) until the tag is `leave_distance` ahead of the camera; when it ends, or whenever a stage fails, times out or is cancelled, the node enables lidar safety and makes sure AMCL is open before `IDLE`. It subscribes only to `/wheelchair/tag_tracing` and checks the tag there against its own `tracking_tag_family` / `tracking_tag_id` (the launch files pass detection's values; `-1`: any id); `valid=false` or a tag that does not match counts as not seen (TAG_LOST after `lost_target_timeout`).

```
 camera/camera/color/image_raw ──► apriltag_detection ──► TF <family>_<id>, apriltag/marked_image
                                          │ detect all families, all tags
                                          ├──► /wheelchair/apriltag_poses (apriltag_interfaces/TagPoseArray, every image; reference)
                                          │ pick tracking_tag_family / tracking_tag_id, closest (smallest z > 0)
                                          ▼
                   /wheelchair/tag_tracing (apriltag_interfaces/TagTracing, every image)
                                          │
 /wheelchair/attach | detach (SetBool)     ──► apriltag_control ──► /wheelchair/status (5 Hz), /wheelchair/attached
                                          │ valid and matches tracking_tag_family / tracking_tag_id? else not seen
   quaternion → R → control error → TrajectoryPlanner → LQR → clamp
                                  │
                                  ▼
      /cmd_vel (Stage 1) / /pre_cmd_vel (Stage 2, leaving)
```

Target loss is checked by a timer in `apriltag_control`: `tag_tracing` messages with `valid=false` or with a tag that does not match are ignored (a mismatch is warned once per tag: `[target] tag_tracing has … tracking …; treated as not seen`), so the tag counts as lost after `control.lost_target_timeout` (config, shipped 1.0 s; checked only in the `running` step, against the time the poses were received).

`TagPoseArray` (`apriltag_interfaces/msg`): `std_msgs/Header header` (image stamp, camera frame) and `TagPose[] tags`; `TagPose`: `string family` (e.g. `tag36h11`), `string id` (decimal, e.g. `"0"`), `geometry_msgs/Pose pose` (camera optical frame, m). `TagTracing`: `std_msgs/Header header` (same as the image's `TagPoseArray`), `bool valid`, `TagPose tag` (empty when `valid` is false), `image_width` / `image_height`. Nothing is published before `camera_info` has been received.

Both nodes get the same `detection.tracking_tag_family` / `tracking_tag_id` from the config file. With `tracking_tag_id` `-1` the closest tag of the family is tracked, whatever its id.

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
 ├─ LEAVING running:   back straight at -max_vx on /pre_cmd_vel until tag forward distance >= calibration.detach.z_ref
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
| `STAGE1` | preparing: enable lidar safety, close AMCL; running: track `calibration.stage1` on `/cmd_vel` | off | enabled |
| `STAGE2` | preparing: disable lidar safety; running: track `calibration.stage2` on `/pre_cmd_vel`; ending (aligned): stop only | off | disabled |
| `LEAVING` | preparing: disable lidar safety, close AMCL; running: back straight at `-max_vx` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `calibration.detach.z_ref` (no planner / LQR); ending: enable lidar safety, open AMCL | off → on | disabled → enabled |

`apriltag_detection` runs in every state; outside `running` its poses are ignored.

| State | `attach` | `detach` |
|---|---|---|
| `IDLE` | accept → `STAGE1` | accept → `LEAVING` |
| `STAGE1` / `STAGE2` / `LEAVING` | reject | reject |

A rejected cmd gets `success=false` with the reason and is logged (`[cmd] <cmd> rejected: …, status unchanged`); the status keeps showing the running cmd. A running cmd is stopped with `data: false` on its own service; see [Wheelchair Interface](#wheelchair-interface).

Every cmd also has a whole-cmd timeout (`dock_timeout` for attach / `undock_timeout` for detach, launch arguments, default 300 s), counted from cmd accepted until the procedure ends (the cleanup is not counted). A service call already in progress is not interrupted, so the cmd may end up to ~10 s later.

**Responsibility boundary:** this package only calls the AMCL services (and checks their responses) and publishes the lidar safety flag at the right time. Whether AMCL or the PLC actually behave as requested belongs to the G7+ providers (`dev_amcl`, ros1_bridge, `ads_bridge_node`).

## Features

- Real-time AprilTag detection via `pupil_apriltags`, several families at once, all tags published with family and id
- Two-stage docking: align at 50 cm (Stage 1), then at 28 cm (Stage 2), then back to `IDLE` with AMCL closed and lidar safety disabled
- Leaving: `detach` backs straight up until the tag is `calibration.detach.z_ref` away, then restores AMCL and lidar safety
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
# another config file:
ros2 launch apriltag april_tag.launch.py config_file:=/path/to/april_tag_config.json
```

`config_file` (default `/mnt/work_space/april_tag_config.json`) is the only launch argument; every setting is in that file (see [Configuration](#configuration)). The included RealSense `rs_launch.py` has a launch argument of the same name (a YAML of camera parameters), so `april_tag.launch.py` passes it `config_file:=''` inside a scoped `GroupAction`; otherwise the camera launch would load the JSON and fail on its `null` values.

To run the nodes alone in the same namespace:

```bash
ros2 run apriltag apriltag_detection --ros-args -r __ns:=/wheelchair -p config_file:=/mnt/work_space/april_tag_config.json
ros2 run apriltag apriltag_control --ros-args -r __ns:=/wheelchair -p config_file:=/mnt/work_space/april_tag_config.json
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
ros2 service call /wheelchair/attach std_srvs/srv/SetBool "{data: true}"    # success=True, message='{"cmd_id": 1, "msg": "attach accepted"}'
ros2 topic echo /wheelchair/status
# stop the running attach (a running detach: /wheelchair/detach with data: false)
ros2 service call /wheelchair/attach std_srvs/srv/SetBool "{data: false}"
# in IDLE (e.g. after a successful attach: cmd_id 1, error_code 0): back up and restore AMCL / lidar safety
ros2 service call /wheelchair/detach std_srvs/srv/SetBool "{data: true}"
```

Or use the test tools, one per service with a required `true` / `false` argument (`tools/attach.py`, `tools/detach.py`, shared code in `tools/wheelchair_cmd.py`; run them with `python3`, they have no entry point):

```bash
python3 src/apriltag/tools/attach.py true    # false: cancel the running attach
python3 src/apriltag/tools/detach.py true    # false: cancel the running detach
```

Like `ros2 service call`, each prints the response (`<cmd> <data>: success=… message=…`) and returns at once; follow the cmd on `/wheelchair/status`. Exit code: 0 accepted, 1 rejected or no response within 2 s, 2 service not found within 5 s (check `ROS_DOMAIN_ID` / `CYCLONEDDS_URI`) or an argparse error.

Run the controller demo module without ROS:

```bash
python -m apriltag.control
```

## Calibration

`apriltag_control` can target an exact tag pose recorded from the real camera at the desired docking / leaving position, instead of the fixed default distances. This corrects for camera mounting offsets (lateral and yaw) and makes docking more repeatable across robot restarts.

### Saving Reference Points

Place the robot at the desired position with the tag visible, then run `calibrate.py` for that stage. Only `apriltag_detection` (and the camera) need to be running; `apriltag_control` does not.

```bash
# Stage 1 (approach) — robot at ~50 cm from the tag, laterally centred
docker exec -it april_tag_serive bash -ic '
  python3 /mnt/work_space/src/apriltag/tools/calibrate.py --stage1'

# Stage 2 (final dock) — robot at ~28 cm
docker exec -it april_tag_serive bash -ic '
  python3 /mnt/work_space/src/apriltag/tools/calibrate.py --stage2'

# Detach (leave end position) — robot backed up to ~1 m
docker exec -it april_tag_serive bash -ic '
  python3 /mnt/work_space/src/apriltag/tools/calibrate.py --detach'
```

`--stage1` / `--stage2` save the tracked tag's `z_ref` (forward distance, m), `x_ref` (optical lateral, m) and `yaw_ref` (rad), `--detach` only `z_ref` (the leave backs up straight and checks only the forward distance), to the `calibration` section of `april_tag_config.json` (default `/mnt/work_space/april_tag_config.json`; override with `--file PATH`; only that stage is replaced, other stages and sections are kept). No image is saved.

`--file` must be an existing file containing a JSON object. If it is missing, not valid JSON or not an object, `calibrate.py` exits 1 at once (before waiting for a tag) and writes nothing.

Stages can be calibrated independently. There are no default distances: `apriltag_control` requires all three points (`stage1` and `stage2` with numeric `z_ref` > 0, `x_ref`, `yaw_ref`; `detach` with numeric `z_ref` > 0 only; other keys, e.g. an old `stamp` / `image_path` or `detach.x_ref` / `yaw_ref`, are ignored), and exits at startup (or rejects the attach / detach on reload) naming the missing one, e.g. `calibration.stage2 is missing or incomplete …`. `apriltag_detection` and `calibrate.py` do not check them, so a stage can be calibrated while `apriltag_control` is down; with a scoped `GroupAction` the launch keeps the camera and detection running when control exits. After saving, restart the container (`docker restart april_tag_serive`) if control had exited, otherwise the next attach / detach re-reads the file.

The shipped `april_tag_config.json` holds the previous defaults as points (`z_ref` 0.50 / 0.28 / 1.0 m; stage1 / stage2 `x_ref` 0.026 m = the former `camera_y_offset`, `yaw_ref` 0), which behave as before calibration; `calibrate.py` overwrites them one by one.

### Using Calibration

Each stage targets its point: `desired_distance = z_ref`, lateral error `y = -t.x + x_ref`, yaw error `yaw - yaw_ref`. Leave uses only `detach.z_ref` (its `x_ref` / `yaw_ref` only shift the y / yaw errors shown in the status). `apriltag_control` re-reads the JSON each time an attach / detach is accepted, so saved points apply without a restart.

The log shows what is used:

```
[calibration] stage1(z_ref=0.5000m x_ref=0.0260m yaw_ref=0.0000rad), stage2(…), detach(…); tolerance stage1(x=0.050m y=0.050m yaw=0.050rad hold=1.00s), stage2(…)
# at each stage start:
[calibration] STAGE1: z_ref=0.5000m x_ref=0.0260m yaw_ref=0.0000rad; tolerance x=0.050m y=0.050m yaw=0.050rad hold=1.00s
```

## Virtual End-to-End Test

`tools/virtual_tracking_sim.py` replaces the RealSense camera and the AMR with a simulation, so the real `apriltag_detection` and `apriltag_control` run unmodified: a virtual tag36h11 tag stands at the world origin, a virtual holonomic AMR integrates the cmd_vel topics of the config's `ros_interfaces` (`/cmd_vel`, `/pre_cmd_vel`), and a synthetic camera image of the tag is rendered from their relative pose and published on `wheelchair/camera/camera/color/image_raw` + `camera_info`.

Run it inside the Docker container (needs `DISPLAY`):

```bash
ros2 launch apriltag virtual_tracking.launch.py
```

All three nodes read the launch argument `config_file`, default the workspace root's `virtual_tracking_config.json` (next to `april_tag_config.json`, `/mnt/work_space/virtual_tracking_config.json`), read directly (not installed), so edits apply without a `colcon build`; `apriltag_control` re-reads it on each accepted attach / detach but the sim and `apriltag_detection` only at start, so relaunch after editing it. It has the same keys as `april_tag_config.json`, with `tag_size` `0.0475`, `manage_amcl_and_lidar_safety` `true` and calibration points `z_ref` 0.50 / 0.28 / 1.0 m, stage1 / stage2 `x_ref` 0.026 m, `yaw_ref` 0. The sim validates it like `apriltag_control` (`validate_control_config`) and takes from it the tag size, drawn tag id (`detection.tracking_tag_id`, `-1` → `0`), the calibration points, in-position tolerances, `max_vx` / `max_vy` / `max_vw` (dashboard) and the `ros_interfaces` names (cmd_vel topics, AMCL services, lidar safety topic). Its ground truth uses each stage's calibration point exactly like `apriltag_control`: `optical_to_control_error(t, z_ref, R, x_ref)` and yaw minus `yaw_ref` (detach: `x_ref` = `yaw_ref` = 0), so any calibrated values work; the factory `x_ref` 0.026 equals the sim's `camera_y_offset`, which puts the goals on the tag's centre line. For variant tests copy the file, change it and pass `config_file:=/path/to/copy.json` (e.g. `control.dock_timeout` `3.0` → TIMEOUT, `error_code` 4).

A run is an `attach` call followed by a `detach` call; the reply has no `cmd_id`, so the sim takes it from the status: the first `cmd_id` above the one seen before the call, once the call is accepted. A cmd has ended when `/wheelchair/status` shows `IDLE` with its `cmd_id` (`error_code` 0 with `error_msg` starting with `Cancelled` CANCELED, other 0 SUCCEEDED, else ABORTED); a rejected call, or one without a response after `cmd_reject_timeout` (sim parameter, 2.0 s), ends the run as REJECTED. When the attach succeeds the sim logs `aligned: amcl=off lidar_safety=disabled (held)` (or an error if they were restored too early) and waits in its `ALIGNED` phase (`apriltag_control` is in `IDLE`); Leave (`l`) sends `detach` (headless: sent automatically). The run ends with the detach. Leave without a previous Start runs a leave-only run. The stage shown (and the Stage 2 time line) follows the status `state` (`Attaching(STAGE2)`).

During a run the sim records `/g7_body/simple_ui/body_status` (sim parameter `body_status_topic`) and, 0.5 s after the run ends, compares it with the values the accepted cmds should give (attach `1, 0`, detach `2, 0`): `body_status=[1, 0, 2, 0] expected=[1, 0, 2, 0] (OK)`, or an error `(NOT OK)`. Headless exits after this check.

A fake node `fake_g7_services` stands in for AMCL and the PLC lidar safety. It serves the same three AMCL services and subscribes to the lidar safety topic (names from `ros_interfaces`), but only keeps two bool flags (AMCL on/off, lidar safety enabled/disabled), shown in the dashboard. At the end of every run the sim logs `after cmd: amcl=… lidar_safety=… (restored)`, or an error if `apriltag_control` did not restore them.

The whole test runs in `ROS_DOMAIN_ID=65` (launch argument `domain_id`). The robot's domain is set in `docker/.ros2_setup`, but do not rely on it being different: **unplug the robot's network cable**, or use `domain_id:=99` with CycloneDDS on loopback only. Use the same domain for CLI debugging, e.g. `ROS_DOMAIN_ID=99 ros2 topic echo /cmd_vel`.

Dashboard:

| Panel | Content |
|---|---|
| Top view | Tag, AMR, camera FOV (yellow = tag in view), trail, Stage 1 / 2 and leave (`L`) goal positions. Drag the AMR body to move it, drag the round handle (or mouse wheel, `a` / `d`) to rotate — only while not running |
| Camera | `marked_image` from detection, or the raw synthetic image while no `marked_image` has arrived yet; `[TAG HIDDEN]` while the tag is hidden |
| Pose error | Ground-truth error (lines) vs. the controller's error from the status `x_error` / `y_error` / `yaw_error` (circles); shaded band = ±max(x, y) in-position tolerance of the stage running at that time (`stage1_tolerance` / `stage2_tolerance`, none while leaving; all tolerances in the title), Stage 2 and leave start times |
| cmd_vel | Lines + dots: every raw `/cmd_vel` / `/pre_cmd_vel` message from `apriltag_control`. Thin horizontal lines: ±`max_vx` / `max_vy` / `max_vw` in each axis' colour |

Keys: `s` start (calls `attach`), `l` leave (calls `detach`: in `ALIGNED`, or a leave-only run when no run is active), `x` stop (`data: false` on the running cmd's service; nothing to stop in `ALIGNED`), `r` reset to the initial pose, `h` hide / show the tag, `+` / `-` zoom, `q` quit.

Every finished run is saved to `virtual_tracking_logs/` (relative to the working directory): `<time>_<result>_sim.csv` (50 Hz pose, ground-truth error, cmd), `<time>_<result>_feedback.csv` (status while the cmd runs: `t, x_err, y_err, yaw_err, cmd_id, state, last_frame_age, cmd_vx, cmd_vy, cmd_wz`; state 1 `Attaching(STAGE1)`, 2 `Attaching(STAGE2)`, 3 `Detaching`; NaN for `null`), `<time>_<result>_cmd_raw.csv` (every raw `/cmd_vel` / `/pre_cmd_vel` message) and a PNG of the dashboard.

Headless (no window, `attach` then `detach`, then exit after saving):

```bash
ros2 launch apriltag virtual_tracking.launch.py headless:=true auto_start:=true \
    init_x:=-1.0 init_y:=0.15 init_yaw_deg:=10.0
```

The sim's camera intrinsics and camera mount offset are ROS parameters of `virtual_tracking_sim`. It always draws a `tag36h11` tag with the config's `tracking_tag_id`, so changing `tracking_tag_id` in a test config does **not** produce TAG_LOST. To test TAG_LOST use the launch argument `hide_tag_after` (s after the run starts, default `-1` = never) or the `h` key: the sim then publishes the background only, detection reports `valid=false` and `apriltag_control` ends with `error_code` 1 after `lost_target_timeout`:

```bash
ros2 launch apriltag virtual_tracking.launch.py headless:=true auto_start:=true hide_tag_after:=5.0
```

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
| `/wheelchair/attach`, `/wheelchair/detach` | `std_srvs/SetBool` | control | Cmds, see [Wheelchair Interface](#wheelchair-interface) |

## Wheelchair Interface

Follows `[service_robot] wheelchair_interface.pdf` (v0.1) with these differences: `detach` is the leaving (no gripper, `attached` is always `false`), the status states are `IDLE` / `Attaching(STAGE1)` / `Attaching(STAGE2)` / `Detaching` (this node's `TrackingState` `IDLE` / `STAGE1` / `STAGE2` / `LEAVING`; no `CANCELLED` / `ERROR`: a cmd that failed ends in `IDLE` with `error_code` ≠ 0, a cancelled one with `error_code` 0 and `error_msg` `Cancelled …`), the error codes are the ones below (not the PDF's yet), there is no `/wheelchair/get_status` service, and the cmds are two `std_srvs/SetBool` services instead of the `/wheelchair/cmd` topic (`data: false` = cancel, no `reset`; `apriltag_control` gives the `cmd_id`, not the caller).

`/wheelchair/attach`, `/wheelchair/detach` (`std_srvs/SetBool`; `data: true` starts the cmd, `data: false` cancels it) answer at once whether the cmd is taken; the result is in the status. `success` = accepted, `message` = the reason as plain text (no JSON, no `cmd_id`): `attach accepted` / `detach accepted`, `cancel requested`, `already ending, result unchanged`, `nothing running` or `rejected: state=… (only accepted in …)`. The `cmd_id` of a cmd is only in the status. Only one cmd runs at a time.

| Service | Effect |
|---|---|
| `attach` `true` | Only in `IDLE`: Stage 1 + 2 (dock). Gets a new `cmd_id` n (1, 2, … since the node started; status `cmd_id` = n), error cleared. Succeeds in `IDLE` with AMCL closed, lidar safety disabled. Else `success=false` |
| `detach` `true` | Only in `IDLE`, with or without a previous attach (new `cmd_id` like attach): back straight up until the tag is `calibration.detach.z_ref` ahead, then restore AMCL / lidar safety |
| `attach` / `detach` `false` | Only the running cmd of the same service (`attach`: `STAGE1` / `STAGE2`, `detach`: `LEAVING`): `success=true` (`cancel requested`); stop at once, restore, `IDLE` with `error_code` 0 and `error_msg` `Cancelled cmd_id=n. elapsed=…s` (`error_code` 2 if the restore failed). If the cmd is already ending (e.g. just aligned) `msg` is `already ending, result unchanged` and the status keeps its own result, so always check the status `error_code` / `error_msg`. In `IDLE`: `success=false` (`nothing running`); the other cmd running: `success=false`, e.g. `rejected: state=LEAVING (only accepted in STAGE1, STAGE2)`, status unchanged |

`/wheelchair/status` (`std_msgs/String`, reliable, transient_local, depth 1, 5 Hz):

```json
{"state": "Attaching(STAGE1)", "cmd_id": 17, "attached": false, "error_code": 0, "error_msg": "", "stamp": 1792389633.11,
 "x_error": 0.12, "y_error": -0.02, "yaw_error": 0.01, "last_frame_age": 0.03,
 "cmd_vx": 0.1, "cmd_vy": -0.02, "cmd_wz": 0.01}
```

`state` is the `TrackingState` as a status string (`STATUS_STATE` in `control_node.py`: `IDLE`, `Attaching(STAGE1)`, `Attaching(STAGE2)`, `Detaching`; logs and service replies keep the enum value, e.g. `STAGE1`); `cmd_id` the current / last cmd (0 after startup); `stamp` the publish time (epoch s); `x_error` / `y_error` (m), `yaw_error` (rad) and `last_frame_age` (s, node clock − `header.stamp` of the newest frame with the tracked tag) and `cmd_vx` / `cmd_vy` (m/s), `cmd_wz` (rad/s) are debug fields, `null` when unknown. `cmd_vx` / `cmd_vy` / `cmd_wz` are the last velocity sent on `/cmd_vel` or `/pre_cmd_vel` (stops included, as `0`; status is 5 Hz, so only the newest of the ~30 Hz commands is shown), `null` if none was sent within the last 1 s (`CMD_VEL_MAX_AGE` in `control_node.py`) or since startup. A cmd has ended when the state is `IDLE` with its `cmd_id`; `error_code` 0 means it succeeded (`error_msg` `Attach succeeded` / `Detach succeeded`) or was cancelled (`error_msg` starts with `Cancelled`); while running `error_msg` is `""`, after a failure it is the reason. `error_code` / `error_msg` are written together with the change to `IDLE`.

| `error_code` | Meaning |
|---|---|
| `OK` 0 | attach aligned / detach reached, or cancelled with `data: false` (see `error_msg`) |
| `TAG_LOST` 1 | tracked tag not seen for `lost_target_timeout` while running |
| `AMCL_SERVICE_FAIL` 2 | AMCL service missing / no response / wrong state, or the restore failed (whatever ended the cmd) |
| `LIDAR_SERVICE_FAIL` 3 | reserved, not reported yet (lidar safety is fire-and-forget) |
| `TIMEOUT` 4 | `dock_timeout` / `undock_timeout` |
| `OTHER` 5 | anything else (e.g. an exception) |
| `NODE_SHUTDOWN` 6 | `apriltag_control` shut down |

### TF Transforms

`apriltag_detection` broadcasts `camera_frame → <family>_<id>` (e.g. `tag36h11_0`) for each detected tag.

## Logging

Both launch files print log lines without time and without ros2 launch's `[<process>-N]` prefix (`output_format='{line}'`). Every `apriltag_control` message starts with its current state:

```
[INFO] [wheelchair.apriltag_control][stage LEAVING] : [publish] /pre_cmd_vel vx=-0.100 vy=+0.000 wz=+0.000
[INFO] [wheelchair.apriltag_detection]: tag_families=tag36h11 tag_size=0.0475 tag_sizes={} tracking=tag36h11 id 0. Detecting continuously; publishing all tags on apriltag_poses, the tracked one on tag_tracing.
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

### `april_tag_config.json`

`april_tag.launch.py` takes only `config_file` (default `/mnt/work_space/april_tag_config.json`, the workspace root) and passes it to `apriltag_detection` and `apriltag_control`. Both nodes read the file at startup; a missing file, invalid JSON or a wrong type / value makes the node exit at startup.

**Reload** — `apriltag_control` re-reads the file each time an attach / detach is accepted in `IDLE`; an invalid file rejects that cmd (`rejected: Config reload failed: …`) and nothing is changed. `apriltag_detection` reads it only at startup. Keys marked *startup only* need a restart (`docker restart april_tag_serive`) to take effect.

| Key | Shipped value | Used by | Reload | Description |
|---|---|---|---|---|
| `detection.detect_tag_families` | `tag36h11` | detection | startup only | Families detected, space separated; one `pupil_apriltags` detector per family (one `Detector` only loads one family), CPU grows with each. `tag_size` is shared by all families |
| `detection.tracking_tag_family` / `tracking_tag_id` | `tag36h11` / `0` | detection, control | detection: startup only; control: each cmd | Tag published on `tag_tracing` (the closest one of this family / id, `-1`: any id) and checked by control. The family must be in `detect_tag_families` and the id ≥ -1. Changing it without restarting detection makes control see a non-matching tag → TAG_LOST |
| `tag_size.tag_size` | `0.0475` m | detection, control | detection: startup only; control: each cmd | **Edge of the tag's black square (8 × 8 cells, without the white border). Must match the printed tag**; control uses it only for the S4 yaw-precision table |
| `tag_size.tag_sizes` | `""` | detection, control | as `tag_size` | Per-id sizes `'id:size ...'` (m, any family), e.g. `'3:0.095'`; other ids use `tag_size` |
| `control.manage_amcl_and_lidar_safety` | `false` | control | each cmd | `false` skips every AMCL and lidar safety call (tracking is never blocked by them) |
| `control.stage1_tolerance` / `stage2_tolerance` | `{"x": 0.05, "y": 0.05, "yaw": 0.05, "hold_seconds": 1.0}` each | control | each cmd | In-position check of Stage 1 / Stage 2: \|x\|, \|y\| (m) and \|yaw\| (rad, filtered size minus `yaw_ref`) within these for `hold_seconds` (s). x / y / yaw > 0, `hold_seconds` ≥ 0. `stage1_tolerance` x / y also gate the S3 yaw probe |
| `control.dock_timeout` / `undock_timeout` | `300.0` s | control | each cmd | attach / detach timeout, from cmd accepted until it ends (> 0) |
| `control.lost_target_timeout` | `1.0` s | control | each cmd | Tracked tag not seen for this long while running → TAG_LOST (> 0) |
| `control.max_vx` / `max_vy` / `max_vw` | `0.1` / `0.1` m/s, `0.05` rad/s | control | each cmd | Stage 1 / 2 per-axis velocity saturation (> 0); leave backs up at `-max_vx` |
| `control.min_vx` / `min_vy` / `min_vw` | `0.01` / `0.01` m/s, `0.01` rad/s | control | each cmd | Stage 1 / 2 per-axis minimum (G7+ deadband): after the max clip, a non-zero \|v\| below min is sent as `sign(v)·min`; `0` stays `0` (≥ 0, ≤ max; `0` disables). Not applied to leave or stop commands |
| `calibration.stage1` / `stage2` / `detach` | `z_ref` 0.50 / 0.28 / 1.0 m; stage1 / stage2 `x_ref` 0.026 m, `yaw_ref` 0 | control | each cmd | **Required.** Target pose of Stage 1 / Stage 2 / leave: `z_ref` = target forward distance (> 0; `detach`: leave distance), `x_ref` (m) / `yaw_ref` (rad) = lateral / yaw reference of stage1 / stage2. `detach` has only `z_ref` (its `x_ref` / `yaw_ref` are 0: during Detaching, status `y_error` is the raw lateral offset); other keys are ignored. Written by `tools/calibrate.py` (see [Calibration](#calibration)) |
| `yaw_estimator.s4_distances` | `[0.3, 0.5, 0.7]` m | control | each cmd | Yaw precision table: equivalent-distance breakpoints; beyond the last one yaw is not used. Non-empty, same length as `s4_sigmas_deg` |
| `yaw_estimator.s4_sigmas_deg` | `[1.0, 2.0, 3.0]` deg | control | each cmd | Yaw precision table: sigma at each breakpoint |
| `ros_interfaces.amcl_check_service` / `amcl_close_service` / `amcl_open_service` | `/check_mcl_if_trigger` / `/close_amcl` / `/open_amcl` | control | startup only | G7+ AMCL services (absolute names) |
| `ros_interfaces.lidar_safety_topic` | `/g7_plc/disable_lidar_safety` | control | startup only | G7+ PLC lidar safety (`Bool`, `true` disables the safety field) |
| `ros_interfaces.stage1_cmd_vel_topic` | `/cmd_vel` | control | startup only | Stage 1 velocity commands |
| `ros_interfaces.stage2_cmd_vel_topic` | `/pre_cmd_vel` | control | startup only | Stage 2 and leaving velocity commands (G7+ precision mode) |

`config_file` is required by both nodes (`config_file is required` otherwise). `apriltag_detection` checks only the shared sections, so it starts with incomplete calibration; `apriltag_control` additionally requires both tolerances and all three calibration points, and rejects removed keys with a hint: `control.stage1_distance` / `stage2_distance` / `leave_distance` (→ `calibration.stage1` / `stage2` / `detach` `.z_ref`) and `calibration.use_calibration` (calibration is always used).

### ROS parameters not in the JSON (`apriltag_control`)

| Parameter | Default |
|---|---|
| `config_file` | `''`, required (`april_tag.launch.py` passes `/mnt/work_space/april_tag_config.json`) |
| `body_status_topic` | `/g7_body/simple_ui/body_status` |
| `external_service_timeout` | `5.0` s (wait for service + wait for response, each) |
| yaw estimator / S3 probe tuning (`quality_border_px`, `ref_tag_size`, `yaw_*`, `probe_*`, …) | see `AprilTagControlNode.__init__()`; `ref_tag_size` (`0.0475`) is the base size of the S4 table, change only when the table is recalibrated |

### Hard-coded values (`apriltag_control`)

| Value | Setting | Where | Description |
|---|---|---|---|
| `max_dt` | `0.2` s | `control_node.py` | Timestep cap (handles frame drops) |
| `smooth_tau` | `0.5` s | `control_node.py` (`TrajectoryPlanner`) | Reference smoothing time constant |
| LQR Q weights | `(1.0, 1.0, 0.5)` | `control.py` default | State cost: x, y, yaw |
| LQR R weights | `(3.0, 1.0, 3.0)` | `control_node.py` | Control cost: vx, vy, vw (vy is penalised least, so lateral correction is fastest) |

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
│       │   ├── attach.py / detach.py    # test tools: call one service each (true / false), print the response
│       │   ├── wheelchair_cmd.py        # shared code of attach.py / detach.py
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

1. **Stage 1** — track to `calibration.stage1` (`z_ref` 0.50 m shipped). When all errors stay within `control.stage1_tolerance` for its `hold_seconds`, the robot stops and Stage 2 disables the lidar safety field before it moves.
2. **Stage 2** — track to `calibration.stage2` (`z_ref` 0.28 m shipped). When within `control.stage2_tolerance` for its `hold_seconds` the robot stops and the node returns to `IDLE` (AMCL closed, lidar safety disabled); then the cmd succeeds.
3. **Leave** (`detach`) — no planner / LQR: every tag pose publishes `vx = -max_vx, vy = 0, wz = 0` on `/pre_cmd_vel` until the camera-to-tag forward distance ≥ `calibration.detach.z_ref`; y / yaw are not checked and there is no hold time. Then the robot stops (`Leave reached 1.00m (forward=…m).`) and AMCL / lidar safety are restored. Losing the tag for `lost_target_timeout` stops the robot and aborts the leave.

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
