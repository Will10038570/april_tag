# Wheelchair Interface: Return Data

## 1. Service `/wheelchair/attach` and `/wheelchair/detach`

**Type:** `std_srvs/srv/SetBool`

### Request

| Field | Type | Value |
|---|---|---|
| `data` | `bool` | `true` = start the cmd<br>`false` = cancel the running cmd of the same service |

### Response

| Field | Type | Value |
|---|---|---|
| `success` | `bool` | `true` = accepted, `false` = rejected |
| `message` | `string` | plain text, see below |

### Every possible `message`

#### `/wheelchair/attach`

- **`"attach accepted"`** — when the state is `IDLE` and attach `true` was received *(success true)*
- **`"rejected: state=STAGE1 (only accepted in IDLE)"`** — when the state is `STAGE1` and attach `true` was received *(success false)*
- **`"rejected: state=STAGE2 (only accepted in IDLE)"`** — when the state is `STAGE2` and attach `true` was received *(success false)*
- **`"rejected: state=LEAVING (only accepted in IDLE)"`** — when the state is `LEAVING` and attach `true` was received *(success false)*
- **`"nothing running"`** — when the state is `IDLE` and attach `false` was received *(success false)*
- **`"cancel requested"`** — when the state is `STAGE1` / `STAGE2` and attach `false` was received *(success true)*
- **`"already ending, result unchanged"`** — when the state is `STAGE1` / `STAGE2` and attach `false` was received, but the attach was already ending (e.g. Stage 2 just aligned); the status keeps the real result *(success true)*
- **`"rejected: state=LEAVING (only accepted in STAGE1, STAGE2)"`** — when the state is `LEAVING` and attach `false` was received *(success false)*

#### `/wheelchair/detach`

- **`"detach accepted"`** — when the state is `IDLE` and detach `true` was received *(success true)*
- **`"rejected: state=STAGE1 (only accepted in IDLE)"`** — when the state is `STAGE1` and detach `true` was received *(success false)*
- **`"rejected: state=STAGE2 (only accepted in IDLE)"`** — when the state is `STAGE2` and detach `true` was received *(success false)*
- **`"rejected: state=LEAVING (only accepted in IDLE)"`** — when the state is `LEAVING` and detach `true` was received *(success false)*
- **`"nothing running"`** — when the state is `IDLE` and detach `false` was received *(success false)*
- **`"cancel requested"`** — when the state is `LEAVING` and detach `false` was received *(success true)*
- **`"already ending, result unchanged"`** — when the state is `LEAVING` and detach `false` was received, but the detach was already ending (e.g. `leave_distance` just reached); the status keeps the real result *(success true)*
- **`"rejected: state=STAGE1 (only accepted in LEAVING)"`** — when the state is `STAGE1` and detach `false` was received *(success false)*
- **`"rejected: state=STAGE2 (only accepted in LEAVING)"`** — when the state is `STAGE2` and detach `false` was received *(success false)*

---

## 2. Topic `/wheelchair/status`

| | |
|---|---|
| **Type** | `std_msgs/msg/String` (`data` = JSON string) |
| **QoS** | reliable, transient_local, depth 1 |
| **Rate** | 5 Hz |

### JSON example

```json
{"state": "Attaching(STAGE2)", "cmd_id": 1, "attached": false, "error_code": 0, "error_msg": "",
 "stamp": 1792828800.123, "x_error": 0.012, "y_error": -0.004, "yaw_error": 0.021,
 "last_frame_age": 0.035}
```

### Fields

| Field | Type | Content |
|---|---|---|
| `state` | `string` | `"IDLE"` \| `"Attaching(STAGE1)"` \| `"Attaching(STAGE2)"` \| `"Detaching"` |
| `cmd_id` | `int` | current / last cmd id (0 after node start, before any cmd) |
| `attached` | `bool` | always `false` (gripper is not sensed) |
| `error_code` | `int` | result code, see [`error_code`](#error_code) (0 while running) |
| `error_msg` | `string` | `""` while running / after node start; result text after the cmd ended, see [`error_msg`](#error_msg) |
| `stamp` | `float` | publish time, epoch seconds (wall clock) |
| `x_error` | `float` \| `null` | debug: forward error to target distance [m]<br>attach: filtered forward distance − stage distance (0.50 / 0.28)<br>detach: raw value from the tag pose |
| `y_error` | `float` \| `null` | debug: lateral error [m] |
| `yaw_error` | `float` \| `null` | debug: yaw error [rad] |
| `last_frame_age` | `float` \| `null` | debug: node time − `header.stamp` of the newest frame with the tracked tag [s] |

`null` = unknown (no tag data yet in this stage / not finite).

### `error_code`

| Code | Name | Meaning |
|---|---|---|
| 0 | `OK` | succeeded, or cancelled by `data:false` (`error_msg` starts with `"Cancelled"`) |
| 1 | `TAG_LOST` | tracked tag not seen for `lost_target_timeout` (default 1.0 s) while running |
| 2 | `AMCL_SERVICE_FAIL` | AMCL service missing / no response / wrong state (`"Failed to close AMCL."`), or restore failed |
| 3 | `LIDAR_SERVICE_FAIL` | reserved, not used |
| 4 | `TIMEOUT` | `dock_timeout` / `undock_timeout` (default 300 s), e.g. `"Attach timed out after 300.0s."` |
| 5 | `OTHER` | any other failure |
| 6 | `NODE_SHUTDOWN` | `apriltag_control` shut down (`"Node shutting down."`) |

### `error_msg`

Every possible value; written together with state `"IDLE"` when the cmd ends.

- `<t>` = seconds since the cmd was accepted, 2 decimals (e.g. `elapsed=12.78s`)
- `<n>` = the cmd_id

#### Normal

- **`""`** — when a cmd is running (state is not `IDLE`), or in `IDLE` after node start (`cmd_id` 0)
- **`"Attach succeeded"`** — when attach succeeded (Stage 2 aligned) *(error_code 0)*
- **`"Detach succeeded"`** — when detach succeeded (`leave_distance` reached) *(error_code 0)*
- **`"Cancelled cmd_id=<n>. elapsed=<t>s"`** — when attach / detach `false` was accepted and the cmd stopped *(error_code 0)*

#### Failure

- **`"AprilTag lost for 1.00s. Tracking stopped. elapsed=<t>s"`** — when the tracked tag was not seen for `lost_target_timeout` while running; the number is `lost_target_timeout` *(error_code 1)*
- **`"Failed to close AMCL. elapsed=<t>s"`** — when attach (`STAGE1`) or detach (`LEAVING`) started and AMCL could not be closed *(error_code 2)*
- **`"Attach timed out after 300.0s. elapsed=<t>s"`** — when attach ran longer than `dock_timeout` *(error_code 4)*
- **`"Detach timed out after 300.0s. elapsed=<t>s"`** — when detach ran longer than `undock_timeout` *(error_code 4)*
- **`"Attach ended unexpectedly. elapsed=<t>s"`** — when the attach ended without a result, e.g. an exception *(error_code 5)*
- **`"Detach ended unexpectedly. elapsed=<t>s"`** — when the detach ended without a result, e.g. an exception *(error_code 5)*
- **`"Node shutting down. elapsed=<t>s"`** — when `apriltag_control` shut down while a cmd was running *(error_code 6)*


### How to tell the result

| Condition | Result |
|---|---|
| `error_code != 0` | failed |
| `error_code == 0` and `error_msg` starts with `"Cancelled"` | cancelled |
| `error_code == 0` otherwise | succeeded |
