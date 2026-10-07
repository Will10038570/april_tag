# AprilTag 開發 / 測試指南(Docker)

> **規則:所有程式碼的 build、執行、測試一律在 Docker container 內進行,不要在 host 的 `/opt/ros/humble` 上跑。**
> Docker 環境定義在 `/home/cobo/Desktop/Will/work_space/docker/`(`Dockerfile`、`docker-compose.yml`、`ros_entrypoint.sh`)。

套件功能、topic / service / action 介面、控制架構請看 `src/apriltag/README.md`;本文件只說明「怎麼在 Docker 裡跑起來與驗證」。

---

## 1. Docker 環境

| 項目 | 值 |
|---|---|
| Image | `ros2_april_tag:v1`(base `ros:humble-ros-base`) |
| Container | `april_tag_serive`(拼字照 compose 設定,不是 service) |
| Workspace 掛載 | host `Will/work_space` → container `/mnt/work_space` |
| 網路 | `network_mode: host`、`ipc: host`、`privileged: true`(RealSense 走 `/dev`) |
| `ROS_DOMAIN_ID` | `63`(與機器人相同) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | 只綁有線網卡 `enP8p1s0`(192.168.6.0/24,本機 192.168.6.11,機器人在 192.168.6.10),peer 為 `192.168.6.10`、`MaxAutoParticipantIndex=50`。**不要再加 `lo`**:多網卡時 Cyclone 會從 `lo` 的 socket 送 unicast 到 192.168.6.10,kernel 拒絕(`ddsi_udp_conn_write to udp/192.168.6.10:… failed with retcode -3`),實測會導致完全找不到機器人 node(2026-10-06)。網路線沒插時 node 會建不起來,離線測試請照 3.2 把 `CYCLONEDDS_URI` 改成只用 `lo` |
| 已安裝 | librealsense2、`pyrealsense2`、`realsense2_camera`、`cv_bridge`、`python3-opencv`、`rqt_image_view`、`rmw_cyclonedds_cpp`、`pupil-apriltags`、`scipy`、`numpy<2` |

`numpy` 必須維持 `<2`:`cv_bridge` 是以 numpy 1.x C-API 編譯的。

Container 的 entrypoint 與 `.bashrc` 會自動 source `/opt/ros/humble/setup.bash` 與 `/mnt/work_space/install/setup.bash`(若存在)。`.bashrc` 只在**互動式** shell 生效:`docker exec -it … bash` 或 `docker exec … bash -ic '…'` 可直接用 `ros2`;`bash -lc` **不會** source(2026-10-02 實測 `which ros2` 找不到),非互動時請用 `bash -ic`,或在指令開頭自行 `source /opt/ros/humble/setup.bash && source /mnt/work_space/install/setup.bash`。

`install/` 內的 Python 套件是複製的(不是 symlink),改了 `.py` 一定要重新 `colcon build` 才會生效。

### 啟動 container

```bash
# 第一次建立(或 Dockerfile 有改)
cd /home/cobo/Desktop/Will/work_space/docker
docker compose build
docker compose up -d

# 已建立過,只是停掉了
docker start april_tag_serive

# 進入 container
docker exec -it april_tag_serive bash
```

GUI(dashboard、`rqt_image_view`)需要在 host 的圖形桌面 session 裡啟動 container,`$DISPLAY` / `$XAUTHORITY` 才會帶進去;必要時先在 host 執行 `xhost +local:docker`。
若 container 不是在桌面 session 建立的(container 內 `echo $DISPLAY` 為空,`/tmp/.docker.xauth` 變成空目錄),不重建的做法:host 執行 `xhost +local:docker`,再 `docker exec -it -e DISPLAY=:1 april_tag_serive bash`,container 內 `unset XAUTHORITY` 後啟動 GUI;永久修正則在桌面 session 執行 `docker compose up -d --force-recreate`。

**透過 SSH(無圖形桌面)** 時改用無 GUI 版 `docker/docker_compose_UI_off.yml`(移除 `DISPLAY`/`XAUTHORITY` 與 X11 mount,其餘設定相同)。它與 `docker-compose.yml` 同 project / service / container 名稱,所以會直接取代現有 container:

```bash
cd /home/cobo/Desktop/Will/work_space/docker
docker compose -f docker_compose_UI_off.yml up -d --force-recreate   # Dockerfile 有改時加 --build
```

這個 container 內 dashboard / `rqt_image_view` 無法使用,虛擬測試請用 `headless:=true`;之後要切回 GUI 版,在桌面 session 執行 `docker compose up -d --force-recreate`。

搬到別台機器的方式見 `Will/docker_copy_guide.txt`(Jetson 是 ARM,`docker save` 的 image 不能用在 x86)。

---

## 2. Build

```bash
docker exec april_tag_serive bash -ic '
  cd /mnt/work_space &&
  colcon build --symlink-install'
```

只 build 單一套件:`colcon build --symlink-install --packages-select apriltag`。
改了 `apriltag_interfaces/action/*.action` 或 `msg/*.msg`(`TagPose`、`TagPoseArray`)要先 build `apriltag_interfaces`。
`realsense` 套件 build 時會有 stderr 輸出,不影響結果。

---

## 3. 測試

### 3.1 Controller 單元 demo(不需 ROS 通訊)

```bash
docker exec april_tag_serive bash -ic '
  cd /mnt/work_space/src/apriltag && python3 -m apriltag.control'
```

預期:先印 `LQR demo …` 與 `initial error`,再印 `step 00 … step 49` 的 `u / ref / err`,error 逐步收斂,exit code 0。

### 3.2 虛擬端到端測試(建議每次改 code 後必跑)

`virtual_tracking.launch.py` 會啟動真正的 `apriltag_detection`、`apriltag_control`,以及取代相機 / AMR / G7+ AMCL 與 lidar safety 的 `virtual_tracking_sim`。一次 run = `start_tracking`(Stage 1 → Stage 2 → 回 `IDLE`,AMCL 關、lidar safety disabled)接著 `leave_cs`(退到 `leave_distance` 後還原 AMCL / lidar safety)。

**安全注意:** `virtual_tracking.launch.py` 的 `domain_id` 預設是 65(機器人在 63,但仍請勿依賴這點),而 `apriltag_control` 會發 `/cmd_vel`、`/pre_cmd_vel`、`/g7_plc/disable_lidar_safety` 並呼叫 AMCL service。跑虛擬測試時:

- 拔掉機器人網路線,**或**
- 改用其他 domain(`domain_id:=99`)並讓 DDS 只走 loopback(見下方指令)。

Headless(無視窗,自動依序送 `start_tracking`、`leave_cs`,存檔後結束):

```bash
docker exec april_tag_serive bash -ic '
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>"
  mkdir -p /mnt/work_space/virtual_tracking_logs && cd /mnt/work_space &&
  ros2 launch apriltag virtual_tracking.launch.py domain_id:=99 \
      headless:=true auto_start:=true \
      init_x:=-1.0 init_y:=0.15 init_yaw_deg:=10.0'
```

通過標準(看 log):

```
[step] STAGE2 running -> ending (Stage 2 aligned at 0.28m)
[state] STAGE2 -> IDLE (cleanup done)
[action] start_tracking result: SUCCEEDED success=True message='Stage 2 aligned at 0.28m. ... AMCL off, lidar safety disabled; send leave_cs to restore. ...'
aligned: amcl=off lidar_safety=disabled (held)
[state] IDLE -> LEAVING (leave_cs goal accepted, back to 1.00m)
[publish] /pre_cmd_vel vx=-0.100 vy=+0.000 wz=+0.000
[action] leave_cs result: SUCCEEDED success=True message='Leave reached 1.00m (forward=1.0xxm). ...'
after goal: amcl=on lidar_safety=enabled (restored)
Run saved to .../virtual_tracking_logs/<time>_succeeded_*.csv
```

`(held)` 表示 Stage 2 成功回 `IDLE` 後 AMCL 仍關、lidar safety 仍 disabled(正確);若印 `(NOT held)` 表示太早還原。
結束時 `apriltag_control` 的 shutdown 一律還原(AMCL check / open),此時 sim 的 fake AMCL 可能已經結束,會印 `Cannot check AMCL: service /check_mcl_if_trigger not available.`、`Restore failed` 與 launch 的 `escalating to 'SIGTERM'`,不影響測試結果。

輸出檔在**執行目錄**下的 `virtual_tracking_logs/`(`log_dir` 預設是相對路徑):`*_sim.csv`、`*_feedback.csv`、`*_cmd_raw.csv`、`*.png`。所以請先 `cd /mnt/work_space` 再 launch,否則 log 會散落到別的目錄(例如 `src/apriltag/launch/`、`src/apriltag/tools/` 底下)。

有 dashboard 的互動版(需 `DISPLAY`):

```bash
docker exec -it april_tag_serive bash -ic '
  cd /mnt/work_space &&
  ros2 launch apriltag virtual_tracking.launch.py'
```

按鍵:`s` 開始、`l` 離開(送 `leave_cs`;sim 在 `ALIGNED` 時接續同一個 run,沒有 run 時為只有 leave 的 run)、`x` 取消進行中的 goal(`ALIGNED` 時沒有東西可停)、`r` 重置、`a`/`d` 旋轉初始 yaw(±2°,僅未執行時)、`+`(或 `=`)/`-` 縮放、`q` 或 `Esc` 離開(會連同整個 launch 一起結束)。未執行時,上半部畫面可用滑鼠拖曳車體移動、拖曳把手或滾輪旋轉,也可點畫面上的按鈕。

| Launch 參數 | 預設 |
|---|---|
| `domain_id` | `65` |
| `init_x` / `init_y` / `init_yaw_deg` | `-1.0` / `0.15` / `10.0` |
| `headless` / `auto_start` | `false` / `false` |
| `log_dir` | `virtual_tracking_logs` |
| `manage_amcl_and_lidar_safety` | `true`(注意:與 `april_tag.launch.py` 的預設 `false` 不同) |
| `stage1_distance` / `stage2_distance` / `leave_distance` | `0.50` / `0.28` / `1.0`(sim 會從 `apriltag_control` 讀回,不直接設在 sim 上) |
| `detect_tag_families` / `tag_family` / `tag_id` | `tag36h11` / `tag36h11` / `0`(sim 畫的是 tag36h11 id 0;`tag_id:=5` 可測過濾:約 1 s 後 `AprilTag lost` ABORTED 並還原) |

手動送 goal 測試(另開終端,同樣設 `ROS_DOMAIN_ID=99` 與 `lo` 的 `CYCLONEDDS_URI`):用 `headless:=true auto_start:=false` 啟動,sim 的物理與相機照常運作,但不會自己送 goal。

**背景 process 注意:** `docker exec` 被中斷(或 Claude Code 的指令被拒絕)時,container 內已啟動的 process **不會**停止;非互動 bash 的背景 job 也會忽略 SIGINT。測完務必確認並清掉殘留(`[a]` 寫法避免 `pkill` 比對到自己):

```bash
docker exec april_tag_serive bash -c 'ps aux | grep -E "[a]priltag_|[v]irtual_tracking" || echo "nothing running"'
docker exec april_tag_serive bash -c 'pkill -INT -f "[a]priltag_control --ros-args"; pkill -INT -f "[a]priltag_detection --ros-args"; pkill -INT -f "[v]irtual_tracking_sim --ros-args"'
```

### 3.3 `colcon test`

```bash
docker exec april_tag_serive bash -ic '
  cd /mnt/work_space &&
  colcon test --packages-select apriltag && colcon test-result --all'
```

`src/apriltag/test/test_state_machine.py` 測 `TrackingState` 轉移表與 goal 接受表(43 tests),加上 `test_leave_step.py`(4)與 `test_target_flow.py`(6,tag 選擇)共 53。新增的 pytest 請放在 `src/apriltag/test/`,同樣用上面的指令在 container 內執行。

---

## 4. 實機執行

前提:有線網卡 `enP8p1s0` 已接上機器人且為 UP,RealSense 已插上。

```bash
# 終端 1:相機 + detection + control
docker exec -it april_tag_serive bash -ic '
  ros2 launch apriltag april_tag.launch.py manage_amcl_and_lidar_safety:=true'
# 指定要追的 tag / 偵測多個 family:tag_family:=tag36h11 tag_id:=3 detect_tag_families:="tag36h11 tag25h9"

# 終端 2:開始對位(成功後回 IDLE,AMCL 關、lidar safety disabled)
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: true}"'
# 離開(IDLE 時接受,不需先對位;退到 leave_distance,完成後開 AMCL、enable lidar safety)
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/leave_cs apriltag_interfaces/action/StartTracking "{start: true}"'
# 停止對位(僅 STAGE1 / STAGE2 接受,停止並還原)
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: false}"'
# 停止離開(僅 LEAVING 接受,停止並還原)
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/leave_cs apriltag_interfaces/action/StartTracking "{start: false}"'
```

`apriltag_control` 依 launch 參數 `tag_family` / `tag_id`(預設 `tag36h11` / `0`,`-1` = 任何 id)從 `/up/apriltag_poses` 挑要追的 tag,同時有多個符合時取前向距離 z 最小(且 > 0)的;沒有符合的就當作 tag 不見(`lost_target_timeout` 1 s 後 abort)。

`apriltag_control` 是狀態機:`IDLE → STAGE1 → STAGE2 → IDLE`(`start_tracking`)與 `IDLE → LEAVING → IDLE`(`leave_cs`),每個非 IDLE state 內有 `preparing → running → ending` 三個步驟(詳見 README 的 State Machine)。AMCL / lidar safety 由各 state 自己切換:`STAGE1` 準備時 enable lidar、關 AMCL;`STAGE2` 準備時 disable lidar;Stage 2 成功只停車就回 `IDLE`(AMCL 維持關、lidar 維持 disabled);`LEAVING` 結束、任何失敗 / 取消 / 停止與 shutdown 都會 enable lidar 並確保 AMCL 打開(不記錄是誰關的)。接受規則:`start_tracking` `start:true` / `leave_cs` `start:true` 只在 `IDLE`;`start_tracking` `start:false` 只在 `STAGE1` / `STAGE2`;`leave_cs` `start:false` 只在 `LEAVING`;action cancel 一律接受。其餘會被拒絕並印 `[action] … goal rejected: … state=…`。

`april_tag.launch.py` 的 `manage_amcl_and_lidar_safety` 預設為 `false`(不動 AMCL / lidar safety);距離寫死在 launch 裡(`stage1_distance=0.50`、`stage2_distance=0.28`、`leave_distance=1.0`),Stage 1 速度發到 `/cmd_vel`,Stage 2 與 leave 發到 `/pre_cmd_vel`(G7+ 精準模式)。leave 不用 planner / LQR:固定發 `vx=-max_vx`(-0.1 m/s)、`vy=wz=0`,直到 camera→tag 前向距離 ≥ `leave_distance` 立即停(不檢查 y / yaw、無 hold)。

`apriltag_detection` 啟動後就一直偵測,不受 `apriltag_control` 控制(沒有 enable service)。每張影像(收到 `camera_info` 之後)發一則 `apriltag_interfaces/msg/TagPoseArray` 到 `/up/apriltag_poses`,包含 `tag_families`(launch 參數 `detect_tag_families`,以空白分隔,預設 `tag36h11`;每個 family 一個 detector,CPU 隨數量增加)內所有偵測到的 tag(`family`、`id` 字串、`pose`),沒有 tag 時 `tags` 為空;每個 tag 廣播 TF `<camera_frame> → <family>_<id>`(如 `tag36h11_0`)。沒送 goal 時 `apriltag_control` 會忽略這些資料。

### 連線 / G7+ 介面檢查(在 container 內)

```bash
ros2 node list                                         # 看得到機器人 node → DDS / domain 通
ros2 service list | grep mcl                           # 三個 AMCL service
ros2 topic info -v /g7_plc/disable_lidar_safety        # ads_bridge_node 有訂閱

ros2 service call /check_mcl_if_trigger std_srvs/srv/Trigger "{}"   # success: true = AMCL 開
ros2 service call /close_amcl std_srvs/srv/Empty "{}"
ros2 service call /open_amcl  std_srvs/srv/Empty "{}"
```

除錯用:

```bash
ros2 topic echo /up/apriltag_poses       # 一直有資料(沒 tag 時 tags: [])
ros2 run apriltag detection_viewer --ros-args -r __ns:=/up   # 最多 5 Hz 印 TagPoseArray:一行 header + 每個 tag 一行
ros2 topic echo /cmd_vel                 # Stage 1
ros2 topic echo /pre_cmd_vel             # Stage 2 / leave(leave 時 vx 為負)
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11_0   # 單一 tag 的 TF(frame 依相機而定)
ros2 run rqt_image_view rqt_image_view /up/apriltag/marked_image
```

---

## 5. 疑難排解

| 現象 | 原因 / 處理 |
|---|---|
| `rmw_create_node: failed to create domain` / `rcl node's rmw handle is invalid` | `CYCLONEDDS_URI` 綁的 `enP8p1s0` 是 DOWN 或不存在(網路線沒插)。接上網路線,或照 3.2 把 `CYCLONEDDS_URI` 改成 `lo` |
| `ddsi_udp_conn_write to udp/192.168.6.10:… failed with retcode -3`,且 `ros2 node list` 看不到機器人(ping 卻通) | `CYCLONEDDS_URI` 同時有 `enP8p1s0` 與 `lo`(或 peer 有 `127.0.0.1`),從 `lo` socket 送往外部位址被 kernel 拒絕。只留 `enP8p1s0`(見第 1 節),改 compose 後 `docker compose up -d --force-recreate` |
| `selected interface "lo" is not multicast-capable` | 用 `lo` 時的正常警告,可忽略 |
| headless 測試結束時 `virtual_tracking_sim` `exit code -11` | 已知:存完 CSV / PNG 後 process 結束時 segfault,不影響測試結果 |
| `ros2: command not found` | `bash -lc` 不會 source ROS;改用 `bash -ic '…'`、`docker exec -it … bash`,或手動 source |
| 改了 code 但行為沒變 | `install/` 是複製的,重新 `colcon build` |
| 虛擬測試結果很亂、goal 被不明 node 處理 | 之前的測試 process 沒清掉,同一 domain 有兩個 `apriltag_control`;見 3.2 的清理指令 |
| `cv_bridge` import 錯誤 / numpy ABI 錯誤 | container 內 numpy 被升到 2.x;`pip3 install "numpy<2"` 並把修正寫回 Dockerfile |
| GUI 開不起來(`Can't initialize GTK backend`) | container 內 `DISPLAY` 為空(不是在圖形桌面 session 中建立)或 X server 未授權;見第 1 節的 `-e DISPLAY=:1` 做法 |

---

## 6. 已驗證紀錄

2026-10-02 在 Jetson(`Linux 5.15.148-tegra`)的 `april_tag_serive` container 內實測:

- `colcon build --symlink-install`:3 packages 成功
- `python3 -m apriltag.control`:exit 0,error 收斂
- 虛擬測試 headless(`domain_id:=99`、DDS 走 `lo`):`SUCCEEDED`,Stage 2 對齊 0.28 m(elapsed 12.2 s),`amcl=on lidar_safety=enabled (restored)`
- `colcon test --packages-select apriltag`:0 tests(尚無測試檔)

2026-10-02 加入狀態機與 `leave_cs` 後,在同一 container 內實測:

- `colcon test --packages-select apriltag`:59 tests,0 failures
- 虛擬測試 headless(`domain_id:=99`、`lo`):`STAGE2 -> IN_POSITION`、`(held)`、`Leave aligned at 0.40m`、`SUCCEEDED`、`(restored)`
- CLI 情境:各狀態拒絕 goal 並有 log;`IN_POSITION` 時 `start_tracking start:false` 放棄對位並還原;`LEAVING` 時 `start_tracking`/`leave_cs` `start:false` 停止 leave 並還原;`IN_POSITION` / `STAGE1` / `LEAVING` 時 SIGINT,還原順序為 lidar enable → open AMCL → disable detection;AMCL service 不存在時 `STARTING -> FINISHING -> IDLE`
- 實機尚未測(G7+ `/pre_cmd_vel` 收到負 vx 的實際動作未確認)

2026-10-05 leave 改為「直線後退」(不用 planner / LQR,`leave_distance=1.0`)後,在同一 container 內實測:

- `colcon test --packages-select apriltag`:63 tests(含 `test_leave_step.py` 4 個),0 failures
- 虛擬測試 headless(`domain_id:=99`、`lo`):`(held)`、`IN_POSITION -> LEAVING (… back to 1.00m)`、leave 期間 `*_cmd_raw.csv` 只有 `(-0.1, 0, 0)` 與結束時的零速、`Leave reached 1.00m (forward=1.005m)`(leave elapsed 7.0 s)、`SUCCEEDED`、`(restored)`
- 實機尚未測

2026-10-06 狀態機改為 `IDLE` / `STAGE1` / `STAGE2` / `LEAVING`(移除 `STARTING` / `IN_POSITION` / `FINISHING`,改用 state 內步驟)後,在同一 container 內實測:

- `colcon test --packages-select apriltag`:47 tests,0 failures
- 虛擬測試 headless(`domain_id:=99`、`lo`):`STAGE2 -> IDLE (cleanup done)`、`aligned: … (held)`、`IDLE -> LEAVING`、`Leave reached 1.00m`、`SUCCEEDED`、`(restored)`;leave 期間 `*_cmd_raw.csv` 只有 `(-0.1, 0, 0)` 與零速
- CLI 情境(`headless:=true auto_start:=false`):`IDLE` 拒絕 `start_tracking`/`leave_cs` `start:false`;`IDLE` 直接 `leave_cs` 成功(AMCL 已開,不呼叫 open);`STAGE1` 拒絕兩個 `start:true`,`start_tracking false` 停止並還原;Stage 2 成功後再 `start_tracking` 會先 enable lidar;`LEAVING` 拒絕 `start_tracking false`,`leave_cs false` 停止並還原;`STAGE2` 時 action cancel → `CANCELED` 並還原;Stage 2 成功後在 `IDLE` SIGINT → enable lidar、AMCL check off → open → check on
- 未測:`STAGE1` preparing 期間 SIGINT(sim 的 fake AMCL 回應太快,難以卡在該時間點);實機尚未測

2026-10-06 detection 改為持續偵測、發布所有 tag(`TagPoseArray` on `/up/apriltag_poses`,移除 `/up/apriltag_pose` 與 `~/enable`,control 以 `tag_family` / `tag_id` 選 tag,刪除 `tools/print_tag_pose.py`)後,在同一 container 內實測:

- `colcon test --packages-select apriltag`:53 tests,0 failures
- 虛擬測試 headless(`domain_id:=99`、`lo`):`STAGE2 -> IDLE`、`(held)`、`Leave reached 1.00m (forward=1.000m)`、`SUCCEEDED`、`(restored)`
- `auto_start:=false` 沒送 goal 時 `/up/apriltag_poses` 有資料(`family: tag36h11`、`id: '0'`)、沒有 `apriltag_detection` 的 enable service、`tf2_echo … tag36h11_0` 有 transform
- `tag_id:=5`:`AprilTag lost for 1.00s` → ABORTED、`max |cmd|` 全 0、`(restored)`(過濾有效,`tag_id:=5` 以字串傳入無型別錯誤)
- `tag_families:="tag36h11 tag25h9"` 啟動正常(多 family 實際偵測未測,sim 只畫 tag36h11);實機尚未測

## 7. `src/apriltag/README.md` 與實際不一致之處

- (2026-10-06 已修正 launch 檔名與 `test/` 目錄;目前無已知不一致)
