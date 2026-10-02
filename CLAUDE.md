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
| `ROS_DOMAIN_ID` | `65`(與機器人相同) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | 綁定有線網卡 `enP8p1s0`(192.168.5.0/24,機器人在 192.168.5.10),另加 `lo`;`enP8p1s0` 設 `presence_required="false"`,網路線沒插時自動只用 `lo`(node 啟動時決定一次,插線後要重新 launch) |
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
改了 `apriltag_interfaces/action/*.action` 要先 build `apriltag_interfaces`。
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

`virtual_tracking.launch.py` 會啟動真正的 `apriltag_detection`、`apriltag_control`,以及取代相機 / AMR / G7+ AMCL 與 lidar safety 的 `virtual_tracking_sim`。一次 run = `start_tracking`(Stage 1 → Stage 2 → `IN_POSITION`)接著 `leave_cs`(退到 `leave_distance` 後還原 AMCL / lidar safety)。

**安全注意:** 預設 domain 是 65(與機器人相同),而 `apriltag_control` 會發 `/cmd_vel`、`/pre_cmd_vel`、`/g7_plc/disable_lidar_safety` 並呼叫 AMCL service。跑虛擬測試時:

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
[state] STAGE2 -> IN_POSITION (Stage 2 aligned)
[action] start_tracking result: SUCCEEDED success=True message='Stage 2 aligned at 0.28m. ...'
in position: amcl=off lidar_safety=disabled (held)
[state] IN_POSITION -> LEAVING (leave_cs goal accepted, back to 0.40m)
[action] leave_cs result: SUCCEEDED success=True message='Leave aligned at 0.40m. ...'
after goal: amcl=on lidar_safety=enabled (restored)
Run saved to .../virtual_tracking_logs/<time>_succeeded_*.csv
```

`(held)` 表示 `IN_POSITION` 時 AMCL 仍關、lidar safety 仍 disabled(正確);若印 `(NOT held)` 表示太早還原。

輸出檔在**執行目錄**下的 `virtual_tracking_logs/`(`log_dir` 預設是相對路徑):`*_sim.csv`、`*_feedback.csv`、`*_cmd_raw.csv`、`*.png`。所以請先 `cd /mnt/work_space` 再 launch,否則 log 會散落到別的目錄(例如 `src/apriltag/launch/`、`src/apriltag/tools/` 底下)。

有 dashboard 的互動版(需 `DISPLAY`):

```bash
docker exec -it april_tag_serive bash -ic '
  cd /mnt/work_space &&
  ros2 launch apriltag virtual_tracking.launch.py'
```

按鍵:`s` 開始、`l` 離開(送 `leave_cs`,僅 `IN_POSITION`)、`x` 取消(`IN_POSITION` 時為放棄對位,送 `start_tracking` `start:false`)、`r` 重置、`a`/`d` 旋轉初始 yaw(±2°,僅未執行時)、`+`(或 `=`)/`-` 縮放、`q` 或 `Esc` 離開(會連同整個 launch 一起結束)。未執行時,上半部畫面可用滑鼠拖曳車體移動、拖曳把手或滾輪旋轉,也可點畫面上的按鈕。

| Launch 參數 | 預設 |
|---|---|
| `domain_id` | `65` |
| `init_x` / `init_y` / `init_yaw_deg` | `-1.0` / `0.15` / `10.0` |
| `headless` / `auto_start` | `false` / `false` |
| `log_dir` | `virtual_tracking_logs` |
| `manage_amcl_and_lidar_safety` | `true`(注意:與 `april_tag.launch.py` 的預設 `false` 不同) |
| `stage1_distance` / `stage2_distance` / `leave_distance` | `0.50` / `0.28` / `0.40`(sim 會從 `apriltag_control` 讀回,不直接設在 sim 上) |

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

`src/apriltag/test/test_state_machine.py` 測 `TrackingState` 轉移表(59 tests)。新增的 pytest 請放在 `src/apriltag/test/`,同樣用上面的指令在 container 內執行。

---

## 4. 實機執行

前提:有線網卡 `enP8p1s0` 已接上機器人且為 UP,RealSense 已插上。

```bash
# 終端 1:相機 + detection + control
docker exec -it april_tag_serive bash -ic '
  ros2 launch apriltag april_tag.launch.py manage_amcl_and_lidar_safety:=true'

# 終端 2:開始對位(成功後停在 IN_POSITION,AMCL 關、lidar safety disabled)
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: true}"'
# 離開(退到 leave_distance,完成後開 AMCL、enable lidar safety)
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/leave_cs apriltag_interfaces/action/StartTracking "{start: true}"'
# 停止任何進行中的對位 / 離開;在 IN_POSITION 時為放棄對位並還原
docker exec -it april_tag_serive bash -ic '
  ros2 action send_goal /up/start_tracking apriltag_interfaces/action/StartTracking "{start: false}"'
```

`apriltag_control` 是狀態機:`IDLE → STARTING → STAGE1 → STAGE2 → IN_POSITION → LEAVING → FINISHING → IDLE`(詳見 README 的 State Machine)。`start_tracking` `start:true` 只在 `IDLE` 接受;`leave_cs` `start:true` 只在 `IN_POSITION`、`start:false` 只在 `LEAVING` 接受;其餘會被拒絕並印 `[action] … goal rejected: … state=…`。`IN_POSITION` 沒有 timeout。

`april_tag.launch.py` 的 `manage_amcl_and_lidar_safety` 預設為 `false`(不動 AMCL / lidar safety);距離寫死在 launch 裡(`stage1_distance=0.50`、`stage2_distance=0.28`、`leave_distance=0.40`),Stage 1 速度發到 `/cmd_vel`,Stage 2 與 leave 發到 `/pre_cmd_vel`(G7+ 精準模式)。

`apriltag_detection` 啟動時是 **disabled**,不訂閱相機;收到 goal 後由 `apriltag_control` 呼叫 `/up/apriltag_detection/enable`(`std_srvs/srv/SetBool`)才開始偵測,`IN_POSITION` 期間維持開啟,流程結束(`FINISHING`)才關。所以沒送 goal 時 `/up/apriltag_pose` 沒有資料是正常的。

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
ros2 topic echo /up/apriltag_pose        # 只有 tracking 中才有資料
ros2 topic echo /cmd_vel                 # Stage 1
ros2 topic echo /pre_cmd_vel             # Stage 2 / leave(leave 時 vx 為負)
ros2 service call /up/apriltag_detection/enable std_srvs/srv/SetBool "{data: true}"   # 手動開偵測
ros2 run rqt_image_view rqt_image_view /up/apriltag/marked_image
```

---

## 5. 疑難排解

| 現象 | 原因 / 處理 |
|---|---|
| `rmw_create_node: failed to create domain` / `rcl node's rmw handle is invalid` | `CYCLONEDDS_URI` 綁的 `enP8p1s0` 是 DOWN(網路線沒插),且 container 是 fallback 設定之前建立的。`docker compose up -d --force-recreate`(在桌面 session 執行),或照 3.2 把 `CYCLONEDDS_URI` 改成 `lo` |
| `enP8p1s0: optional interface was not found` | 網路線沒插,已 fallback 到 `lo`(正常);實機請接上網路線後重新 launch |
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

## 7. `src/apriltag/README.md` 與實際不一致之處

- README 寫 `test_virtual_tracking.launch.py`,實際檔名是 `launch/virtual_tracking.launch.py`。
- README 的 Project Structure 列了 `test/` 目錄,實際不存在。
- README 的指令(第 138、163 行)也用 `test_virtual_tracking.launch.py`,照抄會找不到檔案。
