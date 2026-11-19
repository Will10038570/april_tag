# AprilTag 開發 / 測試指南(Docker)

> **規則:所有程式碼的 build、執行、測試一律在 Docker container 內進行,不要在 host 的 `/opt/ros/humble` 上跑。**
> Docker 環境定義在 `/home/cobo/Desktop/april_tag/docker/`(`Dockerfile`、`docker-compose.yml`、`docker_compose_UI_off.yml`、`ros_entrypoint.sh`)。
>
> **本機(Jetson `april_tag_serive`)不會接實機機器人**:在這台電腦上 restart / recreate container、停掉實機 launch、`pkill` 測試 process 都不需要顧慮對機器人的影響。第 3.2 節的 domain / 網路線安全注意事項是給接上機器人的環境用的。

套件功能、topic / service / cmd・status 介面、控制架構請看 `src/apriltag/README.md`;本文件只說明「怎麼在 Docker 裡跑起來與驗證」。

---

## 1. Docker 環境

| 項目 | 值 |
|---|---|
| Image | `ros2_april_tag:v1`(base `ros:humble-ros-base`) |
| Container | `april_tag_serive`(拼字照 compose 設定,不是 service) |
| Compose 設定檔 | `docker/.ros2_setup`(ROS 參數)。compose 指令一律在 `docker/` 目錄執行並加 `--env-file .ros2_setup`,否則報 `required variable … is missing a value` |
| Workspace 掛載 | host `/home/cobo/Desktop/april_tag`(即 repo 根目錄,compose 的 `..`)→ container `/mnt/work_space` |
| 網路 | `network_mode: host`、`ipc: host`、`privileged: true`(RealSense 走 `/dev`) |
| `ROS_DOMAIN_ID` | 由 `docker/.ros2_setup` 設定,目前 `72`(與機器人相同) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | `docker-compose.yml`:只綁 `lo`,peer `127.0.0.1`(所有 node 都在本機,不需網路線)。`docker_compose_UI_off.yml`:綁 `.ros2_setup` 的 `ROS_NETWORK_INTERFACE` + peer `ROS_PEER_IP`(目前 `lan2` / `192.168.5.10`)、`MaxAutoParticipantIndex=50`;**該設定不要再加 `lo`**:多網卡時 Cyclone 會從 `lo` 的 socket 送 unicast 到外部 peer,kernel 拒絕(`ddsi_udp_conn_write … failed with retcode -3`);該網卡不存在 / DOWN 時 node 建不起來(compose 註解提到的 `presence_required="false"` 實際上沒有寫進 URI) |
| 已安裝 | librealsense2、`pyrealsense2`、`realsense2_camera`、`cv_bridge`、`python3-opencv`、`rqt_image_view`、`rmw_cyclonedds_cpp`、`pupil-apriltags`、`scipy`、`numpy<2` |

`numpy` 必須維持 `<2`:`cv_bridge` 是以 numpy 1.x C-API 編譯的。

Container 的 entrypoint 與 `.bashrc` 會自動 source `/opt/ros/humble/setup.bash` 與 `/mnt/work_space/install/setup.bash`(若存在)。`.bashrc` 只在**互動式** shell 生效:`docker exec -it … bash` 或 `docker exec … bash -ic '…'` 可直接用 `ros2`;`bash -lc` **不會** source(2026-10-02 實測 `which ros2` 找不到),非互動時請用 `bash -ic`,或在指令開頭自行 `source /opt/ros/humble/setup.bash && source /mnt/work_space/install/setup.bash`。

`install/` 內的 Python 套件是複製的(不是 symlink),改了 `.py` 一定要重新 `colcon build` 才會生效。

### 啟動 container

兩個 compose 檔都設定 `restart: unless-stopped` 與 `command: ros2 launch apriltag april_tag.launch.py`:**container 一啟動(含開機自動啟動)就跑實機 launch**(相機 + detection + control,PID 1),停掉 launch 就等於停掉 container。因此:

- workspace 還沒 build(沒有 `install/`)時 launch 會失敗,container 不斷重啟,`docker exec` 進不去。
- 只想 build / 測試、不想啟動實機 launch 時,用 override 把 command 換掉(2026-11-12 首次建 image 時即用此法):

```bash
# override 檔(放在 repo 外即可),內容:
# services:
#   ros2:
#     restart: "no"
#     command: sleep infinity
docker compose --env-file .ros2_setup -f docker-compose.yml -f /path/to/compose.test.override.yml up -d --force-recreate
```

測完要回到開機自動 launch:`docker compose --env-file .ros2_setup up -d --force-recreate`(會開始驅動相機與車子)。

```bash
# 第一次建立(或 Dockerfile 有改;Jetson 上約 5 分鐘)
cd /home/cobo/Desktop/april_tag/docker
docker compose --env-file .ros2_setup build
docker compose --env-file .ros2_setup up -d

# 已建立過,只是停掉了(或 repo 根目錄的 ./apriltag.sh up / down)
docker start april_tag_serive

# 進入 container
docker exec -it april_tag_serive bash
```

GUI(dashboard、`rqt_image_view`)需要在 host 的圖形桌面 session 裡啟動 container,`$DISPLAY` / `$XAUTHORITY` 才會帶進去;必要時先在 host 執行 `xhost +local:docker`。
若 container 不是在桌面 session 建立的(container 內 `echo $DISPLAY` 為空,`/tmp/.docker.xauth` 變成空目錄),不重建的做法:host 執行 `xhost +local:docker`,再 `docker exec -it -e DISPLAY=<桌面 display> april_tag_serive bash`,container 內 `unset XAUTHORITY` 後啟動 GUI。桌面 display 編號不固定,先在 host 用 `who` 看(2026-11-16 為 `:0`;`/tmp/.X11-unix/X1001` 不是桌面,硬塞 `:1` 會 `Can't initialize GTK backend`)。container 內 `DISPLAY` 已有值且 `/tmp/.docker.xauth` 是檔案時,不要加 `-e DISPLAY` / `unset XAUTHORITY`,直接用即可;永久修正則在桌面 session 執行 `docker compose --env-file .ros2_setup up -d --force-recreate`。

**透過 SSH(無圖形桌面)** 時改用無 GUI 版 `docker/docker_compose_UI_off.yml`(移除 `DISPLAY`/`XAUTHORITY` 與 X11 mount,其餘設定相同)。它與 `docker-compose.yml` 同 project / service / container 名稱,所以會直接取代現有 container:

```bash
cd /home/cobo/Desktop/april_tag/docker
docker compose --env-file .ros2_setup -f docker_compose_UI_off.yml up -d --force-recreate   # Dockerfile 有改時加 --build
```

這個 container 內 dashboard / `rqt_image_view` 無法使用,虛擬測試請用 `headless:=true`;之後要切回 GUI 版,在桌面 session 執行 `docker compose --env-file .ros2_setup up -d --force-recreate`。

搬到別台機器時注意 Jetson 是 ARM,`docker save` 的 image 不能用在 x86(原本的 `Will/docker_copy_guide.txt` 已不在本機)。`docker/rebuild_command` 內的路徑仍是舊的 `Will/work_space`,使用前請改成 `/home/cobo/Desktop/april_tag/docker`。

---

## 2. Build

```bash
docker exec april_tag_serive bash -ic '
  cd /mnt/work_space &&
  colcon build --symlink-install'
```

只 build 單一套件:`colcon build --symlink-install --packages-select apriltag`。
改了 `apriltag_interfaces/msg/*.msg`(`TagPose`、`TagPoseArray`、`TagTracing`)要先 build `apriltag_interfaces`(2026-10-19 起已無 action)。刪除 / 改名 interface 後,`build/apriltag_interfaces`、`install/apriltag_interfaces` 會留著舊的生成檔,要在 container 內 `rm -rf` 後重新 build(檔案是 root 建立的,host 上刪不掉)。同理,刪除 / 改名 `apriltag` 的 `.py` 或 entry point 後,`install/apriltag/` 裡的舊複本不會被刪,import 舊模組名仍會成功,要手動在 container 內刪掉。
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

`virtual_tracking.launch.py` 會啟動真正的 `apriltag_detection`、`apriltag_control`,以及取代相機 / AMR / G7+ AMCL 與 lidar safety 的 `virtual_tracking_sim`(全部在 `wheelchair` namespace)。一次 run = sim 呼叫 `/wheelchair/attach` service(`std_srvs/SetBool` `data:true`;response 不帶 `cmd_id`,sim 在被接受後取 status 中第一個大於呼叫前的 `cmd_id`)(Stage 1 → Stage 2 → 回 `IDLE`,AMCL 關、lidar safety disabled)接著呼叫 `/wheelchair/detach`(退到 `calibration.detach.z_ref` 後還原 AMCL / lidar safety);sim 看 `/wheelchair/status` 出現 `IDLE` + 自己的 `cmd_id` 判斷結束。

**安全注意:** `virtual_tracking.launch.py` 的 `domain_id` 預設是 65(機器人在 `.ros2_setup` 的 `ROS_DOMAIN_ID`,目前 72,但仍請勿依賴這點),而 `apriltag_control` 會發 `/cmd_vel`、`/pre_cmd_vel`、`/g7_plc/disable_lidar_safety` 並呼叫 AMCL service。跑虛擬測試時:

- 拔掉機器人網路線,**或**
- 改用其他 domain(`domain_id:=99`)並讓 DDS 只走 loopback(見下方指令)。

用 `docker-compose.yml` 建立的 container 本身就在跑實機 launch(domain 72、`lo`),與 domain 99 的測試互不干擾;清理殘留 process 時會連它一起停掉(見本節最後),在本機無妨(見文件開頭:本機不接機器人)。

Headless(無視窗,自動依序送 `attach`、`detach`,存檔後結束):

```bash
docker exec april_tag_serive bash -ic '
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>"
  mkdir -p /mnt/work_space/virtual_tracking_logs && cd /mnt/work_space &&
  ros2 launch apriltag virtual_tracking.launch.py domain_id:=99 \
      headless:=true auto_start:=true \
      init_x:=-1.0 init_y:=0.15 init_yaw_deg:=10.0'
```

通過標準(看 log;`manage_amcl_and_lidar_safety: true` 時。`false` 時 `(held)` 那行為 `(unmanaged, held check skipped)`、control 訊息為 `AMCL / lidar safety unmanaged`):

```
[state] STAGE1 -> STAGE2 (Stage 1 aligned)
[state] STAGE2 -> IDLE (cleanup done)
[cmd] attach cmd_id=1 result: SUCCEEDED error_code=0 message='Stage 2 aligned at 0.28m. ... AMCL off, lidar safety disabled; send detach to restore. ...' final=(...)
attach cmd_id=1 finished: SUCCEEDED error_code=0 - Attach succeeded
aligned: amcl=off lidar_safety=disabled (held)
[state] IDLE -> LEAVING (detach cmd_id=2 accepted, back to 1.00m)
[cmd] detach cmd_id=2 result: SUCCEEDED error_code=0 message='Leave reached 1.00m (forward=1.0xxm). ...' final=(...)
detach cmd_id=2 finished: SUCCEEDED error_code=0 - Detach succeeded
after cmd: amcl=on lidar_safety=enabled (restored)
Run saved to .../virtual_tracking_logs/<time>_succeeded_*.csv
body_status=[1, 0, 2, 0] expected=[1, 0, 2, 0] (OK)
```

`body_status=… (OK)` 是 sim 在 run 結束 0.5 s 後比對 `/g7_body/simple_ui/body_status` 收到的值與被接受的 cmd 應產生的值(attach `1, 0`、detach `2, 0`);不符時印 error `(NOT OK)`。headless 在這行之後才結束。

`[status] {...}` 在 cmd 執行中每則都印(與 status 同為 5 Hz)。leave 期間偶爾出現 `Error, more than one new minima found.` 是 `pupil_apriltags` 姿態估計的訊息,不影響結果。

`(held)` 表示 Stage 2 成功回 `IDLE` 後 AMCL 仍關、lidar safety 仍 disabled(正確);若印 `(NOT held)` 表示太早還原。
結束時 `apriltag_control` 的 shutdown 一律還原(AMCL check / open),此時 sim 的 fake AMCL 可能已經結束,會印 `Cannot check AMCL: service /check_mcl_if_trigger not available.`、`Restore failed` 與 launch 的 `escalating to 'SIGTERM'`,不影響測試結果。

輸出檔在**執行目錄**下的 `virtual_tracking_logs/`(`log_dir` 預設是相對路徑):`*_sim.csv`、`*_feedback.csv`(status 的 x/y/yaw error、`cmd_id`、state 數字、`last_frame_age`、`cmd_vx/vy/wz`,共 10 欄)、`*_cmd_raw.csv`、`*.png`(檔名是 `<time>_<result>.png`,沒有 `_`)。所以請先 `cd /mnt/work_space` 再 launch,否則 log 會散落到別的目錄(例如 `src/apriltag/launch/`、`src/apriltag/tools/` 底下)。

有 dashboard 的互動版(需 `DISPLAY`):

```bash
docker exec -it april_tag_serive bash -ic '
  cd /mnt/work_space &&
  ros2 launch apriltag virtual_tracking.launch.py'
```

按鍵:`s` 開始(呼叫 `attach`)、`l` 離開(呼叫 `detach`;sim 在 `ALIGNED` 時接續同一個 run,沒有 run 時為只有 leave 的 run)、`x` 對執行中 cmd 的 service 送 `data:false`(`ALIGNED` 時沒有東西可停)、`r` 重置、`h` 隱藏 / 顯示 tag、`a`/`d` 旋轉初始 yaw(±2°,僅未執行時)、`+`(或 `=`)/`-` 縮放、`q` 或 `Esc` 離開(會連同整個 launch 一起結束)。未執行時,上半部畫面可用滑鼠拖曳車體移動、拖曳把手或滾輪旋轉,也可點畫面上的按鈕。

| Launch 參數 | 預設 |
|---|---|
| `domain_id` | `65` |
| `init_x` / `init_y` / `init_yaw_deg` | `-1.0` / `0.15` / `10.0` |
| `headless` / `auto_start` | `false` / `false` |
| `log_dir` | `virtual_tracking_logs` |
| `hide_tag_after` | `-1.0`(run 開始後幾秒停止畫 tag,測 TAG_LOST;`< 0` 不隱藏) |
| `config_file` | workspace 根目錄的 `april_tag_config.json`(與實機同一檔,container 內 `/mnt/work_space/april_tag_config.json`;由 package share 往上推回 workspace,直接讀,不需 build;2026-11-19 起 `virtual_tracking_config.json` 已刪除) |

detection / control / sim 的其他設定全部在 `config_file`(2026-11-12 起 launch 不再有 `tracking_tag_*`、`dock_timeout`、`stage*_distance` 等參數)。虛擬測試與實機共用 `april_tag_config.json`;其 `manage_amcl_and_lidar_safety` 為 `false` 時 sim 不檢查 held(印 `(unmanaged, held check skipped)`),要測 AMCL / lidar safety 流程時在 container `/tmp` 放一份改成 `true` 的副本，用 `config_file:=` 指定。sim 用 `validate_control_config` 讀同一檔(與 control 相同檢查):`tag_size`、畫的 tag id(`tracking_tag_id`,`-1` → 0)、calibration 三點、tolerance、`max_v*`(dashboard 限制線)、`ros_interfaces`(cmd_vel topic、AMCL service、lidar topic 名稱)。sim 的 ground truth 與 control 同公式(每個 stage 用自己的 `z_ref` / `x_ref`,yaw 減 `yaw_ref`;detach 的 `x_ref` / `yaw_ref` 為 0),所以 calibration 可用任意值;出廠 `x_ref` 0.026 = sim 的 `camera_y_offset`,goal 在 tag 中線上。sim 顯示的 stage 依 status `state`(`Attaching(STAGE2)`)。

改根目錄的 `april_tag_config.json` 後重新 launch 即生效,不需 `colcon build`。control 在每次 attach / detach 被接受時會重讀,但 sim(ground truth、畫的 tag)與 detection 只在啟動時讀,所以 launch 執行中改檔兩邊會不一致,改完一律重新 launch。

變化測試:複製該檔、改值後 `config_file:=/tmp/xxx.json`(檔案放 container 的 `/tmp`)。例:`control.dock_timeout: 3.0` → `error_code=4`。TAG_LOST:`hide_tag_after:=5.0`(或互動版按 `h`)→ `tag hidden at t=5.0s` → 約 1 s 後 attach `ABORTED error_code=1`、`(restored)`、`body_status=[1, 0] … (OK)`。改 `tracking_tag_id` 測不到 TAG_LOST(sim 會跟著畫同一個 id)。

手動送 cmd 測試(另開終端,同樣設 `ROS_DOMAIN_ID=99` 與 `lo` 的 `CYCLONEDDS_URI`):用 `headless:=true auto_start:=false` 啟動,sim 的物理與相機照常運作,但不會自己送 cmd。指令見第 4 節(`tools/attach.py` / `detach.py` `true|false`,或 `ros2 service call /wheelchair/attach std_srvs/srv/SetBool "{data: true}"`、`ros2 topic echo /wheelchair/status`)。工具呼叫完 service 就結束(不會卡住終端),結果看 status。

**背景 process 注意:** `docker exec` 被中斷(或 Claude Code 的指令被拒絕)時,container 內已啟動的 process **不會**停止;非互動 bash 的背景 job 也會忽略 SIGINT。測完務必確認並清掉殘留(`[a]` 寫法避免 `pkill` 比對到自己)。**先看 `ps` 有沒有實機 launch 的 node 或別人正在跑的測試**:下面的 `pkill` 會連它們一起停掉(2026-10-19 發生過),要保留時改用 `kill -INT <pid>` 只停自己的 PID(PID 用 `ps -eo pid,args` 看，不要用 `pgrep -f` 的結果直接 kill:命令列含相同字串時會比對到執行 kill 的 shell 本身,2026-11-17 這樣做時同一 container 另一組 domain 99 測試也同時收到 SIGINT 停掉)(本機不接機器人，停掉實機 launch 無妨;同一 container 已有別的 domain 99 測試時，自己的測試改用其他 domain,如 98):

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

`src/apriltag/test/` 內有 `test_state_machine.py`(轉移表、cmd 接受表與取消表)、`test_cmd_flow.py`(status JSON、參數檢查、outcome / error_code)、`test_leave_step.py`、`test_target_flow.py`(`choose_best_target`、`tag_matches`)、`test_yaw_*.py`、`test_tag_size.py`、`test_config_loader.py`(`april_tag_config.json` 驗證、calibration 讀寫)、`test_speed_limits.py`(max / min 速度限制;`test_config_loader.py` 另含 tolerance、calibration 必填、removed key 檢查)、`test_sim_geometry.py`(sim 的 `goal_pose` / `ground_truth`)、`test_approach.py`(Stage 1 進場線橫向偏移，以 sim 幾何驗證正負號;`ramp_scale` / `approach_vx_scale`)、`test_lqr_weights.py`(`control.lqr` 出廠 q 與舊 Q/R 等效、`K ≈ √q`),`test_config_loader.py` 另含 `yaw_estimator` / `yaw_probe` 驗證與 `_doc` key 檢查,2026-11-17 共 359 tests。新增的 pytest 請放在 `src/apriltag/test/`,同樣用上面的指令在 container 內執行。

---

## 4. 實機執行

前提:RealSense 已插上;用 `docker-compose.yml` 時機器人端 node 須在本機(DDS 只走 `lo`),用 `docker_compose_UI_off.yml` 時 `ROS_NETWORK_INTERFACE`(目前 `lan2`)須已接上機器人且為 UP。

正常情況下 container 啟動時已自動執行 `april_tag.launch.py`(見第 1 節),不需再開終端 1;下面終端 1 的指令只在 container 是以 `sleep infinity` override 建立時使用,否則會在同一 domain 跑出第二組 node。

```bash
# 終端 1:相機 + detection + control(全部在 /wheelchair namespace)
docker exec -it april_tag_serive bash -ic '
  ros2 launch apriltag april_tag.launch.py'
# 所有參數統一由 /mnt/work_space/april_tag_config.json 設定(manage_amcl_and_lidar_safety、
# stage1/2_tolerance、tracking_tag_*、timeout、max_/min_vx/vy/vw、tag_size、calibration(stage / leave 距離)等)
# 若要換設定檔:config_file:=/path/to/april_tag_config.json
# 每次送 attach / detach service 時 control node 會自動重新讀取 JSON,不需重啟即可套用變更

# 校正定點存檔(擺好位置後執行,只需 detection + camera):
# docker exec -it april_tag_serive bash -ic 'python3 /mnt/work_space/src/apriltag/tools/calibrate.py --stage1'
# --stage1 / --stage2 / --detach;可加 --file PATH(預設 /mnt/work_space/april_tag_config.json)
# --file 必須是已存在且為 JSON object 的檔案,否則立刻 exit 1(不等 tag、不寫檔)
# --stage1 / --stage2 存 z_ref / x_ref / yaw_ref,--detach 只存 z_ref;不存 PNG(2026-11-13 起)

# 終端 2:看狀態
docker exec -it april_tag_serive bash -ic 'ros2 topic echo /wheelchair/status'

# 終端 3:測試工具,每個 service 一支、必填 true(開始)/ false(取消);像 ros2 service call 一樣印出 response 後立即結束,結果看終端 2 的 status(exit code:0 被接受、1 被拒 / 無回應、2 找不到 service 或參數錯誤)
docker exec -it april_tag_serive bash -ic 'python3 /mnt/work_space/src/apriltag/tools/attach.py true'    # false:取消執行中的 attach
docker exec -it april_tag_serive bash -ic 'python3 /mnt/work_space/src/apriltag/tools/detach.py true'    # false:取消執行中的 detach

# 或直接呼叫 service(std_srvs/SetBool;response 立即回 success=接受與否、message=純文字原因,不帶 cmd_id;cmd_id 看 status)
# 對位(成功後 status 為 IDLE + 該 cmd_id + error_code 0,AMCL 關、lidar safety disabled)
docker exec -it april_tag_serive bash -ic 'ros2 service call /wheelchair/attach std_srvs/srv/SetBool "{data: true}"'
# 離開:/wheelchair/detach data:true(IDLE 時接受,不需先對位;退到 calibration.detach.z_ref,完成後開 AMCL、enable lidar safety)
# 停止:同一個 service 送 data:false(attach 停 STAGE1 / STAGE2、detach 停 LEAVING;success=true → IDLE、error_code=0、error_msg 為 `Cancelled cmd_id=n. …`,停止並還原;已在 ending 時 msg 為 already ending,結果以 status 為準;IDLE 或另一種 cmd 在跑時 success=false)
```

介面依 `[service_robot] wheelchair_interface.pdf`(v0.1),差異:`detach` = leaving(不管夾爪,`/wheelchair/attached` 固定發 `false`)、status 的 state 字串為 `IDLE` / `Attaching(STAGE1)` / `Attaching(STAGE2)` / `Detaching`(`control_node.py` 的 `STATUS_STATE`,對應 enum `IDLE` / `STAGE1` / `STAGE2` / `LEAVING`;log 與 service response 仍用 enum 名稱;沒有 `CANCELLED` / `ERROR`,結束一律回 `IDLE`,看 `error_code` / `error_msg`)、錯誤碼沿用下列舊編號(之後再改成 PDF 的)、沒有 `/wheelchair/get_status`、cmd 改成兩個 `std_srvs/SetBool` service(取代 `/wheelchair/cmd` topic,`data:false` = cancel,沒有 `reset`,`cmd_id` 由 `apriltag_control` 給)。

- `/wheelchair/attach`、`/wheelchair/detach`(`std_srvs/SetBool`):立即回 `success`(接受與否)與純文字 `message`(`attach accepted` / `detach accepted`、`cancel requested`、`already ending, result unchanged`、`nothing running`、`rejected: state=… (only accepted in …)`;2026-10-27 起不是 JSON、不帶 `cmd_id`)。`data:true` 只在 `IDLE` 接受,每次接受 `cmd_id` +1(開機後從 1 起算,重啟重算;只出現在 status);非 `IDLE` → `success=false`、`[cmd] … rejected …, status unchanged`。`data:false` 只取消同一個 service 的執行中 cmd(`CANCEL_RULES`:attach → `STAGE1` / `STAGE2`、detach → `LEAVING`),回 `cancel requested`;`IDLE` 時 `success=false`(`nothing running`);另一種 cmd 在跑時 `success=false`、`rejected: state=LEAVING (only accepted in STAGE1, STAGE2)`。
- `/wheelchair/status`(`std_msgs/String` JSON,reliable、transient_local、depth 1,5 Hz):`state`、`cmd_id`(開機後 0)、`attached`(false)、`error_code`、`error_msg`、`stamp`(epoch s),以及除錯欄位 `x_error` / `y_error` / `yaw_error` / `last_frame_age` / `cmd_vx` / `cmd_vy` / `cmd_wz`(未知時 `null`)。`cmd_v*` 是最近一次送到 `/cmd_vel` 或 `/pre_cmd_vel` 的速度(停車零速也算,兩個 topic 記一筆;5 Hz 只帶最新一筆),超過 1 s(`control_node.py` 的 `CMD_VEL_MAX_AGE`)或開機後沒送過時為 `null`。cmd 結束 = `IDLE` + 該 `cmd_id`;`error_code` / `error_msg` 與轉到 `IDLE` 在同一個 lock 內寫入。`error_msg`:執行中 `""`、成功 `Attach succeeded` / `Detach succeeded`(`cmd_flow.py` 的 `SUCCESS_MSG`)、取消 `Cancelled cmd_id=n. elapsed=…s`、失敗為原因。
- `error_code`:0 OK(成功或 `data:false` 取消,以 `error_msg` 是否以 `Cancelled` 開頭區分)、1 TAG_LOST、2 AMCL_SERVICE_FAIL(restore 失敗也是,取消後 restore 失敗亦同)、3 LIDAR_SERVICE_FAIL(保留未用)、4 TIMEOUT、5 OTHER、6 NODE_SHUTDOWN(2026-10-27 起;之前 5 是 ACTION_CANCELED、6 OTHER、7 NODE_SHUTDOWN)。
- `/wheelchair/attached`(`std_msgs/Bool`,transient_local,5 Hz):固定 `false`。
- `/g7_body/simple_ui/body_status`(`std_msgs/Int32`,reliable、transient_local、depth 1,參數 `body_status_topic`;絕對路徑,不在 `wheelchair` namespace):只在 state 改變時發一次,進 `STAGE1` → `1`、進 `LEAVING` → `2`、回 `IDLE` → `0`(成功或失敗都一樣);進 `STAGE2` 與開機時不發。

選 tag 在 **detection**:依 config 的 `detection.tracking_tag_family` / `tracking_tag_id`(預設 `tag36h11` / `0`,`-1` = 任何 id),每張影像取前向距離 z 最小(且 > 0)的一個,發到 `/wheelchair/tag_tracing`(`TagTracing`:`header`(同該影像的 `TagPoseArray`)、`valid`、`tag`(`TagPose`)、`image_width/height`;沒有符合的 tag 時 `valid=false`、`tag` 為空)。launch 把同一組 `tracking_tag_*` 也傳給 `apriltag_control`,它只訂閱 `/wheelchair/tag_tracing` 並以此比對:`valid=false` 或不符都當作 tag 不見(不符時每個 tag 只 warn 一次;RUNNING 步驟中超過 `lost_target_timeout`,預設 1 s,以收到資料的時間計 → `TAG_LOST`)。yaw estimator / probe 在 cmd 開始與 `tag_tracing` 的 id 改變時 reset。leave 距離是 `calibration.detach.z_ref`(出廠 1.0 m),不由指令帶入。

`apriltag_control` 是狀態機:`IDLE → STAGE1 → STAGE2 → IDLE`(`attach`)與 `IDLE → LEAVING → IDLE`(`detach`),每個非 IDLE state 內有 `preparing → running → ending` 三個步驟(詳見 README 的 State Machine)。每個被接受的 attach / detach 在一條 worker thread 跑。AMCL / lidar safety 由各 state 自己切換:`STAGE1` 準備時 enable lidar、關 AMCL;`STAGE2` 準備時 disable lidar;Stage 2 成功只停車就回 `IDLE`(AMCL 維持關、lidar 維持 disabled);`LEAVING` 結束、任何失敗 / cancel / timeout 與 shutdown 都會 enable lidar 並確保 AMCL 打開(不記錄是誰關的)。整個 cmd 有 timeout(`dock_timeout` / `undock_timeout`,預設 300 s,從 cmd 接受算到進入 ending)。

`april_tag.launch.py` 只接受 `config_file` 一個參數(include RealSense 時以 scoped `GroupAction` 傳 `config_file:=''` 給它:`rs_launch.py` 也有同名 launch 參數,會把我們的 JSON 當相機 YAML 讀,遇 `null` 讓整個 launch 崩潰,2026-11-12 修正前 HEAD 即如此);`manage_amcl_and_lidar_safety`(預設 `false`)等由 `april_tag_config.json` 的 `control` 區塊設定。**stage / leave 目標只來自 `calibration` 的 `stage1` / `stage2` / `detach` 三個點(必填,`z_ref` = 目標前向距離、`x_ref` / `yaw_ref` = 橫向 / yaw 參考;`detach` 只有 `z_ref`(記憶體中 `x_ref` / `yaw_ref` 為 0,Detaching 時 status 的 `y_error` 是未加偏移的橫向值);其他 key(舊檔的 `stamp` / `image_path`、`detach.x_ref` / `yaw_ref`)一律忽略;出廠值 `z_ref` 0.50 / 0.28 / 1.0、stage1 / stage2 `x_ref` 0.026、`yaw_ref` 0,等同舊預設)**,沒有 `stage*_distance` / `leave_distance` / `use_calibration`(留著會被 control 拒絕並提示替代欄位)。到位判斷用 `control.stage1_tolerance` / `stage2_tolerance`(`x` / `y` m、`yaw` rad、`hold_seconds` s,出廠皆 0.05 / 0.05 / 0.05 / 1.0;Stage 1 的 x / y 也是 S3 yaw probe 的啟動條件)。`config_file` 對兩個 node 都是必填;detection 只驗證共用區塊,calibration 不完整時只有 control 啟動失敗(camera / detection 照跑,可用 `calibrate.py` 校正後 `docker restart`),Stage 1 速度發到 `/cmd_vel`,Stage 2 與 leave 發到 `/pre_cmd_vel`(G7+ 精準模式)。Stage 1 / 2 的速度每軸先限制在 `max_vx` / `max_vy` / `max_vw`(0.1 / 0.1 m/s、0.05 rad/s),再把非 0 且 `|v| < min_*`(`min_vx` / `min_vy` / `min_vw`,皆 0.01;補償 G7+ 低速 deadband)的值改成 `sign(v)·min`,0 維持 0(`control_flow.clamp_control`);六個值都在 JSON `control` 區塊、缺一不可,須 max > 0、0 ≤ min ≤ max。分軸 hold(`control.per_axis_hold`,必填 bool,出廠 `true`;只有 control 檢查):Stage 1 / 2 每張影像各軸判斷(與到位判斷同值:`x_f`、`y_f`、`yaw_abs_f − yaw_ref`,同一組 tolerance),在 tolerance 內的軸送 0(在 LQR / S3 `wz_override` 之後、`clamp_control` 之前，不會被 min 拉起),漂出即恢復修正;yaw 在 tolerance 內時 S3 probe 不啟動;三軸都在內時不發任何 cmd(planner / LQR 不跑)直到有軸離開(status `cmd_v*` 1 s 後為 `null`);成功條件不變(三軸都在內滿 `hold_seconds`);log `[axis_hold] x=hold|fix …`(狀態改變時印);`false` = 舊行為。yaw deadband(`control.yaw_deadband`,必填 `{"in", "out"}` rad、`0 < in < out`,出廠 0.03 / 0.05;只有 control 檢查):Stage 1 / 2 依 `|yaw_f − yaw_ref|`(LQR 的 yaw 輸入，不是 `yaw_abs_f`)加 hysteresis,`< in` 起 wz 送 0(LQR 與 S3 `wz_override` 都會被歸零)直到 `> out`,避免極小的 `yaw_f` 被 `min_vw` 拉成每張翻號的 ±`min_vw`;每個 stage 開始時為 hold;與分軸 hold 合併(x / y 也 hold 時不發 cmd),不影響到位判斷;log `[yaw_deadband] hold|fix`。Stage 1 直線進場(`control.stage1_approach`,必填:`enabled`、`lateral_full` / `lateral_zero` m、`yaw_full` / `yaw_zero` rad,出廠 `true`、0.03 / 0.08、0.035 / 0.105,僅在 sim 調過):Stage 1 前進的 `vx`(> 0)在 max 限制後、min 之前乘上 `ramp(|lat|) × ramp(|yaw|)`(≤ full 為 1、≥ zero 為 0、中間線性),後退不縮放;yaw 還無效時(`YawEstimate.yaw_valid` false,沒有 S4 表內的 frame,0.0475 m tag 約 0.7 m 外)`lat` = 相機 `y_error`、不看 yaw,有效後(鎖住到換 target)`lat` = 相機到 stage1 進場線的距離(`math_utils.approach_lateral_error`)並看 `|yaw_f − yaw_ref|`;log `[approach] …`。`enabled: false` = 舊行為。LQR(`control.lqr`,必填):`q_weights` `[x, y, yaw]`(皆 > 0)、`smooth_tau`(s,> 0,planner 三軸共用);R 固定為 1(`control_node.LQR_R_WEIGHTS`),三軸解耦，每軸 gain `K ≈ √q`(1/s,時間常數 `1/K`,收斂約 `smooth_tau + 1/K`;q 乘 4 → 快 2 倍),只在 `|error| < max_v* / K`(未飽和)時有影響。出廠 `[0.3333, 1.0, 0.1667]` / `0.5` 等同舊寫死的 Q (1, 1, 0.5) / R (3, 1, 3)(K ≈ 0.56 / 0.95 / 0.40)。每次 attach / detach reload 時重建 `LQRTracker` / `TrajectoryPlanner`,啟動與 reload 時 log `[lqr] q=[…] (K~=[…])`。yaw 估計與 S3 probe 的參數全部在 JSON(2026-11-17 起，不再是 ROS 參數):`yaw_estimator`(s4 表 + `border_px`、`margin_lo/hi`、`sigma_dh`、`snr_lo/hi`、`ref_tag_size`、`q_proc`、`gate_sigma`、`gate_min_deg`、`accept_frames`、`consistency_deg`、`abs_window`、`xy_tau`)與 `yaw_probe`(`trigger_frames`、`c_sign_low`、`trigger_yaw`、`wz`、`turn_time`、`settle_frames`、`end_yaw_deg`、`change_deg`、`end_c_sign`、`initial_direction`(1 / -1)、`max_turns`),key = `YawEstimatorConfig` / `YawProbeConfig` 欄位名，全部必填(只有 control 檢查：`config_loader.parse_yaw_estimator` / `parse_yaw_probe`;detection / sim 不受影響),出廠值 = dataclass 預設;每個區塊的 `_doc` 物件是各 key 的說明(選填、loader 忽略，測試檢查與數值 key 一致)。reload 時重建 `YawEstimator` / `YawProbe`,log `[yaw_estimator] …` / `[yaw_probe] …`。leave 不用 planner / LQR:固定發 `vx=-max_vx`(-0.1 m/s)、`vy=wz=0`,直到 camera→tag 前向距離 ≥ `leave_distance` 立即停(不檢查 y / yaw、無 hold)。

`apriltag_detection` 啟動後就一直偵測,不受 `apriltag_control` 控制(沒有 enable service)。debug 影像所有 tag 一律畫綠框。每張影像(收到 `camera_info` 之後)發一則 `apriltag_interfaces/msg/TagPoseArray` 到 `/wheelchair/apriltag_poses`,包含 `tag_families`(launch 參數 `detect_tag_families`,以空白分隔,預設 `tag36h11`;每個 family 一個 detector,CPU 隨數量增加)內所有偵測到的 tag(`family`、`id` 字串、`pose`),沒有 tag 時 `tags` 為空;每個 tag 廣播 TF `<camera_frame> → <family>_<id>`(如 `tag36h11_0`)。`/wheelchair/apriltag_poses` 只作參考 / 除錯,control 不訂閱。沒有 cmd 執行時 `apriltag_control` 會忽略 `tag_tracing`。RealSense 相機(`realsense/launch/camera.launch.py`)也在 `wheelchair` namespace:`/wheelchair/camera/camera/color/*`。

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
ros2 topic echo /wheelchair/status         # 5 Hz JSON
ros2 topic hz /wheelchair/status           # 約 5 Hz
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /wheelchair/attached   # data: false
ros2 topic echo /wheelchair/apriltag_poses # 一直有資料(沒 tag 時 tags: [])
ros2 topic echo /wheelchair/tag_tracing    # 一直有資料(沒有追蹤的 tag 時 valid: false)
ros2 run apriltag detection_viewer --ros-args -r __ns:=/wheelchair   # 最多 5 Hz 印 tag_tracing(預設):header + valid + 追蹤的 tag
ros2 run apriltag detection_viewer --ros-args -r __ns:=/wheelchair -p topic:=apriltag_poses   # 改印 TagPoseArray:header + 每個 tag 一行
ros2 topic echo /cmd_vel                   # Stage 1
ros2 topic echo /pre_cmd_vel               # Stage 2 / leave(leave 時 vx 為負)
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11_0   # 單一 tag 的 TF(frame 依相機而定)
ros2 run rqt_image_view rqt_image_view /wheelchair/apriltag/marked_image
```

---

## 5. 疑難排解

| 現象 | 原因 / 處理 |
|---|---|
| `rmw_create_node: failed to create domain` / `rcl node's rmw handle is invalid` | `CYCLONEDDS_URI` 綁的網卡(`docker_compose_UI_off.yml` 的 `ROS_NETWORK_INTERFACE`,目前 `lan2`)是 DOWN 或不存在(網路線沒插)。接上網路線,或照 3.2 把 `CYCLONEDDS_URI` 改成 `lo` |
| `ddsi_udp_conn_write to udp/<ROS_PEER_IP>:… failed with retcode -3`,且 `ros2 node list` 看不到機器人(ping 卻通) | `CYCLONEDDS_URI` 同時有實體網卡與 `lo`(或 peer 有 `127.0.0.1`),從 `lo` socket 送往外部位址被 kernel 拒絕。只留實體網卡(見第 1 節),改 compose 後 `docker compose --env-file .ros2_setup -f docker_compose_UI_off.yml up -d --force-recreate` |
| `selected interface "lo" is not multicast-capable` | 用 `lo` 時的正常警告,可忽略 |
| headless 測試結束時 `virtual_tracking_sim` `exit code -11` | 已知:存完 CSV / PNG 後 process 結束時 segfault,不影響測試結果 |
| `ros2: command not found` | `bash -lc` 不會 source ROS;改用 `bash -ic '…'`、`docker exec -it … bash`,或手動 source |
| 改了 code 但行為沒變 | `install/` 是複製的,重新 `colcon build`;實機 launch 是 container 的 PID 1,build 後要 `docker restart april_tag_serive` 才會載入新 code |
| container 一直 `Restarting`,`docker exec` 失敗 | 預設 command 是實機 launch,workspace 未 build 或 launch 啟動失敗就會重啟循環;用第 1 節的 `sleep infinity` override 建立後再 build / 除錯 |
| 寫入 `virtual_tracking_logs/`、`build/`、`install/` 時 `Permission denied`(host) | 這些目錄由 container 內 root 建立;在 container 內操作,或把暫存檔放 container 的 `/tmp` |
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

2026-10-07 action 介面改為 `DockWheelchair`(`/up/dock_wheelchair`)/ `UndockWheelchair`(`/up/undock_wheelchair`)、刪除 `StartTracking` 與 `start:false`、tag / 後退距離改由 goal 指定、新增 `error_code` / 10 Hz feedback / `dock_timeout` / `undock_timeout` / `lost_target_timeout` 後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:118 tests,0 failures
- 虛擬測試 headless:dock `SUCCEEDED error_code=0`(Stage 2 elapsed 12.7 s)、`(held)`、undock `Leave reached 1.00m (forward=1.000m)` `SUCCEEDED error_code=0`、`(restored)`;`*_feedback.csv` 7 欄
- CLI:`tag_id:-2`、`tag_family:""`、`tag_family:tag25h9`、undock `distance_m:0.0` 皆 rejected 並印原因;dock 執行中 feedback `10.0 Hz`、送 undock 被拒(`state=STAGE1`)、Ctrl+C → `CANCELED error_code=5`;`tag_id:5` → `ABORTED error_code=1`(1.03 s);`lost_target_timeout:=0.5` → 0.52 s `error_code=1`;`dock_timeout:=3.0` → `ABORTED error_code=4`;只開 control(`manage_amcl_and_lidar_safety:=true`、無 AMCL)→ PREPARING 期間 feedback 仍 10 Hz,`ABORTED error_code=2`(restore 也失敗);dock 中對 control SIGINT → `ABORTED error_code=7`,client 有收到
- 未測:undock 到位後 restore 失敗(只有 `final_outcome` 單元測試);實機尚未測(`last_frame_age` 用 node clock − 相機 `header.stamp`,實機時鐘是否一致待確認)

2026-10-12 選 tag 移到 detection(新增 `TagTracing.msg` / `/up/tag_tracing`、detection 參數 `tracking_tag_family` / `tracking_tag_id`;control 只訂閱 `tag_tracing`、以 goal 比對,不符當作 tag 不見;刪除 `TargetSelector`(防抖動)與 control 的 `detect_tag_families`、`target_*` 參數;goal 只檢查格式)後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:114 tests,0 failures
- 虛擬測試 headless:dock `SUCCEEDED error_code=0`(Stage 2 elapsed 12.7 s)、`(held)`、undock `Leave reached 1.00m (forward=1.001m)` `SUCCEEDED`、`(restored)`;`[target] tracking tag36h11 id 0 … yaw estimator reset` 在 goal 開始時各一次
- `tracking_tag_id:=-1 tag_id:=-1`:同上 `SUCCEEDED`
- `tag_id:=5`:goal 被接受、mismatch warn 一次、1.04 s 後 `ABORTED error_code=1`、`max |cmd|` 全 0、`(restored)`
- `tracking_tag_id:=5`:無 warn,1.10 s 後 `ABORTED error_code=1`、`(restored)`
- `tracking_tag_family:=tag25h9`:detection `ValueError: Parameter tracking_tag_family 'tag25h9' is not in tag_families (tag36h11).` 結束
- `auto_start:=false`:`/up/tag_tracing` `valid: true`、`id: '0'`、header 有 stamp 與 `frame_id: camera_color_optical_frame`、`image_width: 640`;`tag_tracing` 與 `apriltag_poses` 都約 29.9 Hz;手動送 `tag_family: tag25h9` → 接受、warn、`ABORTED`(TAG_LOST)
- 實機尚未測(防抖動移除後,多個 tag 同時在畫面時的行為待確認)

2026-10-19 介面改為 `/wheelchair/cmd` / `/wheelchair/status`(`std_msgs/String` JSON,依 `[service_robot] wheelchair_interface.pdf`;刪除 `DockWheelchair` / `UndockWheelchair` action、`tools/WheelChair*.py`、`dock.sh` / `undock.sh`;`action_flow.py` → `cmd_flow.py`;所有 node 與 RealSense 改到 `wheelchair` namespace;control 的 tag / 後退距離改成 launch 參數 `tracking_tag_*` / `leave_distance`)後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:121 tests,0 failures
- 虛擬測試 headless:attach `SUCCEEDED error_code=0`(Stage 2 elapsed 12.9 s)、`(held)`、detach `Leave reached 1.00m (forward=1.000m)` `SUCCEEDED`、`(restored)`;`*_feedback.csv` 的 x/y/yaw 為數值、state 1/2/3
- CLI(`auto_start:=false`):status 4.998 Hz、合法 JSON、無資料時 `null`;`/wheelchair/attached` transient_local echo 得 `false`;`not json`、`cmd_id:0` → `[cmd] ignored`;`IDLE` 時 cancel → ignored;STAGE1 中 attach id 2、STAGE2 中 detach id 3 → rejected、status 仍為 `cmd_id 1`;cancel id 9(不符)→ ignored;cancel id 1(相符)→ 同一 callback 立即發零速,status `IDLE` `cmd_id 1` `error_code 5`;`IDLE` 時 reset → `cmd_id 0`;STAGE2 中 reset → `IDLE`、`cmd_id 0` `error_code 0`;`tracking_tag_id:=5` → `error_code 1`;`dock_timeout:=3.0` → `error_code 4`
- 實機尚未測(RealSense 改到 `/wheelchair/camera/...` 後 detection 是否收到影像待確認)

2026-10-19 新增測試工具 `tools/attach.py` / `detach.py`(共用 `tools/wheelchair_cmd.py`,`python3` 直接執行,無 entry point)後,在同一 container 內實測(domain 99、`lo`):

- `attach --goal --cmd_id 1` → STAGE1 … `IDLE cmd_id=1 error_code=0`,exit 0;`detach --goal --cmd_id 2` → LEAVING … `IDLE error_code=0`,exit 0
- goal 3 執行中 `--cancel --cmd_id 3` → cancel exit 0、goal 最後一行 `error_code=5` exit 1;`timeout --preserve-status -s INT 3 attach --goal --cmd_id 4` → 自動送 cancel,`error_code=5`,exit 1
- goal 5 執行中 `--reset --cmd_id 6` → reset 印到 `IDLE cmd_id=0` exit 0,goal exit 1
- `--goal --cmd_id 0` → exit 1(會印 `cmd_id=0` 的 IDLE status,因為平常 status 就是 0);`IDLE` 時 `--cancel --cmd_id 9` → 無輸出 exit 1;沒有 launch → 無輸出 exit 2;沒給 `--cmd_id` → usage exit 2

2026-10-20 新增 `/g7_body/simple_ui/body_status`(`control_node.py` 的 `BODY_STATUS` 表,在 `_transition_locked` 發送)後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:122 tests,0 failures
- 虛擬測試 headless(另開 `ros2 topic echo /g7_body/simple_ui/body_status std_msgs/msg/Int32`):依序 `1, 0, 2, 0`;attach / detach 皆 `SUCCEEDED`、`(held)`、`(restored)`
- `tracking_tag_id:=5`:`1, 0`,`ABORTED error_code=1`、`(restored)`
- 實機尚未測(G7+ `simple_ui` 是否收到)

2026-10-21 `/wheelchair/cmd` topic 改為 `std_srvs/Trigger` service `/wheelchair/attach` / `detach` / `cancel`(`cmd_id` 由 control 遞增、在 response `message` JSON 回傳;刪除 `reset`、`parse_cmd`;工具刪除 `--cmd_id` / `--reset`)後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:110 tests,0 failures
- 虛擬測試 headless:`attach accepted: cmd_id=1`、attach `SUCCEEDED`、`(held)`、`detach accepted: cmd_id=2`、`Leave reached 1.00m (forward=1.001m)` `SUCCEEDED`、`(restored)`
- CLI(`auto_start:=false`):`ros2 topic list` 無 `/wheelchair/cmd`、三個 service 存在;`IDLE` cancel → `success=False` `nothing running`;attach → `{"cmd_id": 1}`、status `STAGE1 cmd_id 1`;執行中 attach / detach → `success=False` `rejected: state=STAGE1`;cancel → `success=True` `{"cmd_id": 1, "msg": "cancel requested"}`、status `IDLE error_code 5`;之後 cmd_id 依序 2、3、4
- 工具(改為每個 service 一支、無參數、只印 response 立即結束:`attach.py` / `detach.py` / `cancel.py`):`IDLE` 時 `cancel.py` exit 1;`attach.py` exit 0、status `STAGE1 cmd_id 1`;執行中 `detach.py` exit 1;`cancel.py` exit 0、status `IDLE error_code 5`;`detach.py` exit 0、status `LEAVING cmd_id 2`;多給參數 exit 2;沒有 launch exit 2
- 未測:cancel 回 `already ending`(時間點難卡);互動版 `x` 鍵(無 `DISPLAY`);實機尚未測(Shane 端 `bridge.yaml` 改 service 橋接)

2026-10-22 `/wheelchair/attach` / `detach` 改為 `std_srvs/SetBool`(`data:true` 開始、`data:false` 只取消同一個 service 的執行中 cmd;刪除 `/wheelchair/cancel` 與 `tools/cancel.py`;工具改為必填 `true|false`)後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:112 tests,0 failures
- 虛擬測試 headless:`attach accepted: cmd_id=1`、attach `SUCCEEDED`(Stage 2 elapsed 12.8 s)、`(held)`、detach `Leave reached 1.00m (forward=1.001m)` `SUCCEEDED`、`(restored)`
- CLI(`auto_start:=false`):`ros2 service list` 只有 `/wheelchair/attach`、`/wheelchair/detach`,type `std_srvs/srv/SetBool`;`IDLE` 時 `attach false` → log `attach false rejected: nothing running`;`attach.py true` exit 0、`STAGE1 cmd_id 1`;`detach.py false` exit 1 `rejected: state=STAGE1 (only accepted in LEAVING)`、status 不變;`attach.py true` exit 1 `(only accepted in IDLE)`;`attach.py false` exit 0 `cancel requested`、`IDLE error_code 5`;`detach.py true` exit 0 `LEAVING cmd_id 2`;`attach.py false` exit 1 `(only accepted in STAGE1, STAGE2)`;`detach.py false` exit 0、`IDLE error_code 5`;沒給參數 / `yes` → exit 2;沒有 launch → exit 2
- 未測:`data:false` 回 `already ending`;互動版 `x` 鍵(無 `DISPLAY`);實機尚未測(Shane 端 `bridge.yaml` 需改 SetBool、移除 cancel)

2026-10-27 刪除 `error_code` 5 `ACTION_CANCELED`(取消改回 0,`OTHER` 5、`NODE_SHUTDOWN` 6)、status `state` 改發 `Attaching(STAGE1)` / `Attaching(STAGE2)` / `Detaching`(enum 不變)、成功時 `error_msg` 為 `Attach succeeded` / `Detach succeeded` 後,在同一個 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:114 tests,0 failures
- 虛擬測試 headless:status `Attaching(STAGE1)` → `Attaching(STAGE2)` → `Detaching`;`attach cmd_id=1 finished: SUCCEEDED error_code=0 - Attach succeeded`、`(held)`、`detach cmd_id=2 finished: SUCCEEDED error_code=0 - Detach succeeded`、`(restored)`;`*_feedback.csv` state 只有 1/2/3;log 仍為 `[state] STAGE1 -> STAGE2`
- CLI(`auto_start:=false`):`attach.py true` → `Attaching(STAGE1)`;`attach.py false` → `IDLE` `error_code 0` `error_msg "Cancelled cmd_id=1. elapsed=5.55s"`;`detach.py true` → `Detaching`,完成後 `error_msg "Detach succeeded"`
- 未測:`detach false` 取消(leave 在送出前已完成)、取消後 restore 失敗回 2 的實際流程(只有 `final_outcome` 單元測試);實機尚未測(Shane 端需改 state 字串、碼值與取消判斷)

2026-10-27 service response `message` 改為純文字(刪除 `reply_json`,不帶 `cmd_id`;sim 改從 status 取 `cmd_id`)後,在同一 container 內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:112 tests,0 failures
- 虛擬測試 headless:`attach accepted.`、`attach cmd_id=1 finished: SUCCEEDED … Attach succeeded`、`(held)`、`detach accepted.`、`detach cmd_id=2 finished: SUCCEEDED … Detach succeeded`、`(restored)`
- CLI:`attach false`(IDLE)→ `success=False message=nothing running`;`attach true` → `success=True message=attach accepted`;執行中 `detach true` → `rejected: state=STAGE1 (only accepted in IDLE)`、`detach false` → `rejected: state=STAGE1 (only accepted in LEAVING)`;status `Attaching(STAGE1)` `cmd_id 1`;`attach false` → `cancel requested`、status `IDLE` `error_code 0` `Cancelled cmd_id=1. …`
- 實機尚未測(Shane 端不可再 `json.loads` response)

2026-11-12 參數統一改由 `april_tag_config.json`(`config_loader.py`,`april_tag.launch.py` 只剩 `config_file`;attach / detach 被接受時重新讀取)後,在新 build 的 `ros2_april_tag:v1` container 內實測(domain 99、`lo`;workspace 為 host `/home/cobo/Desktop/april_tag`):

- 修正:原本 `_start_cmd` 在檢查 state **前** reload,執行中收到 `data:true` 也會改掉 stage 距離 / tag / `yaw_estimator` 再被拒;改為 `_read_config()`(lock 外讀檔驗證)+ `_apply_config_locked()`(lock 內、確認接受後才套用);非 `IDLE` 以 state 原因拒絕,config 錯誤回 `rejected: Config reload failed: …`
- `colcon build --symlink-install`:3 packages 成功;`colcon test --packages-select apriltag`:142 tests,0 failures
- 虛擬測試 headless(不帶 `config_file`,走 ROS 參數 fallback):attach `SUCCEEDED`、`(held)`、`Leave reached 1.00m (forward=1.001m)`、detach `SUCCEEDED`、`(restored)`
- `april_tag.launch.py --show-args`:自身只有 `config_file`(預設 `/mnt/work_space/april_tag_config.json`),其餘為 include 的 `realsense/camera.launch.py` 參數
- 單獨啟動 `apriltag_control -p config_file:=<副本>`:`IDLE` attach → `[config] Reloaded`;`STAGE1` 中改 JSON 再 attach → `rejected: state=STAGE1`、無 reload;`stage2_distance: true`、`tag_sizes: 'abc'`、JSON 語法錯誤 → 皆拒絕並印原因;修好後接受
- `apriltag_detection -p config_file:=…`:正常啟動;`tracking_tag_family: tag25h9` → 啟動即 `ValueError`
- 未測:calibration 套用在實際對位(`use_calibration=true` 但 config 內 `stage1/2/detach` 皆 `null`)、`tools/calibrate.py` 寫回 JSON 的實際流程(只有單元測試);實機尚未測

2026-11-12 修正 `save_calibration_point` 遇到壞檔會覆蓋整個 `april_tag_config.json`(新增 `calibration.read_config_dict`:檔案不存在 / JSON 錯誤 / 最外層非 object → `ValueError`、不寫不建檔;`calibrate.py` 在 `rclpy.init()` 前先檢查)後,在同一 container 內實測:

- `colcon test --packages-select apriltag`:146 tests,0 failures
- `calibrate.py --stage1 --file`:不存在 → `ERROR: Config file not found: '/tmp/none.json'`、`{bad` → `ERROR: Cannot parse …`(檔案內容不變)、`[]` → `ERROR: … must contain a JSON object, got 'list'`,皆立刻 exit 1、沒有 `Waiting up to …`、沒有產生檔案或 PNG
- 未測:合法檔案的實際存檔(container 的實機 detection 在跑,避免寫入;由單元測試涵蓋)

2026-11-12 `max_vx` / `max_vy` / `max_vw` 改由 `april_tag_config.json` `control` 區塊設定,新增 `min_vx` / `min_vy` / `min_vw`(0.01;Stage 1 / 2 非 0 且小於 min 時送 `sign(v)·min`)後,在同一 container(`sleep infinity`)內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:177 tests(新增 `test_speed_limits.py`),0 failures
- 虛擬測試 headless:attach `SUCCEEDED`(Stage 2 elapsed 14.8 s,之前約 12.7 s)、`(held)`、`Leave reached 1.00m (forward=1.001m)`、detach `SUCCEEDED`、`(restored)`;`*_cmd_raw.csv` 651 筆,每軸為 0 或 `|v| ∈ [0.01, max]`
- 單獨啟動 `apriltag_control -p config_file:=<副本>`:IDLE 時把副本改成 `min_vx: 0.2` 再 attach → `rejected: Config reload failed: control.min_vx (0.2) must be <= max_vx (0.1)`;用該檔啟動 → `ValueError` 結束
- 未測:實機(min 是否足以讓 G7+ 在低速動起來、目標附近是否來回擺動)

2026-11-12 對位 tolerance 移到 `april_tag_config.json`(`control.stage1_tolerance` / `stage2_tolerance`)、刪除 `stage1_distance` / `stage2_distance` / `leave_distance` / `use_calibration`(目標只來自 calibration 三個點,必填)、`config_file` 改必填(刪除 ROS 參數 fallback)、虛擬測試改用 `config/virtual_tracking_config.json`、`april_tag.launch.py` 不再把 `config_file` 漏給 RealSense 後,在同一 container(`sleep infinity`)內實測(domain 99、`lo`):

- `colcon test --packages-select apriltag`:212 tests,0 failures
- 虛擬測試 headless:`[calibration] STAGE1: z_ref=0.5000m x_ref=0.0260m … tolerance x=0.050m …`、`Stage 2 aligned at 0.28m`(elapsed 16.4 s)、`(held)`、`Leave reached 1.00m (forward=1.000m)`、`SUCCEEDED` ×2、`(restored)`;副本 `dock_timeout: 3.0` → `error_code=4`
- 單獨啟動 control:加回 `stage1_distance` / `use_calibration` → `… was removed; …`;`calibration.stage2: null` → `calibration.stage2 is missing or incomplete …`;`stage2_tolerance.yaw: 0` → `must be positive`;IDLE 時改壞檔再 attach / detach → `rejected: Config reload failed: …`;沒給 `config_file`(control、detection)→ `config_file is required`;calibration 全 `null` 時 detection 正常啟動
- `april_tag.launch.py`(無相機):修正前 HEAD 的 config 與新 config 皆 `Caught exception in launch: Unexpected type for parameter value None`;修正後 calibration 全 `null` → 只有 control `ValueError` 結束,launch / detection 繼續跑;出廠 config → detection、control 都起來。用 3.2 的 `lo` URI(沒有 `MaxAutoParticipantIndex`)跑實機 launch 時偶爾 `rmw handle is invalid`,改用 container 預設 `CYCLONEDDS_URI`(`MaxAutoParticipantIndex=50`)即正常
- 已知限制:改 `tracking_tag_id` 測不到 TAG_LOST(sim 跟著畫同 id)
- 未測:實機(相機、對位)

2026-11-12 `/wheelchair/status` 新增 `cmd_vx` / `cmd_vy` / `cmd_wz`(`cmd_flow.recent_cmd`、`control_node._last_cmd`)後,在同一 container(`sleep infinity`)內實測(domain 99、container 預設 `CYCLONEDDS_URI`;3.2 的 `lo` URI 這次 `Failed to find a free participant index`,因為沒有 `MaxAutoParticipantIndex`):

- `colcon test --packages-select apriltag`:217 tests,0 failures
- 虛擬測試 headless:attach `SUCCEEDED`(Stage 2 elapsed 12.6 s)、`(held)`、`Leave reached 1.00m (forward=1.002m)`、detach `SUCCEEDED`、`(restored)`;`ros2 topic echo /wheelchair/status`:開機 `null`,Stage 1 / 2 為實際速度(每軸 0 或 `|v| ∈ [0.01, max]`),`Detaching` 為 `-0.1, 0.0, 0.0`,結束後 `IDLE` 連續 5 則 `0.0`(1 s)後變 `null`
- 未測:實機

2026-11-12 `tools/virtual_tracking_sim.py` 對齊現行架構(只改 sim / launch / config 註解,control / detection 未動):ground truth 與 goal 改用每個 stage 的 calibration `x_ref` / `yaw_ref`(與 control 同公式)、cmd_vel / AMCL / lidar 名稱改讀 `ros_interfaces`、stage 改依 status `state`、feedback CSV 加 `cmd_vx/vy/wz`(10 欄)、新增 body_status 檢查、`hide_tag_after` / `h` 隱藏 tag(TAG_LOST)、dashboard 限制線 / tolerance 帶改讀 config、fake G7 改由 sim 建立。在同一 container(`sleep infinity`)內實測(domain 99、container 預設 `CYCLONEDDS_URI`):

- `colcon test --packages-select apriltag`:240 tests(新增 `test_sim_geometry.py` 23 個),0 failures
- 虛擬測試 headless:attach `SUCCEEDED`(Stage 2 elapsed 12.6 s)、`(held)`、detach `SUCCEEDED`、`(restored)`、`body_status=[1, 0, 2, 0] … (OK)`;`*_feedback.csv` 10 欄、state 1/2/3;PNG 有分段 tolerance 帶與 ±0.1 / ±0.05 限制線
- `hide_tag_after:=5.0`:`tag hidden at t=5.0s`、`AprilTag lost for 1.00s`、`ABORTED error_code=1`、`(restored)`、`body_status=[1, 0] … (OK)`
- 副本 stage2 `x_ref: 0.05, yaw_ref: 0.03`:attach `SUCCEEDED`;Stage 2 結束時 GT `(+0.024, -0.000, +0.001)` 與 control `final=(+0.026, -0.000, +0.000)` 一致
- 副本 `stage1_cmd_vel_topic: /cmd_vel_sim_test`:control 發到 `/cmd_vel_sim_test`,attach / detach `SUCCEEDED`
- 未測:互動版 dashboard(`h` 鍵、按鈕;需 `DISPLAY`)

2026-11-13 `virtual_tracking.launch.py` 的 `config_file` 預設改為直接讀 workspace 的 `src/apriltag/config/virtual_tracking_config.json`(之前讀 `install/` 的複本,改 src 不 build 就不生效:當天兩次 run 以為 `stage1.x_ref` 是 ±0.8,實際用的是 install 的 0.026)、`setup.py` 不再安裝 `config/*.json` 後,在同一 container(`sleep infinity`)內實測(domain 99、container 預設 `CYCLONEDDS_URI`):

- `rm -rf install/apriltag/share/apriltag/config` 後 build:`share/apriltag/` 無 `config/`;`--show-args` 的 `config_file` 預設 `/mnt/work_space/src/apriltag/config/virtual_tracking_config.json`
- 不 build、只改 src JSON:control log `[calibration] STAGE1: … x_ref=-0.8000m yaw_ref=30.0000rad` 與 src 一致;改成 `z_ref 0.8, x_ref 0.3, yaw_ref 0` → attach / detach `SUCCEEDED`、`body_status … (OK)`,Stage 1 結束時 tag `t_x=+0.280`(像素 u≈533,右側);`x_ref -0.3` → x / y 收斂(`y_error≈0`,`t_x≈-0.30`,左側)
- 已知問題(未處理):`z_ref 0.8, x_ref -0.3, yaw_ref 0` 時 Stage 1 的 yaw 在約 2–5° 擺盪、`[S3]` probe 反覆觸發,147 s 未進 Stage 2(`x_ref +0.3` 正常);另 2026-11-13 兩次 run 在 Stage 1 → 2 切換時 sim GT yaw 為 5.7° / 11.9°,超出 tolerance 0.05 rad,control 的 yaw 卻在 tolerance 內
- `colcon test --packages-select apriltag`:240 tests,1 failure(`test_virtual_config_is_valid`:src JSON 的 `stage1.z_ref` 當時為 0.8,測試預期出廠值 0.5;與本次修改無關)

2026-11-13 刪除 calibration 的 `stamp` / `image_path` 與 `detach.x_ref` / `yaw_ref`(`CalibrationPoint` 只剩 `z_ref` / `x_ref` / `yaw_ref`;detach 只讀寫 `z_ref`、記憶體中 x / yaw 為 0;多餘 key 忽略;`calibrate.py` 不再存 PNG、不訂閱 `marked_image`)、`virtual_tracking_config.json` 移到 repo 根目錄(刪除 `src/apriltag/config/`,stage1 改回出廠值)後,在同一 container(`sleep infinity`)內實測(domain 99、container 預設 `CYCLONEDDS_URI`):

- `colcon test --packages-select apriltag`:252 tests,0 failures
- `--show-args`:`config_file` 預設 `/mnt/work_space/virtual_tracking_config.json`
- 虛擬測試 headless:`[calibration] … detach(z_ref=1.0000m)`、`[calibration] LEAVING: z_ref=1.0000m`、attach `SUCCEEDED`、`(held)`、`Leave reached 1.00m (forward=1.005m)`、detach `SUCCEEDED`(`final=(+0.005, -0.025, +0.083)`,y 為未加偏移的值)、`(restored)`、`body_status=[1, 0, 2, 0] … (OK)`
- `calibrate.py`(對 sim,`auto_start:=false`,`--file /tmp/cal_test.json`):`--detach` → 檔內 `detach` 只剩 `{"z_ref": …}`、exit 0;`--stage2` → 三個 key、exit 0;沒有產生 `calibration_*.png`;不存在的 `--file` → exit 1
- 未測:實機


2026-11-16 新增 Stage 1 直線進場(`control.stage1_approach`;`YawEstimate.yaw_valid`、`math_utils.approach_lateral_error`、`control_flow.ramp_scale` / `approach_vx_scale`、`clamp_control` / `publish_control` 的 `vx_scale`)後，在同一 container(`sleep infinity`)內實測(domain 99、container 預設 `CYCLONEDDS_URI`;config 用 `/tmp` 出廠值副本，只改 `enabled`):

- `colcon test --packages-select apriltag`:301 tests,1 failure(`test_virtual_config_is_valid`,`will_testing` 的 virtual config `detach.z_ref` 0.8,既有問題，見 3.2)
- 虛擬測試 headless 4 run 皆 attach / detach `SUCCEEDED`、`(held)`、`(restored)`、`body_status … (OK)`;`[approach] yaw valid at d≈0.70m`。下表由 `*_sim.csv` 算相機到 stage1 進場線的距離:

  | 起點 (y, yaw) | `enabled` | Stage 1 時間 | 前向 0.6–0.5 m 最大橫向偏移 | 前向 0.55 m 時 yaw |
  |---|---|---|---|---|
  | 0.15, 10° | true / false | 12.0 / 16.1 s | 0.029 / 0.088 m | 1.5° / 8.6° |
  | -0.25, -10° | true / false | 13.0 / 10.0 s | 0.039 / 0.054 m | -2.0° / -5.7° |

  `true` 時遠距離(> 0.72 m)`|y_err| > 0.09` 的 sample 都沒有前進(`false` 時約 116 個);1.0 → 0.7 m 是「tag 置中、朝 tag 前進」(此時 yaw 無效、LQR 不轉，距進場線約 0.12 m),到 0.7 m 停下轉正、橫移再直線進入
- 未測：實機(門檻值只在 sim 調過、實機 yaw 雜訊下 `e_lat` 是否穩定)
2026-11-17 LQR 權重移到 JSON(`control.lqr` 的 `q_weights` / `smooth_tau`,必填;R 固定 1;`config_loader.LqrConfig` / `parse_lqr`,只有 control 檢查，detection / sim 不受影響)後，在同一 container(`sleep infinity` override)內實測(domain 98、container 預設 `CYCLONEDDS_URI`;同時有別的 domain 99 測試在跑;config 用 `/tmp` 的 `april_tag_config.json` 副本加 `manage_amcl_and_lidar_safety: true`):

- `colcon test --packages-select apriltag`:323 tests,1 failure(`test_virtual_config_is_valid`,未 commit 的 virtual config `detach.z_ref` 0.8,既有問題)
- 虛擬測試 headless 4 run 皆 attach / detach `SUCCEEDED`、`(held)`、`(restored)`、`body_status … (OK)`;由 `*_feedback.csv` 算(y / yaw settle = Stage 1 中最後一次超過 0.05 的時間):

  | `q_weights` | 起點 (y, yaw) | Stage 1 時間 | y settle | yaw settle |
  |---|---|---|---|---|
  | `[0.3333, 1.0, 0.1667]`(出廠) | 0.15, 10° | 12.2 s(11-16 為 12.0 s) | 2.8 s | 8.4 s |
  | 出廠 | -0.25, -10° | 14.8 s(11-16 為 13.0 s) | 3.8 s | 9.4 s |
  | `[0.3333, 0.5, 0.9]` | 0.15, 10° | 12.4 s | 3.4 s | 9.0 s |
  | `[0.3333, 0.5, 0.9]` | -0.25, -10° | 13.4 s | 4.0 s | 9.4 s |

  y 依預期變慢;yaw 的 K 加倍(0.41 → 0.95)但 settle 幾乎不變:約 0.7 m 外 yaw 無效(wz≈0),有效後 10° 級誤差仍在 `max_vw` 0.05 飽和區。要讓 yaw 變快需同時提高 `max_vw`
- `auto_start:=false` 手動:副本 `smooth_tau: 0` → `attach.py true` exit 1、`rejected: Config reload failed: control.lqr.smooth_tau (0) must be positive`;改成 `q_weights [0.5, 0.5, 2.0]`、`smooth_tau 0.3` → 接受、`[config] Reloaded`、`[lqr] q=[0.5000, 0.5000, 2.0000] (K~=[0.707, 0.707, 1.414] …) smooth_tau=0.30s`
- `docker compose --env-file .ros2_setup up -d --force-recreate`(切回實機 launch):control 啟動印 `[lqr] q=[0.3333, 1.0000, 0.1667] …`、`State IDLE`
- 未測：實機

2026-11-17 yaw 估計 / S3 probe 參數移到 JSON(`yaw_estimator` 補 14 個 key、新增 `yaw_probe` 11 個 key(含原本不是參數的 `end_c_sign`),皆必填、附 `_doc`;刪除 25 個 ROS 參數)後，在同一 container 內實測(domain 98、container 預設 `CYCLONEDDS_URI`;config 為 `/tmp` 的 `april_tag_config.json` 副本加 `manage_amcl_and_lidar_safety: true`):

- `colcon test --packages-select apriltag`:359 tests,1 failure(既有的 `test_virtual_config_is_valid`)
- 虛擬測試 headless(0.15, 10°):`[yaw_estimator]` / `[yaw_probe]` 印出預設值、attach / detach `SUCCEEDED`、`(held)`、`(restored)`、`body_status … (OK)`,Stage 1 12.4 s(LQR 驗證時 12.2 s)
- `auto_start:=false`:副本 `yaw_probe.wz: 0` → `rejected: Config reload failed: yaw_probe.wz (0) must be positive`;改 `wz 0.02`、`initial_direction -1`、`yaw_estimator.q_proc 0.0001` → 接受、log 顯示新值、attach `SUCCEEDED`(此 run 未觸發 S3,`turning -` 未實際看到)
- `docker restart april_tag_serive`:實機 launch 的 control 印 `[lqr]`、`[yaw_estimator]`、`[yaw_probe]`、`State IDLE`
- 未測：實機

2026-11-18 新增分軸 hold(`control.per_axis_hold`,出廠 `true`;`config_loader.parse_per_axis_hold`、`control_flow.axes_in_tolerance`、`publish_control` 的 `hold_axes`、`control_node._publish_hold_stop`)後，在同一 container 內實測(domain 99、container 預設 `CYCLONEDDS_URI`;config 為 `/tmp` 的 `april_tag_config.json` 副本加 `manage_amcl_and_lidar_safety: true`,只改 `per_axis_hold`):

- `colcon test --packages-select apriltag`:381 tests,2 failures(`test_virtual_config_is_valid`、`test_shipped_yaw_defaults[virtual]`:committed 的 virtual config `detach.z_ref` 0.8、`yaw_probe.trigger_frames` 50,非出廠值，既有問題)
- 虛擬測試 headless 4 run 皆 attach / detach `SUCCEEDED`、`(held)`、`(restored)`、`body_status … (OK)`:

  | `per_axis_hold` | 起點 (y, yaw) | Stage 1 / Stage 2 時間 | Stage 2 結束 `x_f` / `y_f` / `yaw_abs_f` |
  |---|---|---|---|
  | true | 0.15, 10° | 15.4 / 4.2 s | 0.046 / 0.049 / 0.014 |
  | true | -0.25, -10° | 15.6 / 4.2 s | 0.047 / -0.049 / 0.003 |
  | false | 0.15, 10° | 15.4 / 4.0 s | 0.027 / -0.001 / 0.007 |
  | false | -0.25, -10° | 13.2 / 4.0 s | 0.027 / 0.000 / 0.005 |

  `true` 時 log 有 `[axis_hold] x=… y=… yaw=…` 變化與 `[publish] … axis hold vx=0 vy=0 wz=0`;`*_cmd_raw.csv` 在三軸都 hold 後只有一筆零速，接著約 1.0 s(`hold_seconds`)無 cmd,再來是 stage 結束的停車。各軸一進 tolerance 就停，所以結束時誤差停在 tolerance 邊緣附近(約 0.05),`false` 時會繼續收斂到接近 0
- 未測：實機

## 7. `src/apriltag/README.md` 與實際不一致之處

- (2026-10-06 已修正 launch 檔名與 `test/` 目錄;目前無已知不一致)

2026-11-19 刪除 `virtual_tracking_config.json`,`virtual_tracking.launch.py` 的 `config_file` 預設改為 workspace 的 `april_tag_config.json`;sim 在 `manage_amcl_and_lidar_safety: false` 時略過 held 檢查;`test_config_loader.py` 刪除 `test_virtual_config_is_valid` 與 `virtual` 參數化。在同一 container 內實測(domain 99、`lo` + `MaxAutoParticipantIndex=50`):

- `colcon test --packages-select apriltag`:376 tests,1 failure(`test_shipped_yaw_defaults[april_tag]`:`april_tag_config.json` 的 `yaw_probe.trigger_frames` 50 非出廠值 10,既有問題)
- 虛擬測試 headless(預設 config):`[config] Reloaded from '/mnt/work_space/april_tag_config.json'`、attach `SUCCEEDED`(elapsed 14.5 s)、`(unmanaged, held check skipped)`、detach `SUCCEEDED`、`(restored)`、`body_status=[1, 0, 2, 0] … (OK)`

2026-11-19 新增 yaw deadband(`control.yaw_deadband` `in` 0.03 / `out` 0.05;`config_loader.YawDeadband` / `parse_yaw_deadband`、`control_flow.yaw_deadband_hold`、`control_node` 併入 `hold_axes`)後，在同一 container 內實測(domain 99、`lo` + `MaxAutoParticipantIndex=50`;repo config,`min_vw` 0.07):

- `colcon test --packages-select apriltag`:414 tests,1 failure(既有的 `test_shipped_yaw_defaults[april_tag]`);新增 `test_yaw_deadband.py`
- 虛擬測試 headless 2 run((0.15, 10°)、(-0.25, -10°))皆 attach / detach `SUCCEEDED`、`body_status … (OK)`,attach elapsed 14.1 s;Stage 2 結束 `yaw_abs_f` 0.017 / 0.027(≤ 0.03,未卡住)
- `*_cmd_raw.csv`:wz 相鄰翻號 0 次(加之前 `20261119_113559` 129 筆中 45 次 ±0.07 翻號),wz 非 0 約 60 筆，集中在 `[yaw_deadband] fix` 期間(`yaw_f` 跳到約 ±0.2 時)
- 三軸全 hold(不發 cmd)前最後一筆 wz 皆為 0,沒有 `yaw_f` 漂移造成的重新修正;但最後一筆 `vx` / `vy` 可能為 ±0.02(分軸 hold 既有行為，未處理)
- 已知：遠距離 `yaw_f` ≈ 0(方向不明)時 deadband 為 hold,GT yaw 約 -0.15 rad 時也不修，直到 `yaw_f` 跳出 `out`;S3 probe 的 wz 會被歸零
- 未測：實機
