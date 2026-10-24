# 模擬障礙安全鏈路

適用 ROS 2 Humble、Gazebo Fortress、`ros_gz`。所有流程預設模擬，沒有新增實體裝置連線；不得與獨立 comm-sim、yaw-check 或其他底盤流程同時啟動。

## 接線與模式

預設為 `/cmd_vel/{nav,follow,teleop}` → mux → `/cmd_vel_mux/output` → obstacle_guard → `/cmd_vel_guard/output` → watchdog → `/cmd_vel`。選用 `collision_monitor:=true` 時，guard 後面加入 Nav2 collision_monitor，其輸出 `/cmd_vel_collision/output` 接 watchdog。只有 watchdog 發布最終 `/cmd_vel`；guard 或 collision_monitor 停止發布後，watchdog 以單調壁鐘 0.25 秒逾時輸出零速。急停仍直接在 watchdog 鎖定。

`guide`、`follow` 及獨立 `mode_mux` 預設 `obstacle_guard:=true`、`collision_monitor:=false`；可明確使用 `obstacle_guard:=false` 比較效果，collision_monitor 可獨立啟用。整合 launch 的 `use_mux` 現在只接受 true，避免歷史上的直接 `/cmd_vel` 旁路。獨立通訊模擬沿用自己的 mux／watchdog，不需要雷射。Nav2 controller、BT 與 navigation.launch.py 未修改。

GUIDE 的 Nav2、FOLLOW 的直接控制／NAV2_HANDOFF，以及 TELEOP 都經同一個 guard。模式仲裁與 Nav2 目標取消仍由既有管理器處理。跟隨訂閱 `/safety/obstacle_state`：新鮮 STOP 可觸發既有持續障礙交接；STALE 使 scan_valid 為 false，停止並取消交接，不把感測器失效當成可繞行障礙。原有 LOS、人物過近／遺失／UWB 過期保護保留。STOP 是當下要求速度的判定；直接控制停車後，狀態可能重新評估，持續繞行判定仍有原有 LOS 去抖與延遲。

## 幾何與失效行為

- 整合 launch 在啟動時直接讀取已安裝 Nav2 YAML 的 local／global footprint，要求兩者相同且為置中矩形；目前半長 0.30、半寬 0.25 公尺，與 xacro 的 0.60 × 0.50 公尺及兩個 Nav2 costmap 矩形一致；pytest 檢查來源一致性。修改車體時須同步 xacro 與 Nav2 足跡；guard 自動採用 Nav2 值。獨立節點未指定 footprint_file 時使用上述預設尺寸。
- 雷射在訊息時間以 TF 轉至 base_footprint，沒有假設雷射在車體中心（現有 laser_link 的 x 為 0.18 公尺）。缺 TF、非平面安裝、NaN、負值、低於量測下限、超出上限、非完整周向或過疏掃描均停止。正無限大視為未命中；不補值。
- 以要求線／角速度進行保守的等速矩形掃掠；前視時間為 `reaction + max(abs(v)/deceleration, abs(w)/angular_deceleration)`。這比相同比例煞車的預測距離長，並非 Nav2 軌跡規劃。掃掠間隔最多 0.025 秒，以車體邊界最大速度乘半間隔加寬，避免只檢查離散位置。直行／倒車只忽略移動反方向且不接觸安全足跡的回波；預設 mux 仍不允許倒車。
- 純原地旋轉使用矩形外接圓，涵蓋全部旋轉方向，淨空時允許旋轉；它可能比實際有限角度掃掠保守。曲線使用等速圓弧掃掠。
- 掃掠至障礙間距 ≤ 0.10 公尺立即停止；再往外 0.50 公尺為線性減速區，線／角速度套用相同比例。此比例保留要求曲率。降低限制立即生效；放寬須額外 0.05 公尺且持續 0.30 秒。啟動／恢復期間先維持零速。
- 雷射 header 的模擬時間與接收單調壁鐘都須小於 0.50 秒；重送舊 header 不能繞過保護。guard 及 watchdog 的 20 Hz 定時器使用單調壁鐘，暫停模擬也會逾時。mux 輸入停止後，guard 自身以 0.25 秒逾時輸出零速，不能一直重送舊速度。
- watchdog 的最終恢復限制保留曲率，線／角速度增幅上限分別為 0.6 m/s²、1.2 rad/s²；安全減速／零速立即套用，不能因加速度平滑而超出 guard 限制。

`/safety/obstacle_state` 為 String JSON，20 Hz 包含 `t`（模擬秒）、`state`（CLEAR／SLOW／STOP／STALE）、`scale`、`swept_clearance`（公尺，無可用值為 null）。後者是要求速度掃掠的保守間距，**不是**真值足跡最小間距。

參數可透過 ROS parameters 調整；包含 footprint_file、上述幾何、margin、reaction、deceleration、angular_deceleration、slow_extra、hysteresis、clear_time、scan_timeout。模擬煞車能力與 DDS／排程延遲尚未辨識，這些值不是硬體安全認證，也不能保證雷射平面外、漏測、透明物體或真實輪胎滑移的安全。

## 選用 Humble collision_monitor

compute 映像加入 apt `ros-humble-nav2-collision-monitor`，預設不啟動。參數在 bringup/config/collision_monitor.yaml，使用 [Nav2 Humble 官方參數範例](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/params/collision_monitor_params.yaml) 的平坦 points 陣列、max_points 與 LaserScan source；專用 lifecycle_manager 負責 configure／activate。固定停止矩形為車體外加 0.10 公尺，並非另一套速度規劃器。啟用後若 lifecycle、TF 或輸入失效導致沒有輸出，最終 watchdog 逾時停止。這個選項不能替代 guard 的 acquisition header 過期檢查。

## 維護者驗收（無視窗）

以下由維護者在有 Docker 的主機執行；本次開發環境沒有執行容器或 ROS。

```bash
./scripts/dev.sh build
./scripts/dev.sh up
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh follow headless:=true fault_injection:=true route_around:=false
```

另一個終端機執行；工具只注入模擬感測器故障，不發布速度。`--suite obstacle` 依序測試前方減速、前方停止、雷射掉訊、舊 header 重送，每次均要求移動基線與解除後恢復；停止列另要求命令 ≤ 0.8 秒、真值 ≤ 1.5 秒且持續停止 ≥ 0.3 秒。減速列要求觀測 SLOW、0 < scale < 0.95 與非零命令，不能視為真值減速度驗收。

```bash
./scripts/dev.sh fault-check --suite obstacle --motion-timeout 120
./scripts/dev.sh fault-check --suite follow --motion-timeout 120
./scripts/dev.sh record obstacle1 --duration 120
./scripts/dev.sh eval-bag obstacle1
```

停止原先 follow，再逐次重開以下兩種組合，分別重跑 obstacle suite。不要同時啟動：

```bash
./scripts/dev.sh follow headless:=true fault_injection:=true route_around:=false collision_monitor:=true
./scripts/dev.sh follow headless:=true fault_injection:=true route_around:=true collision_monitor:=true
```

route_around=true 時可能改道或停止輸出直行要求，使人工假障礙的 SLOW 列無法得到穩定減速證據；保留 FAIL 並另外檢查 `/follow/state`、handoff goal 與真值。不能改低門檻宣告通過。真實貨架繞行須確認 Nav2 handoff／返回直接跟隨，並另驗證平行分支的 Shrink+Wait recovery。

guide 與 teleop 的共用安全鏈路可另開下列模擬；在容器 shell 內使用模式請求及 teleop topic，禁止直接發布 `/cmd_vel`。按 Ctrl-C 停止 teleop 後，mux／watchdog 會逾時停車。

```bash
./scripts/dev.sh guide headless:=true fault_injection:=true
./scripts/dev.sh goto drinks --timeout 60
./scripts/dev.sh shell
# In the container, while the guide simulation is running:
ros2 topic pub --once /mode_manager/request std_msgs/msg/String '{data: teleop}'
ros2 topic pub -r 20 /cmd_vel/teleop geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

第三個容器 shell 可依序發布故障（先停止前一列指令再換下一列）；clear 恢復真實模擬雷射，檢查是否平滑恢復。`lidar_front` 的距離從雷射原點算起，且只改正前方 ±0.20 rad，因此測試前先確認車子要求向前。

```bash
ros2 topic pub --once /sim/fault std_msgs/msg/String '{data: "lidar_front:0.9"}'
ros2 topic pub --once /sim/fault std_msgs/msg/String '{data: "lidar_front:0.2"}'
ros2 topic pub --once /sim/fault std_msgs/msg/String '{data: lidar_freeze}'
ros2 topic pub --once /sim/fault std_msgs/msg/String '{data: clear}'
ros2 topic info /cmd_vel --verbose
ros2 topic echo /safety/obstacle_state
```

確認 `/cmd_vel` 唯一 publisher 為 cmd_vel_watchdog；另測停止 guard 程序／collision_monitor 程序、急停與明確 reset、原地旋轉與貼近側邊障礙。倒車純邏輯已有 pytest，但預設 mux 禁止倒車，不透過額外 `/cmd_vel` 發布者繞過限制。
