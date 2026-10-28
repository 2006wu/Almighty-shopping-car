# 操作手冊（自 README 移出）

回到 [README](../README.md)。

## 里程計來源與漂移比較

`sim`／`mapping`／`guide`／`follow` 預設 `odom_source:=ekf`，由 robot_localization 40 Hz 融合有噪聲的輪速與 IMU 偏航角速度，發布 `/odom` 及唯一的 `odom → base_footprint`。六軸 IMU 不提供絕對航向；位置、航向仍會漂移。Nav2 velocity_smoother 與 SLAM 仍使用 `/odom`／`odom`。此次新增 apt 相依套件，需先重建 compute 映像與 colcon 建置。

比較時每次停止整個模擬，再以相同初始姿態、速度命令及路線各跑一次：

```fish
./scripts/dev.sh mapping odom_source:=ekf
# Stop the previous simulation before starting the comparison
./scripts/dev.sh mapping odom_source:=ground_truth
```

`ground_truth` 保留舊版模擬真值 `/odom` 與 TF；EKF 模式不發布 `odom → sim_world`，避免宣稱估測車體與世界真值永遠一致。兩種模式都發布 `/sim/ground_truth_odom`（`sim_world → base_footprint_gt`），且不橋接真值 TF 或輪速 TF。RViz 預設 `odom`，導航切成 `map`；人物真值仍在 `/uwb/person_ground_truth`（`sim_world`），可記錄供評估，不能直接疊到 EKF 的 `odom` 畫面。

在另一個容器終端機觀察自動啟動的漂移監測器：

```bash
ros2 topic echo /sim/odom_drift
ros2 topic hz /odom
ros2 topic echo /wheel_odom_noisy --once
ros2 topic echo /imu/with_covariance --once
```

`/sim/odom_drift` 每 5 秒發布一次 Float32MultiArray，依序為平移誤差（m）、絕對航向誤差（rad）、累積真值路程（m）、平移誤差／路程（m/m）、航向誤差／路程（rad/m）。未移動時最後兩項為 NaN。監測器依時間插值真值，跨越 0.1 秒以上的資料缺口便略過，不外插；資料過期時不重播舊統計。請從出生點開始整段量測，至少比較直線、轉彎與返回起點；真值模式誤差應接近零（含插值殘差），EKF 應呈現非零漂移，預期比單靠輪速積分的偏航穩定，但尚無容器量測數字或保證門檻。種子 42 只固定輪速噪聲序列，不保證物理模擬完全重現。參數、covariance 與離線 `odom_metrics.py` 介面見[架構文件](design/ARCHITECTURE.md)。

## 模式管理與速度仲裁

`guide`／`follow` 預設 `use_mux:=true`，分別以 `initial_mode:=guide`／`initial_mode:=follow` 請求啟動模式，保留既有 `./scripts/dev.sh goto` 操作。等待 `/mode_manager/state` 顯示 `GUIDE` 後再送區域目標；切換期間顯示 `IDLE` 並持續輸出零速，會取消舊導航、停用跟隨及清除舊命令。若要每次啟動都先待機，傳入 `initial_mode:=idle`。

```bash
./scripts/dev.sh mode idle
./scripts/dev.sh mode guide
./scripts/dev.sh mode follow
./scripts/dev.sh mode teleop
./scripts/dev.sh estop
./scripts/dev.sh reset
```

`estop` 鎖定零速，`reset` 僅回到 `IDLE`，需再明確請求移動模式。也可發布 `std_msgs/Bool` 的 `true` 至 `/safety/stop`；`false` 不會解除急停。模式請求使用 `/mode_manager/request`（`std_msgs/String`，`idle|guide|follow|teleop|estop|reset`）；`/mode_manager/state` 與 `/cmd_vel_mux/active` 為保留最新值的 String。follow 程序只在 follow bringup 啟動；guide bringup 無法憑模式請求建立缺少的跟隨節點或人物。

鍵盤在仲裁流程中必須改送專用輸入，先請求 `teleop`，再於已載入 ROS 環境的終端機執行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel/teleop
```

Nav2 平滑器輸出 `/cmd_vel/nav`、直接跟隨輸出 `/cmd_vel/follow`；只有獨立程序 `cmd_vel_watchdog` 發布底盤 `/cmd_vel`。仲裁器以 20 Hz 發布內部 `/cmd_vel_mux/output`；nav／follow／teleop 的接收逾時分別為 0.5／0.3／0.5 秒，資料過期後下一次更新歸零。預設由 obstacle_guard 檢查完整雷射、速度相關掃掠足跡及過期資料，再將安全速度交給看門狗；選用 collision_monitor 也接在看門狗之前。看門狗以單調時鐘檢查安全鏈路輸出，超過 0.25 秒未收到命令，於下一個 50 毫秒更新發布零速，模擬暫停不會停止其計時。限速為 0.6 m/s、1.2 rad/s，一般加速度為 0.6 m/s²、1.2 rad/s²，零速安全命令立即生效，預設不倒車。

純邏輯與單獨執行 `cmd_vel_mux` 都預設 `IDLE`，程序重啟不自行恢復先前模式。launch 的 `initial_mode` 是明確的便利性例外：重新執行整個 guide／follow launch（或以相同參數重啟節點）會重新請求該模式；有重啟風險的驗收請使用 `initial_mode:=idle`。launch 未設定自動 respawn。若獨立看門狗已鎖定急停，重啟仲裁器不會解除，仍須 `reset`。

`use_mux:=false` 僅供引導／導航舊接線，恢復 Nav2 直接發布 `/cmd_vel`，不啟動模式管理與看門狗，必須維持舊有流程互斥。follow 已要求 `use_mux:=true`，以免繞過安全轉送與看門狗。mapping 沒有 Nav2，維持直接鍵盤操作；如需為建圖加上看門狗，可另啟動 `ros2 launch shopping_robot_bringup mode_mux.launch.py initial_mode:=teleop` 並將鍵盤改送 `/cmd_vel/teleop`。禁止同時保留直接 `/cmd_vel` 發布端。看門狗本身、橋接或 Gazebo 同時故障時，軟體無法保證停止；目前未新增 Fortress 的底盤逾時外掛，也沒有硬體安全功能。完整參數與容器驗收步驟見[架構文件](design/ARCHITECTURE.md)。
`guide`／`follow`／`mode_mux` 已停用 `use_mux:=false` 旁路。障礙防護預設 `obstacle_guard:=true`，Nav2 collision_monitor 預設 `collision_monitor:=false`；幾何、故障注入與無視窗驗收命令見 [障礙安全鏈路](design/OBSTACLE_SAFETY.md)。mapping 沒有 Nav2，維持直接鍵盤操作；如需為建圖加上看門狗，可另啟動 `ros2 launch shopping_robot_bringup mode_mux.launch.py initial_mode:=teleop` 並將鍵盤改送 `/cmd_vel/teleop`。禁止同時保留直接 `/cmd_vel` 發布端。看門狗本身、橋接或 Gazebo 同時故障時，軟體無法保證停止；目前未新增 Fortress 的底盤逾時外掛，也沒有硬體安全功能。完整參數與容器驗收步驟見[架構文件](design/ARCHITECTURE.md)。

## 受限網路的 ROS 套件鏡像

只有無法連線 `packages.ros.org` 的網路才需要設定。`ROS_APT_MIRROR` 填入可用鏡像的完整 ROS 2 Ubuntu 套件庫網址（結尾通常為 `/ros2/ubuntu`），fish 範例：

```fish
set -gx ROS_APT_MIRROR https://your-mirror.example/ros2/ubuntu
./scripts/dev.sh build
```

請將範例網址換成實際鏡像。兩個 Dockerfile 都在 `apt-get update` 前處理 deb822 `ros2.sources` 與舊版 `ros2-latest.list`；空值沿用官方來源，保留原本的套件簽章設定。此設定不代理 Docker Hub 或 Ubuntu 套件庫。

## 操作與連接埠

`./scripts/dev.sh help` 列出所有指令。`mapping`／`guide` 分別保留 `slam`／`nav` 別名。`colcon-build`、`colcon-test` 與 `test` 使用暫時運算容器，因此可以先建置、測試，再執行 `up`。`sim` 僅啟動模擬；`mapping/guide/follow` 各自包含模擬，請勿同時執行。模擬指令維持前景執行，Ctrl-C 停止該次工作。`down` 停止容器但保留建置磁碟區。程式修改後重新執行 `colcon-build`；使用 `colcon-test` 後會顯示測試摘要。

| 用途 | 主機位置 | 說明 |
| --- | --- | --- |
| noVNC | `127.0.0.1:6080` | 瀏覽器桌面；被占用時設定 `NOVNC_HOST_PORT` |
| VNC | `127.0.0.1:5910` | 對應容器 5900；可用 `VNC_HOST_PORT` 修改 |
| ROS DDS | 共用網路命名空間 | 顯示容器使用 `network_mode: service:compute`，UDPv4；`ROS_DOMAIN_ID` 預設 28 |

VNC 密碼讀取環境變數，未設定時為 `shopping`；主機連接埠僅綁定迴送介面。兩個容器共享安裝產物，使用 Fast DDS UDPv4，停用共享記憶體傳輸。顯示容器共用運算容器的網路命名空間（連接埠因此發布在運算容器上），避免 Docker 橋接網路上的多播探索失敗；運算容器的 Xvfb 使用顯示編號 50 起跳，避免與顯示容器的 `:99` 衝突。RViz 與 Gazebo 感測器採軟體繪圖；`sim/mapping/guide/follow` 會為運算容器開啟 Xvfb。`gz-gui` 可從顯示容器啟動 Gazebo GUI，用於觀察既有模擬；容器間的 Gazebo 探索仍需實機環境驗證。`AUTOSTART_RVIZ=1` 需先完成建置再啟動顯示容器。

Linux 的操作與 Mac 相同（bash 或 fish 皆可）；若需 sudo，可執行 `env DOCKER='sudo docker' ./scripts/dev.sh up`。自訂世界須保留 `supermarket` 世界名稱、`person` 模型及對應橋接主題；目前 world 參數提供替換檔案路徑，尚未自動推導世界名稱。

### RViz 與 Gazebo 顯示配置

`./scripts/dev.sh rviz` 預設以 map 為固定框架，俯視整間商場；深色背景搭配橘色雷射、青色全域路徑與薄荷綠車體足跡。Views 面板可切換已存的商場全景與車體近景。`Optional diagnostics` 內可手動開啟 TF、Fast／Slow TEB 軌跡與區域成本圖；TEB 主題只在 fork 發布，僅 apt 時保留關閉。只有 sim、尚未發布 map→odom 時，可在 Global Options 將 Fixed Frame 改為 odom。

SLAM 專用的 `slam.rviz` 採相同配色與全景尺度，姿態圖預設關閉，需要時勾選 `SLAM Pose Graph`。Gazebo 保留原有碰撞盒、模型姿態與感測器，統一地板、貨架、地標與人物材質；`./scripts/dev.sh gz-gui` 以專案的 Fortress GUI 配置啟動深色介面與商場斜俯視鏡頭，連接目前既有模擬。需要自訂配置時可設定 `GAZEBO_GUI_CONFIG`，路徑須能在顯示容器內讀取。顯示配置不等於功能驗收，實測限制仍以進度紀錄為準。

## 模擬故障注入與停車驗證

重新 colcon 建置後，停止其他移動流程，只啟動一個跟隨模擬：

```bash
./scripts/dev.sh follow fault_injection:=true
# In another terminal, against the same running simulation
./scripts/dev.sh fault-check
```

`fault_injection` 預設為 `false`；一般 sim／mapping／guide／follow 的 topic 路徑維持原狀。啟用後，模擬套件的 `fault_injector` 透過 `/sim/raw/*` 中繼 UWB、雷射與里程計，獨立的 `/sim/ground_truth_odom` 不受影響。只在 follow 啟用里程計新鮮度保護；guide 的故障停止機制不在本次驗收範圍。可先用 `route_around:=false` 驗證直接跟隨，再以預設 `route_around:=true` 重測 Nav2 交接；測試時不得另外送導航目標或鍵盤速度。

也可在已載入 ROS 環境的容器終端機手動注入：

```bash
ros2 topic pub --once /sim/fault std_msgs/msg/String '{data: "uwb_stale"}'
ros2 topic echo /sim/fault_status --qos-durability transient_local
# In another terminal
ros2 topic pub --once /sim/fault std_msgs/msg/String '{data: "clear"}'
```

支援 `uwb_stale`、`uwb_dropout:3.0`（秒）、`uwb_noise:0.5`（測距高斯標準差，公尺）、`lidar_drop`、`lidar_freeze`、`odom_freeze`、`clear`。凍結保留舊時間戳；UWB 原始測距沒有時間戳，因此 stale 在估測目標上注入。除 dropout 外，手動故障持續到 clear；不同故障可以疊加。狀態為目前故障名稱的 JSON 陣列，使用 transient_local 保留最後一筆。

排程使用 YAML 字串參數清單，`time` 是收到第一筆非零模擬時間後的相對秒數，`duration: 0` 表示持續到清除：

```bash
./scripts/dev.sh follow fault_injection:=true 'fault_schedule:=[{time: 20.0, fault: lidar_drop, duration: 3.0}, {time: 40.0, fault: "uwb_noise:0.5", duration: 5.0}]'
```

`fault-check` 請使用空排程。工具依序測六種故障，每次保持約 4 秒模擬時間、清除並等待恢復，輸出 PASS／FAIL 表與非零失敗結束碼。使用 `python3 scripts/eval/fault_check.py --help` 可查看等待選項；門檻、預期反應及限制見[架構文件](design/ARCHITECTURE.md#模擬故障注入與驗收)。目前只有純 Python／靜態測試，尚未取得 Gazebo 故障量測結果。

## 安全與驗證狀態

所有預設流程僅使用模擬，沒有硬體驅動、裝置掛載或真實車體連線。跟隨節點單獨執行時預設停用，由 follow launch 的模式管理器啟用；可用 `/follow/enable` 的 `std_srvs/SetBool` 服務切換。目標太近、遺失或資料過期時立即輸出零 Twist；雷射未收到、前方無有效資料或超時也完全停止。直接跟隨時，前方有效障礙讀值不超過 0.65 公尺，`OBSTACLE_STOP` 立即將線速度固定為零。`obstacle_stop_rotate` 預設啟用；僅在未啟用繞行或 Nav2 不可用、目標新鮮、雷射有效且 TF 與完整 360° 掃描確認至少 0.45 公尺旋轉圓盤淨空時，允許依原角速度／角加速度上限朝人物轉向；條件不符或參數關閉時輸出完整零 Twist。過近停止須人物距離持續大於 0.75 公尺達 0.3 秒才解除；前方清空後，若方位誤差仍達 45°，會先依原加速度限制減速並原地對準，直到誤差不超過 0.25 rad 才恢復平移。啟用繞行時仍可交由 Nav2 規劃，輸出經障礙安全層與 watchdog。急停優先於一般加速度限制；一般追蹤限制線速度、角速度及加速度，預設不倒車。

跟隨 launch 同時啟動 `uwb_eval_node`，有時間相近的 `/sim/ground_truth_odom` 與 `/uwb/person_ground_truth` 時，發布 `/uwb/estimate_error`（Float32，公尺）。它用車體真值姿態將 base_link 估測值轉到 sim_world 與人物真值比較，不依賴估測 TF，因此不混入 EKF 漂移；結果仍包含取樣時間差。三筆時間戳最大差預設為 0.15 秒，不插值，缺資料或過期便略過。離線可用不依賴 ROS 的 `follow_metrics.py`，傳入依時間遞增的 `FollowSample` 序列，以 `evaluate_series` 計算估測平均誤差、RMSE、最大誤差及真實跟隨距離的平均／最小／最大值；統計按樣本加權。

跟隨模式可將持續受阻的有效目標交給 Nav2 繞行。一般 LOS 走廊半寬為 0.25 公尺；連續受阻 0.3 秒才進入 `BLOCKED_WAIT`，此前維持 `FOLLOWING` 並減速。已判定受阻後，須以加寬至 0.35 公尺的走廊連續暢通 0.6 秒才能解除，避免貨架末端的雷射擦邊反覆停車。人物距離超過 1.3 公尺且判定受阻再持續 0.5 秒後交接；前方障礙停止仍立即生效，不等待 LOS 計時。`/follow/state` 發布 `STATE: 原因`，`/follow/handoff_goal` 為人物 map 座標的 PoseStamped，可加入 RViz。

NAV2 至少保持 1.5 秒；之後須 LOS 遲滯確認暢通、人物距離不超過 1.3 公尺、前方淨空大於 0.8 公尺，並持續 0.5 秒才恢復 DIRECT。人物已在 1.0 公尺跟隨距離內時，可略過額外 0.5 秒，仍須 LOS 確認與最短保持時間。導航成功／失敗／取消或安全事件可立即結束保持時間。恢復 DIRECT 後至少保持 1 秒才再次交接；若 LOS 已連續判定受阻達 1 秒，可提前交接，但仍須滿足受阻交接延遲。導航中停止直接追逐，由跟隨節點轉送經安全檢查的 Nav2 命令；遺失、過近、停用或雷射失效時立即取消並輸出零 Twist。目標更新直接使用 Nav2 搶占；切回時等待動作終止，且外部速度命令安靜 0.3 秒或最新命令近零，終止後最多等 2 秒，第一筆直接命令為零。apt 與 fork 的 Smac2D 共用容差 0.8 公尺，目標未偏移；引導模式受阻的區域目標也可能停在較遠處，不能只依成功狀態認定到達原始座標。

新增的 `uwb_follow.yaml`／節點參數如下；時間單位為秒、距離為公尺、角度為弧度。LOS 與保持時間及額外寬度接受有限非負值；減速比例限於 0～1，側向角度須滿足 `0 <= min < max <= pi`，側向距離須滿足 `0 <= slow < free`。

| 參數 | 預設值 | 用途 |
| --- | --- | --- |
| `los_block_time`／`los_clear_time` | 0.3／0.6 | 受阻／暢通連續確認時間；時間倒退重設 |
| `los_clear_extra_width` | 0.10 | 已受阻時恢復所需的額外走廊半寬 |
| `los_uncertain_speed_factor` | 0.6 | 原始 LOS 受阻但尚未確認時的線速度倍率 |
| `min_nav_hold`／`min_direct_hold` | 1.5／1.0 | 導航／直接模式最短保持時間 |
| `side_sector_min`／`side_sector_max` | 0.523599／1.745329 | 左右各 30°～100° 的側向扇區 |
| `side_slow_clearance`／`side_free_clearance` | 0.35／0.75 | 最小側向淨空的減速／全速門檻 |
| `side_min_speed_factor` | 0.4 | 淨空不超過減速門檻時的最低線速度倍率 |

側向淨空取雷射回波的最小 `range * abs(sin(angle))`；兩門檻間線性插值，缺少側向涵蓋時採最低倍率。側向與 LOS 倍率相乘，套用在線速度及其上限，仍受既有加速度限制。直接上限 0.5 m/s 已高於人物 0.35 m/s，且等於 Nav2 velocity_smoother 上限，因此本次不提高。比例控制 `kp_distance=0.7` 在無減速、正前方等速追蹤時本來就有約 0.5 公尺的穩態距離誤差；不能把 1.0 公尺設定解讀為動態距離保證。遲滯及側向減速可能延後恢復或追近，須比較下一輪 240 秒模擬的真實距離及切換率，尚未驗證改善幅度。

可直接分析 `ros2 topic echo /follow/state` 文字紀錄，不需 rosbag：

```bash
python3 scripts/eval/follow_state_stats.py follow-state.log
python3 scripts/eval/follow_state_stats.py follow-state.log --sample-period 0.05
```

工具解析 `data: STATE: 原因` 或 `state: STATE` 紀錄，可在同一行前綴 `[秒數]` 或 `秒數 `。有時間戳時須有限且嚴格遞增；無時間戳時依序假設每筆 0.05 秒（20 Hz），遺漏樣本或僅記錄狀態變更時須提供時間戳，否則每分鐘切換率與停留時間僅為估算。混用有／無時間戳會拒絕分析。輸出切換總數、每分鐘切換率與各狀態停留次數／總計／平均／最小／最大秒數；首尾停留區間只計算觀測範圍，最後一筆不外推。亦可呼叫 ROS-free 的 `follow_metrics.state_statistics([(t, state), ...])`。

guide／follow 預設啟用全域模式管理、速度仲裁與獨立輸出看門狗，操作方式見下節；一次只啟動一個 bringup，跟隨期間仍不可另外送出 Nav2 目標，以免干擾跟隨自行管理的導航動作。UWB 訊息目前用無時間戳的 Float32MultiArray，模擬本機接收時間視為量測時間；硬體介面前必須改用有時間戳及錨點識別的訊息。

純 Python 測試可在容器執行（`./scripts/dev.sh test`），也可在任何已安裝 pytest、pyyaml、xacro 的 Python 3.10 環境執行：

```bash
python3 -m pytest -q
./scripts/run_unit_tests.sh python3
python3 scripts/checks/check_static.py
```

此次 EKF、噪聲與漂移監測尚未執行映像重建、colcon 或 Gazebo 驗證。先前真值里程計版本於 2026 年 10 月已在 Linux amd64 環境以 Docker 實際驗證：映像建置、七套件 colcon 建置與測試、建圖、RViz（經 noVNC）、六個區域導航與跟隨流程；Mac arm64 建置、Gazebo GUI 與真實硬體仍未驗證，詳見 [進度紀錄](PROGRESS.md)。指定環境的 xacro 套件若不支援 `python -m xacro`，可改用 `python -c 'import xacro; xacro.main()' 模型路徑`，兩者呼叫相同的 xacro 處理程式。

[模擬評估指標與離線報表](design/METRICS.md) · [UWB 錨點離線研究](research/UWB_STUDY.md) · [架構](design/ARCHITECTURE.md) · [時程](ROADMAP.md) · [協作規範](../AGENTS.md)

本次合併保留 develop 的 Smac2D、YAML 數值與人物模型服務。一般與加寬 LOS 的檢查長度在人物距離大於跟隨距離時改為 `distance - max(los_margin, follow_distance)`；近距離仍檢查到 `distance - los_margin`。`route_around:=false` 也建立雷射到車體的 TF listener，旋轉判定不需要 map 或真值。下列舊分支設計僅保留歷史脈絡，與本段衝突時以本段及目前設定為準。

目前 Nav2 在 LOS 經既有遲滯確認且人物距離不超過 `follow_distance=1.0` 公尺時，至少持有 1.5 秒即可返回 DIRECT，由直接跟隨在跟隨距離停止／保持；導航成功時可立即返回，不受側向淨空與朝向門檻限制。人物仍超過跟隨距離時，正常返回 DIRECT 須同時滿足：至少持有 1.5 秒、人物距離不超過 1.3 公尺、LOS 經既有遲滯確認、前方淨空大於 0.8 公尺、側向淨空達既有 `side_free_clearance=0.75` 公尺、目標朝向誤差不超過既有 `rotate_to_target_release_angle=0.25` rad，且返回條件持續 0.5 秒。側向淨空使用目前雷射的左右 30°～100° 扇區最小側向距離；此返回門檻讓 Nav2 先帶車體離開貼架位置，保留原有下游 guard 的掃掠防護。人物仍超過跟隨距離時，導航成功也須滿足相同條件；人物移動時，目標仍依原有 0.4 公尺／0.5 秒門檻搶占更新。

雷射／TF 失效仍立即停止並取消 Nav2，但保留尚未完成的繞行意圖。資料恢復後，若側向或朝向尚未適合直接跟隨，DIRECT 以 `BLOCKED_WAIT` 保持零命令，人物距離超過 1.3 公尺且滿足原有交接延遲、失敗冷卻後，即使 LOS 已清楚也重新交給 Nav2；取消、動作終止與速度排空完成前不得轉送新導航命令。目標遺失、過近鎖定、停用或時間倒退會清除恢復意圖；`0bdb4c1` 的跨 DIRECT／Nav2 近停鎖定持續優先。所有控制僅使用 UWB 估測、雷射與估測 TF。此修補尚待 `follow_turns`、`follow_stop_go` 與跟隨／障礙 `fault-check` 模擬驗收，詳見[進度紀錄](PROGRESS.md)。

<details>
<summary>合併前 follow-los 分支設計紀錄</summary>

所有預設流程僅使用模擬，沒有硬體驅動、裝置掛載或真實車體連線。跟隨節點單獨執行時預設停用，由 follow launch 的模式管理器啟用；follow 僅接受 `use_mux:=true`；可用 `/follow/enable` 的 `std_srvs/SetBool` 服務切換。目標太近、遺失或資料過期時立即輸出零 Twist；雷射未收到、前方無有效資料或超時也完全停止。前方有效障礙讀值不超過 0.65 公尺時，直接控制器的 `OBSTACLE_STOP` 立即輸出完整零 Twist，包含角速度。交接至 Nav2 後仍禁止前進，但完整雷射與 TF 確認車體旋轉淨空時，可轉送 Nav2 的限速角速度，讓車體轉離貨架；淨空不足或未知仍雙軸停止。急停優先於一般加速度限制；一般追蹤限制線速度、角速度及加速度，預設不倒車。
所有預設流程僅使用模擬，沒有硬體驅動、裝置掛載或真實車體連線。跟隨節點單獨執行時預設停用，由 follow launch 的模式管理器啟用；可用 `/follow/enable` 的 `std_srvs/SetBool` 服務切換。目標太近、遺失或資料過期時立即輸出零 Twist；雷射未收到、前方無有效資料或超時也完全停止。前方有效障礙讀值不超過 0.65 公尺時，`OBSTACLE_STOP` 立即將線速度歸零；目標有效且距離大於 0.6 公尺時，允許原地轉向目標，角速度及角加速度仍受限。急停優先於一般加速度限制；一般追蹤限制線速度、角速度及加速度，預設不倒車。

跟隨 launch 同時啟動 `uwb_eval_node`，有時間相近的 `/sim/ground_truth_odom` 與 `/uwb/person_ground_truth` 時，發布 `/uwb/estimate_error`（Float32，公尺）。它用車體真值姿態將 base_link 估測值轉到 sim_world 與人物真值比較，不依賴估測 TF，因此不混入 EKF 漂移；結果仍包含取樣時間差。三筆時間戳最大差預設為 0.15 秒，不插值，缺資料或過期便略過。離線可用不依賴 ROS 的 `follow_metrics.py`，傳入依時間遞增的 `FollowSample` 序列，以 `evaluate_series` 計算估測平均誤差、RMSE、最大誤差及真實跟隨距離的平均／最小／最大值；統計按樣本加權。

跟隨模式可將持續受阻的有效目標交給 Nav2 繞行。一般 LOS 走廊半寬為 0.4 公尺；連續受阻 0.3 秒才進入 `BLOCKED_WAIT`，此前維持 `FOLLOWING` 並減速。已判定受阻後，須以加寬至 0.5 公尺的走廊連續暢通 0.6 秒才能解除，避免貨架末端的雷射擦邊反覆停車。人物距離超過 1.3 公尺且判定受阻再持續 0.5 秒後交接；前方障礙停止仍立即生效，不等待 LOS 計時。`/follow/state` 發布 `STATE: 原因`，`/follow/handoff_goal` 為人物前方保留 1.0 公尺距離的 map 終點 PoseStamped，可加入 RViz。

NAV2 至少保持 1.5 秒；人物距離不超過 1.3 公尺、視線角度不超過 0.25 rad、LOS 經較寬走廊確認、前方淨空大於 0.8 公尺，且返回條件持續 0.5 秒後才恢復 DIRECT。導航成功本身不切換模式；人物繼續移動時，成功的目標也依更新週期重新送出。移動目標以 Nav2 搶占更新，等待接受回覆時合併新位置，不取消／排空；只有離開 NAV2 才要求取消。切回後等待動作終止與速度排空，第一筆直接命令為零。目標遺失、過近、停用或雷射失效立即雙軸停止並取消；導航失敗另有 3 秒重試冷卻。

NAV2 速度由 `/cmd_vel/nav` 輸入跟隨節點，檢查命令年齡不超過 0.2 秒、安全狀態及速度／加速度上限後，以 `/cmd_vel/follow` 轉送。FOLLOW 仲裁僅接受 follow 與安全停止，原始 nav 不再作為逾時備援，避免安全節點中斷後繞過檢查；GUIDE 仍接受 nav。前方停止值 0.65 公尺從前移 0.18 公尺的雷射原點量測，保留原值。旋轉須完整 360° 掃描確認 base_link 周圍半徑至少 0.45 公尺圓盤無障礙，涵蓋 0.60 × 0.50 公尺車體外接圓與餘裕；偏移雷射、後方與側方回波都納入。這只允許 Nav2 規劃／控制的轉向，不自行產生盲轉或倒車。NavFn 共用容差仍為 0.8 公尺，須核對實際終點。

人物路徑恢復 0.35 m/s 瀏覽速度。直接控制與 Nav2 巡航上限均為 0.5 m/s；窄處最低 0.2 m/s 與 Nav2 曲率限速最低 0.25 m/s 仍可能造成落後，未藉提高安全限速保證追蹤通過。完整 180 秒評估仍待維護者重跑。

新增的 `uwb_follow.yaml`／節點參數如下；時間單位為秒、距離為公尺、角度為弧度。LOS 與保持時間及額外寬度接受有限非負值；減速比例限於 0～1，側向角度須滿足 `0 <= min < max <= pi`，側向距離須滿足 `0 <= slow < free`。

| 參數 | 預設值 | 用途 |
| --- | --- | --- |
| `los_block_time`／`los_clear_time` | 0.3／0.6 | 受阻／暢通連續確認時間；時間倒退重設 |
| `los_clear_extra_width` | 0.10 | 已受阻時恢復所需的額外走廊半寬 |
| `los_uncertain_speed_factor` | 0.6 | 原始 LOS 受阻但尚未確認時的線速度倍率 |
| `min_nav_hold`／`min_direct_hold` | 1.5／1.0 | 導航／直接模式最短保持時間 |
| `side_sector_min`／`side_sector_max` | 0.523599／1.745329 | 左右各 30°～100° 的側向扇區 |
| `side_slow_clearance`／`side_free_clearance` | 0.35／0.75 | 最小側向淨空的減速／全速門檻 |
| `side_min_speed_factor` | 0.4 | 淨空不超過減速門檻時的最低線速度倍率 |
| `velocity_feedforward` | 1.0 | 目標絕對速度在視線方向上的前饋增益；0 停用 |
| `distance_hysteresis_enter`／`distance_hysteresis_exit` | 0.04／0.08 | 距離誤差進入／離開保持區的門檻；保持時只抑制距離修正，仍保留前饋 |

側向淨空取雷射回波的最小 `range * abs(sin(angle))`；兩門檻間線性插值，缺少側向涵蓋時採最低倍率。側向與 LOS 倍率取較小值，用來限制線速度上限，再套用朝向餘弦閘控與加速度限制；不再將未飽和的距離修正與前饋一併縮小。實體減速上限不變，直接控制仍為 0.5 m/s、1.0 rad/s，仲裁／看門狗管線與唯一 `/cmd_vel` 發布者維持原狀。

目標速度由 `/uwb/target` 估測位置搭配 `/odom` 位姿轉入 odom 後差分，經 EMA（alpha=0.2）、0.6 m/s 向量限幅，再轉回車體座標，避免把車體移動誤認成人物移動。里程計與目標來源時間差或控制時里程計年齡超過 0.15 s 時不使用前饋；資料缺口超過 0.5 s 或時間倒退會丟棄差分歷史。一般模式會退回距離控制；`monitor_odom` 的故障停止規則維持原狀。此估速使用最新里程計，沒有補間；EKF 雜訊、漂移與兩來源時間差仍須容器驗證。距離保持遲滯須滿足 `0 <= enter <= exit`。

純 Python 閉迴路診斷可在儲存庫根目錄執行：

```bash
/workspace/scratch/venv-shop/bin/python scripts/eval/follow_loop_sim.py
/workspace/scratch/venv-shop/bin/python scripts/eval/follow_loop_sim.py --world
/workspace/scratch/venv-shop/bin/python scripts/eval/follow_loop_sim.py --speed-factor 0.4
```

工具直接讀取唯一 `person_path.yaml` 與控制 YAML，包含整個循環及啟動暫態；使用現有 TargetFilter、控制器、速度仲裁、加速度限制與看門狗的純邏輯。輸出保留舊版公式、修正版、移除前饋與移除控制器加速度限制的對照。預設無障礙情境的平均距離由 1.397 m 改為 1.003 m；`--world` 以 SDF 碰撞盒及人物圓柱近似雷射，舊版全程平均距離為 2.766 m，修正版為 1.807 m；修正版排除 LOS 鎖定與安全停止後，平均距離誤差 0.171 m、RMSE 0.198 m，有效樣本 69.3%。兩者都不是 Gazebo 驗收。模型不執行 Nav2，也不模擬車體動力學或 UWB 雜訊；全程統計與排除計數必須一起閱讀，詳見 [指標說明](design/METRICS.md)。

目前人物速度固定為 0.35 m/s，路徑與速度均未修改。比例控制原本在直線等速情境需要約 `0.35/0.7=0.5 m` 落後才能維持前進；轉角朝向閘控、側向減速與 LOS 等待會進一步放大落後。當減速倍率長期為 0.4，上限只有 0.2 m/s，前饋也無法追上 0.35 m/s 的人物；該壓力情境刻意保留失敗。預設走道中央淨空約 0.8 m，可使用完整速度，因此未全面放慢人物。貨架末端仍須驗證既有 Nav2 交接，不能由此模型推論已解決繞行。人物本身也是雷射障礙：正前方距離約 `0.18+0.25+0.65=1.08 m` 時就可能觸發停止，因此目前幾何與安全門檻下不能保證持續精確維持 1.0 m；距離遲滯不延後安全停止。

人物模型沿用 Fortress `/world/supermarket/set_pose`，每 0.05 s 嘗試更新，但同時最多一個 `ign service` 子程序，未完成便略過。原流程在發出請求前取樣當下路徑，之後才啟動子程序、等待服務套用，模型再保持位置直到下次完成；這會累積命令與保持延遲，且不是 ROS bridge 傳送移動命令。新增 `/person_path` 節點參數 `model_lag_compensation_s=0.30`，只將模型命令取樣改為 `t+補償`；UWB 真值與同步檢查仍使用 t，不修改觀測時間戳。服務完成觀察耗時以模擬秒寫入 DEBUG 紀錄，但不能分離程序啟動與實際套用時間。0.30 s 依 2026-10-24 實測總延遲校準；不同模擬速度／主機負載可能需要重校，必須以 `person-sync-check` 確認平均誤差 <0.05 m、延遲絕對值 <0.1 s。可用 `ros2 param set /person_path model_lag_compensation_s 0.0` 關閉，或設為新的非負補償值；設定不代表已通過驗收。

可直接分析 `ros2 topic echo /follow/state` 文字紀錄，不需 rosbag：

```bash
python3 scripts/eval/follow_state_stats.py follow-state.log
python3 scripts/eval/follow_state_stats.py follow-state.log --sample-period 0.05
```

工具解析 `data: STATE: 原因` 或 `state: STATE` 紀錄，可在同一行前綴 `[秒數]` 或 `秒數 `。有時間戳時須有限且嚴格遞增；無時間戳時依序假設每筆 0.05 秒（20 Hz），遺漏樣本或僅記錄狀態變更時須提供時間戳，否則每分鐘切換率與停留時間僅為估算。混用有／無時間戳會拒絕分析。輸出切換總數、每分鐘切換率與各狀態停留次數／總計／平均／最小／最大秒數；首尾停留區間只計算觀測範圍，最後一筆不外推。亦可呼叫 ROS-free 的 `follow_metrics.state_statistics([(t, state), ...])`。

guide／follow 預設啟用全域模式管理、速度仲裁與獨立輸出看門狗，操作方式見下節；一次只啟動一個 bringup，跟隨期間仍不可另外送出 Nav2 目標，以免干擾跟隨自行管理的導航動作。UWB 訊息目前用無時間戳的 Float32MultiArray，模擬本機接收時間視為量測時間；硬體介面前必須改用有時間戳及錨點識別的訊息。

純 Python 測試可在容器執行（`./scripts/dev.sh test`），也可在任何已安裝 pytest、pyyaml、xacro 的 Python 3.10 環境執行：

```bash
python3 -m pytest -q
./scripts/run_unit_tests.sh python3
python3 scripts/checks/check_static.py
```

此次 EKF、噪聲與漂移監測尚未執行映像重建、colcon 或 Gazebo 驗證。先前真值里程計版本於 2026 年 10 月已在 Linux amd64 環境以 Docker 實際驗證：映像建置、七套件 colcon 建置與測試、建圖、RViz（經 noVNC）、六個區域導航與跟隨流程；Mac arm64 建置、Gazebo GUI 與真實硬體仍未驗證，詳見 [進度紀錄](PROGRESS.md)。指定環境的 xacro 套件若不支援 `python -m xacro`，可改用 `python -c 'import xacro; xacro.main()' 模型路徑`，兩者呼叫相同的 xacro 處理程式。

[模擬評估指標與離線報表](design/METRICS.md) · [UWB 錨點離線研究](research/UWB_STUDY.md) · [架構](design/ARCHITECTURE.md) · [時程](ROADMAP.md) · [協作規範](../AGENTS.md)

</details>

## rosbag 記錄與離線報告（僅模擬）

先在既有 compute 容器執行 `./scripts/dev.sh colcon-build`，重新啟動 launch。在另一個終端機錄製；固定清單位於 `shopping_zone_manager/config/record_topics.yaml`，預設不錄 `/scan`、TF 或 action 服務。輸出 `bags/NAME/` 由 `/repo` 掛載回主機，已加入 Git 忽略清單，不會覆寫既有 bag。

```bash
# 終端機 A：全新模擬，預設起點與 map／世界原點一致
./scripts/dev.sh guide
# 終端機 B：等待導航就緒後，先開始記錄
./scripts/dev.sh record guide1 --duration 120
# 終端機 C：確認 B 顯示正在錄製，再依序執行；每次等待結果
./scripts/dev.sh goto drinks --timeout 30
./scripts/dev.sh goto snacks --timeout 30
./scripts/dev.sh goto entrance --timeout 30
# B 正常結束並產生 metadata.yaml 後
./scripts/dev.sh eval-bag guide1
```

先停止 guide，再以全新模擬執行 follow；兩種 launch 不可同時控制底盤：

```bash
./scripts/dev.sh follow
# 另一個終端機，確認 follow 與人物真值已開始發布
./scripts/dev.sh record follow1 --duration 120
# 錄製完成後
./scripts/dev.sh eval-bag follow1 --target 1.0 --band 0.2
```

`--duration` 為正值的牆鐘秒數，省略時以 Ctrl-C 正常結束；暫停模擬仍會消耗此時限。可在尾端加入額外 topic，例如 `./scripts/dev.sh record diagnostic --duration 30 /scan`。必須等待 recorder 完成寫檔，不能以強制終止取代。記錄使用 Humble sqlite3 與 `--use-sim-time`，啟動 `/clock` 前不會記錄資料。

`eval-bag` 依序產生 `observations.jsonl`、`observations.jsonl.meta.json`、`report.json` 與 `report.md`；重跑會覆寫這四個檔案。轉換中繼資料揭露缺少人物真值、同時間合併樣本與截止回合，須和報告一起檢視。`eval-bag` 使用目前固定場景的 map→世界 `(0,0,0)` 對齊；自訂地圖原點須改用 `bag_to_eval --map-world-transform X Y YAW`，不能直接套用。

完整時間規則、已支援與無法還原的觀測、固定情境及兩次報告比較方式見 [評估指標](design/METRICS.md#rosbag-記錄轉換與可重現流程)。沙盒僅驗證純 Python 與靜態配置，ROS／Gazebo 實際錄製回放須由維護者在容器驗證。

### 人物同步與跟隨評估（純模擬）

重新建置後，只啟動一個 follow 模擬；兩個檢查工具都是唯讀訂閱，不會發布速度、改人物路徑或注入故障。以下各指令在不同終端機執行，檢查工具須在模擬時間 120 秒前開始，才能觀察排程的 UWB 缺口：

```bash
./scripts/dev.sh colcon-build
./scripts/dev.sh follow fault_injection:=true 'fault_schedule:=[{time: 120.0, fault: uwb_stale, duration: 3.0}]'
./scripts/dev.sh person-sync-check --duration 60 --output /tmp/person-sync.json
./scripts/dev.sh follow-eval --duration 180 --output /tmp/follow-eval.json
```

JSON 路徑位於 compute 容器內；可改成 `/repo/` 下可寫路徑以保留於主機。工具各自等待 `/clock`、略過開始後 2 秒，再量測指定的模擬秒數；`--wall-timeout` 預設為 `4*duration+30` 秒，避免模擬暫停時無限等待。直接在已 source 的容器執行也可以：

```bash
ros2 run shopping_uwb_follow person_sync_check --duration 60 --output /tmp/person-sync.json --ros-args -p use_sim_time:=true
ros2 run shopping_uwb_follow follow_eval --duration 180 --output /tmp/follow-eval.json --ros-args -p use_sim_time:=true
```

`person_sync_check` 比較 `/uwb/person_ground_truth`、`/simulation/person_pose` 的人物幾何位置，以及共用 `person_path.yaml` 在各筆來源模擬時間的位置；輸出平面偏差的平均、RMSE、最大值及延遲搜尋結果。每組平均偏差須小於 0.05 m、延遲絕對值須小於 0.1 s，可用 `--mean-limit`、`--lag-limit` 更改驗收門檻。人物模型父框架必須為 `supermarket`、子框架為 `person`，對應 SDF 世界原點；人物真值必須為 `sim_world`。這是同一個模擬世界座標，不拿 EKF 的 `odom` 當真值。

既有 `/simulation/poses` 保留原用途。Fortress [SceneBroadcaster](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/scene_broadcaster/SceneBroadcaster.cc) 的時間戳在外層 Pose_V，而 Humble [TFMessage 轉換](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/src/convert/tf2_msgs.cpp) 讀取各筆 pose，無法保留該外層時間。此次增加人物唯讀 [PosePublisher](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc)，產生各筆時間與框架；只有 follow launch 將 `/model/person/pose` 單向橋接至 `/simulation/person_pose`（TFMessage），不接入 `/tf`。PosePublisher 僅補足量測來源時間；人物命令延遲補償見跟隨控制說明。SDF 的觀察外掛在其他模擬也會載入，但沒有新增 ROS 橋接。

`follow_eval` 使用人物真值與 `/sim/ground_truth_odom`（`sim_world → base_footprint_gt`），將真實相對位置轉到車體平面座標，再和 `/uwb/target`（`base_link`）比較。平移原點相同的假設來自目前 URDF 的固定關節。表格包含下列指標，門檻是初始驗收提案，並非控制器調參：

| 項目 | 定義與預設門檻 |
| --- | --- |
| 濾波延遲 | 相對位置向量最佳時間位移，絕對值 <0.3 s；`--latency-limit` 可改 |
| 直線／轉彎追蹤 | 共用人物路徑在前後各 0.5 s 的航向變化率 >0.3 rad/s 視為轉彎；各類至少 10 筆真值，真實距離減 `follow_distance` 的 RMSE <1.5 m（`--tracking-limit`）；另列距離及估測位置誤差統計 |
| 近距停止 | 真實距離嚴格小於 YAML 的 `stop_distance`；0.5 s（`--stop-deadline`）後平面速度須 ≤0.03 m/s、角速度絕對值須 ≤0.05 rad/s，並維持到離開近距區間；列出所有違規時間 |
| 重新取得 | 估測來源時間缺口超過 YAML 的 `lost_timeout` 或狀態進入 LOST，從下一筆新鮮估測到 `/follow/state` 回到 FOLLOWING 須 ≤2 s（`--reacquire-limit`）；另列遺失至恢復的總時間，NAV2_HANDOFF 不當作 FOLLOWING；schema 2 另列 `target_reacquire`，描述已辨識目標但仍安全停車／交接的時間，不取代本門檻 |

停止距離、期望跟隨距離和 stale 門檻讀取安裝的 `shopping_uwb_follow/config/uwb_follow.yaml`；若模擬使用自訂設定，量測時也須指定相同的 `--follow-params`。`--person-path` 只供指定模擬已使用的同一份路徑，不能用另一條路徑作為真值。JSON 會記錄設定內容、門檻、來源路徑與各項結果。

延遲以整段紀錄為搜尋窗，在 ±0.5 s 內每 0.01 s 搜尋向量均方差；正值代表觀察訊號落後真值。每個候選使用相同樣本集合，至多跨 0.25 s 缺口插值，不外插；至少需要 10 筆與 0.02 m 的運動激發，邊界最佳值視為無法辨識。空間偏差、取樣與服務呼叫排程可能混入延遲，故不是純濾波器內部耗時，也不能把 0.01 s 搜尋格距當成實際精度。統計依樣本加權；轉彎分類依預期人物路徑，因此應先通過同步檢查。

缺資料、框架不符、時間倒退、時鐘逾時或不符合門檻會 FAIL；沒有轉彎／近停／恢復事件，或近停區間碰到紀錄邊界、太短或有資料缺口，顯示 `NOT_COVERED`。恢復期間狀態缺資料或尚未觀察到恢復也會標記未涵蓋；新鮮估測已恢復但在充分觀察後仍未回到 FOLLOWING 則 FAIL。結束碼為 0（全部 PASS）、1（有 FAIL）、2（僅有未涵蓋）。一般路徑不保證出現近距停止，工具也不會為此改動控制器；應由維護者安排可重現的模擬情境，重新量測未涵蓋項目。以上純 Python 測試不代表 ROS／Gazebo 驗收，兩錨點歧義與 NLOS 限制仍適用。
### 偏航率校正與格網幾何檢查

`yaw-check` 用於**全新、單獨執行的純 sim**。純 sim 沒有 mux／watchdog，命令由 `/cmd_vel` 經 `ros_gz` 直接送至 Fortress DiffDrive；不要同時執行 guide、follow、mapping、鍵盤或其他速度發布器。兩個終端機分別執行：

```bash
./scripts/dev.sh sim headless:=true odom_source:=ekf person:=false
./scripts/dev.sh yaw-check --settle 2 --sample 3 --old-factor 2.0
```

工具以 20 Hz 持續送命令，依序測試 `(linear.x, angular.z)`：`(0, 0.2/0.4/0.6/0.8)` 與 `(0.2/0.4, 0.3)`；每列前零速 1 秒、排除 2 秒暫態，再採樣 3 秒（可調）。由 `/sim/ground_truth_odom` 與 EKF `/odom` 的來源時間戳、位置及解纏偏航估計角速度與平面路徑速率，列出實測／命令比值；原地線速比值不適用。速率來自 pose 差分，不依賴兩種 odometry 的 twist 框架慣例。平面速率含側滑，EKF 位置雜訊也會使路徑長稍增。來源時間須覆蓋採樣窗至少 80%，間隔不得大於 0.3 秒。

完成、Ctrl-C 或錯誤時重複發布零 Twist；資料過期 0.5 秒、時鐘倒退／停止、每列超過 `--wall-timeout 45` 秒或雷射距離低於 `--clearance 0.6` 公尺均中止。弧線可能接近貨架而提前停止，這是失敗的量測，不能作為完整六列驗收；需在模擬中重新選擇空曠起點與朝向後再量測。`--in-place-only` 可先單獨完成四列原地測試；若弧線中止，仍保留四列原地擬合結果。此工具沒有底盤程序失效保護，不能用於硬體；停止工具後也應確認車體停止。

擬合只採四列原地真值比值的算術平均。固定輪速時，里程計輪距校正為 `factor_old × ratio`；但 Fortress **命令轉輪速**使用 `wheel_speed = (v ± w × separation / 2) / radius`，本專案 xacro 的試調方向是 `factor_old / ratio`。工具分別列出 `fixed_wheel_rate_separation` 與 `fortress_command_factor`，不可混用。此公式已核對 [Fortress DiffDrive 原始碼](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/diff_drive/DiffDrive.cc)。例如單點 0.855/0.6 推得候選約 1.404，並非 2.85；摩擦起轉門檻與有線速時的側向滑移不保證符合單一倍率，目前仍保留 2.0。若日後校正，必須重驗低速起轉、Nav2 與跟隨調校。

離線幾何工具不需 ROS，僅需既有 Python 與 PyYAML：

```bash
python3 scripts/checks/map_geometry_check.py
python3 scripts/checks/map_geometry_check.py --map ros2_ws/src/shopping_robot_navigation/maps/supermarket.yaml --json /tmp/map_geometry.json
# Or, inside the existing compute container:
./scripts/dev.sh shell compute
python3 /repo/scripts/checks/map_geometry_check.py --json /tmp/map_geometry.json
```

工具讀取 P2／P5（含 16 位元）PGM、YAML 的解析度／原點／旋轉／占用門檻，直接使用淨空測試所依據的 world SDF 碰撞盒。每個面在預測邊緣附近 `--search 0.3` 公尺內，以切向格子各取最近占用格中心、排除角落並擬合直線；搜尋距離另限制小於物件半厚度，避免把內牆面誤當外牆面。輸出四面偏移中位數、覆蓋率、物件中心平移、旋轉與尺寸倍率，JSON 另有每面旋轉及擬合 RMS。平移／尺寸要求兩面覆蓋至少 50%，缺面標為 `missing`，不補零。這是有對應搜尋範圍限制的估計，單面、厚邊／重影、遮蔽與未知格都限制推論；格中心約有半格量化誤差，不等同量到實際物體表面。

另依 `zones.yaml` 與貨架幾何，輸出區域附近一公尺長走道邊緣的橫向平移／旋轉／寬度倍率。snacks 本次為 +0.005 m／0°／1.000，兩側覆蓋皆 100%；平行貨架無法單獨約束沿走道的定位誤差。詳細全店量測與 AMCL 調整理由見 [進度紀錄](PROGRESS.md)。AMCL 此次為待模擬確認的試調，須先用同一提交格網重新啟動 `guide slam_mode:=map_server` 並跑 `loc-check`；尚無證據需要重建地圖。若仍失敗，保留 JSON 並比較誤差方向、AMCL 更新與 EKF 殘差，再考慮重新 map-tour，勿手改格網或移動區域目標來通過門檻。
## MCU 通訊（預設模擬，選用啟動）

新增 `shopping_mcu_bridge`：標準 ROS 訊息介面、具逾時與急停鎖定的模擬韌體、雙向 heartbeat monitor，以及不用實體裝置的 PTY serial 備援。`./scripts/dev.sh comm-sim` 可在沒有 Gazebo 時獨立啟動；另一終端執行 `./scripts/dev.sh comm-check` 會注入測試運動／故障並列出 PASS／FAIL。先以 `dev.sh up`、`dev.sh colcon-build` 準備容器；兩種傳輸的完整指令見 [micro-ROS 入門](hardware/MICROROS_GUIDE.md)。

`dev.sh comm-up` 明確選用 `comm` profile 啟動 UDP4 agent，`comm-down` 只停止 agent；一般 `dev.sh up` 不受影響。模擬韌體直接使用 DDS，**不經過 agent**，尚未驗證真正 micro-ROS client。follow／guide 可明確傳入 `mcu_comm:=true`，預設 false；資料保持 `/mcu/*`，不更動 Gazebo／EKF。請勿同時啟動 standalone comm-sim 與既有移動流程，也不要對 Gazebo 執行通訊注入驗收。介面、QoS、停止與恢復規則見 [MCU 介面](hardware/MCU_INTERFACE.md)。所有預設都不開實體串列埠。
## 未來硬體介面（預設全部模擬）

[硬體介面規格](hardware/HARDWARE_INTERFACES.md) 定義感測器、底盤、急停與心跳契約，以及 `shopping_interfaces` 的四種 UWB 訊息。`hardware.launch.py` 是含 Gazebo 的待機介面驗證入口，`use_real_lidar`、`use_real_imu`、`use_real_uwb`、`use_real_base` 全部預設 `false`；只有明確選用並提供對應 driver package／launch 才會引入未來驅動，缺設定只記錄錯誤，不探測裝置。入口使用 EKF、待機模式與唯一 `/cmd_vel` watchdog，沒有移動命令來源。現有 UWB 模擬／估測仍使用原訊息；目前沒有實體驅動、裝置連線或完整實車 bringup，guide／follow 也尚未整合硬體選項。

## Nav2 fork、TEB 與 apt 對照

`third_party/eurobot_nav2` 是私人儲存庫 `2006wu/Eurobot-2027-Navigation2` 的 git submodule（Humble／Nav2 1.1.16，目前固定於 fork 的 `shopping-car` 分支 `bf60ee0`）。有權限的使用者在**主機**用自己的 Git 憑證初始化；沒有權限的人直接略過，選 `nav2_stack:=auto` 時若找不到 `custom_controller` 與 `custom_nav_to_pose` 會自動使用 apt Nav2。

```bash
git submodule update --init third_party/eurobot_nav2
./scripts/dev.sh build compute
./scripts/dev.sh nav2-build
./scripts/dev.sh nav2-status
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh test
./scripts/dev.sh up
```

私人原始碼以唯讀掛載進 compute 的 `/nav2_ws/src`，產物留在具名 volume；Dockerfile 不 clone、不接收 GitHub token，`.dockerignore` 排除整個 `third_party`。這能讓公開主專案保留整合設定與固定版本指標，而不複製私人實作。顯示容器維持 apt RViz plugins。`nav2-build` 預設 `--parallel-workers 2` 與 `MAKEFLAGS=-j3`；記憶體較少時可用 `env NAV2_PARALLEL_WORKERS=1 MAKEFLAGS=-j1 ./scripts/dev.sh nav2-build`，不指定架構。

**目前固定版本：** fork `shopping-car` 分支 `bf60ee0` 已含非全向／gentle TEB 參數合約與 opt-in 的 Shrink `safe_recovery`（遵守 `shrink_to`、0.40 m 下限、local／global 各自的 inflation service、optional rival／object layer 逾時、結束或取消時還原）。建置前檢查仍會拒絕缺少合約的版本。2026-10-24 模擬中 fork 建置、連結、lifecycle、BT 與 Shrink 均正常，但 gentle TEB 在店內走道仍常判定阻擋、未能抵達區域目標，因此 launch 預設維持 `apt`（見 PROGRESS）。

`guide`、`follow` 與直接的 `shopping_robot_navigation navigation.launch.py` 都接受 `nav2_stack:=auto|fork|apt`，預設 `auto`（2026-10-27 起：已建置 fork underlay 就用 fork TEB＋Smac2D，否則自動回退 apt；要比對 apt 請明確指定 `nav2_stack:=apt`），啟動日誌會顯示結果；明確選 `fork` 而找不到兩個套件時會失敗並提示建置。`apt` 保留既有 `nav2_params.yaml` 與 RPP；即使 fork 已建置，也會在移除 fork 搜尋路徑的子程序重新啟動導航，使用 apt binaries。先停掉上一個 bringup，再執行另一種模式：

```bash
./scripts/dev.sh guide nav2_stack:=fork slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
# After stopping the previous bringup with Ctrl-C:
./scripts/dev.sh guide nav2_stack:=apt slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
```

fork 使用 `Slow`（0.30 m/s）／`Fast`（0.40 m/s）TEB、`holonomic: false`、`avoidance_mode: gentle` 與 `use_rival_topic: false`，預設 selector 選 `Slow`／`Loose`，不需額外發布者。`Loose` 維持區域目標 0.15 m／0.25 rad 容差；`NonStopGoalChecker` 沿用 fork 中以 `SimpleGoalChecker` 定義、忽略朝向的 profile 名稱，不是獨立 plugin class。Smac2D 的既有 0.8 m 可接受終點容差維持不變，仍須量測原始目標誤差。跟隨交接、區域引導、nav-check 與 map-tour 都使用 `NavigateToPose`，空白 `behavior_tree` 會選新的預設樹。

to_pose 與 through_poses recovery 皆為 `GoalUpdated` 或 `Sequence(Shrink, Wait)`，沒有 Escape。Shrink 使用 fork 的 `safe_recovery: true`：BT `shrink_to=0.45 m`，下限 0.40 m（大於 0.60 × 0.50 m 車體半對角線約 0.391 m），兩個 costmap 各用自己的 inflation service，結束或取消時還原；缺少的 rival／object layer service 只等待 0.05 s 即略過。詳見 [架構文件](design/ARCHITECTURE.md)。

### 記錄兩種導航的同一路線

每輪從全新 guide 模擬、相同出生點、同一張已儲存格網與 `slam_mode:=map_server` 開始，人物預設關閉，保持 GUIDE 模式；比較期間不送其他 goto、map-tour、nav-check、跟隨或鍵盤命令。以下步驟分別在兩個終端機執行；每輪檢查結束後先停止 guide，再重新開始，不能在執行途中切換 stack。

```bash
# Terminal 1: first run
./scripts/dev.sh guide nav2_stack:=apt slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
# Terminal 2: record drinks, snacks, produce, household, checkout, entrance
./scripts/dev.sh nav-compare --stack apt --output /repo/output/nav-apt.json
# Stop Terminal 1, restart from spawn with fork, then record again:
./scripts/dev.sh guide nav2_stack:=fork slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh nav-compare --stack fork --output /repo/output/nav-fork.json
./scripts/dev.sh nav-compare --reports /repo/output/nav-apt.json /repo/output/nav-fork.json
```

工具重用 nav-check 的 `ZoneNavigator(enable_interfaces=False)` 與區域清單，先確認執行中的 controller profile 符合 `--stack`，逐一記錄 action 結果、模擬時間、真值軌跡折線長度及終點平面距離誤差，輸出並列表。真值只接受 `/sim/ground_truth_odom` 的 `sim_world`，不退回 EKF odom；缺值、倒退或超過 0.5 模擬秒的取樣缺口會報錯。時間涵蓋送目標前最近一筆至結果後第一筆真值；失敗／逾時也保留結果，逾時會等待取消確認，無法確認則停止後續目標。`SUCCEEDED` 表示 action 成功，未額外保證終點誤差門檻；需一起檢視 error 欄。不同電腦負載、定位漂移與規劃隨機性仍會影響結果，建議重複多輪。JSON 預設例子存入忽略版控的 `output/`。

完整回歸請在各自獨立的模擬場次執行 `dev.sh nav-check --tolerance-check`、`dev.sh loc-check`、mapping 加 map-tour，以及 `dev.sh follow nav2_stack:=fork fault_injection:=true` 加 `dev.sh fault-check`。Nav2 underlay 的 ABI 檢查與限制見架構文件；本機 Python／靜態 PASS 不代表容器或 Gazebo 通過。
`dev.sh comm-up` 明確選用 `comm` profile 啟動 UDP4 agent，`comm-down` 停止 client 與 agent；一般 `dev.sh up` 不受影響。Python 模擬韌體直接使用 DDS，**不經過 agent**。另以 `./scripts/dev.sh comm-e2e-up` 建置／啟動真正 C／rclc Linux XRCE client、agent 與前景 standalone mux／watchdog／monitor；第二終端執行 `./scripts/dev.sh comm-e2e-check`，驗證心跳、輪速、命令逾時與 PC 心跳失聯。C client 只模擬輪子，沒有裝置連線；本次尚未執行 Docker／ROS 驗收。測試前須停止 Gazebo 及其他韌體，結束後 Ctrl-C 前景 launch 再執行 `comm-down`。follow／guide 可明確傳入 `mcu_comm:=true`，預設 false；資料保持 `/mcu/*`，不更動 Gazebo／EKF。請勿同時啟動 standalone comm-sim 與既有移動流程，也不要對 Gazebo 執行通訊注入驗收。介面、QoS、停止與恢復規則見 [MCU 介面](hardware/MCU_INTERFACE.md)。所有預設都不開實體串列埠。


## 模擬情境

`sim`、`mapping`、`guide`、`follow` 都接受 `scenario:=default`。使用 `./scripts/dev.sh scenario-list` 可在本機列出並驗證七種設定，不啟動 Docker；可用 `PYTHON=/path/to/python` 指定已有 PyYAML 的 Python。情境涵蓋中間走道穿越、1.6 公尺窄走道迎面行人、結帳排隊、急轉／走道末端迴轉、人物停走與不同出生位置／朝向。

```bash
./scripts/dev.sh guide scenario:=aisle_crossing headless:=true \
  slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
# In another terminal, record before sending goals.
./scripts/dev.sh record aisle_crossing_01 --duration 120 /scan /simulation/poses
./scripts/dev.sh goto household --timeout 90
./scripts/dev.sh eval-bag aisle_crossing_01
```

錄製命令會阻塞，`goto` 請在第三個終端機執行。每次僅啟動一個移動流程，前一情境 Ctrl-C 完全結束後才啟動下一個。導航情境建議先用標準座標的已存地圖；線上建圖仍可選用。`alt_start` 的 EKF 初始狀態、Gazebo 出生姿態、AMCL `/initialpose` 與 slam_toolbox 定位起點同步選取；`loc-check` 如需再次發布起點，必須加 `-p scenario:=alt_start`。所有驗收仍待維護者在 Fortress 容器實測，操作表與通過條件見 [模擬情境驗收程序](design/SCENARIOS.md) 及 [評估指標](design/METRICS.md#模擬情境與動態行人)。

情境可與 `nav2_stack:=apt|fork|auto` 同時選用，預設為 `auto`；`mcu_comm` 維持預設 `false`。已有 fork underlay 而明確選 apt 時，隔離的導航子程序會保留情境出生姿態，供 slam_toolbox 定位使用。人物同步與跟隨評估若用於非 default 情境，須以 `--person-path` 指定本次 launch 產生的同一份人物路徑；不能套用預設路徑的解析值。

SLAM 建圖、專用 RViz 視圖、四檔儲存與重載定位的完整操作見 [SLAM 操作說明](design/SLAM.md)；Mac arm64 的本次實測記錄見 [驗收報告](hardware/VALIDATION_ARM64.md)。
