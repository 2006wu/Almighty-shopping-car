# Almighty-shopping-car

清華大學動力機械系28級畢業專題項目

本專案開發購物機器人的「引導購物」與「人物跟隨」功能。2006wu 負責 SLAM、Nav2 與跟隨；購物介面、商品辨識等其他子系統不在此骨架範圍。預計 2027 年 2 月才取得硬體，此前全部使用模擬。

使用 Ubuntu 22.04 容器、ROS 2 Humble、Gazebo Fortress（`ros_gz`）、slam_toolbox 與 Nav2。四輪滑移轉向車體約 0.60 × 0.50 公尺，包含 360 度雷射、IMU、輪式里程計及可調整的兩個或三個 UWB 錨點。容器使用主機原生架構，不指定 `--platform`：支援 Mac（Apple Silicon，arm64）的 Docker Desktop 與 Linux（amd64／arm64）的 Docker Engine。車載電腦不在本骨架範圍。

目標展示為 **12 × 10 公尺的小型模擬商店**，配置恰好四排貨架、三條貨架間走道（各寬 1.6 公尺）、入口與結帳區，另有四個不對稱地標（端架、柱子、冷藏櫃、展示台），避免 SLAM 把相同的走道互相誤認。這是畢業專題的模擬展示場景。機器人出生姿態為世界座標 `(x, y, z, yaw) = (0, 0, 0.02, 0)`，朝向正 x；地面穩定後 base_footprint 接近 z = 0。以此初始原點建立 `odom`／`map`，六個區域目標使用與世界相同的平面座標，既有外部地圖須先對齊，不能直接套用。

## 2027 年 1 月在 Mac 上執行

先安裝並啟動 Docker Desktop，確認 `docker compose version` 可用，並預留足夠記憶體供 Gazebo 軟體繪圖。以下指令可以直接在 fish 執行；腳本會由 Bash 解譯，不必修改 fish 設定，也不需在 macOS 安裝 ROS。

```fish
git clone https://github.com/2006wu/Almighty-shopping-car.git
cd Almighty-shopping-car
set -gx VNC_PASSWORD myshop42
./scripts/dev.sh build
./scripts/dev.sh colcon-build
./scripts/dev.sh test
./scripts/dev.sh up
./scripts/dev.sh status
./scripts/dev.sh mapping
```

另開終端機啟動 RViz：

```fish
./scripts/dev.sh rviz
./scripts/dev.sh novnc
```

瀏覽器開啟 <http://localhost:6080/vnc.html>，輸入相同 VNC 密碼。RViz 預設固定座標為 `odom`，純跟隨也可顯示；使用 Nav2 時請在 Global Options 將 Fixed Frame 切成 `map`。

### 自動建圖與離線定位檢查

先停止其他模擬，再用第一個主機終端機啟動含 Nav2 的建圖流程；第二個終端機執行巡航：

```fish
# Terminal 1; keep running until the tour has saved all four files
./scripts/dev.sh guide slam_mode:=mapping
# Terminal 2; default map name is supermarket
./scripts/dev.sh map-tour supermarket
```

`map_tour` 依 `shopping_zone_manager/config/map_tour.yaml` 逐一送出 NavigateToPose 目標，涵蓋三條走道全長、南北橫向通道、入口及結帳區，最後回到原點。北側橫向路段採 y=6.1，避開 y=5.3 的端架；pytest 以 SDF 所有碰撞盒、車體外接圓加 0.15 公尺餘裕檢查航點及相鄰連線。這是幾何檢查，實際路徑由 Nav2 決定，仍須在模擬驗證。

`map_tour` 與共用 `experiment_node` 只透過 Nav2 action 送出地圖目標，不讀取 `/odom` 作為真值，可沿用預設 EKF 流程；建圖品質取決於 EKF 與 slam_toolbox。納入版本管理的標準地圖應使用預設 `odom_source:=ekf` 產生，以保留含漂移里程計的模擬條件。若要比較理想真值里程計，停止原模擬後以 `./scripts/dev.sh guide slam_mode:=mapping odom_source:=ground_truth` 重新啟動，再於另一終端機執行 `./scripts/dev.sh map-tour supermarket_ground_truth`，另存比較地圖。

預設每個航點最多失敗兩次（重試一次），之後跳過；單一目標逾時 150 秒，整趟導航逾時 1800 秒，皆使用實際經過時間，模擬暫停也會逾時。目標逾時先取消並等待終止，取消無法確認時直接結束。完整跑完序列後會等待最後掃描更新，再以 `map_saver_cli` 儲存 `.yaml/.pgm`，呼叫 `/slam_toolbox/serialize_map` 儲存 `.posegraph/.data`；整趟逾時或中斷不自動儲圖。JSON 摘要列出到達、跳過、尚未執行、逾時及儲檔結果；有跳過或儲檔失敗皆以非零結束碼退出。跳過航點後仍可能產生不完整地圖，確認全部到達再採用。

產物位於 `ros2_ws/src/shopping_robot_navigation/maps/`；標準格網 `supermarket.yaml`、`supermarket.pgm` 已加入 Git（供 AMCL／map_server 模式）；位姿圖 `supermarket.posegraph`、`supermarket.data` 約 27 MB，不加入 Git，使用 slam_toolbox 定位前須先在本機執行 `map-tour` 重新產生；其他名稱仍忽略。四檔須來自同一次建圖，重建與檢核方式見 [地圖說明](ros2_ws/src/shopping_robot_navigation/maps/README.md)。也保留 `./scripts/dev.sh save-map NAME` 供手動儲存；需要手動建圖時改用 `mapping` 與鍵盤遙控，完成後再儲圖。

巡航完成後以 Ctrl-C 停止第一個終端機的整套 `guide`，每次重新從出生點啟動一種定位模式。第一種載入 slam_toolbox 位姿圖前綴，不加副檔名：

```fish
# Terminal 1
./scripts/dev.sh guide slam_mode:=localization map:=/ws/src/shopping_robot_navigation/maps/supermarket
# Terminal 2, after Nav2 is ready
./scripts/dev.sh loc-check
```

完成後停止這套 `guide`，再驗證第二種 AMCL 格網定位：

```fish
# Terminal 1
./scripts/dev.sh guide slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
# Terminal 2; publish the known spawn pose to AMCL before moving
./scripts/dev.sh loc-check -p publish_initial_pose:=true
```

`localization_check` 依 `zones.yaml` 順序走訪全部六區；每區目標結束後等待 2 秒，再採樣 3 秒。每筆採樣以真值訊息時間戳查詢 `map → base_footprint`，只接受新鮮資料（模擬時間及接收時間皆檢查），不足五筆、缺 TF、導航失敗或未走完六區均為 FAIL。每區輸出最後 TF／真值距區域座標的平面距離，以及定位誤差平均值、RMSE、最大值；最後另列所有採樣的統計。PASS 要求每區所有採樣定位誤差 **< 0.15 公尺、< 5 度**，最後真值距區域目標 **≤ 0.3 公尺**；整體 PASS 才回傳零結束碼。

真值參數 `truth_topic` 預設 `/sim/ground_truth_odom`，兩種里程計模式都使用此獨立真值，不再探索主題或回退至 `/odom`。可用 `-p truth_topic:=/custom_truth` 覆寫；只接受 `(sim_world, base_footprint_gt)` 或舊版 `(odom|world|map, base_footprint)` 框架配對，其餘會明確報錯。真值與地圖必須共用出生原點／朝向；EKF 模式下 `sim_world` 與估測 TF 樹沒有連線，檢查直接比較座標，不會以待測的 map 校正轉換真值。預設 `/odom` 是會漂移的 EKF 估測，不能當作真值；強制選用時會警告並拒絕執行。只有模擬已用 `odom_source:=ground_truth` 啟動，且檢查另以 `./scripts/dev.sh loc-check -p truth_topic:=/odom -p odom_source:=ground_truth` 明確宣告相同模式時，才允許舊版來源；檢查參數不會切換模擬模式。這些模擬結果仍不代表真實硬體精度。

兩個工具都透過 `dev.sh` 設定 `use_sim_time:=true`。直接於已載入 ROS 環境的容器內執行可調整參數：

```bash
ros2 run shopping_zone_manager map_tour --ros-args -p use_sim_time:=true -p map_name:=supermarket -p max_failures:=2 -p goal_timeout:=150.0 -p overall_timeout:=1800.0
ros2 run shopping_zone_manager localization_check --ros-args -p use_sim_time:=true -p report_file:=/tmp/localization_check.json
```

巡航、定位檢查、`goto`、`/shopping/goto_zone`、跟隨與鍵盤控制必須互斥操作；尚無模式管理器或速度仲裁。`map → odom` 在每次啟動時只由所選 slam_toolbox 或 AMCL 發布。定位用解析度、雷射上限及座標框架與 mapper 一致；slam_toolbox 保留 `map_start_at_dock: true`，從出生點的第一筆建圖掃描開始，不同時設定優先權更高的 `map_start_pose`（[Humble 實作](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)）。AMCL 使用 `(0,0,0)` 初始姿態，檢查工具的 `publish_initial_pose` 會在移動前另外發布 `/initialpose`。本次未調整控制器與規劃器，也尚未執行這套巡航及兩種離線定位的 ROS／Gazebo 驗收。

導航就緒後，從另一個終端機送出目標：

```fish
./scripts/dev.sh zones
./scripts/dev.sh goto 飲料區
./scripts/dev.sh goto 生鮮區 --timeout 120
./scripts/dev.sh cancel
```

`goto` 將區域名稱送至常駐 `zone_navigator` 的 `/shopping/goto_zone`（`std_msgs/String`），由同一節點持有 NavigateToPose 目標，印出回饋及結果；只有 `SUCCEEDED` 回傳 0，其餘結果回傳非零。Nav2 伺服器等待上限為 15 秒，`--timeout` 預設 300 秒；Ctrl-C 或操作期限到達時，先要求取消並有限等待最終結果，無法確認時會明確提示。第一次建圖的原點與朝向應維持模擬初始姿態，重建地圖後要核對區域座標。

`cancel` 呼叫 `/shopping/cancel_goal`（`std_srvs/Trigger`）；也可發布 `std_msgs/Empty` 至 `/shopping/cancel`。服務回覆代表已處理取消要求，須等 `/shopping/zone_status` 的 `CANCELED` 才代表動作已取消；閒置時回覆 `IDLE`。明確執行 `cancel` 也會清除搶先流程的排隊目標。取消尚待接受的目標會在收到接受回覆後立刻執行；若取消回覆遺失，可再次執行 `cancel` 重送要求。操作介面目前使用共用狀態主題，沒有每位呼叫者的識別碼，請一次使用一個 `goto` 終端機，避免混用其他直接操作 Nav2 的客戶端。

狀態格式固定為 `CODE: 說明`（實際說明為英文），距離回饋約每秒一筆：

| 狀態碼 | 意義 |
| --- | --- |
| IDLE／PENDING／ACTIVE／CANCELING | 閒置／等待目標接受／移動中／等待取消結果 |
| SUCCEEDED／ABORTED／CANCELED／REJECTED | 成功／執行失敗／已取消／目標被拒絕 |
| TIMEOUT | 超過 `response_timeout`（預設 5 秒）未收到目標接受回覆 |
| UNAVAILABLE／UNKNOWN_ZONE | Nav2 尚未就緒／區域不存在 |
| BUSY／CANCEL_FAILED | 舊目標尚未結束，拒絕新要求／取消未獲確認，目標可能仍在執行 |

預設忙碌時拒絕新目標。節點啟動參數 `preempt:=true`（預設 `false`）可改成先取消舊目標，收到舊動作最終結果後才送出最新排隊目標；也可透過 `ros2 param set /zone_navigator preempt true` 切換；Python 介面 `navigate(name, preempt=True)` 可逐次選用。接受回覆逾時後保留 future；延遲接受的目標立即取消，舊世代的結果不影響新目標。若 DDS 回覆永遠遺失，節點會持續拒絕新目標，避免同時移動；`TIMEOUT` 本身不保證機器人已停，須檢查 Nav2 與模擬狀態，不能只重啟區域節點就視為安全。

維護者可在 `guide` 啟動、Nav2 啟用且地圖已涵蓋商店後執行：

```fish
./scripts/dev.sh nav-check
# Optional per-case deadline (seconds)
./scripts/dev.sh nav-check --case-timeout 180
# Optional reachable-zone position check (metres)
./scripts/dev.sh nav-check --tolerance-check --max-final-error 0.35
```

檢查工具會實際移動模擬機器人，請先結束既有導航，保持常駐節點閒置，並互斥使用鍵盤、跟隨與其他導航客戶端。工具使用同一套區域節點程式，但不重複建立指令訂閱與取消服務，依序檢查未知區域不送 action、不可達目標（`unreachable goal`）遭 `ABORTED`／`REJECTED`、遠端目標有至少 0.15 公尺進展後取消、取消後 2 秒內收到 `CANCELED` 且新鮮 `/odom` 速度接近零（各平面分量低於 0.02）、有效目標成功，以及第二個目標回覆 `BUSY`。最後印出 PASS／FAIL 表，任何失敗皆回傳非零；無法取消舊目標時停止後續案例。不可達目標使用 map 座標 `(50, 50)`，NavFn 容許值讀取安裝的 `shopping_robot_navigation/config/nav2_params.yaml`；送出前從安裝的 `supermarket.sdf` 牆面盒體取得商店外框，並讀取 `/global_costmap/costmap`，確認目標距離兩個外框皆超過「容許值＋1 公尺」。成本地圖外框涵蓋所有格子，因此也保證距離每個自由格子超過此門檻；缺少成本地圖、座標框架或幾何假設不符時回報失敗，不送出此目標。單項等待保留 `--case-timeout`（預設 120 秒），逾時後要求取消。這是待容器實測的工具，離線測試不代表模擬驗收。

`--tolerance-check` 預設關閉，啟用後增加第六項 `zone tolerance`：送出一個正常區域目標，須收到 `SUCCEEDED`，再於 5 秒內取得結果之後接收的新鮮 `/odom`，計算與區域座標的平面距離。`--max-final-error` 預設 0.35 公尺，超標即失敗；PASS 與超標 FAIL 的表格都列出實測誤差與門檻，用來檢查共用 NavFn 0.8 公尺容許值是否使導覽提早停止。此量測直接將 odom 與 map 座標比較，僅適用模擬初始原點與朝向對齊且漂移可忽略的情況；不補償 AMCL／SLAM 的 map→odom 修正，不能當作一般定位精度驗收。

停止其他移動流程後測試 UWB 跟隨（包含建圖與 Nav2 繞行）：

```fish
./scripts/dev.sh follow
```

`sim.launch.py`、`mapping.launch.py` 與 `guide.launch.py` 預設 `person:=false`：人物停放在店外 `(20, 20)`，不啟動路徑節點，避免人物碰撞幾何寫入地圖或擋住導航。需要動態障礙測試時可明確啟用，例如 `ros2 launch shopping_robot_bringup guide.launch.py person:=true`。`follow.launch.py` 預設 `slam_mode:=mapping`、`map:=`（空值）、`route_around:=true`，會啟動 slam_toolbox 與 Nav2，但不啟動區域導航節點。使用已存地圖時，與引導模式相同，可設 `slam_mode:=localization map:=/maps/store`（slam_toolbox 序列化地圖）或 `slam_mode:=map_server map:=/maps/store.yaml`（AMCL）。例如 `ros2 launch shopping_robot_bringup follow.launch.py route_around:=false` 可恢復原本直接跟隨控制，節點不建立動作客戶端；launch 仍會啟動導航服務。`follow.launch.py` 固定傳入 `person:=true`，人物路徑從出生點正前方約 1.5 公尺的 `(1.5, 0)` 開始，首段沿前方橫向通道向 +x 移動。

人物是本機圓柱幾何模型，不需連線下載 Fuel 素材。路徑唯一來源為 `shopping_robot_gazebo/config/person_path.yaml`；模擬人物與 UWB 使用相同模擬時間插值。人物會循環走過三條走道；測試直接解析 SDF 碰撞盒，驗證區域有 0.55 公尺淨空、人物整條路徑有 0.30 公尺淨空。`uwb_follow.yaml` 可調整噪聲、偏差、NLOS、掉包及錨點陣列；兩錨點實驗須同時縮短 `anchor_x/anchor_y`，並將車體 xacro 的 `anchor_count` 改成 2 或以 xacro 參數產生相符模型。錨點移位時也需同步修改 xacro 的錨點屬性。這是平面測距模型，尚未模擬高度差與完整無線電傳播。

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

`/sim/odom_drift` 每 5 秒發布一次 Float32MultiArray，依序為平移誤差（m）、絕對航向誤差（rad）、累積真值路程（m）、平移誤差／路程（m/m）、航向誤差／路程（rad/m）。未移動時最後兩項為 NaN。監測器依時間插值真值，跨越 0.1 秒以上的資料缺口便略過，不外插；資料過期時不重播舊統計。請從出生點開始整段量測，至少比較直線、轉彎與返回起點；真值模式誤差應接近零（含插值殘差），EKF 應呈現非零漂移，預期比單靠輪速積分的偏航穩定，但尚無容器量測數字或保證門檻。種子 42 只固定輪速噪聲序列，不保證物理模擬完全重現。參數、covariance 與離線 `odom_metrics.py` 介面見[架構文件](docs/ARCHITECTURE.md)。

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

Nav2 平滑器輸出 `/cmd_vel/nav`、直接跟隨輸出 `/cmd_vel/follow`；只有獨立程序 `cmd_vel_watchdog` 發布底盤 `/cmd_vel`。仲裁器以 20 Hz 發布內部 `/cmd_vel_mux/output`；nav／follow／teleop 的接收逾時分別為 0.5／0.3／0.5 秒，資料過期後下一次更新歸零。看門狗以單調時鐘檢查仲裁輸出，超過 0.25 秒未收到命令，於下一個 50 毫秒更新發布零速，模擬暫停不會停止其計時。限速為 0.6 m/s、1.2 rad/s，一般加速度為 0.6 m/s²、1.2 rad/s²，零速安全命令立即生效，預設不倒車。

純邏輯與單獨執行 `cmd_vel_mux` 都預設 `IDLE`，程序重啟不自行恢復先前模式。launch 的 `initial_mode` 是明確的便利性例外：重新執行整個 guide／follow launch（或以相同參數重啟節點）會重新請求該模式；有重啟風險的驗收請使用 `initial_mode:=idle`。launch 未設定自動 respawn。若獨立看門狗已鎖定急停，重啟仲裁器不會解除，仍須 `reset`。

`use_mux:=false` 恢復 Nav2／跟隨直接發布 `/cmd_vel`，不啟動模式管理與看門狗，必須維持舊有流程互斥。mapping 沒有 Nav2，維持直接鍵盤操作；如需為建圖加上看門狗，可另啟動 `ros2 launch shopping_robot_bringup mode_mux.launch.py initial_mode:=teleop` 並將鍵盤改送 `/cmd_vel/teleop`。禁止同時保留直接 `/cmd_vel` 發布端。看門狗本身、橋接或 Gazebo 同時故障時，軟體無法保證停止；目前未新增 Fortress 的底盤逾時外掛，也沒有硬體安全功能。完整參數與容器驗收步驟見[架構文件](docs/ARCHITECTURE.md)。

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

`fault-check` 請使用空排程。工具依序測六種故障，每次保持約 4 秒模擬時間、清除並等待恢復，輸出 PASS／FAIL 表與非零失敗結束碼。使用 `python3 scripts/fault_check.py --help` 可查看等待選項；門檻、預期反應及限制見[架構文件](docs/ARCHITECTURE.md#模擬故障注入與驗收)。目前只有純 Python／靜態測試，尚未取得 Gazebo 故障量測結果。

## 安全與驗證狀態

所有預設流程僅使用模擬，沒有硬體驅動、裝置掛載或真實車體連線。跟隨節點單獨執行時預設停用，由 follow launch 的模式管理器啟用（`use_mux:=false` 時維持 launch 直接啟用）；可用 `/follow/enable` 的 `std_srvs/SetBool` 服務切換。目標太近、遺失或資料過期時立即輸出零 Twist；雷射未收到、前方無有效資料或超時也完全停止。前方有效障礙讀值不超過 0.65 公尺時，`OBSTACLE_STOP` 立即將線速度歸零；目標有效且距離大於 0.6 公尺時，允許原地轉向目標，角速度及角加速度仍受限。急停優先於一般加速度限制；一般追蹤限制線速度、角速度及加速度，預設不倒車。

跟隨 launch 同時啟動 `uwb_eval_node`，有時間相近的 `/sim/ground_truth_odom` 與 `/uwb/person_ground_truth` 時，發布 `/uwb/estimate_error`（Float32，公尺）。它用車體真值姿態將 base_link 估測值轉到 sim_world 與人物真值比較，不依賴估測 TF，因此不混入 EKF 漂移；結果仍包含取樣時間差。三筆時間戳最大差預設為 0.15 秒，不插值，缺資料或過期便略過。離線可用不依賴 ROS 的 `follow_metrics.py`，傳入依時間遞增的 `FollowSample` 序列，以 `evaluate_series` 計算估測平均誤差、RMSE、最大誤差及真實跟隨距離的平均／最小／最大值；統計按樣本加權。

跟隨模式可將持續受阻的有效目標交給 Nav2 繞行。一般 LOS 走廊半寬為 0.25 公尺；連續受阻 0.3 秒才進入 `BLOCKED_WAIT`，此前維持 `FOLLOWING` 並減速。已判定受阻後，須以加寬至 0.35 公尺的走廊連續暢通 0.6 秒才能解除，避免貨架末端的雷射擦邊反覆停車。人物距離超過 1.3 公尺且判定受阻再持續 0.5 秒後交接；前方障礙停止仍立即生效，不等待 LOS 計時。`/follow/state` 發布 `STATE: 原因`，`/follow/handoff_goal` 為人物 map 座標的 PoseStamped，可加入 RViz。

NAV2 至少保持 1.5 秒，然後以遲滯確認暢通的 LOS 加上前方淨空大於 0.8 公尺、持續 0.5 秒恢復 DIRECT；若人物距離已不超過 1.3 公尺，可略過這額外 0.5 秒。導航成功／失敗／取消或安全事件可立即結束保持時間。恢復 DIRECT 後至少保持 1 秒才再次交接；若 LOS 已連續判定受阻達 1 秒，可提前交接，但仍須滿足受阻交接延遲。導航中直接控制器不發布速度；遺失、過近、停用或雷射失效時立即取消並輸出零 Twist。目標更新直接使用 Nav2 搶占；切回時等待動作終止，且外部速度命令安靜 0.3 秒或最新命令近零，終止後最多等 2 秒，第一筆直接命令為零。NavFn 共用容差仍為 0.8 公尺，目標未偏移；引導模式受阻的區域目標也可能停在較遠處，不能只依成功狀態認定到達原始座標。

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
python3 scripts/follow_state_stats.py follow-state.log
python3 scripts/follow_state_stats.py follow-state.log --sample-period 0.05
```

工具解析 `data: STATE: 原因` 或 `state: STATE` 紀錄，可在同一行前綴 `[秒數]` 或 `秒數 `。有時間戳時須有限且嚴格遞增；無時間戳時依序假設每筆 0.05 秒（20 Hz），遺漏樣本或僅記錄狀態變更時須提供時間戳，否則每分鐘切換率與停留時間僅為估算。混用有／無時間戳會拒絕分析。輸出切換總數、每分鐘切換率與各狀態停留次數／總計／平均／最小／最大秒數；首尾停留區間只計算觀測範圍，最後一筆不外推。亦可呼叫 ROS-free 的 `follow_metrics.state_statistics([(t, state), ...])`。

guide／follow 預設啟用全域模式管理、速度仲裁與獨立輸出看門狗，操作方式見下節；一次只啟動一個 bringup，跟隨期間仍不可另外送出 Nav2 目標，以免干擾跟隨自行管理的導航動作。UWB 訊息目前用無時間戳的 Float32MultiArray，模擬本機接收時間視為量測時間；硬體介面前必須改用有時間戳及錨點識別的訊息。

純 Python 測試可在容器執行（`./scripts/dev.sh test`），也可在任何已安裝 pytest、pyyaml、xacro 的 Python 3.10 環境執行：

```bash
python3 -m pytest -q
./scripts/run_unit_tests.sh python3
python3 scripts/check_static.py
```

此次 EKF、噪聲與漂移監測尚未執行映像重建、colcon 或 Gazebo 驗證。先前真值里程計版本於 2026 年 10 月已在 Linux amd64 環境以 Docker 實際驗證：映像建置、七套件 colcon 建置與測試、建圖、RViz（經 noVNC）、六個區域導航與跟隨流程；Mac arm64 建置、Gazebo GUI 與真實硬體仍未驗證，詳見 [進度紀錄](docs/PROGRESS.md)。指定環境的 xacro 套件若不支援 `python -m xacro`，可改用 `python -c 'import xacro; xacro.main()' 模型路徑`，兩者呼叫相同的 xacro 處理程式。

[模擬評估指標與離線報表](docs/METRICS.md) · [UWB 錨點離線研究](docs/UWB_STUDY.md) · [架構](docs/ARCHITECTURE.md) · [時程](docs/ROADMAP.md) · [協作規範](AGENTS.md)

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

完整時間規則、已支援與無法還原的觀測、固定情境及兩次報告比較方式見 [評估指標](docs/METRICS.md#rosbag-記錄轉換與可重現流程)。沙盒僅驗證純 Python 與靜態配置，ROS／Gazebo 實際錄製回放須由維護者在容器驗證。

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

既有 `/simulation/poses` 保留原用途。Fortress [SceneBroadcaster](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/scene_broadcaster/SceneBroadcaster.cc) 的時間戳在外層 Pose_V，而 Humble [TFMessage 轉換](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/src/convert/tf2_msgs.cpp) 讀取各筆 pose，無法保留該外層時間。此次增加人物唯讀 [PosePublisher](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc)，產生各筆時間與框架；只有 follow launch 將 `/world/supermarket/model/person/pose` 單向橋接至 `/simulation/person_pose`（TFMessage），不接入 `/tf`。這是量測來源時間的補足，沒有修改人物移動或 UWB 生成演算法；SDF 的觀察外掛在其他模擬也會載入，但沒有新增 ROS 橋接。

`follow_eval` 使用人物真值與 `/sim/ground_truth_odom`（`sim_world → base_footprint_gt`），將真實相對位置轉到車體平面座標，再和 `/uwb/target`（`base_link`）比較。平移原點相同的假設來自目前 URDF 的固定關節。表格包含下列指標，門檻是初始驗收提案，並非控制器調參：

| 項目 | 定義與預設門檻 |
| --- | --- |
| 濾波延遲 | 相對位置向量最佳時間位移，絕對值 <0.3 s；`--latency-limit` 可改 |
| 直線／轉彎追蹤 | 共用人物路徑在前後各 0.5 s 的航向變化率 >0.3 rad/s 視為轉彎；各類至少 10 筆真值，真實距離減 `follow_distance` 的 RMSE <1.5 m（`--tracking-limit`）；另列距離及估測位置誤差統計 |
| 近距停止 | 真實距離嚴格小於 YAML 的 `stop_distance`；0.5 s（`--stop-deadline`）後平面速度須 ≤0.03 m/s、角速度絕對值須 ≤0.05 rad/s，並維持到離開近距區間；列出所有違規時間 |
| 重新取得 | 估測來源時間缺口超過 YAML 的 `lost_timeout` 或狀態進入 LOST，從下一筆新鮮估測到 `/follow/state` 回到 FOLLOWING 須 ≤2 s（`--reacquire-limit`）；另列遺失至恢復的總時間，NAV2_HANDOFF 不當作 FOLLOWING |

停止距離、期望跟隨距離和 stale 門檻讀取安裝的 `shopping_uwb_follow/config/uwb_follow.yaml`；若模擬使用自訂設定，量測時也須指定相同的 `--follow-params`。`--person-path` 只供指定模擬已使用的同一份路徑，不能用另一條路徑作為真值。JSON 會記錄設定內容、門檻、來源路徑與各項結果。

延遲以整段紀錄為搜尋窗，在 ±0.5 s 內每 0.01 s 搜尋向量均方差；正值代表觀察訊號落後真值。每個候選使用相同樣本集合，至多跨 0.25 s 缺口插值，不外插；至少需要 10 筆與 0.02 m 的運動激發，邊界最佳值視為無法辨識。空間偏差、取樣與服務呼叫排程可能混入延遲，故不是純濾波器內部耗時，也不能把 0.01 s 搜尋格距當成實際精度。統計依樣本加權；轉彎分類依預期人物路徑，因此應先通過同步檢查。

缺資料、框架不符、時間倒退、時鐘逾時或不符合門檻會 FAIL；沒有轉彎／近停／恢復事件，或近停區間碰到紀錄邊界、太短或有資料缺口，顯示 `NOT_COVERED`。恢復期間狀態缺資料或尚未觀察到恢復也會標記未涵蓋；新鮮估測已恢復但在充分觀察後仍未回到 FOLLOWING 則 FAIL。結束碼為 0（全部 PASS）、1（有 FAIL）、2（僅有未涵蓋）。一般路徑不保證出現近距停止，工具也不會為此改動控制器；應由維護者安排可重現的模擬情境，重新量測未涵蓋項目。以上純 Python 測試不代表 ROS／Gazebo 驗收，兩錨點歧義與 NLOS 限制仍適用。
