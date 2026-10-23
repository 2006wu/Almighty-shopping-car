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

瀏覽器開啟 <http://localhost:6080/vnc.html>，輸入相同 VNC 密碼。RViz 預設固定座標為 `odom`，純跟隨也可顯示；使用 Nav2 時請在 Global Options 將 Fixed Frame 切成 `map`。建圖時，可在另一個終端機操作：

```fish
./scripts/dev.sh shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

先用低速走遍走道，然後在主機終端機儲存地圖：

```fish
./scripts/dev.sh save-map supermarket
```

這會在 `ros2_ws/src/shopping_robot_navigation/maps/` 儲存格網 `.yaml/.pgm` 以及 slam_toolbox `.posegraph/.data`。不要在 AMCL 模式使用此指令要求序列化 SLAM；此時沒有 slam_toolbox 服務。

停止建圖（Ctrl-C）後，選擇一種導航方式，每次只啟動一個模擬：

```fish
# Live mapping with Nav2; no saved map required
./scripts/dev.sh guide
# Saved SLAM pose graph; map is a prefix without an extension
./scripts/dev.sh guide slam_mode:=localization map:=/ws/src/shopping_robot_navigation/maps/supermarket
# Saved occupancy grid with AMCL
./scripts/dev.sh guide slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
```

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

## 安全與驗證狀態

所有預設流程僅使用模擬，沒有硬體驅動、裝置掛載或真實車體連線。跟隨節點單獨執行時預設停用，只有 follow launch 主動啟用；可用 `/follow/enable` 的 `std_srvs/SetBool` 服務切換。目標太近、遺失或資料過期時立即輸出零 Twist；雷射未收到、前方無有效資料或超時也完全停止。前方有效障礙讀值不超過 0.65 公尺時，`OBSTACLE_STOP` 立即將線速度歸零；目標有效且距離大於 0.6 公尺時，允許原地轉向目標，角速度及角加速度仍受限。急停優先於一般加速度限制；一般追蹤限制線速度、角速度及加速度，預設不倒車。

跟隨 launch 同時啟動 `uwb_eval_node`，有時間相近的 `/sim/ground_truth_odom` 與 `/uwb/person_ground_truth` 時，發布 `/uwb/estimate_error`（Float32，公尺）。它用車體真值姿態將 base_link 估測值轉到 sim_world 與人物真值比較，不依賴估測 TF，因此不混入 EKF 漂移；結果仍包含取樣時間差。三筆時間戳最大差預設為 0.15 秒，不插值，缺資料或過期便略過。離線可用不依賴 ROS 的 `follow_metrics.py`，傳入依時間遞增的 `FollowSample` 序列，以 `evaluate_series` 計算估測平均誤差、RMSE、最大誤差及真實跟隨距離的平均／最小／最大值；統計按樣本加權。

跟隨模式現在可將持續受阻的有效目標交給 Nav2 繞行：距離超過 1.3 公尺且持續受阻 0.5 秒後交接；視線受阻期間直接模式原地等待（`BLOCKED_WAIT`），避免貼近貨架角落；視線暢通且前方淨空大於 0.8 公尺持續 0.5 秒後，或視線暢通且目標已接近、導航成功時，取消目標並恢復直接跟隨。`/follow/state` 增加 `NAV2_HANDOFF` 與 `BLOCKED_WAIT`，`/follow/handoff_goal` 為 map 座標的 PoseStamped，可加入 RViz；跟隨模式可將 Fixed Frame 改為 `map`。導航中直接控制器不發布速度；遺失、過近、停用或雷射失效時立即取消並輸出零 Twist。目標更新直接使用 Nav2 搶占；切回時等待動作終止，且外部速度命令安靜 0.3 秒或最新命令近零，終止後最多等 2 秒，第一筆直接命令為零。NavFn 共用容差由 0.3 改為 0.8 公尺，以允許規劃至人物旁的可通行終點；引導模式受阻的區域目標也可能停在較遠處，不能只依成功狀態認定到達原始座標。已於 Docker／Gazebo 跑 240 秒驗證（見 PROGRESS）；視線抖動造成走走停停仍待改善。

本階段尚無全域模式管理器；follow 執行期間不可另外送出 Nav2 目標、執行 guide 或鍵盤控制。交接僅管理自己送出的動作，獨立命令逾時看門狗與跨流程仲裁仍待後續整合。UWB 訊息目前用無時間戳的 Float32MultiArray，模擬本機接收時間視為量測時間；硬體介面前必須改用有時間戳及錨點識別的訊息。

純 Python 測試可在容器執行（`./scripts/dev.sh test`），也可在任何已安裝 pytest、pyyaml、xacro 的 Python 3.10 環境執行：

```bash
python3 -m pytest -q
./scripts/run_unit_tests.sh python3
python3 scripts/check_static.py
```

此次 EKF、噪聲與漂移監測尚未執行映像重建、colcon 或 Gazebo 驗證。先前真值里程計版本於 2026 年 10 月已在 Linux amd64 環境以 Docker 實際驗證：映像建置、七套件 colcon 建置與測試、建圖、RViz（經 noVNC）、六個區域導航與跟隨流程；Mac arm64 建置、Gazebo GUI 與真實硬體仍未驗證，詳見 [進度紀錄](docs/PROGRESS.md)。指定環境的 xacro 套件若不支援 `python -m xacro`，可改用 `python -c 'import xacro; xacro.main()' 模型路徑`，兩者呼叫相同的 xacro 處理程式。

[模擬評估指標與離線報表](docs/METRICS.md) · [UWB 錨點離線研究](docs/UWB_STUDY.md) · [架構](docs/ARCHITECTURE.md) · [時程](docs/ROADMAP.md) · [協作規範](AGENTS.md)
