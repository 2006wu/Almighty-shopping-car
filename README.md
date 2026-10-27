# Almighty-shopping-car

清華大學動力機械系28級畢業專題項目

本專案開發購物機器人的「引導購物」與「人物跟隨」功能。2006wu 負責 SLAM、Nav2 與跟隨；購物介面、商品辨識等其他子系統不在此骨架範圍。預計 2027 年 2 月才取得硬體，此前全部使用模擬。

使用 Ubuntu 22.04 容器、ROS 2 Humble、Gazebo Fortress（`ros_gz`）、slam_toolbox 與 Nav2。四輪滑移轉向車體約 0.60 × 0.50 公尺，包含 360 度雷射、IMU、輪式里程計及可調整的兩個或三個 UWB 錨點。容器使用主機原生架構，不指定 `--platform`：支援 Mac（Apple Silicon，arm64）的 Docker Desktop 與 Linux（amd64／arm64）的 Docker Engine。車載電腦不在本骨架範圍。

目標展示為 **12 × 10 公尺的小型模擬商店**，配置恰好四排貨架、三條貨架間走道（各寬 1.6 公尺）、入口與結帳區，另有四個不對稱地標（端架、柱子、冷藏櫃、展示台），避免 SLAM 把相同的走道互相誤認。這是畢業專題的模擬展示場景。機器人出生姿態為世界座標 `(x, y, z, yaw) = (0, 0, 0.02, 0)`，朝向正 x；地面穩定後 base_footprint 接近 z = 0。以此初始原點建立 `odom`／`map`，六個區域目標使用與世界相同的平面座標，既有外部地圖須先對齊，不能直接套用。

## 選用角色容器

預設維持 `compute`＋`display`。要驗證容器拆分，先執行 `./scripts/dev.sh build compute display`、`./scripts/dev.sh colcon-build`，再執行 `./scripts/dev.sh split-up guide headless:=true`；用 `split-status` 查看狀態、`split-logs navigation` 看日誌、`nav-check --case-timeout 180` 檢查導航，最後 `split-down`。也支援 `split-up follow`，沿用 `nav2_stack:=apt|auto|fork`。

`split` 分別啟動 sim、localization、navigation、app 與既有 display；comm、record 另外明確選用。所有服務共用 compute 網路命名空間與 UDP-only DDS，適用 Mac arm64／Linux。`split-up` 會先停止同一專案的既有服務，避免重複控制節點；切換前先結束錄製工作。Mac 日常開發優先使用預設單容器模式，驗證拆分時保持 headless、暫不啟用額外 profiles。角色表、主題流向、限制與完整維護者驗收命令見 [容器操作說明](docs/design/CONTAINERS.md)。

## 在 Mac Apple Silicon 與 Linux 上執行

先安裝並啟動 Docker Desktop，確認 `docker compose version` 可用，並預留足夠記憶體供 Gazebo 軟體繪圖。以下指令可以直接在 fish 執行；腳本會由 Bash 解譯，不必修改 fish 設定，也不需在 macOS 安裝 ROS。

```fish
git clone https://github.com/2006wu/Almighty-shopping-car.git
cd Almighty-shopping-car
git switch develop
cp .env.example .env
set -gx VNC_PASSWORD myshop42
./scripts/dev.sh build compute display
./scripts/dev.sh colcon-build
./scripts/dev.sh test
./scripts/dev.sh up
./scripts/dev.sh status
./scripts/dev.sh mapping
```

另開第二個終端機啟動 RViz；這個指令會維持前景執行：

```fish
./scripts/dev.sh rviz
```

在第三個終端機查詢實際瀏覽器位址：

```fish
./scripts/dev.sh novnc
```

開啟指令顯示的位址（預設為 <http://localhost:6080/vnc.html>），輸入相同 VNC 密碼。RViz 預設固定座標為 `odom`，純跟隨也可顯示；使用 Nav2 時請在 Global Options 將 Fixed Frame 切成 `map`。

若連接埠已有其他專案使用，修改根目錄 `.env` 的 `NOVNC_HOST_PORT` 與 `VNC_HOST_PORT`，例如 `6081`／`5970`，再執行 `dev.sh up`。`dev.sh` 從任何工作目錄都會載入這份檔案；服務啟動後，`dev.sh novnc` 顯示實際連接位址。Linux Bash 使用相同指令，將 `set -gx VNC_PASSWORD myshop42` 改為 `export VNC_PASSWORD=myshop42`。一般開發不需建置選用的 micro-ROS C client，只有 `comm-e2e-up` 才會啟用。

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

巡航、定位檢查、`goto`、`/shopping/goto_zone`、跟隨與鍵盤控制必須互斥操作；尚無模式管理器或速度仲裁。`map → odom` 在每次啟動時只由所選 slam_toolbox 或 AMCL 發布。定位用解析度、雷射上限及座標框架與 mapper 一致；單獨的 slam_toolbox launch 保留 `map_start_at_dock: true`；情境 bringup 則以選定起點的 `map_start_pose` 覆寫定位起點（[Humble 實作](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)）。AMCL 由情境初始化節點發布選定的世界／標準地圖起點（default 為 `(0,0,0)`），檢查工具的 `publish_initial_pose` 可在移動前再次發布相同起點。本次未調整控制器與規劃器，也尚未執行這套巡航及兩種離線定位的 ROS／Gazebo 驗收。

使用前述 `guide` 流程，等終端機出現 `Managed nodes are active` 後，再從另一個終端機送出目標：

```fish
./scripts/dev.sh zones
./scripts/dev.sh goto 飲料區
./scripts/dev.sh goto 生鮮區 --timeout 300
./scripts/dev.sh cancel
```

操作期限以壁鐘時間計算；本次 Mac 實測曾在 120 秒上限逾時，因此範例保留預設 300 秒。模擬速度與路徑會影響所需時間，應依實測調整；較長期限不取代抵達位置與安全檢查。

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

檢查工具會實際移動模擬機器人，請先結束既有導航，保持常駐節點閒置，並互斥使用鍵盤、跟隨與其他導航客戶端。工具使用同一套區域節點程式，但不重複建立指令訂閱與取消服務，依序檢查未知區域不送 action、不可達目標（`unreachable goal`）遭 `ABORTED`／`REJECTED`、遠端目標有至少 0.15 公尺進展後取消、取消後 2 秒內收到 `CANCELED` 且新鮮 `/odom` 速度接近零（各平面分量低於 0.02）、有效目標成功，以及第二個目標回覆 `BUSY`。最後印出 PASS／FAIL 表，任何失敗皆回傳非零；無法取消舊目標時停止後續案例。不可達目標使用 map 座標 `(50, 50)`，Smac2D 容許值讀取安裝的 `shopping_robot_navigation/config/nav2_params.yaml`；送出前從安裝的 `supermarket.sdf` 牆面盒體取得商店外框，並讀取 `/global_costmap/costmap`，確認目標距離兩個外框皆超過「容許值＋1 公尺」。成本地圖外框涵蓋所有格子，因此也保證距離每個自由格子超過此門檻；缺少成本地圖、座標框架或幾何假設不符時回報失敗，不送出此目標。單項等待保留 `--case-timeout`（預設 120 秒），逾時後要求取消。這是待容器實測的工具，離線測試不代表模擬驗收。

`--tolerance-check` 預設關閉，啟用後增加第六項 `zone tolerance`：送出一個正常區域目標，須收到 `SUCCEEDED`，再於 5 秒內取得結果之後接收的新鮮 `/odom`，計算與區域座標的平面距離。`--max-final-error` 預設 0.35 公尺，超標即失敗；PASS 與超標 FAIL 的表格都列出實測誤差與門檻，用來檢查共用 Smac2D 0.8 公尺容許值是否使導覽提早停止。此量測直接將 odom 與 map 座標比較，僅適用模擬初始原點與朝向對齊且漂移可忽略的情況；不補償 AMCL／SLAM 的 map→odom 修正，不能當作一般定位精度驗收。

停止其他移動流程後測試 UWB 跟隨（包含建圖與 Nav2 繞行）：

```fish
./scripts/dev.sh follow
```

`sim.launch.py`、`mapping.launch.py` 與 `guide.launch.py` 預設 `person:=false`：人物停放在店外 `(20, 20)`，不啟動路徑節點，避免人物碰撞幾何寫入地圖或擋住導航。需要動態障礙測試時可明確啟用，例如 `ros2 launch shopping_robot_bringup guide.launch.py person:=true`。`follow.launch.py` 預設 `slam_mode:=mapping`、`map:=`（空值）、`route_around:=true`，會啟動 slam_toolbox 與 Nav2，但不啟動區域導航節點。使用已存地圖時，與引導模式相同，可設 `slam_mode:=localization map:=/maps/store`（slam_toolbox 序列化地圖）或 `slam_mode:=map_server map:=/maps/store.yaml`（AMCL）。例如 `ros2 launch shopping_robot_bringup follow.launch.py route_around:=false` 可恢復原本直接跟隨控制，節點不建立動作客戶端；launch 仍會啟動導航服務。`follow.launch.py` 固定傳入 `person:=true`，人物路徑從出生點正前方約 1.5 公尺的 `(1.5, 0)` 開始，首段沿前方橫向通道向 +x 移動。

人物是本機圓柱幾何模型，不需連線下載 Fuel 素材。路徑唯一來源為 `shopping_robot_gazebo/config/person_path.yaml`；模擬人物與 UWB 使用相同模擬時間插值。人物會循環走過三條走道；測試直接解析 SDF 碰撞盒，驗證區域有 0.55 公尺淨空、人物整條路徑有 0.30 公尺淨空。`uwb_follow.yaml` 可調整噪聲、偏差、NLOS、掉包及錨點陣列；兩錨點實驗須同時縮短 `anchor_x/anchor_y`，並將車體 xacro 的 `anchor_count` 改成 2 或以 xacro 參數產生相符模型。錨點移位時也需同步修改 xacro 的錨點屬性。這是平面測距模型，尚未模擬高度差與完整無線電傳播。


## 更多操作說明

里程計比較、模式管理、鏡像、連接埠、故障注入、安全狀態、rosbag 報告、MCU 通訊、Nav2 fork 對照與模擬情境等細節已移至 [docs/USAGE.md](docs/USAGE.md)；文件索引見 [docs/README.md](docs/README.md)。
