# 實車介面準備

本文件定義 2027 年 2 月後可採用的介面，不代表驅動、實車或安全驗收已完成。目前只使用 ROS 2 Humble、Ubuntu 22.04、Gazebo Fortress 與 `ros_gz`；不綁定板卡、SDK、CPU 架構或裝置名稱。新增入口預設全部模擬，不開啟裝置，不偵測硬體。

## Topic、座標與資料責任

以下頻率為介面目標，模擬感測頻率依 ROS 模擬時間；MCU 的發布排程與安全期限依單調壁鐘。[MCU 契約](../ros2_ws/src/shopping_mcu_bridge/config/interface.yaml) 是 MCU topic、輪序、頻率與 QoS 的唯一來源。QoS 格式為 reliability／keep_last depth／durability；未列 durability 時為 volatile。未來驅動必須滿足既有訂閱者的 QoS，不能直接將 reliable 訂閱者接到 best_effort 發布者。

| 介面與 topic | 型別 | frame／單位 | Hz；QoS | 模擬發布者 → 未來實車發布者 |
| --- | --- | --- | --- | --- |
| 2D 雷射 `/scan` | `sensor_msgs/msg/LaserScan` | `laser_link`；m、rad、s | 10；reliable／10 | Fortress lidar 經 ros_gz → 明確選用的雷射驅動／正規化層 |
| 原始 IMU `/imu` | `sensor_msgs/msg/Imu` | `imu_link`；rad/s、m/s² | 50；reliable／10 | Fortress IMU 經 ros_gz → 可選原始輸出；實車 EKF 使用下列正規化輸出 |
| EKF IMU `/imu/with_covariance` | `sensor_msgs/msg/Imu` | `imu_link`；rad/s、m/s² | 50；reliable／10 | `wheel_odom_noise` → IMU 驅動的正規化層 |
| MCU IMU `/mcu/imu` | `sensor_msgs/msg/Imu` | `imu_link`；rad/s、m/s² | 50；best_effort／5 | 模擬韌體／loopback bridge（須另選用）→ MCU／通訊 bridge |
| 舊 UWB `/uwb/ranges` | `std_msgs/msg/Float32MultiArray` | 無 header；依 anchor_x/y 順序、平面 m | 10；reliable／10 | `uwb_sim_node` → 不供實車直接接入 |
| 新 UWB `/uwb/ranges_stamped` | `shopping_interfaces/msg/UwbRangeArray` | 批次 `base_link`，每筆 header 為錨點 frame；斜距 m | 目標至少 10；reliable／10 | 本次未新增 ROS 發布者 → 未來 UWB 驅動／時間正規化層 |
| 錨點 `/uwb/anchors` | `shopping_interfaces/msg/UwbAnchorArray` | `base_link`；m | 啟動／校正變更；reliable／1／transient_local | 目前由 YAML 與 xacro 定義，未發布新訊息 → 校正資料發布者 |
| 輪狀態 `/mcu/wheel_state` | `sensor_msgs/msg/JointState` | `base_link`；輪角 rad、輪速 rad/s | 50；best_effort／5 | 模擬韌體／bridge → MCU／bridge |
| encoder `/mcu/encoder_ticks` | `std_msgs/msg/Int64MultiArray` | 無 header；契約輪序；有號累計 tick | 50；best_effort／5 | 模擬韌體／bridge → MCU／bridge |
| 原始輪式里程計 `/wheel_odom` | `nav_msgs/msg/Odometry` | pose `odom`、child/twist `base_footprint`；m、rad、m/s、rad/s | 50；reliable／10 | Fortress DiffDrive 經 ros_gz → 未來輪式里程計轉接層；不得發布 TF |
| EKF 輪速輸入 `/wheel_odom_noisy` | `nav_msgs/msg/Odometry` | 同上；含量測 covariance | 50；reliable／10 | `wheel_odom_noise` → 輪式里程計轉接層的相容輸出（名稱暫保留） |
| 融合里程計 `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_footprint` | 40；reliable／10 | EKF → 同一 EKF；模擬 ground_truth 替代模式為 50 Hz |
| 底盤命令 `/cmd_vel` | `geometry_msgs/msg/Twist` | 無 header；約定 `base_footprint`；m/s、rad/s | 20；reliable／1 | **唯一發布者 `cmd_vel_watchdog`**；ros_gz 或未來 MCU bridge 只訂閱 |
| 急停 `/mcu/estop` | `std_msgs/msg/Bool` | 無 frame；true 表示鎖定 | 10；reliable／1／transient_local | 模擬韌體／bridge → MCU／bridge |
| 復歸 `/mcu/reset_estop` | `std_msgs/msg/Bool` | 無 frame；明確要求 | 事件；reliable／1 | PC 操作端 → 同一操作端；不是自動復歸 |
| 雙向心跳 `/mcu/heartbeat`、`/pc/heartbeat` | `std_msgs/msg/UInt32` | 無 frame；遞增 counter | 各 10；reliable／1 | 韌體／PC monitor → MCU／PC monitor |
| 通訊狀態 `/mcu/status` | `std_msgs/msg/String` | 無 frame；契約列舉 | 10；reliable／1 | 模擬韌體／bridge → MCU／bridge |
| 輪關節 `/joint_states` | `sensor_msgs/msg/JointState` | `base_link`；rad、rad/s | 目標 50；reliable／10 | Fortress 經 ros_gz（實際依模擬步進）→ 輪狀態轉接層 |

MCU 輪序為 `front_left_joint, rear_left_joint, front_right_joint, rear_right_joint`。position、velocity 各四筆，effort 留空；tick 不能直接作為 rad。0.10 m 輪半徑、0.92 m 模擬有效輪距與 4096 ticks/rev 都須實車重校，不能將滑移補償係數當實體幾何。既有通訊模擬不產生 EKF 輪速輸入；`use_real_base` placeholder 要求未來套件同時提供輪速轉接、關節狀態、雙向 heartbeat 與急停整合。若使用 MCU IMU，由 `use_real_imu` 所選的單一轉接層提供 EKF 輸入，避免另外啟用第二個 IMU 正規化層。

### 安裝與 covariance

xacro 的 `base_footprint → base_link` 為 z=0.20 m。雷射相對 base_link 為 `(0.18, 0, 0.16)` m，水平安裝、x 向前、y 向左、z 向上；掃描角為 −π 至 π，720 點、10 Hz，範圍 0.12–12 m，解析度 0.01 m。實體安裝變更須修改同一 xacro 並校正，不讓驅動重複發布安裝 TF。LaserScan 的 stamp 應表示第一束量測時刻；設定正確 angle_increment、time_increment、scan_time，超出範圍與無回波不可偽造成近距離零值。跟隨的雷射過期保護為 0.5 秒。

IMU 相對 base_link 為 `(0, 0, 0.05)` m；資料先轉到 `imu_link` 軸向。covariance 為各自單位的平方，有限、對稱、半正定；已提供量測的對角須為經量測的正值。未知 covariance 不可用零宣稱精確；orientation 不可用時依 Imu 慣例將 `orientation_covariance[0]` 設為 −1。目前模擬角速度對角為 0.005²，加速度為 0.05²，僅是模擬值。實車須校正 bias、溫漂與噪聲，不能直接複製。六軸 IMU 無絕對 yaw 依據，EKF 僅融合 angular_velocity.z；不融合 orientation 與加速度，也不重複融合輪式積分 pose。現有 serial 契約只攜帶 yaw rate，不能宣稱具有完整實體 Imu 的量測與同步能力。

### 命令與急停

Nav2、跟隨、teleop 必須經既有 mux 與 watchdog；硬體入口固定 `use_mux:=true`、`initial_mode:=idle`，不啟動任何移動來源。未來驅動只能訂閱 `/cmd_vel`，不得從 `/cmd_vel/nav` 等上游繞過 watchdog，也不得新增 `/cmd_vel` publisher。實體 MCU 仍須獨立以單調時鐘檢查 0.3 秒命令逾時及 0.5 秒 PC heartbeat 逾時，並在失聯、急停、無效命令時立即零輸出。一般命令限制速度與加速度、預設不倒車；立即停止不可受一般減速斜率拖延。跟隨遇過近、遺失、UWB 過期、障礙或雷射失效均須零 Twist。

硬體急停應在 MCU／電氣端直接停止動力，ROS Bool 只回報其狀態；PC monitor 另外向模式管理器請求 estop。按鈕放開不等於解鎖，必須確認新鮮 heartbeat、明確復歸且重新取得新命令。沒有本機驅動、電氣接線與實車測試，不能將上述設計視為急停已驗收。

## TF 與時鐘所有權

```text
map --slam_toolbox 或 AMCL（互斥）--> odom
odom --robot_localization EKF--> base_footprint
base_footprint --robot_state_publisher--> base_link
base_link --robot_state_publisher--> laser_link / imu_link / uwb_anchor_0 / uwb_anchor_1 / uwb_anchor_2
base_link --robot_state_publisher + joint_states--> 四輪 link
```

模擬與實車預設都由 EKF 唯一發布 `odom → base_footprint`。輪速轉接、MCU、DiffDrive 不發布這條 TF，輪式 TF 不橋接。僅純模擬 `odom_source:=ground_truth` 改由 Fortress OdometryPublisher 發布 `/odom` 與該 TF，並停用 EKF；`hardware.launch.py` 固定 ekf，不能用真值代替實車里程計。硬體選項與 ground_truth 或模擬故障發布者不可搭配；launch 會拒絕，避免真值 TF 或故障轉送造成重複來源。硬體入口固定關閉 fault_injection。診斷 `/sim/ground_truth_odom` 為 `sim_world → base_footprint_gt`，其 TF 不橋接，也不接入實車 TF 樹。混合介面測試時，模擬車體不跟隨實體底盤，其真值不能拿來評估實車 EKF 漂移。

所有節點與未來 driver launch 都須傳遞一致的 `use_sim_time`。入口預設 true，使用 Gazebo `/clock`；未來純實車應使用 false，且不以 Gazebo 真值或時鐘充當量測來源。目前此入口仍會啟動 Gazebo 作介面測試，並非完整的無 Gazebo 實車 bringup。

## 預設關閉的啟動與擴充

`shopping_robot_bringup/launch/hardware.launch.py` 提供 `use_real_lidar`、`use_real_imu`、`use_real_uwb`、`use_real_base`，預設全部 false。每個 true 只會 include 對應的 `*_driver_placeholder.launch.py`。placeholder 的 `driver_package` 與 `driver_launch` 都預設空；未給完整配對時 LogError，完全不啟動驅動，也不回退成模擬來源。此行為是安全的缺資料狀態，不是驗收通過。

| 選項為 true | 被關閉的模擬 ROS 來源 | 未來驅動責任 |
| --- | --- | --- |
| `use_real_lidar` | `lidar_bridge`，不再橋接 `/scan` | 發布 `/scan`，不發布安裝 TF |
| `use_real_imu` | `imu_bridge` 及模擬 IMU 正規化 publisher/subscriber | 發布 `/imu/with_covariance`；可另發原始量測 |
| `use_real_uwb` | `uwb_sim_node` 與舊 `uwb_estimator_node` | 新 stamped ranges；待遷移的新估測器，不直接進舊估測器 |
| `use_real_base` | `base_bridge`（`/wheel_odom`、`/joint_states`、送 Gazebo 的 `/cmd_vel`）及模擬輪速噪聲 publisher/subscriber | MCU 命令接收、遙測、輪速／關節轉接、安全與心跳；不發布 odom TF |

Gazebo 內部感測器仍可運作，但相應 bridge 不建立，所以上述正式 ROS topic 無模擬 publisher；不使用重新命名的模擬資料填補實體缺訊。`bridge_common.yaml` 保留 clock、診斷真值與模型姿態；四個分檔與原始 `bridge.yaml` 的內容一致性由靜態檢查核對。選項可獨立組合，未選用部分繼續模擬。缺少實車來源時 EKF 可能只憑另一個感測器推進，不能視為導航就緒。

未來套件須在自己的 package.xml 宣告依賴 `shopping_interfaces`（若使用新訊息）及所需 ROS 套件，將 launch 安裝在 share/<package>/launch。使用 `<kind>_driver_package` 與 `<kind>_driver_launch` 明確指定，後者只能是套件 launch 目錄內的單一 `*.launch.py` 檔名。該檔必須接受 `use_sim_time`，自行提供所有必要的明確設定；本入口不注入埠號、不傳硬體啟用參數、不掃描裝置。將來驅動若需 `use_hardware`，應維持預設 false，另經專屬設定明確選用；不能因被 include 或發現裝置而自行開啟。

例如未來已審查的雷射套件，可明確傳入 `use_real_lidar:=true lidar_driver_package:=<套件> lidar_driver_launch:=<已審查檔名>.launch.py`；目前沒有提供這個驅動。僅給 true、不指定套件的測試安全且不開裝置。新增入口不啟動 MCU 通訊模擬，guide／follow 暫不轉送硬體選項；避免在尚未遷移 UWB 與實車輪速前啟用移動流程。既有 guide／follow 模擬預設不變，請分開執行，不能與 hardware 入口同時啟動第二組模擬或 watchdog。

## UWB 訊息與有效性

新 `shopping_interfaces` 是獨立 ament_cmake／rosidl 套件，只依賴 std_msgs、geometry_msgs 與 rosidl，不依賴板卡 SDK。`UwbRange` 每筆保留量測時間、錨點／標籤 ID、斜距 range_m、range_std_m、quality、valid、reason、nlos 與 nlos_known。quality 為 0–1 診斷分數，−1 表示未知，不是統計機率；未知標準差為 −1。`nlos_known=false` 時不能將 `nlos=false` 解讀為 LOS 保證。

`UwbRangeArray.header` 是批次參考時間與共用配置 frame；每筆 header 是實際量測時間與錨點 frame。不同時間的樣本不能因裝在同一陣列就當作同步。錨點 `UwbAnchor.header.frame_id` 為位置的父框架，`frame_id` 為錨點自身框架，`position` 為三維安裝位置，`height_m` 為相對校正地面的高度；兩種 z 參考不可混淆。Array header 為本批校正版本的時間與共同父框架，陣列內 ID 必須唯一，所有發布者使用同一份校正資料，不另外發布重複 TF。

reason 列舉為 OK=0、DROPOUT=1、NONFINITE=2、OUT_OF_RANGE=3、STALE=4、CLOCK_UNSYNCED=5、NLOS=6、UNCALIBRATED=7。valid=true 必須 reason=OK，且 ID 已知、量測有限且在模組已校正範圍、時間同步可信、未過期、校正有效。初期採保守策略，已知 NLOS 設 valid=false、reason=NLOS；無 NLOS 分類不代表排除了多重路徑。未知 ID／高度設定視為 UNCALIBRATED。驅動提供可調且明確記錄的有效距離上下限，不把非法數值裁成有效距離。

估測器須以量測 stamp 拒絕未來資料、非遞增／重複量測與超過 0.5 秒的舊資料，不可以接收時刻刷新舊量測。批次內錨點時間差初始上限建議 0.05 秒，須實測再調；只在量測時間完成對齊後解算，不能讓異步距離形成假的位置。無效筆保留原始距離供診斷，但控制與估測不得使用。

### 時鐘同步

模組 tick 必須先展開 rollover，分辨重啟與倒退，再以 `t_ros = a * t_module + b` 映射到選定 ROS 時鐘。透過雙向請求／回覆記錄 PC 發送、接收與模組時間，用短 RTT 樣本估測 offset b，較長視窗估測 drift a，保留 RTT／殘差／同步不確定度；單向接收時間包含傳輸延遲，不能宣稱精確量測時間。多錨點模組必須各自映射到同一時間基準。

啟動、重啟、ROS 時間倒退或同步誤差超出預算時，清空估測與濾波歷史，標記 CLOCK_UNSYNCED，重新收集同步樣本，不沿用舊 offset。以 0.05 秒批次時間預算為起點，尚未實測同步精度；如果模組沒有時間戳，先記接收時間與此限制，禁止假裝完成時鐘同步。原始 tick、RTT、a/b 與不確定度可另存診斷／記錄，不擠入 range_m。模擬直接使用同一 ROS 模擬時鐘，不能將 wall time 與 `/clock` 混用。

### 高度與幾何校正

xacro 錨點相對 base_link 為 `(0, ±0.22, 0.12)` 與 `(-0.25, 0, 0.12)` m；地面高度為 0.32 m。修改錨點時須同步 xacro 與 `uwb_follow.yaml` 的平面位置，並另記量測到的實際高度、天線相位中心與標籤配戴高度。人物路徑仍只使用 Gazebo 的 `config/person_path.yaml`。

實體量測是斜距 r。平面距離應為 `sqrt(r² − (h_tag − h_anchor)²)`；若根號內非正、標籤高度未知或校正超期則拒收，不能取絕對值或直接套現有平面解。近垂直幾何下高度誤差會放大；需要依 r 與高度的不確定度傳播 covariance，不能只沿用模組 range_std_m。先在多個已知距離／角度估計每錨點偏差與標準差，再測人體遮蔽、貨架 NLOS、掉訊與高度變化。

兩錨點有前後交點歧義；現有前半平面初始化與前次位置選解只適用既有配置／假設，不是一般定位保證。三錨點仍可能因幾何、掉訊與 NLOS 翻側；品質分數與低殘差都不能證明沒有 NLOS，見 [UWB_STUDY](UWB_STUDY.md)。

### 相容轉接與遷移順序

`shopping_uwb_follow/uwb_adapter.py` 的 `adapt_sim_ranges` 是無 ROS 匯入的純函式：輸入目前 `Float32MultiArray.data` 等價的數列、明確錨點 ID 與呼叫者提供的 ROS stamp，輸出與新訊息欄位同形的 dict。NaN 對應掉訊、Infinity 對應非有限、零／負值對應超界；不得壓縮掉訊筆而錯置 ID。它不會從陣列推算量測時間、NLOS 或品質，不知道的欄位保留未知值。

此轉接輸出仍是舊模擬的**平面距離**，僅供離線遷移測試；不得直接發布成聲稱真實斜距的新 topic。舊模擬未保留每筆 NLOS 判定與模組時鐘，adapter 無法還原；正有限值的 valid 只代表通過舊模擬數值規則，不宣稱通過實車同步／校正驗證。

後續先增加含高度與 stamp 的模擬資料及新 ROS 發布薄層，再實作具時鐘有效性／高度轉換的純估測轉接並測試，最後用新估測器替換 `/uwb/ranges` 訂閱，維持 `/uwb/target` 的 base_link 與量測時間語意。加入重啟、未來／過期 stamp、異步錨點、未知 ID、NLOS 與高度不確定度測試，再於 Fortress 重跑停止與指標流程；完成前不將實體 UWB 連到舊估測器，也不啟動真實跟隨。

## 維護者驗證指令

以下由維護者執行；本次沙盒只執行純 Python 與靜態檢查。compute Dockerfile 明列 `ros-humble-rosidl-default-generators`，需由維護者重建映像；本次沒有安裝套件或執行映像建置。新訊息以 rosidl 建置；現有 adapter 不匯入 generated messages，故沒有假造 runtime 相依。bringup 宣告 shopping_interfaces 依賴，colcon 依相依圖先建置介面；未來真正的訊息使用套件也必須宣告依賴。

```bash
# Repo root, no ROS required
/workspace/scratch/venv-shop/bin/python -m pytest -q
/workspace/scratch/venv-shop/bin/python scripts/check_static.py

# Maintainer host, build all packages including generated messages
./scripts/dev.sh build compute
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh up
./scripts/dev.sh shell compute
```

以下在該 compute shell 執行，先停止其他 launch。每次完成以 Ctrl-C 停止再換下一個案例，不並行啟動：

```bash
ros2 interface show shopping_interfaces/msg/UwbRange
ros2 interface show shopping_interfaces/msg/UwbRangeArray
ros2 interface show shopping_interfaces/msg/UwbAnchor
ros2 interface show shopping_interfaces/msg/UwbAnchorArray
xvfb-run -a -n 50 ros2 launch shopping_robot_bringup hardware.launch.py
# Stop, then verify missing-driver errors and absence of simulated sources
xvfb-run -a -n 50 ros2 launch shopping_robot_bringup hardware.launch.py use_real_lidar:=true use_real_imu:=true use_real_uwb:=true use_real_base:=true
```

另一個 compute shell 用 `ros2 topic info /cmd_vel --verbose` 確認發布者僅 watchdog；檢查 `/scan`、`/imu/with_covariance`、`/wheel_odom_noisy`、`/joint_states`、`/uwb/ranges` 的 publisher 數：全部 false 為各一，全部 true 且無驅動為零；topic 不存在亦符合缺資料預期。逐項開啟四個選項、檢查其他來源保留，並以 `ros2 run tf2_ros tf2_echo odom base_footprint` 檢查預設 EKF；缺少所有融合輸入時不可要求有有效 odom。用 `ros2 node info /ekf_filter_node` 與 `/tf` 的發布來源核對 TF 所有權，不能只憑 topic 存在認定正確。

停止 hardware 入口後，分別重新執行 `./scripts/dev.sh guide` 與 `./scripts/dev.sh follow` 驗證原本模擬仍能運作。這些檢查不能取代硬體急停、實車輪式轉接、DDS QoS 與 amd64／arm64 原生建置驗收。
