# MCU 通訊介面（版本 1）

本專案目前只開發模擬；2027 年 2 月前沒有實車。介面與 MCU 型號無關，僅使用 ROS 2 Humble 標準訊息，不需要自訂 msg 建置。

唯一規格來源是 [interface.yaml](../ros2_ws/src/shopping_mcu_bridge/config/interface.yaml)。節點直接讀取其 topic、型別、頻率、QoS、輪序、協定編號與時限；下表是該檔的文件呈現。修改介面時須同步本文與測試，不能另外建立互相矛盾的節點參數。

## Topic 規格

頻率單位為 Hz，事件表示收到才處理；安全計時與發布排程使用單調壁鐘。`use_sim_time` 只影響訊息 stamp，所有節點仍一致設定，暫停 `/clock` 也不會停用看門狗。QoS 欄依序為 reliability／depth／durability；兩端必須相容。

| Topic | 方向 | 訊息型別 | Hz | QoS | 單位／frame |
| --- | --- | --- | --- | --- | --- |
| `/cmd_vel` | PC → MCU | `geometry_msgs/msg/Twist` | 20 | reliable／1／volatile | m/s; rad/s／base_footprint |
| `/mcu/wheel_state` | MCU → PC | `sensor_msgs/msg/JointState` | 50 | best_effort／5／volatile | rad; rad/s／base_link |
| `/mcu/encoder_ticks` | MCU → PC | `std_msgs/msg/Int64MultiArray` | 50 | best_effort／5／volatile | ticks／wheel_joint_order |
| `/mcu/imu` | MCU → PC | `sensor_msgs/msg/Imu` | 50 | best_effort／5／volatile | rad/s; m/s^2／imu_link |
| `/mcu/estop` | MCU → PC | `std_msgs/msg/Bool` | 10 | reliable／1／transient_local | bool／none |
| `/mcu/heartbeat` | MCU → PC | `std_msgs/msg/UInt32` | 10 | reliable／1／volatile | counter／none |
| `/pc/heartbeat` | PC → MCU | `std_msgs/msg/UInt32` | 10 | reliable／1／volatile | counter／none |
| `/mcu/status` | MCU → PC | `std_msgs/msg/String` | 10 | reliable／1／volatile | enum／none |
| `/mcu/reset_estop` | PC → MCU | `std_msgs/msg/Bool` | 事件 | reliable／1／volatile | bool／none |
| `/mcu/sim/estop_button` | 僅模擬控制 | `std_msgs/msg/Bool` | 事件 | reliable／1／volatile | bool／none |
| `/mcu/sim/drop_link` | 僅模擬控制 | `std_msgs/msg/Float32` | 事件 | reliable／1／volatile | seconds／none |
| `/mcu/sim/drop_commands` | 僅模擬控制 | `std_msgs/msg/Float32` | 事件 | reliable／1／volatile | seconds／none |
| `/mcu/sim/active` | 僅模擬標記 | `std_msgs/msg/Bool` | 1 | reliable／1／transient_local | bool／none |
| `/mode_manager/request` | PC 本機 | `std_msgs/msg/String` | 事件 | reliable／10／volatile | enum／none |

`/cmd_vel` 的唯一發布者仍是既有 `cmd_vel_watchdog`；MCU／serial bridge 只訂閱，不得再加一個速度發布者。測試命令必須送 `/cmd_vel/teleop`，並經 `/mode_manager/request` 選用 teleop。Nav2、follow、teleop 依模式互斥。通訊 launch 不更動任何 Gazebo、EKF topic 或 TF，也不發布 `/odom`。

輪序固定為 `front_left_joint, rear_left_joint, front_right_joint, rear_right_joint`。JointState 的 position、velocity 各四筆，effort 留空；encoder ticks 依相同順序、帶正負號、從開機起累計，4096 ticks/rev 是模擬設定，實際 encoder 倍頻與減速比待確認。JointState 的位置已換算為輸出輪 rad，不能把原始 tick 直接填入。Int64MultiArray 的 layout 留空，順序由本規格定義；重啟後歸零，PC 必須處理不連續。

平面速度使用 linear.x（m/s，前方正）與 angular.z（rad/s，逆時針正），其餘四分量必須為零。IMU 的 angular_velocity.z 來自輪模型，orientation_covariance[0] 與 linear_acceleration_covariance[0] 為 -1，表示沒有這兩種估測；不把假的絕對航向或重力資料交給 EKF。角速度 covariance 對角為 0.001，僅模擬示意，不能當作實際感測器標定。

## 時序與安全

- MCU 開機為 LINK_LOST／零輪速。連續三次向前遞增的 PC heartbeat（10 Hz），且相鄰間隔小於 0.5 秒後才認定連線恢復。UInt32 以模 2^32 遞增；重複與倒序不延長期限。逾時後允許對端重新從任意 counter 開始，仍須三筆新值。
- 命令接收超過 0.3 秒未更新，狀態 CMD_TIMEOUT，立即設四輪速度為零。PC heartbeat 超過 0.5 秒或模擬斷線，狀態 LINK_LOST，立即零速；安全停止略過一般加速度限制。
- 連線恢復、MCU 急停重設後，都需要重新接收命令，先前非零命令不會重播。PC 模式管理器 ESTOP 不因連線恢復自動解除，須明確 reset 再選模式。
- MCU 急停按鈕 true 立即鎖定，false 只代表放開，不能解除鎖定；連線正常且按鈕已放開，再向 `/mcu/reset_estop` 送 true 才解除。此 topic 是指令，不採 transient_local，避免重播舊的 reset。MCU `/mcu/estop` 為保留最新值的 transient_local／reliable，重新訂閱者可取得鎖定狀態。serial bridge 重啟後須等 MCU 再回報，不能自行視為已解除。
- NaN、Inf 或非平面命令整筆拒絕並立即零速／INVALID_COMMAND；有限超界值飽和到 linear.x 0–0.6 m/s、angular.z ±1.2 rad/s。預設不倒車；轉彎仍可讓內側輪反轉。
- 一般運動採一階輪速響應（時間常數 0.12 秒），輪加速度上限 12 rad/s²。輪徑與有效輪距直接讀取 description xacro：0.10 m、(0.50−0.04)×2.0 = 0.92 m。這是與 Fortress 模型一致的 skid-steer 近似，不是實車滑移辨識或馬達控制器。
- PC monitor 自行執行時 `estop_on_link_loss=false`，comm launch 明確預設 true。MCU 急停、LINK_LOST、INVALID_COMMAND、heartbeat 或 status 過期會持續請求 `estop`；啟動連線有 0.5 秒寬限，但急停立即請求。CMD_TIMEOUT 本身只在 MCU 停車。monitor 不自行 reset、不發布速度。

`/mcu/status` 為有限字串：`LINK_LOST`、`ESTOP`、`CMD_TIMEOUT`、`INVALID_COMMAND`、`READY`。優先序為 ESTOP → LINK_LOST → INVALID_COMMAND → CMD_TIMEOUT → READY。READY 表示可執行新鮮命令，也可能執行零命令，不代表一定在移動。

模擬 fault topic 的 Float32 是持續秒數，接受 0–60 的有限值；超界忽略。`drop_link` 同時丟棄兩向輸入與 MCU 所有遙測；恢復後重走三筆 heartbeat。`drop_commands` 只丟 `/cmd_vel`，保留 heartbeat，用於驗證自然命令逾時。`/mcu/sim/*` 只用於模擬，不是板端契約；真實 serial 模式不建立這些端點。`/mode_manager/request` 是既有 PC 本機介面，送字串 `estop`，不傳到 MCU。

## 訊息與注入範例

以下僅用於獨立的 `dev.sh comm-sim`，不要同時開 Gazebo、follow 或 guide：

```bash
ros2 topic echo /mcu/wheel_state --qos-reliability best_effort
ros2 topic echo /mcu/estop --qos-durability transient_local
ros2 topic pub --once /mcu/sim/drop_link std_msgs/msg/Float32 '{data: 1.5}'
ros2 topic pub --once /mcu/sim/drop_commands std_msgs/msg/Float32 '{data: 1.5}'
ros2 topic pub --once /mcu/sim/estop_button std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /mcu/sim/estop_button std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /mcu/reset_estop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /mode_manager/request std_msgs/msg/String '{data: reset}'
```

JointState 範例（順序見上文）：`name: [front_left_joint, rear_left_joint, front_right_joint, rear_right_joint]`、`position: [1.0, 1.0, 1.0, 1.0]`、`velocity: [2.0, 2.0, 2.0, 2.0]`。輪速應與同一 frame 時刻的 tick 增量一致；目前跨 topic 沒有共同序號，不能聲稱硬體同步取樣。

## 純 serial 備援協定

此協定不是 XRCE-DDS，也不能交給 micro-ROS agent。使用固定起始位元組與固定長度欄位，便於小型 MCU 以狀態機解析，不依賴 COBS 函式庫。沒有 escaping；解碼器在未知 id、錯誤長度或 CRC 時逐 byte 重新搜尋，長度必須符合 id 的固定大小，避免損壞長度造成無限等待。適用可信任的本機連線，不提供認證或防重播。

訊框：`A5 | id:u8 | length:u8 | payload | crc:u16 little-endian`。CRC16-CCITT-FALSE：poly 0x1021、init 0xffff、不反射、xorout 0；涵蓋 id、length、payload，不含 A5／CRC。標準測試向量 `123456789 → 0x29B1`。所有數值 little-endian；`d` 為 IEEE754 float64、`f` 為 float32、`q` 為 int64、`I` 為 uint32、`B` 為 uint8、`?` 為一 byte bool（0 或 1）。

| id | 內容 | 固定 payload bytes |
| --- | --- | --- |
| 1 | Twist：linear xyz、angular xyz，6d | 48 |
| 2 | 四輪 position 接四輪 velocity，8d | 64 |
| 3 | 四輪 tick，4q | 32 |
| 4 | IMU yaw rate，d；其餘依本版固定未知值重建 | 8 |
| 5 | MCU estop，? | 1 |
| 6／7 | MCU／PC heartbeat，I | 4 |
| 8 | status，B，對照 YAML status_values 索引 0–4 | 1 |
| 9 | reset_estop，? | 1 |
| 10／11／12 | 僅模擬：按鈕 ?／斷線 f／命令丟棄 f | 1／4／4 |

header stamp 與 frame 由 PC 收到有效訊框時重建；這不是 MCU 時鐘同步。版本 1 IMU 只攜帶 yaw rate；未來真實六軸／九軸 IMU 若擴充須升版，不能悄悄變更 payload。CRC 錯誤完全丟棄、不刷新心跳。斷線不自動掃描或重新開啟埠；PC 由 monitor 請求急停，MCU 必須在自己的控制迴圈獨立實作相同逾時規則。

50 Hz wheel／tick／IMU 與 10 Hz 狀態心跳合計約 6.4 kB/s，加上下行指令與心跳約 7.6 kB/s；115200 baud 的 8N1 預算約 11.5 kB/s，仍須實測 USB／UART 排程，micro-ROS 的 XRCE overhead 另計。PTY 不代表實體線路吞吐量。

## 未來實車 EKF 接法（尚未啟用）

另建明確 opt-in 的硬體 launch：先將 `/mcu/wheel_state` 換算成帶 covariance 的輪式 Odometry，將 `/mcu/imu` 接到實車 EKF 輸入，停止 Gazebo 同類輸入。不能把 JointState 直接餵 robot_localization；須先處理重啟跳點、減速比、符號、校正、時間同步與失效 covariance。維持 EKF 唯一發布 odom → base_footprint，AMCL／slam_toolbox 二選一發布 map → odom，robot_state_publisher 負責車體 TF。這次沒有改 EKF、加入硬體 launch 或控制真實馬達。

## Linux XRCE-DDS 子集

`firmware/sim_client` 以 C／rclc 經 UDP agent 實作 command、兩向 heartbeat、wheel、status；常數由本契約與 description xacro 產生。沒有 encoder、IMU、MCU 按鈕鎖定／reset 或 `/mcu/estop`，因此不宣稱符合完整板端契約。只用於獨立、壁鐘時間的 `dev.sh comm-e2e-up`／`comm-e2e-check`，不能接入實車或當作 Gazebo 底盤控制。完整測試與限制見 [micro-ROS 入門](MICROROS_GUIDE.md)。
