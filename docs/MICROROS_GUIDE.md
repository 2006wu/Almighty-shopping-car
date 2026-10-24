# 從 rosserial 到 micro-ROS：模擬優先入門

本文件對象是曾用 rosserial 連接 STM32 的使用者。目前只驗證純 Python 邏輯與靜態配置；沒有執行容器、MCU 韌體、燒錄或實車測試。ROS 2 固定 Humble、Ubuntu 22.04，模擬固定 Gazebo Fortress／ros_gz。

## 1. 先對照熟悉的元件

| 以前的經驗 | micro-ROS 對應 | 本專案的位置 |
| --- | --- | --- |
| MCU 的 rosserial client | rcl／rclc 與 Micro XRCE-DDS client | 未實作板端韌體 |
| PC `serial_node.py`／rosserial_server | micro-ROS agent，代理 MCU 加入 DDS | compose 的 `microros_agent` |
| ROS 1 master 與 topic | ROS 2 DDS discovery、domain、QoS | 既有 compute 容器 |
| 自訂 UART 字串 bridge | 可另用固定二進位備援協定 | `serial_bridge`，與 agent 無關 |
| 暫時沒有板子 | PC 模擬韌體取代控制與回報邏輯 | `sim_mcu_firmware`，直接使用 DDS |

agent 是 XRCE client 與 DDS 網路之間的代理；它不會自動產生輪速、操作馬達或替 MCU 做逾時停止。[官方 Humble agent 說明](https://github.com/micro-ROS/micro-ROS-Agent/tree/humble)。

參考資料只讀取舊競賽 repo 的 PC 端：主要是 `tdk_peace_and_joy/catkin_ws/src/stm32_bridge.py`、`docker/bridge.yml` 與 `compose.yml`，另對照 `tdk_communication` 的 rosserial launch 與 `tdk_machine_v1` 的 compose。當時的 topic 清單混有任務命令，`stm32_bridge.py` 本身是直接開 serial 的 ROS 2 字串／整數解析器；不能把它當作已找到 MCU 韌體或可直接沿用的 micro-ROS 實作。本次不沿用舊版預設開 `/dev/ttyACM0`、掛整個 `/dev` 或 privileged 的方式。

## 2. 沒有板子也能跑：獨立通訊模擬

於 repo 根目錄執行（這些是維護者要執行的容器驗收指令）：

```bash
./scripts/dev.sh up
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh comm-sim
```

第二個終端：

```bash
./scripts/dev.sh comm-check
```

`comm-sim` 使用 `standalone:=true use_sim_time:=false`，啟動模擬韌體、PC monitor、既有 mux 與 watchdog，沒有 Gazebo 也可運作。`comm-check` 會短暫要求 teleop、注入 0.2 m/s 命令與斷線／急停，最後保持 ESTOP。它只對獨立通訊模擬使用；若 `/cmd_vel` 不只有 watchdog 一個發布者、接收者不只有模擬韌體或 loopback bridge，或沒有模擬標記，會 FAIL 並拒絕注入。

通過表包含：graph 隔離、雙向心跳 counter 與 wheel rate、速度經 mux 抵達、命令逾時零速、命令恢復、斷線急停請求、連線恢復仍停車、MCU 按鈕鎖定、按住拒絕 reset、明確 reset 後恢復。任一項失敗 exit code 1。這是 ROS 層驗收，與本機 pytest 不同；目前結果待維護者執行。故障條件用單調壁鐘計時，暫停模擬時間仍能停車。

獨立模擬停止後可改測相同協定的 PTY loopback：

```bash
./scripts/dev.sh comm-sim transport:=serial_loopback
# 第二個終端
./scripts/dev.sh comm-check
```

PTY 用 `pty.openpty()` 動態建立，兩端都在程序內，不指定或掃描 `/dev/tty*`，不需要 pyserial。這個模式真的編解碼 CRC 訊框，再由模擬韌體回覆；不是直接把 ROS 訊息回傳。

整合既有模擬時另開一個流程即可：

```bash
./scripts/dev.sh follow mcu_comm:=true
# 或
./scripts/dev.sh guide mcu_comm:=true
```

兩者預設 `mcu_comm:=false`，既有行為不變；啟用時沿用該流程的 `use_sim_time`，不重複啟動 mux。維持 `use_mux:=true`。此時不可再啟動 standalone comm-sim，也不要執行會注入運動的 comm-check。MCU 遙測仍獨立於 Gazebo，輪模型不驅動 Gazebo 車體，也不能當作 Gazebo 真值驗證。

## 3. 啟動 agent 容器

```bash
./scripts/dev.sh comm-up
# 等價指令
# docker compose -f docker/docker-compose.yaml --profile comm up -d microros_agent
# 查看日誌
docker compose -f docker/docker-compose.yaml --profile comm logs -f microros_agent
# 停止 agent，不停止 compute/display
./scripts/dev.sh comm-down
```

預設 `dev.sh up` 仍只啟動 compute/display。agent 使用官方映像 `microros/micro-ros-agent:humble`、`udp4 -p 8888`，同一 compose `ros` 網路與 ROS_DOMAIN_ID（預設 28），無 host port、裝置掛載或固定 `--platform`。[官方 PlatformIO agent 指令](https://github.com/micro-ROS/micro_ros_platformio#using-the-micro-ros-agent) 使用相同 image 命名與 ROS 發行版 tag；本次沒有 pull 映像或驗證 registry manifest／amd64、arm64 實際啟動，須於兩種原生主機確認，不能把設定無 `--platform` 說成已通過雙架構驗收。

目前的 sim_ros 與 serial_loopback **都沒有經過 agent**。前者是一般 rclpy／DDS 節點，後者是自訂 serial codec；即使 agent 已啟動，也不代表 XRCE-DDS 已驗證。ROADMAP 的「模擬韌體透過 agent」項目仍待後續真正 client 驗證，這次不勾選。

要驗證 agent，下一步可依 [官方 Linux host client 教學原始文件](https://github.com/micro-ROS/micro-ROS.github.io/blob/master/_docs/tutorials/core/first_application_linux/index.md)，在另一個測試環境以 `micro_ros_setup` 建立 host／Linux client，或使用已確認的 MCU 板；這次不下載或實作該 client。client 須連到 agent 可達 IP:8888；同一 compose 網路的 Linux client 可解析 `microros_agent`，板子通常無法解析容器 DNS。外部 UDP client 需要另行明確 opt-in 的 UDP port mapping、主機 IP 與防火牆配置，目前預設沒有對外入口。

## 4. 之後才選 STM32 開發路徑

先確認完整晶片與開發板編號、Flash／RAM、UART／USB 腳位、時鐘、RTOS、encoder 與馬達控制責任，再決定工具鏈。已知用過 H7 與 F446（可能 F446RE）；「451d」為**待確認**，不能據此選 linker script 或 SDK。

| 選項 | 適用方向 | 限制與待確認 |
| --- | --- | --- |
| STM32CubeIDE／CubeMX + micro_ros_stm32cubemx_utils | 要自己控制 DMA、FreeRTOS、HAL 的 STM32 專案 | 本專案未建板端專案，須鎖 Humble 分支與編譯器／FPU ABI |
| PlatformIO + micro_ros_platformio | 板型在其支援清單、工具鏈已成熟時 | 必須明確 `board_microros_distro = humble`；不能假設所有 STM32／F446RE 都支援 |

CubeMX 路徑概要：

1. 以實際晶片建立 CubeMX／CubeIDE 專案，先讓 RTOS、時鐘與通訊介面單獨穩定工作。本專案的 STM32 方案預計用 FreeRTOS，把通訊 executor 與固定週期底盤控制／watchdog 分開；micro-ROS 並非在所有平台都強制需要 FreeRTOS。
2. 依 [Humble CubeMX 工具文件](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble) 產生對應架構的靜態 libmicroros.a，加入 include、allocator、time source 與 UART DMA／interrupt 或 USB-CDC transport。官方建議 FreeRTOS 的 micro-ROS task stack 大於 10 kB；這只是起點，仍需量測 stack high-water mark。
3. 以最小 heartbeat publisher／subscriber 驗證連線，再加入 [MCU_INTERFACE.md](MCU_INTERFACE.md) 的端點與安全狀態機。MCU 內部控制迴圈必須獨立判定命令與 PC heartbeat 逾時，不能依賴 PC monitor 或 agent。
4. 初始化時配置 JointState 的四個 name 字串、position／velocity 陣列、String 容量及所有 executor handle；執行控制迴圈期間避免動態配置。標準訊息不代表沒有 sequence／string 記憶體管理需求。限制 XRCE history、MTU、session／publisher／subscriber 數量與 reliable queue，並量測最大序列化大小。
5. 上線前先於無馬達輸出的環境量測 heartbeat、重連、QoS、CPU／RAM 與串列吞吐，再另外審查實際底盤控制流程。本次沒有 PWM、GPIO、encoder 或 IMU 驅動。

H7 是系列而非單一容量，通常較有餘裕，但精確 RAM／Flash、DMA 可達 RAM 與 cache coherency 都要看具體型號；目前無法給本車 H7 的確定記憶體預算。F446RE 官方規格是最高 512 KB Flash、128 KB SRAM，micro-ROS、FreeRTOS、stack、DDS 訊息緩衝、HAL 與控制程式須共用，是否容得下本介面仍待編譯 map／實測。[ST F446RE 規格](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)。不要把 H7 成功案例直接套到 F446RE；PlatformIO 官方現行清單也不是 F446RE 的驗收證明。[PlatformIO 支援清單與配置](https://github.com/micro-ROS/micro_ros_platformio)。

UART 需要匹配 baud、8N1、電位與 GND；USB-CDC 是 USB device transport，與 ST-LINK 的虛擬 COM 或原生 UART 不同，不能因都顯示 ttyACM 就視為同一路徑。具體接法待板型確認。

## 5. 常見問題與重連

- 先開 agent 再啟動 client，方便觀察 session 建立；client 也應以 ping／重連狀態機處理 agent 尚未就緒、重啟、USB 拔除，不能永遠卡在初始化。典型做法是 `rmw_uros_ping_agent`、建立 entities、執行、失聯銷毀 entities 再等待；每次恢復都維持零命令，重新取得新鮮 heartbeat／cmd 才允許移動。
- 沒有 topic：先確認 agent session 日誌、相同 domain、網路路由與 DDS discovery；不能只看容器 alive。跨容器使用 UDP transport 配置，不能假設 shared memory 可跨 namespace 使用。
- topic 有但收不到：檢查 offered／requested QoS。本介面高頻 wheel／IMU／tick 使用 best_effort，避免遙測重傳壅塞；command／heartbeat／急停使用 reliable，仍必須有時間期限。best_effort publisher 無法滿足 reliable subscriber。急停保留值需 transient_local 相容，MCU 端也須確認所選 rmw micro-ROS 版本支援方式。
- 大訊息／低 RAM：標準 Imu 的 covariance 陣列也會增加序列化尺寸；核對 MTU 與分片／history 設定，不要以 Python 節點能送就推定 MCU 能送。減少字串容量、topic 數量、queue 與 rate 時須同步規格，不能私自改 message type。
- 串列 agent 掉包：官方 Humble agent 文件列有 Docker serial 掉包限制；需實測版本、baud、USB 與排程。[Humble agent 已知限制](https://github.com/micro-ROS/micro-ROS-Agent/tree/humble#known-issueslimitations)。

## 6. 明確選用實體串列（未驗證，預設不開）

兩條路互斥：micro-ROS MCU 使用 XRCE agent；純 serial MCU 使用本專案 codec／serial_bridge。不能同時開啟同一埠，也不能把本專案二進位訊框送 agent。

micro-ROS serial agent 的範本是 [docker-compose.serial.yaml](../docker/docker-compose.serial.yaml)。只有使用者確認硬體與埠後，複製成自己的 override，取消該單一 devices mapping 註解，再明確執行：

```bash
docker compose -f docker/docker-compose.yaml -f /path/to/confirmed-serial.yaml --profile comm up -d microros_agent
```

範本 command 為 `serial --dev /dev/ttyACM0 -b 115200`；直接使用原範本沒有 devices，預期無法連到裝置。勿掛整個 `/dev`、勿 privileged、勿自動偵測。這是未來硬體選用步驟，目前開發仍用 sim_ros／PTY。

如果 MCU 記憶體、RTOS 整合或 micro-ROS transport 支援不合適，可選用固定 binary fallback。它較小、容易在舊韌體實作，但 MCU 需自行維護 codec／CRC、資料與安全狀態機，沒有 DDS discovery。真正連到板子的 bridge 必須同時設定 **`use_hardware:=true` 與非空明確 `port`**；預設 false／空字串，任一條件缺少就拒絕開埠。該程序還需在自己的 opt-in 容器 override 映射單一已確認的 port，並由維護者預先提供 pyserial（套件不自動安裝；只有硬體分支延遲匯入）。範例僅供未來確認硬體後使用：

```bash
ros2 run shopping_mcu_bridge serial_bridge --ros-args -p use_hardware:=true -p port:=/dev/ttyACM0 -p baud:=115200 -p use_sim_time:=false
ros2 run shopping_mcu_bridge mcu_link_monitor --ros-args -p estop_on_link_loss:=true -p use_sim_time:=false
```

必須另有既有 mux／watchdog，且 MCU 自行實作逾時停車；此模式不會啟動模擬 peer。連線失敗會保持 LINK_LOST，由 monitor 請求急停，不自動重開其他裝置。上述不是實車操作授權或硬體驗收紀錄。板端韌體、電氣急停與實際馬達控制仍未實作。
