# 從 rosserial 到 micro-ROS：模擬優先入門

本文件對象是曾用 rosserial 連接 STM32 的使用者。目前已有純 Python 通訊模擬及 Linux C／rclc XRCE 客戶端；本機檢查涵蓋純邏輯與靜態配置，尚未執行容器、ROS 通訊、板端韌體、燒錄或實車測試。ROS 2 固定 Humble、Ubuntu 22.04，模擬固定 Gazebo Fortress／ros_gz。

## 1. 先對照熟悉的元件

| 以前的經驗 | micro-ROS 對應 | 本專案的位置 |
| --- | --- | --- |
| MCU 的 rosserial client | rcl／rclc 與 Micro XRCE-DDS client | `firmware/sim_client`（Linux 模擬，尚無板端韌體） |
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
# 停止 client 與 agent，不停止 compute/display
./scripts/dev.sh comm-down
```

預設 `dev.sh up` 仍只啟動 compute/display。agent 使用官方映像 `microros/micro-ros-agent:humble`、`udp4 -p 8888`，同一 compose `ros` 網路與 ROS_DOMAIN_ID（預設 28），無 host port、裝置掛載或固定 `--platform`。[官方 PlatformIO agent 指令](https://github.com/micro-ROS/micro_ros_platformio#using-the-micro-ros-agent) 使用相同 image 命名與 ROS 發行版 tag；本次沒有 pull 映像或驗證 registry manifest／amd64、arm64 實際啟動，須於兩種原生主機確認，不能把設定無 `--platform` 說成已通過雙架構驗收。

`sim_ros` 與 `serial_loopback` 都沒有經過 agent。以下另有真正的 XRCE-DDS Linux client；新增實作與離線檢查不代表容器驗收已通過，ROADMAP 仍由審查者驗證後處理，本次不勾選。

### 3.1 不用板子的 XRCE-DDS 端到端驗收

先停止 Gazebo、follow、guide、`comm-sim` 與其他通訊 launch；同一 ROS domain 只執行這一組流程。於 repo 根目錄執行：

```bash
./scripts/dev.sh up
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh comm-e2e-up
```

`comm-e2e-up` 會建置並背景啟動 `microros_agent`、`microros_client`，接著於前景執行 `comm.launch.py standalone:=true transport:=agent use_sim_time:=false`，提供既有 mux、watchdog 與 PC monitor；此 transport 不啟動 Python 韌體。保留此終端，另開第二個終端：

```bash
./scripts/dev.sh comm-e2e-check
```

資料路徑如下：

```text
檢查器 → /cmd_vel/teleop → mux → watchdog（唯一 /cmd_vel 發布者）
                                       ↓ DDS
                              micro-ROS agent
                                       ↓ UDP XRCE-DDS
                         Linux C／rclc 模擬 client
                         ↓ wheel／heartbeat／status
                              agent → DDS → 檢查器
```

客戶端使用 `rmw_microxrcedds`，與 agent 共用 compute 網路命名空間，以 `127.0.0.1` 連 UDP 8888；沒有 host port、devices、主機網路或固定架構。`ROS_DOMAIN_ID` 會明確設入 rcl 初始化選項，預設與 compute／agent 同為 28。Linux client 只支援此獨立壁鐘流程，不能混入 Gazebo 的模擬時間。

檢查器最多等 10 秒探索 DDS；要求 XRCE 模擬標記、唯一 C 客戶端、唯一 mux／watchdog／monitor、相符的遙測發布者與唯一 `/cmd_vel` 接收者。若有 `/clock` 發布者、Gazebo、`sim_mcu_firmware` 或 `serial_bridge`，即 FAIL 並拒絕注入。注入期間也會重查 graph；請勿在檢查途中啟動其他流程。它不是實體裝置辨識或操作工具。

驗收列出七項 PASS／FAIL：

1. 獨立 graph 與 watchdog 唯一發布權。
2. 兩向 heartbeat counter 前進、約 10 Hz，wheel 約 50 Hz（容許 35–65 Hz）。
3. teleop 命令經 mux／watchdog 後輪子移動。
4. 模擬丟棄客戶端命令，超過 0.3 秒後 `CMD_TIMEOUT` 且持續零速。
5. 新鮮命令恢復後輪子移動。
6. 暫停 **PC monitor 的實際心跳發布**，超過 0.5 秒後 `LINK_LOST` 且持續零速。
7. 恢復 PC heartbeat 後仍為 ESTOP／零輪速，需要明確 reset 才能再移動。

心跳故障使用 monitor 的 `simulate_pc_heartbeat_loss` 參數；只有 standalone agent launch 明確啟用 `allow_simulation_faults` 才會生效，一般模式預設 false。結束或失敗時檢查器請求 ESTOP 並嘗試恢復心跳；任一項失敗 exit code 1。若檢查器被強制 kill，須停止並重啟前景 launch，以清除故障參數，不能直接沿用殘留狀態。

這個 client 是契約子集：沒有 encoder、IMU、MCU 按鈕／急停 latch／reset；這些仍由 Python 模擬流程驗證。此處驗證 PC 失聯、命令逾時與 PC ESTOP，不宣稱完整 MCU 急停韌體已完成。模型只回報輪速，不驅動 Gazebo 或真實車體。

停止時先 Ctrl-C 結束 `comm-e2e-up` 的前景 launch，再執行：

```bash
./scripts/dev.sh comm-down
docker compose -f docker/docker-compose.yaml --profile comm logs --tail=100 microros_agent microros_client
```

`comm-down` 會同時停止 client 與 agent。agent 重啟或尚未就緒時，client 可能先退出；Compose 會重試，每次重建 session 都從零命令、三筆新心跳開始。映像建置、agent session、實際 DDS rate、重連與 amd64／arm64 原生執行仍待維護者於 Docker 驗收。

### 3.2 和 rosserial、未來 STM32 的關係

rosserial client 透過專用協定由 PC bridge 轉入 ROS；micro-ROS 讓 MCU 使用精簡的 ROS 2 C API，再由 XRCE agent 代理 DDS entities。agent 負責通訊，逾時停止仍是 client 的責任。本次 Linux client 與 STM32 韌體會使用同類的 rclc 初始化、publisher／subscription、executor 與標準訊息配置，但 Linux 程式不是可直接燒錄的 STM32 binary。

[客戶端原始碼說明](../../firmware/sim_client/README.md) 記錄官方 host 工具工作區、外部 app 複製位置及契約 header 產生方式。流程依 [官方 Linux 教學](https://micro.ros.org/docs/tutorials/core/first_application_linux/) 與 [Humble host 建置腳本](https://github.com/micro-ROS/micro_ros_setup/blob/humble/config/host/generic/build.sh)，使用 `micro_ros_setup` humble 分支、`create_firmware_ws.sh host`／`build_firmware.sh`，沒有板卡 SDK 或交叉編譯。

未來 STM32 需將 Linux DNS／UDP、POSIX clock 與容器重啟換成已確認的 transport、RTOS 時鐘及重連狀態機，將輪模型替換成獨立的控制與安全迴圈。板上必須自行在失聯時關閉馬達輸出，不能靠 agent 或 Docker 幫忙停車。外部 MCU 通常無法解析容器 DNS，還需另外明確 opt-in 的 port mapping、IP、防火牆及單一裝置配置；目前沒有對外入口或預設硬體連線。

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

寒假若只測 STM32 串列通訊與左右馬達，可明確選用隔離的 [micro-ROS 實機測試](../../hardware/winter_microros/README.md)（PlatformIO／F446RE 社群實驗性路徑，採簡化契約；已編譯，尚未實機驗收）。

## 5. 常見問題與重連

- 先開 agent 再啟動 client，方便觀察 session 建立；client 也應以 ping／重連狀態機處理 agent 尚未就緒、重啟、USB 拔除，不能永遠卡在初始化。典型做法是 `rmw_uros_ping_agent`、建立 entities、執行、失聯銷毀 entities 再等待；每次恢復都維持零命令，重新取得新鮮 heartbeat／cmd 才允許移動。
- 沒有 topic：先確認 agent session 日誌、相同 domain、網路路由與 DDS discovery；不能只看容器 alive。跨容器使用 UDP transport 配置，不能假設 shared memory 可跨 namespace 使用。
- topic 有但收不到：檢查 offered／requested QoS。本介面高頻 wheel／IMU／tick 使用 best_effort，避免遙測重傳壅塞；command／heartbeat／急停使用 reliable，仍必須有時間期限。best_effort publisher 無法滿足 reliable subscriber。急停保留值需 transient_local 相容，MCU 端也須確認所選 rmw micro-ROS 版本支援方式。
- 大訊息／低 RAM：標準 Imu 的 covariance 陣列也會增加序列化尺寸；核對 MTU 與分片／history 設定，不要以 Python 節點能送就推定 MCU 能送。減少字串容量、topic 數量、queue 與 rate 時須同步規格，不能私自改 message type。
- 串列 agent 掉包：官方 Humble agent 文件列有 Docker serial 掉包限制；需實測版本、baud、USB 與排程。[Humble agent 已知限制](https://github.com/micro-ROS/micro-ROS-Agent/tree/humble#known-issueslimitations)。

## 6. 明確選用實體串列（未驗證，預設不開）

兩條路互斥：micro-ROS MCU 使用 XRCE agent；純 serial MCU 使用本專案 codec／serial_bridge。不能同時開啟同一埠，也不能把本專案二進位訊框送 agent。

micro-ROS serial agent 的範本是 [docker-compose.serial.yaml](../../docker/docker-compose.serial.yaml)。只有使用者確認硬體與埠後，複製成自己的 override，取消該單一 devices mapping 註解，設定 `MICROROS_SERIAL_DEVICE` 為同一個已確認埠，再明確執行：

```bash
docker compose -f docker/docker-compose.yaml -f /path/to/confirmed-serial.yaml --profile comm up -d microros_agent
```

範本 command 的 `--dev` 由明確的 `MICROROS_SERIAL_DEVICE` 提供，未設定時 Compose 直接拒絕解析；即使設定，原範本仍沒有 devices，無法連到裝置。勿掛整個 `/dev`、勿 privileged、勿自動偵測。這是未來硬體選用步驟，目前開發仍用 sim_ros／PTY。

如果 MCU 記憶體、RTOS 整合或 micro-ROS transport 支援不合適，可選用固定 binary fallback。它較小、容易在舊韌體實作，但 MCU 需自行維護 codec／CRC、資料與安全狀態機，沒有 DDS discovery。真正連到板子的 bridge 必須同時設定 **`use_hardware:=true` 與非空明確 `port`**；預設 false／空字串，任一條件缺少就拒絕開埠。該程序還需在自己的 opt-in 容器 override 映射單一已確認的 port，並由維護者預先提供 pyserial（套件不自動安裝；只有硬體分支延遲匯入）。範例僅供未來確認硬體後使用：

```bash
ros2 run shopping_mcu_bridge serial_bridge --ros-args -p use_hardware:=true -p port:=/dev/ttyACM0 -p baud:=115200 -p use_sim_time:=false
ros2 run shopping_mcu_bridge mcu_link_monitor --ros-args -p estop_on_link_loss:=true -p use_sim_time:=false
```

必須另有既有 mux／watchdog，且 MCU 自行實作逾時停車；此模式不會啟動模擬 peer。連線失敗會保持 LINK_LOST，由 monitor 請求急停，不自動重開其他裝置。上述不是實車操作授權或硬體驗收紀錄。板端韌體、電氣急停與實際馬達控制仍未實作。
