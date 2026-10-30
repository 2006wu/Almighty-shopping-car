# 寒假實機測試：micro-ROS

## 目的與範圍

只驗證 Linux PC（ROS 2 Humble）↔ 串列 micro-ROS agent ↔ STM32 通訊，以及 PC 發布 `/cmd_vel` 能讓差速底盤左右馬達轉動。沒有導航、UWB、雷射、IMU、里程計或 TF。這是使用者明確選用的獨立實機測試，不會由既有模擬流程自動啟動。**韌體已在審查環境以 PlatformIO 編譯通過（預設設定 RAM 約 22.6 KB／32 KB（68.8%）、Flash 約 96.7 KB／128 KB（73.8%）），尚未燒錄或實機驗收。**

範例板為 Nucleo-G431RB，使用 Arduino／PlatformIO 與 ST-LINK VCP（LPUART1，PA2/PA3，主機通常顯示為 `/dev/ttyACM0`），不是 MCU 原生 USB-CDC。STM32／Nucleo 不在 micro_ros_platformio 的官方支援板清單，本路徑屬於 **community/experimental（社群／實驗性）**，可能需要處理工具鏈、靜態函式庫與記憶體相容問題。若此路徑無法建置，可改用 STM32CubeMX／CubeIDE + `micro_ros_stm32cubemx_utils` 的 Humble 路徑，另行設定 HAL、transport 與記憶體；不是直接改一個 board 名稱就能替換。

板型假設：使用者表示寒假用 STM32G431，此處假設為 Nucleo-G431RB；若是其他 G431 板（例如自製板或 G431KB），須重新核對腳位、序列埠與 platformio.ini 的 board。STM32G431RB 為 Cortex-M4F、170 MHz，具 32 KB SRAM 與 128 KB Flash。

### 記憶體餘裕（G431RB）

2026-10-30 以 `ststm32@20.0.0`、micro_ros_platformio `cfee17f`、Humble 預設 colcon meta（serial transport）編譯 `nucleo_g431rb` 的結果：

| 設定 | 靜態 RAM（.data＋.bss） | Flash |
| --- | --- | --- |
| 預設（`USE_ENCODERS 0`） | 22,560 bytes／32,768 bytes（68.8%） | 96,748 bytes／131,072 bytes（73.8%） |
| 啟用 encoder（`USE_ENCODERS 1`） | 22,816 bytes／32,768 bytes（69.6%） | 100,432 bytes／131,072 bytes（76.6%） |

之前 F446RE（128 KB SRAM、512 KB Flash）約 22.4 KB RAM、93 KB Flash，絕對用量幾乎相同，但 G431RB 的總量小很多：靜態配置後只剩約 10 KB 給 heap（micro-ROS 建立 entities、重連時的動態配置）與 stack，Flash 約剩 30 KB。**餘裕偏緊但可建置**；編譯通過不代表執行期不會 heap／stack 不足，實機須觀察多次斷線重連後是否仍穩定。

RAM 大宗是 micro-ROS 靜態池（XRCE session 與串列緩衝約 5.4 KB、靜態訊息緩衝約 8.3 KB、publisher／subscription 池約 3.2 KB）。若實機出現建立失敗、重連後當機或要再增加功能，可在 `platformio.ini` 加 `board_microros_user_meta` 自訂 colcon meta，降低 `RMW_UXRCE_MAX_PUBLISHERS`（程式最多只用 4 個）、`RMW_UXRCE_MAX_SUBSCRIPTIONS`（只用 1 個）、`RMW_UXRCE_MAX_HISTORY` 或 MTU，再執行 `pio run -t clean_microros` 重新建置；此縮減目前未實作也未驗證。

## 與完整 MCU 契約的差異

本目錄是隔離的簡化測試，沒有取代 [MCU_INTERFACE.md](../../docs/hardware/MCU_INTERFACE.md) 或修改其唯一規格。

| 項目 | 完整 MCU 契約 | 本測試 |
| --- | --- | --- |
| 輪資料 | 四輪 JointState、四筆 encoder ticks | 左右兩側命令回報，選用兩筆原始 encoder ticks |
| 命令逾時 | 0.3 秒 | 500 ms，由 `millis()` 計時 |
| PC 心跳 | 必須，三筆恢復判斷 | 無；使用 agent ping 與命令逾時 |
| `/cmd_vel` 發布者 | 唯一 `cmd_vel_watchdog`，測試送 `/cmd_vel/teleop` | 手動 `ros2 topic pub` 或鍵盤直接送 `/cmd_vel`，一次只用一種 |
| 倒車與速度 | 不倒車，線速上限 0.6 m/s，有一般加速度限制 | 允許倒車，線速與輪速上限 1.0 m/s，角速上限 2.0 rad/s，沒有加速度限制 |
| 狀態 | 完整有限狀態集合，10 Hz | `READY`／`CMD_TIMEOUT`，約 2 Hz；非法命令也停車並顯示 `CMD_TIMEOUT` |

所有端點使用 volatile durability。`/cmd_vel` 為 `geometry_msgs/msg/Twist`、reliable、depth 1；`/mcu/heartbeat` 為 `std_msgs/msg/UInt32`、reliable、depth 1、10 Hz，從開機累加。`/mcu/status` 為 `std_msgs/msg/String`、reliable、depth 1。`/mcu/wheel_cmd` 為 `std_msgs/msg/Float32MultiArray`、best_effort、depth 5、10 Hz，順序為 `[left_mps, right_mps, left_pwm, right_pwm]`，PWM 為非負 duty 數值（預設 0–255），方向見帶正負號的輪速。這是命令回報，**不是實測速度**。

`USE_ENCODERS=1` 才有 `/mcu/encoder_ticks`（`std_msgs/msg/Int64MultiArray`、best_effort、depth 5、10 Hz），順序 `[left, right]`，僅 A 上升緣計數（1 倍頻），B 判斷正負，開機歸零，尚未換算速度或閉迴路控制。不可讓完整契約的四輪接收者讀取這個兩筆版本。

**不要與專案模擬通訊堆疊／Compose、Nav2、跟隨或其他速度發布者在同一 domain 同時執行。** 預設 domain 為 28；本測試使用實際時間，不需要 `/clock`，PC 工具保持 `use_sim_time=false`。

## 硬體與接線

先關閉馬達電源。所有硬體占位設定集中在 [firmware/include/robot_config.h](firmware/include/robot_config.h)，每項都有 `TODO(hardware)`。唯一必須同步的工具鏈選項是 `firmware/platformio.ini` 的板型、串列監看 baud 與燒錄方式。

| 功能／連線 | Nucleo-G431RB Arduino 腳 | MCU 腳 | 計時器／說明 | 驅動器端 |
| --- | --- | --- | --- | --- |
| `LEFT_PWM_PIN` | D6 | PB10 | TIM2_CH3 PWM | 左 PWM／ENA |
| `RIGHT_PWM_PIN` | D9 | PC7 | TIM3_CH2 PWM | 右 PWM／ENB |
| `LEFT_DIR_PIN` | D4 | PB5 | GPIO | 左 DIR／IN1 |
| `RIGHT_DIR_PIN` | D7 | PA8 | GPIO | 右 DIR／IN1 |
| `LEFT_DIR2_PIN` | D8 | PA9 | GPIO，僅 IN1/IN2 模式使用 | 左 IN2 |
| `RIGHT_DIR2_PIN` | D3 | PB3 | GPIO，僅 IN1/IN2 模式使用；亦為 SWO，使用不含追蹤的 SWD 時可用 | 右 IN2 |
| `LEFT_ENCODER_A_PIN` | D2 | PA10 | EXTI10 中斷 | 左側編碼器 A |
| `LEFT_ENCODER_B_PIN` | D12 | PA6 | GPIO 輸入 | 左側編碼器 B |
| `RIGHT_ENCODER_A_PIN` | D11 | PA7 | EXTI7 中斷 | 右側編碼器 A |
| `RIGHT_ENCODER_B_PIN` | D5 | PB4 | GPIO 輸入；Arduino core 於開機時停用 UCPD dead-battery 下拉 | 右側編碼器 B |
| `STATUS_LED_PIN` | LED_BUILTIN = D13 | PA5 | 使用者 LED LD2 | — |
| 串列 | ST-LINK USB 接 PC | PA2/PA3 | LPUART1，ST-LINK VCP；保留，不作其他用途 | 不接馬達驅動器；確認 VCP 跳線／焊橋 |
| 共地 | GND | — | 邏輯地 | 驅動器邏輯地、電源負端共地 |
| 馬達供電 | 獨立、合規格且有斷電開關的電源 | — | 不得由 Nucleo 供馬達電力 | 驅動器馬達供電端 |

兩個 PWM 腳分別使用 TIM2／TIM3，皆維持 20 kHz。此 variant 的 Arduino `Serial` 使用 LPUART1（PA2/PA3，ST-LINK VCP）。PA13/PA14 保留給 SWD；D14/D15（PB9/PB8）保留給未來的 I2C IMU（BNO085），其中 PB8 亦為 G4 的 BOOT0；D0/D1（PC5/PC4，USART1）保留備用。

使用 3.3 V 相容的邏輯輸入與 encoder 訊號，確認模組電位與驅動能力。PWM+DIR 模式假設 PWM=0 就停止；IN1/IN2+PWM 模式適用有獨立 enable/PWM 的 L298N、TB6612 類模組，移除強制 EN 全速跳線。TB6612 的 STBY 或其他額外 enable 必須依模組資料確認接法，確保重設時停用，不可懸空；本程式沒有額外 enable 腳位控制。每側若有多顆馬達，要先核對總堵轉電流，不能直接假設模組可承受。

逐一確認所有 TODO：

- 板型：`nucleo_g431rb` 與實際板子相符，並確認 LPUART1（PA2/PA3）確實連到 ST-LINK VCP。
- 驅動器：`DRIVER_TYPE` 選 `PWM_DIR` 或 `IN1_IN2_PWM`。
- 腳位：`LEFT_PWM_PIN`、`RIGHT_PWM_PIN`、`LEFT_DIR_PIN`、`RIGHT_DIR_PIN`、`LEFT_DIR2_PIN`、`RIGHT_DIR2_PIN`，確認 PWM、排針及其他周邊沒有衝突。
- 編碼器：`USE_ENCODERS` 預設 0；`LEFT_ENCODER_A_PIN`、`LEFT_ENCODER_B_PIN`、`RIGHT_ENCODER_A_PIN`、`RIGHT_ENCODER_B_PIN`；`ENCODER_TICKS_PER_REV` 為輸出輪每圈的 A 上升緣數（含減速比）。回報是原始符號，不受馬達反相旗標影響，必要時調整 A/B 接線。
- 方向：`LEFT_MOTOR_INVERT`、`RIGHT_MOTOR_INVERT`，先架空確認正向；左右鏡像安裝常需要其中一側反相。
- 幾何與速度：`WHEEL_RADIUS_M=0.05`（目前只預留）、`TRACK_WIDTH_M=0.30`、`MAX_WHEEL_SPEED_MPS=1.0`（100% PWM 的估計輪速）、`MAX_LINEAR_MPS=1.0`、`MAX_ANGULAR_RADPS=2.0`、`ALLOW_REVERSE=true`。差速換算後超界會同比縮放兩輪；停用倒車時也會限制內側輪不反轉。
- PWM：`PWM_FREQUENCY_HZ=20000`、`PWM_RESOLUTION_BITS=8`、`PWM_MIN_DUTY=0`、`PWM_DEADBAND_MPS=0.01`。確認驅動器支援 20 kHz；提高最小 duty 會使低速命令突然跳動。
- 時序與識別：`CMD_VEL_TIMEOUT_MS=500`、`HEARTBEAT_PERIOD_MS=100`、`SERIAL_BAUD=115200`、`STATUS_LED_PIN=LED_BUILTIN`、`ROS_DOMAIN_ID=28`、`NODE_NAME="winter_base"`。修改 baud 必須同步 agent；修改 domain 必須同步 PC 並重新燒錄。

## 安裝、建置與燒錄

以下在可連網的 Linux 主機執行。可安裝 VS Code PlatformIO 擴充套件，或使用獨立 CLI：

```bash
sudo apt update
sudo apt install git cmake build-essential python3-venv
python3 -m venv ~/.venvs/winter-pio
. ~/.venvs/winter-pio/bin/activate
pip install platformio
cd hardware/winter_microros/firmware
pio run
pio run -t upload
```

首次建置會下載工具鏈與相依套件並編譯 micro-ROS 靜態函式庫，需數分鐘；不需要 Docker 來建函式庫。`platformio.ini` 已鎖定審查時編譯成功的版本：ST STM32 平台 20.0.0（Arduino STM32 core 3.0.0）與 micro_ros_platformio commit `cfee17f`；升級前先確認仍可編譯。韌體內有一個丟棄 printf 輸出的 `_write` stub，因為 micro-ROS 函式庫會連結到 newlib 的輸出函式，而 Serial 只能承載 XRCE。ST-Link 燒錄使用 `upload_protocol=stlink`，Linux 也可能需要安裝 PlatformIO／ST-Link 的 udev 規則與重新插拔。

改過 distro／transport 後清除 micro-ROS 建置產物再建：

```bash
pio run -t clean_microros
pio run
pio run -t upload
```

程式使用靜態字串與陣列；每次 session 初始化建立 ROS entities 與預先準備 executor wait set，正常控制迴圈不主動配置動態記憶體。斷線重連會銷毀並重新配置 entities，必須量測多次重連後的 RAM 與堆積碎片，不能宣稱完全不使用 heap。馬達在建立／銷毀 session 期間保持停止。串列承載 XRCE，**不得加入 `Serial.print` 除錯文字**。

## 啟動主機端

使用 Linux 筆電上的 Docker Engine；這份 Compose 獨立於 repo 的 `docker/`，只映射明確指定的一個串列裝置，沒有 privileged 或整個 `/dev` 掛載。先確認裝置與接線，再明確執行：

```bash
cd hardware/winter_microros/host
export MICROROS_SERIAL_DEVICE=/dev/ttyACM0
export SERIAL_BAUD=115200
export ROS_DOMAIN_ID=28
docker compose build tools
docker compose up agent
```

保留 agent 日誌終端機；應出現 session established、`create_participant`、建立 publisher/subscriber 等訊息（文字依版本略有不同），必要時按板子 reset。等待時 LED 約每 100 ms 切換；連線後隨 10 Hz heartbeat 切換。LED 只能輔助判讀，應以 topic 與馬達結果為準。

另開終端機，進入同一 `host/` 目錄並設定相同環境變數，再執行：

```bash
docker compose run --rm tools
```

以下所有 `ros2` 指令都在這個 tools shell 執行；需要同時觀察與發布時，多開一個 tools shell。`ros:humble-ros-base` 的 entrypoint 會載入 ROS。agent 操作指令則在主機的 `host/` 目錄執行。

`network_mode: host` 加上 Mac Docker Desktop **不能直接傳遞 USB serial**，這份流程請用 Linux 筆電。也可在已用 apt 安裝 Humble 的 Linux 主機上，以 `micro_ros_setup` 的 Humble 工具建置並載入 agent 工作區後執行：

```bash
export ROS_DOMAIN_ID=28
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

另一選項是已安裝且獲串列存取權限的 snap：`micro-ros-agent serial --dev /dev/ttyACM0 -b 115200`。只選一種 agent，不要同時占用埠。這些替代方法仍需 Linux 的裝置權限設定。

## 逐步測試與預期結果

1. **馬達電源關閉：確認通訊。**

   ```bash
   ros2 topic list
   ros2 topic echo /mcu/heartbeat
   # Ctrl-C, then measure rate
   ros2 topic hz /mcu/heartbeat
   ```

   預期出現 `/cmd_vel`、`/mcu/heartbeat`、`/mcu/status`、`/mcu/wheel_cmd`；heartbeat counter 持續增加、約 10 Hz。未送命令時 status 為 `CMD_TIMEOUT`，輪命令全零。選用 encoder 才會出現其 topic。

2. **輪子架空，才開馬達電源。** 先在一個 tools shell 觀察：

   ```bash
   ros2 topic echo /mcu/wheel_cmd --qos-reliability best_effort
   ```

   在另一個 tools shell 發布：

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
   ```

   預期兩側朝前，預設回報約 `[0.2, 0.2, 51, 51]`，status 為 `READY`。這代表 PWM 命令，不保證實際達到 0.2 m/s；低 duty 可能無法克服靜摩擦。方向錯誤時先停發布、切馬達電源，調整 invert 後重新燒錄。Ctrl-C 停止前一個發布者，等馬達停止，再測轉彎：

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
   ```

   預期左側 0.125 m/s、右側 0.275 m/s，duty 約 32／70，朝左轉。正 angular.z 表示逆時針。

3. **驗證逾時、agent 重啟與拔插。** Ctrl-C 停發布，預期約 0.5 秒內停止、回報全零及 `CMD_TIMEOUT`（約 2 Hz 更新）。重新啟動 10 Hz 發布者使輪子轉動，在主機執行：

   ```bash
   docker compose stop agent
   ```

   預期馬達停止；agent ping 約每 200 ms 檢查，偵測失聯即零 PWM，命令逾時則約 500 ms，實際延遲需量測。先 Ctrl-C 結束發布者，再啟動 agent：

   ```bash
   docker compose up -d agent
   docker compose logs -f agent
   ```

   預期自動重建 session、heartbeat 恢復、馬達仍停止，直到收到新的 `/cmd_vel`。若舊發布者還在 10 Hz 發送，重連後會被視為新命令而移動，所以務必先停掉。輪子仍架空時另測 USB 拔插；板子若重新供電，heartbeat counter 會歸零。Linux 的 devices mapping 可能因重插產生新裝置節點而失效；確認仍是原本的明確埠後執行 `docker compose up -d --force-recreate agent`。不要自動掃描或映射其他埠。

4. **落地慢速測試。** 清空場地、有人手持馬達斷電開關，先短距離用上述 0.2 m/s 指令，再 Ctrl-C 停止。確認停止有效後，結束所有 `topic pub`，才用鍵盤：

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
   ```

   預期按鍵能移動／轉向，停止按鍵後也會逾時停車。teleop 預設每次按鍵才送一次，500 ms 後停下是預期行為；按 `k` 可送零命令，Ctrl-C 結束。不要用加速鍵提高到未驗證速度。完成後先關閉馬達電源，在主機執行 `docker compose down`。

## 疑難排解

| 現象 | 檢查方向 |
| --- | --- |
| `/dev/ttyACM0` 不存在／權限不足 | 確認 ST-Link USB 線可傳資料、實際裝置路徑；主機使用者可用 `sudo usermod -aG dialout "$USER"` 後重新登入，確認 udev 規則；不要以 privileged 取代正確權限。改埠時明確設定 `MICROROS_SERIAL_DEVICE`。 |
| agent 沒有 session 或卡住 | 核對兩端 115200 baud、LPUART1（PA2/PA3）／ST-LINK VCP 接線，按板子 reset；確認沒有其他 agent、ModemManager 或程式占埠。**不要同時開 `pio device monitor`**。GPIO／馬達雜訊也可能造成重啟。 |
| agent 有 session，但看不到 topics | 韌體 config 與 agent／tools 的 `ROS_DOMAIN_ID` 都必須為 28；改韌體值需重燒。確認 host 網路與 Linux DDS discovery／防火牆設定。 |
| wheel／encoder echo 沒資料 | 使用 `ros2 topic echo /mcu/wheel_cmd --qos-reliability best_effort` 或 `ros2 topic echo /mcu/encoder_ticks --qos-reliability best_effort`；encoder 需先啟用並接線。 |
| 首次建置找不到 cmake／compiler／venv | 安裝 `git cmake build-essential python3-venv`，確認 PlatformIO 所用環境可執行這些工具；保留第一筆錯誤供排查，不要混用 ROS 發行版。 |
| STM32／micro-ROS 建置失敗 | 這是實驗性板型；提供完整錯誤、依賴版本、RAM／Flash 使用量。Nucleo-G431RB 只有 32 KB SRAM、128 KB Flash，**記憶體餘裕較緊**，須檢查 map、堆疊與重連記憶體；不夠時先關閉 encoder 或評估 CubeMX 路徑。 |
| 有 PWM 命令卻不轉 | 核對共地、外部電源、EN／STBY、DIR 模式、PWM 腳位與靜摩擦；先斷電再改線。命令回報不代表驅動器實際有輸出。 |

## 安全注意

- 馬達電源必須有可立即操作的急停／斷電開關；軟體 timeout 不是電氣急停。
- 先架空輪子，再以 0.2 m/s 開始；設定與測試速度皆不得超過 1.0 m/s，開迴路實際速度仍需確認。此範例沒有加速度斜坡、障礙防護或閉迴路調速。
- 必須共地，馬達不可由板子供電。供電與驅動器須能承受堵轉電流，避免 USB 回灌。
- **重設／燒錄／未上電時 GPIO 可能浮接，驅動器必須以硬體下拉或 enable/STBY 電路預設停用。** 先確認 PWM=0 的實際停止行為；韌體無法保護尚未執行 setup 的期間。
- 平常只有收到新鮮且有限的平面命令才輸出；NaN／Inf、非平面輸入、命令逾時與偵測失聯都停車。控制迴圈可能受 transport 或 MCU 故障阻塞，因此停止時間與斷電急停都必須實測。
