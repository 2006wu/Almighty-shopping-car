# 寒假底盤測試：STM32 與 rosserial

## 目的與範圍

只驗證 **PC ↔ STM32 串列通訊**及 **PC 發布 `/cmd_vel` 讓左右兩側馬達轉動**。硬體只有差速底盤、直流馬達、PWM 驅動器及 STM32；沒有導航、UWB、雷射、IMU、里程計或 TF。此資料夾獨立操作，只有明確執行本文件的燒錄與啟動程序才會使用硬體；不接入既有模擬流程。

rosserial 是 ROS 1 通訊方案，沒有 ROS 2 版本，所以主機端使用 ROS 1 Noetic 容器。既有專案仍使用 ROS 2 Humble；不能將 ROS 2 指令直接套在此測試。Noetic 已停止官方維護，此環境僅供獨立寒假測試。

[完整 MCU 契約](../../docs/hardware/MCU_INTERFACE.md) 是未來四輪介面：0.3 秒命令逾時、PC heartbeat、預設不倒車。此測試刻意簡化為**左右兩側、500 ms 命令逾時、沒有 PC heartbeat、允許倒車**，直接訂閱 `/cmd_vel`，**不經 `cmd_vel_watchdog`**。它不是完整契約的實作；不要同時執行 Nav2、跟隨、既有通訊節點或第二個速度發布者。

## 硬體與接線

先切斷馬達電源，完成接線及設定核對後再上電。預設板型為 Nucleo-F446RE，以下腳位是可編譯的範例，並非已確認的實車接線。

| 接點 | Nucleo Arduino 排針預設 | 接法／注意 |
| --- | --- | --- |
| 左 PWM／右 PWM | D3（PB3）／D5（PB4） | 接驅動器 PWM；L298N 接 ENA／ENB，移除常開 EN 跳線 |
| 左 DIR／右 DIR | D4（PB5）／D7（PA8） | PWM+DIR 的方向；雙方向輸入驅動器接各側 IN1 |
| 左 DIR2／右 DIR2 | D8（PA9）／D9（PC7） | `DRIVER_L298N` 模式各側 IN2；預設模式不使用 |
| 左 encoder A／B | D2（PA10）／D10（PB6） | 選用，預設關閉；輸入須符合 3.3 V 電位 |
| 右 encoder A／B | D11（PA7）／D12（PA6） | 選用，預設關閉；A 的上升及下降緣計數 |
| 狀態 LED | LED_BUILTIN（D13／PA5） | 每次 heartbeat 翻轉，100 ms 一次，完整閃爍週期 200 ms |
| PC USB | 板上 ST-Link USB 插座 | ST-Link 虛擬 COM 經 USART2 PA2／PA3；保留 D0／D1，不接馬達 |
| GND | STM32 GND ↔ 驅動器 GND | 必須共地；馬達由獨立合適電源供電 |

`DRIVER_PWM_DIR` 適用真正的 PWM+DIR 輸入。**裸 TB6612 模組有 AIN1／AIN2、BIN1／BIN2、PWMA／PWMB**，應選 `DRIVER_L298N` 雙方向輸入模式，並依模組規格將 STBY 接到明確有效的邏輯電位；不能只接一根方向線。L298N 也使用這個模式，且須確認模組 EN、邏輯供電及電壓降。零 PWM 的實際停止／滑行行為取決於驅動器，程式不提供主動煞車。

### 上電前逐項填寫

唯一硬體設定檔是 [firmware/include/robot_config.h](firmware/include/robot_config.h)，每個佔位均標有 `// TODO(hardware):`。請核對全部項目：

- 板型與 ST-Link VCP 路徑；板型選擇須同步修改 `platformio.ini`。預設 `Serial` 使用 USART2，不是 MCU 原生 USB CDC。
- `DRIVER_TYPE`：PWM+DIR 或 IN1／IN2+EN；驅動器使能接法。
- `LEFT_PWM_PIN`、`RIGHT_PWM_PIN`、`LEFT_DIR_PIN`、`RIGHT_DIR_PIN`、`LEFT_DIR2_PIN`、`RIGHT_DIR2_PIN`：依接線填寫；PWM 必須支援計時器。
- `USE_ENCODERS`（預設 0）、四個 `*_ENCODER_*_PIN`、`ENCODER_TICKS_PER_REV`：含減速比、每個輸出輪轉一圈的 A CHANGE 計數（x2）。
- `LEFT_MOTOR_INVERT`、`RIGHT_MOTOR_INVERT`：以車體前進方向核對正負。
- `WHEEL_RADIUS_M`（0.05 m）、`TRACK_WIDTH_M`（0.30 m）：量測實際輪半徑與左右輪距。
- `MAX_WHEEL_SPEED_MPS`（1.0 m/s）：滿 PWM 的輪線速度校正值；`MAX_LINEAR_MPS`（1.0 m/s）、`MAX_ANGULAR_RADPS`（2.0 rad/s）：測試上限。
- `ALLOW_REVERSE`（預設 1／true）：方便測試正反轉；設為 0／false 只禁止負的底盤前進速度，原地旋轉仍可能讓單側倒轉。
- `PWM_FREQUENCY_HZ`（20000 Hz）、`PWM_RESOLUTION_BITS`（8）、`PWM_MIN_DUTY`（0 個計數）、`PWM_DEADBAND_MPS`（0.01 m/s）：依驅動器規格與起轉需求調整。提高最小 PWM 會使低速突然起轉，須先架空測試。
- `CMD_VEL_TIMEOUT_MS`（500 ms）、`HEARTBEAT_PERIOD_MS`（100 ms）、`SERIAL_BAUD`（115200）、`STATUS_LED_PIN`：確認板端與主機一致。

這是開迴路 PWM 測試：輪半徑與 ticks/rev 只保留校正資料，不參與控制；速度命令不等於實測車速。差速換算後若超過滿 PWM 輪速，左右同比縮小以保留轉彎比例。沒有加速度限制，也沒有軟體急停鎖定；落地前先確認起轉不會暴衝。

## 安裝、建置與燒錄

在 Linux 主機安裝 Python 3、venv 與 USB／ST-Link 存取規則，再於專案根目錄執行（不需要 ROS 即可編譯）：

```bash
python3 -m venv /tmp/winter-pio
source /tmp/winter-pio/bin/activate
python -m pip install platformio
cd hardware/winter_rosserial/firmware
pio run
# Connect the ST-Link USB cable with motor power OFF.
pio run -t upload
```

預設環境 `nucleo_f446re`，以 ST-Link 燒錄；必要時安裝 PlatformIO 的 Linux udev 規則後重新插拔。`platformio.ini` 有註解掉的 F407 替代環境；它只是起點，須重新核對排針、PWM、LED、序列埠路徑與 ST-Link 接線，並明確用 `pio run -e black_f407ve` 選用。F407 板子的 USB 不保證提供 ST-Link VCP。

## 啟動主機端

使用 Linux 筆電的 Docker Engine 與 Compose v2。**Docker Desktop on Mac 無法直接傳入 USB serial 裝置**；請改用 Linux 筆電，或在相容 Linux 系統原生安裝 Noetic 與 rosserial。此處不固定 CPU 架構；amd64／arm64 映像與實機執行仍須各自驗證。

確認實際裝置名稱（例如 `/dev/ttyACM0`），不要讓其他 serial monitor 或 agent 佔用同一埠。以下均在 `host/` 執行：

```bash
cd hardware/winter_rosserial/host
export SERIAL_DEVICE=/dev/ttyACM0
export SERIAL_BAUD=115200
docker compose build
docker compose up
```

Compose 只映射這一個已確認的裝置，使用主機網路，沒有 privileged 或整個 `/dev` 掛載。launch 自動啟動 ROS master 與 `serial_node.py`，等效於 `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200` 加上獨立 roscore。不要同時使用已有的 ROS master；本測試使用 `localhost:11311`、`use_sim_time=false`，不需要 `/clock`。

保留前景終端；第二個終端進入容器執行測試指令：

```bash
cd hardware/winter_rosserial/host
docker compose exec rosserial bash
source /opt/ros/noetic/setup.bash
```

原生 Noetic 可改成先 `roscore`，另一終端 `roslaunch /absolute/path/to/host/launch/winter_rosserial.launch port:=/dev/ttyACM0 baud:=115200`，再於已載入 Noetic 環境的終端執行下列相同 ROS 1 指令。

## 測試流程與預期結果

1. **通訊檢查，馬達電源先關閉**。於容器終端依序執行，各用 Ctrl-C 結束：

   ```bash
   rostopic echo /mcu/heartbeat
   rostopic hz /mcu/heartbeat
   rostopic echo /mcu/status
   ```

   預期 counter 持續增加、頻率約 10 Hz、LED 持續閃爍；未送命令時狀態為 `CMD_TIMEOUT`。LED 單獨閃爍只代表 MCU 迴圈在跑，收到 heartbeat 才能確認 PC 通訊。

2. **架空輪子，確認電源切斷開關可及，再開馬達電源**。僅一個終端發布：

   ```bash
   rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'
   ```

   預期兩側朝車體前進方向轉動，狀態 `READY`；另開容器終端可用 `rostopic echo /mcu/wheel_cmd` 查看 `[left_mps, right_mps, left_pwm, right_pwm]`。預設設定應約為 `[0.2, 0.2, 51, 51]`，PWM 是含反相後正負號的 duty 計數，非百分比。這是命令回報，不是實測輪速；死區內 m/s 仍保留命令值而 PWM 為零。

   停止前一發布者，等輪子停妥，再測轉彎：

   ```bash
   rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'
   ```

   預期左側後退、右側前進，車體呈逆時針旋轉方向。選用 encoder 時，`rostopic echo /mcu/encoder_ticks` 應見 `[left, right]` 累計值變動；重啟歸零、int32 溢位回繞，正負取決於 A/B 接線，馬達 invert 不會改 encoder 符號。

3. **逾時與斷線**。在輪子架空、持續送 0.2 m/s 時 Ctrl-C 停止發布，預期約 0.5 秒內 PWM 歸零、輪子停止受驅動、狀態成為 `CMD_TIMEOUT`；實際停轉時間另受慣性影響。重新發布後拔掉 ST-Link USB，預期同樣在約 0.5 秒後停止受驅動。`nh.connected()` 判定失聯時也會立即清除命令，但 rosserial 的連線狀態判定可能較慢，主要停止期限由 MCU 命令逾時保證。若 MCU 由 USB 供電，拔線將使 MCU 斷電，不能靠程式保證 GPIO；驅動器 EN／PWM 須有硬體下拉或停用電路，先以獨立 MCU 電源驗證軟體逾時，再驗證整機斷電行為。重新插線後先結束速度發布者、切斷馬達電源，再 `docker compose down`／`docker compose up` 重新映射裝置；確認零輸出後才重測。持續發布的命令在重連時屬於新命令，可能再次起轉。

4. **落地慢速測試**。只有前述項目通過，才在淨空地面以 0.2 m/s 短距離測試；備妥馬達電源切斷開關。停止 `rostopic pub` 後再用鍵盤：

   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.2 _turn:=0.5
   ```

   必須降低鍵盤預設速度，勿超過 1.0 m/s 線速度上限。預期前進、轉彎方向正確，空白鍵停車；鍵盤預設不持續重送，因此單次按鍵約 0.5 秒後會因逾時停止，可短按逐步確認。勿為了持續行走而關閉 MCU 逾時。

結束時先停止速度發布、確認停車、切斷馬達電源，再於主機端 `docker compose down`。

## 疑難排解

| 現象 | 檢查方式 |
| --- | --- |
| 沒有 `/dev/ttyACM0`／埠錯誤 | 核對 ST-Link 插座、可傳資料 USB 線及 Linux 的 `ls -l /dev/ttyACM*`；明確修改 `SERIAL_DEVICE` 後重建容器，不自動掃描開埠 |
| Permission denied | 查看 `ls -l /dev/ttyACM0`；Linux 使用者可執行 `sudo usermod -aG dialout "$USER"`，登出再登入；燒錄另需 ST-Link udev 規則。不要以 privileged 解決 |
| Unable to sync with device | 核對韌體已燒錄、板型、ST-Link VCP 的 USART2 接線／跳線、兩端 baud 相同、沒有其他程序佔埠；啟動容器後可按板上 reset |
| lost sync／雜亂資料 | 不要向 rosserial 的 Serial 寫入除協定外的除錯文字；檢查供電、共地、馬達雜訊與 USB 線；先關閉馬達電源重測 heartbeat |
| 方向相反 | 斷開馬達電源後改對應 `*_MOTOR_INVERT` 再燒錄；不要靠混亂的正負速度補償接線 |
| 馬達不轉或發熱 | 確認 EN／STBY、L298N 跳線、獨立電源與驅動能力；核對 `DRIVER_TYPE`、PWM 腳位與 `PWM_FREQUENCY_HZ` 規格，20 kHz 未必適合每款驅動器；謹慎校正最小 duty |
| 發布後很快停止 | `/cmd_vel` 應約 10 Hz 持續發布；停止發布後的 500 ms 停止是預期行為 |

## 安全注意

馬達電源必須有可立即操作的急停／切斷開關；本測試沒有軟體急停功能，USB 拔除不能取代電氣急停。先架空輪子，再以 0.2 m/s 起測；1.0 m/s 是命令上限，不是實測限速保證。STM32 與驅動器共地，**不能從開發板供電給馬達**。重啟、燒錄、MCU 斷電期間 GPIO 可能浮接，驅動器輸入須以硬體保持停用。確認驅動器支援 3.3 V 邏輯並處理反電動勢與電源雜訊；變更接線前切斷電源。

## 日後接 ROS 2（選讀）

主要測試全程留在 ROS 1，不需要 `ros1_bridge`。若日後要接 Humble，通常需要 Foxy／Galactic 時期可同時安裝 ROS 1／ROS 2 的橋接映像，或自行從原始碼建置相容橋接環境；不能假設 Humble 的 Ubuntu 22.04 容器可直接 apt 安裝 Noetic 與 bridge。須另外確認訊息型別、版本及安全控制權，此處不提供橋接實作。
