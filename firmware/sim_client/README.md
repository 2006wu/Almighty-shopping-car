# Linux micro-ROS 模擬韌體

這是使用 C、rclc 與 `rmw_microxrcedds` 的 Linux 程式，只模擬輪子，不開啟串列埠、不操作馬達。資料路徑是 Linux client → UDP4 → micro-ROS agent → DDS；與直接加入 DDS 的 Python `sim_mcu_firmware` 不同。

從 repo 根目錄使用 `./scripts/dev.sh comm-e2e-up` 建置／啟動；另一終端使用 `./scripts/dev.sh comm-e2e-check`。完整準備、停止與驗收指令見 [micro-ROS 入門](../../docs/hardware/MICROROS_GUIDE.md)。一般 compute 的 colcon 不會建置此目錄；客戶端在自己的映像內建置。

## 原始碼與契約

- `main.c` 負責 DNS、UDP XRCE session、rclc executor、訊息記憶體與發布排程。只支援獨立通訊驗收的壁鐘時間，不能與 `use_sim_time:=true` 的 Gazebo 流程混用。
- `model.c`／`model.h` 不依賴 ROS；三筆遞增 PC heartbeat 才恢復、拒絕重複／倒序 counter（支援 UInt32 回繞）、失聯後不重播舊命令。NaN／Inf／非平面輸入立即零速；預設不倒車。一般運動採一階模型與加速度限制，安全停止立即歸零。
- `generate_contract.py` 從唯一規格 `interface.yaml` 產生時限、頻率、topic、depth、輪序與模型常數，再從 description xacro 讀取輪半徑與有效輪距。`contract.h` 是已納入版本管理的產物，Docker 建置也會重新產生；pytest 比對兩者並編譯執行純 C 模型。

修改規格後在既有、已具 PyYAML 的環境執行：

```bash
python3 firmware/sim_client/generate_contract.py \
  ros2_ws/src/shopping_mcu_bridge/config/interface.yaml \
  ros2_ws/src/shopping_robot_description/urdf/shopping_robot.urdf.xacro \
  firmware/sim_client/contract.h
```

本次只實作 `/cmd_vel`、兩向 heartbeat、`/mcu/wheel_state`、`/mcu/status` 子集，另有模擬限定的 `/mcu/sim/drop_commands` 與 `/mcu/sim/xrce_active`。沒有 encoder、IMU、MCU 急停按鈕／reset；PC mux ESTOP 仍有效，不能把這個子集當成完整 STM32 安全韌體。狀態只使用 `LINK_LOST`、`CMD_TIMEOUT`、`INVALID_COMMAND`、`READY`。

## 為什麼複製進工具工作區

[官方 Linux 教學](https://micro.ros.org/docs/tutorials/core/first_application_linux/) 的 `create_firmware_ws.sh host` 在目前工作目錄建立 `firmware/PLATFORM`，並將 ROS／micro-ROS 套件放到 `src/`；`build_firmware.sh` 在同一個工作區呼叫 colcon。

Dockerfile 使用 `micro_ros_setup` 的 `humble` 分支，先建立 `/uros_ws`，再將本目錄複製成 `/uros_ws/src/uros/shopping_microros_client`。這是獨立 ament C 套件，無須修改官方 demos 的 `export_executable()` 清單。標準訊息由工具的 host 套件清單提供。執行檔安裝到 `/uros_ws/install/shopping_microros_client/lib/shopping_microros_client/sim_client`；入口明確載入該工作區並設定 XRCE middleware。

所有字串與 JointState 的四筆陣列在 executor 運作前配置。heartbeat／status 的 reliable 發布等待上限各 10 ms，避免預設長等待阻塞模擬安全迴圈。初始化失敗、發布失敗或每秒 agent ping 失敗時退出，由 Compose `restart: on-failure` 建立全新程序及零命令模型；這是 Linux 容器的重試策略，不是 MCU 的 RTOS 重連實作。

移植 STM32 時可沿用 rclc 的 publisher／subscription 與純 C 模型；Linux DNS、POSIX clock、程序重啟及模擬輪模型需要替換成已確認的 RTOS／transport／時鐘與獨立底盤控制。訊息緩衝、RAM、可靠傳輸延遲、失聯／急停與重連都須重新量測，不能以本 Linux 測試替代。
