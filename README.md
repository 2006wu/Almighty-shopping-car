# Almighty-shopping-car

清華大學動力機械系28級畢業專題項目

本專案開發購物機器人的「引導購物」與「人物跟隨」功能。2006wu 負責 SLAM、Nav2 與跟隨；購物介面、商品辨識等其他子系統不在此骨架範圍。預計 2027 年 2 月才取得硬體，此前全部使用模擬。

使用 Ubuntu 22.04 容器、ROS 2 Humble、Gazebo Fortress（`ros_gz`）、slam_toolbox 與 Nav2。四輪滑移轉向車體約 0.60 × 0.50 公尺，包含 360 度雷射、IMU、輪式里程計及可調整的兩個或三個 UWB 錨點。容器使用主機原生 amd64／arm64 架構，不指定 `--platform`；未來的 R2 板規格仍待確認。

目標展示為 **12 × 10 公尺的小型模擬商店**，配置恰好四排貨架、三條貨架間走道（各寬 1.6 公尺）、入口與結帳區。這是畢業專題的模擬展示場景。機器人出生姿態為世界座標 `(x, y, z, yaw) = (0, 0, 0.02, 0)`，朝向正 x；地面穩定後 base_footprint 接近 z = 0。以此初始原點建立 `odom`／`map`，六個區域目標使用與世界相同的平面座標，既有外部地圖須先對齊，不能直接套用。

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
```

`goto` 直接呼叫 NavigateToPose，印出回饋及結果，失敗會回傳非零結束碼。應用程式也可發布 `std_msgs/String` 至 `/shopping/goto_zone`，由常駐區域節點處理。第一次建圖的原點與朝向應維持模擬初始姿態，重建地圖後要核對區域座標。

停止導航後測試 UWB 跟隨：

```fish
./scripts/dev.sh follow
```

人物是本機圓柱幾何模型，不需連線下載 Fuel 素材。路徑唯一來源為 `shopping_robot_gazebo/config/person_path.yaml`；模擬人物與 UWB 使用相同模擬時間插值。人物會循環走過三條走道；測試直接解析 SDF 碰撞盒，驗證區域有 0.55 公尺淨空、人物整條路徑有 0.30 公尺淨空。`uwb_follow.yaml` 可調整噪聲、偏差、NLOS、掉包及錨點陣列；兩錨點實驗須同時縮短 `anchor_x/anchor_y`，並將車體 xacro 的 `anchor_count` 改成 2 或以 xacro 參數產生相符模型。錨點移位時也需同步修改 xacro 的錨點屬性。這是平面測距模型，尚未模擬高度差與完整無線電傳播。

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
| noVNC | `127.0.0.1:6080` | 瀏覽器桌面 |
| VNC | `127.0.0.1:5910` | 對應容器 5900 |
| ROS DDS | 容器橋接網路 | UDPv4；相同 `ROS_DOMAIN_ID`，預設 28 |

VNC 密碼讀取環境變數，未設定時為 `shopping`；主機連接埠僅綁定迴送介面。兩個容器共享安裝產物，使用 Fast DDS UDPv4，停用共享記憶體傳輸。RViz 與 Gazebo 感測器採軟體繪圖；`sim/mapping/guide/follow` 會為運算容器開啟 Xvfb。`gz-gui` 可從顯示容器啟動 Gazebo GUI，用於觀察既有模擬；容器間的 Gazebo 探索仍需實機環境驗證。`AUTOSTART_RVIZ=1` 需先完成建置再啟動顯示容器。

Linux 若需 sudo，可執行 `env DOCKER='sudo docker' ./scripts/dev.sh up`。自訂世界須保留 `supermarket` 世界名稱、`person` 模型及對應橋接主題；目前 world 參數提供替換檔案路徑，尚未自動推導世界名稱。

## 安全與驗證狀態

所有預設流程僅使用模擬，沒有硬體驅動、裝置掛載或真實車體連線。跟隨節點單獨執行時預設停用，只有 follow launch 主動啟用；可用 `/follow/enable` 的 `std_srvs/SetBool` 服務切換。目標太近、遺失、資料過期或前方障礙過近時立即輸出零速度；雷射未收到、前方無有效資料或超時也停止。急停優先於一般加速度限制；一般追蹤限制線速度、角速度及加速度，預設不倒車。

本階段尚無模式管理器，請勿讓 Nav2、鍵盤控制與跟隨同時發布 `/cmd_vel`。跟隨目前是前方障礙停車，不會自主繞行；模式切換、命令仲裁、獨立命令逾時看門狗與恢復行為列於後續階段。UWB 訊息目前用無時間戳的 Float32MultiArray，模擬本機接收時間視為量測時間；硬體介面前必須改用有時間戳及錨點識別的訊息。

可在容器執行純 Python 測試，也可在指定沙盒環境執行：

```bash
/workspace/scratch/venv-shop/bin/python -m pytest -q
./scripts/run_unit_tests.sh /workspace/scratch/venv-shop/bin/python
/workspace/scratch/venv-shop/bin/python scripts/check_static.py
```

此骨架的靜態檢查與純邏輯測試不等於 ROS／Gazebo 整合驗證。Docker 映像建置、DDS、雷射繪圖、SLAM 與導航成功率須在具備 Docker Desktop 的環境實際確認，詳見 [進度紀錄](docs/PROGRESS.md)。指定環境的 xacro 套件若不支援 `python -m xacro`，可改用 `python -c 'import xacro; xacro.main()' 模型路徑`，兩者呼叫相同的 xacro 處理程式。

[架構](docs/ARCHITECTURE.md) · [時程](docs/ROADMAP.md) · [協作規範](AGENTS.md)
