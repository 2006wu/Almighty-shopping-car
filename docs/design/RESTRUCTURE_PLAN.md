# Almighty-shopping-car 資料夾重整提案（唯讀研究，2026-10-27 16:3x 台北）

基準：`/workspace/wt/shop-dev` @ develop `c637777`。參考：pomelo925/TTennis-Pickup-Robot（HEAD）。
**本文件只是提案，repo 未做任何修改。**

## 1. 參考 repo 的分類方式（TTennis-Pickup-Robot）

```
.github/workflows/   每個子系統一個 workflow（coordinator/nav2/edgenn/rtabmap/communication.yml）
1.Hardware/          PCB、電源板（原理圖 PDF + 每板一個 README）
2.Firmware/          STM32CubeIDE 專案（Core-STM32H7/…）
3.Software/          每個子系統一個資料夾，且結構相同：
   A.Coordinator/  B.Navigation/  C.EdgeNN/  D.RTABMap/  E.Communication/  F.NextjsApp/
     ├── docker/{Dockerfile, compose.yml}   ← 每個子系統自己的容器
     ├── app/<mode>/run.sh                  ← 啟動腳本
     ├── <name>-ws/src/                     ← 該子系統自己的 colcon workspace
     └── build_and_run.sh / pull_and_run.sh
   Readme.md        子系統總覽
4.Model/             機構 CAD（.ipt/.stl）
5.Docs/              Introduction.md + assets/（架構圖、照片、gif）
README.md  LICENSE
```
值得學的：①最上層按「領域」分（硬體/韌體/軟體/機構/文件）；②軟體按「子系統」分，每個子系統自帶 docker + 啟動腳本；③文件集中 + assets 資料夾；④README 精簡、細節分到子 README。
**不要學的**：它把 colcon `build/`、`install/` 也 commit 進去（20k 檔案）；數字+點號的資料夾名（`3.Software`）在 shell/docker 路徑裡不好打，也不利 tab 補完。

## 2. 目前 layout（develop c637777）

```
.dockerignore .env.example .gitignore .gitmodules AGENTS.md README.md(56 KB) pytest.ini
docker/        3 Dockerfile + 3 entrypoint + compose(.serial).yaml + fastdds xml + mirror 腳本 + nav2_fork_packages.txt（全部平放）
docs/          13 個 .md + uwb_study.csv（平放；設計、硬體、研究、進度混在一起）
firmware/sim_client/        micro-ROS 模擬 MCU client
ros2_ws/src/   9 個 shopping_* 套件（平放）
scripts/       16 個腳本平放（dev.sh 是總入口）
tests/         15 個 repo 層級 pytest
third_party/eurobot_nav2    submodule（使用者的 Nav2 fork）
tools/         只有 1 個檔（nav2_inflation_probe.py）
bags/          本地 rosbag（已 gitignore）
```
沒有 `.github/`（無 CI）。整體已經不亂，問題主要是：docker/、docs/、scripts/ 平放太多；tools/ 只有一個檔；沒有 hardware/ 的位置；README 太長。

## 3. 目標結構（最小、可逐步做）

不用數字前綴；保留 `ros2_ws/`、`firmware/`、`third_party/`、`tests/`、`scripts/dev.sh` 這些高引用路徑不動。

```
Almighty-shopping-car/
├── README.md                 精簡：簡介、架構圖、Quick start、文件索引（細節移到 docs/）
├── AGENTS.md  .env.example  .gitignore  .dockerignore  .gitmodules  pytest.ini
├── hardware/                 [新] 硬體：BOM、接線、UWB 錨點擺放、lidar 安裝位置（目前放 README 佔位）
│   └── README.md
├── firmware/
│   └── sim_client/           (不動) 之後真 MCU 韌體放 firmware/<mcu名>/
├── docker/
│   ├── docker-compose.yaml          (不動，dev.sh 依賴)
│   ├── docker-compose.serial.yaml   (不動)
│   ├── common/        configure-ros-mirror.py, fastdds_udp.xml
│   ├── compute/       Dockerfile, entrypoint, nav2_fork_packages.txt
│   ├── display/       Dockerfile, entrypoint
│   └── microros-client/ Dockerfile, entrypoint
├── ros2_ws/src/              (第 4 階段，可選) 套件按子系統分組；套件名不變
│   ├── core/          shopping_interfaces, shopping_robot_description, shopping_robot_bringup
│   ├── simulation/    shopping_robot_gazebo
│   ├── navigation/    shopping_robot_navigation, shopping_robot_slam
│   ├── follow/        shopping_uwb_follow
│   ├── guide/         shopping_zone_manager
│   └── communication/ shopping_mcu_bridge
├── scripts/
│   ├── dev.sh  run_unit_tests.sh    (不動：總入口)
│   ├── container/     split_role.sh, sim_health.py
│   ├── nav2/          nav2_build.sh, check_nav2_contract.py, check_nav2_linkage.py, nav2_inflation_probe.py(←tools/)
│   ├── eval/          fault_check.py, follow_state_stats.py, record_bag.py, scenario_list.py, uwb_study.py
│   └── checks/        check_static.py, check_ros_shutdown.py, map_geometry_check.py, yaw_rate_check.py
├── tests/                    (不動)
├── third_party/eurobot_nav2  (不動，submodule)
└── docs/
    ├── README.md      [新] 文件索引
    ├── PROGRESS.md  ROADMAP.md      (不動：AGENTS.md 與工作流程依賴)
    ├── design/        ARCHITECTURE, CONTAINERS, OBSTACLE_SAFETY, SCENARIOS, METRICS, SLAM
    ├── hardware/      HARDWARE_INTERFACES, MCU_INTERFACE, MICROROS_GUIDE, VALIDATION_ARM64
    ├── research/      UWB_STUDY.md, uwb_study.csv
    └── assets/        [新] 架構圖、截圖、模擬影片 gif
```
（`tools/` 刪除；`bags/` 維持 gitignore 不進 repo。）

## 4. 舊路徑 → 新路徑 對照與引用處

### 階段 1：docs（低風險）
| 舊 | 新 | 需要改的引用 |
|---|---|---|
| docs/ARCHITECTURE.md | docs/design/ARCHITECTURE.md | README.md、docs/*.md 互相連結（相對連結要加 `../`） |
| docs/CONTAINERS.md | docs/design/CONTAINERS.md | README、docs 互連 |
| docs/OBSTACLE_SAFETY.md | docs/design/OBSTACLE_SAFETY.md | README、docs 互連 |
| docs/SCENARIOS.md | docs/design/SCENARIOS.md | README、docs 互連 |
| docs/METRICS.md | docs/design/METRICS.md | README、docs、**ros2_ws/src/shopping_zone_manager/shopping_zone_manager/eval_report.py:91**（報告文字） |
| docs/SLAM.md | docs/design/SLAM.md | README、docs 互連 |
| docs/HARDWARE_INTERFACES.md | docs/hardware/HARDWARE_INTERFACES.md | README、docs |
| docs/MCU_INTERFACE.md | docs/hardware/MCU_INTERFACE.md | README、docs、**ros2_ws/src/shopping_mcu_bridge/test/test_comm_integration_contract.py:74**（測試會讀這個檔！） |
| docs/MICROROS_GUIDE.md | docs/hardware/MICROROS_GUIDE.md | README、docs、firmware/sim_client/README.md |
| docs/VALIDATION_ARM64.md | docs/hardware/VALIDATION_ARM64.md | README、docs |
| docs/UWB_STUDY.md, docs/uwb_study.csv | docs/research/… | README、**scripts/uwb_study.py:19-20**（預設輸出路徑）、ros2_ws/.../shopping_uwb_follow/uwb_study.py、test_uwb_study.py |
| docs/PROGRESS.md, ROADMAP.md | 不動 | — |

### 階段 2：scripts + tools（中低風險）
| 舊 | 新 | 引用處（不含 docs/PROGRESS.md 歷史紀錄，那些不改） |
|---|---|---|
| scripts/split_role.sh, sim_health.py | scripts/container/ | **docker/docker-compose.yaml**（`/repo/scripts/split_role.sh`、healthcheck）、tests/test_container_split.py |
| scripts/nav2_build.sh | scripts/nav2/ | scripts/dev.sh、tests/test_nav2_fork.py |
| scripts/check_nav2_contract.py, check_nav2_linkage.py | scripts/nav2/ | scripts/nav2_build.sh、tests/test_nav2_fork.py、docs/ARCHITECTURE.md |
| tools/nav2_inflation_probe.py | scripts/nav2/ | tests/test_nav2_fork.py |
| scripts/fault_check.py | scripts/eval/ | scripts/dev.sh、README、docs |
| scripts/follow_state_stats.py | scripts/eval/ | tests/test_follow_state_stats_cli.py、README、docs |
| scripts/record_bag.py | scripts/eval/ | scripts/dev.sh、tests/test_bag_pipeline.py、test_eval_repro.py、docs |
| scripts/scenario_list.py | scripts/eval/ | scripts/dev.sh |
| scripts/uwb_study.py | scripts/eval/ | shopping_uwb_follow/uwb_study.py、test_uwb_study.py、docs |
| scripts/check_static.py, check_ros_shutdown.py | scripts/checks/ | README、docs |
| scripts/map_geometry_check.py, yaw_rate_check.py | scripts/checks/ | scripts/dev.sh、tests/test_calibration_tools.py、urdf xacro 註解、README、docs |
| scripts/dev.sh, run_unit_tests.sh | 不動 | — |
**注意**：幾乎每支 python 腳本都用 `ROOT = Path(__file__).resolve().parents[1]` 之類找 repo 根目錄，搬進子資料夾後要改成 `parents[2]`（每支都要檢查）。

### 階段 3：docker（中風險）
| 舊 | 新 | 引用處 |
|---|---|---|
| docker/Dockerfile.compute | docker/compute/Dockerfile | docker-compose.yaml（`dockerfile:`）、scripts/dev.sh、scripts/nav2_build.sh、tests/test_container_config.py、test_nav2_fork.py、docs/CONTAINERS.md、VALIDATION_ARM64.md |
| docker/compute-entrypoint | docker/compute/entrypoint | Dockerfile.compute、**Dockerfile.display**（也 COPY 這支）的 COPY 行、tests/test_container_config.py |
| docker/nav2_fork_packages.txt | docker/compute/ | scripts/nav2_build.sh、tests/test_nav2_fork.py |
| docker/Dockerfile.display, display-entrypoint | docker/display/{Dockerfile,entrypoint} | docker-compose.yaml、test_container_config.py |
| docker/Dockerfile.microros-client, microros-client-entrypoint | docker/microros-client/{Dockerfile,entrypoint} | docker-compose.yaml、test_agent_e2e.py、test_comm_integration_contract.py、docs/MICROROS_GUIDE.md |
| docker/configure-ros-mirror.py | docker/common/ | 3 個 Dockerfile 的 COPY |
| docker/fastdds_udp.xml | docker/common/ | docker-compose.yaml 兩處 volume（`./fastdds_udp.xml`）、docker-compose.serial.yaml、tests |
build context 仍是 repo 根（`context: ..`），COPY 路徑改成 `docker/compute/entrypoint` 等即可；image 名不變所以不用重建流程以外的改動。

### 階段 4（可選）：ros2_ws/src 子系統分組（中高風險，建議最後或不做）
colcon 會遞迴找 package.xml，套件名不變，`ros2 launch/run` 不受影響。但要改：
- **pytest.ini** `testpaths`/`pythonpath`（5 條路徑）
- **.gitignore** 6 行 `ros2_ws/src/shopping_robot_navigation/maps/...`
- **docker/Dockerfile.microros-client** 2 行 COPY（mcu_bridge interface.yaml、urdf xacro）
- tests/*.py 9 支、ros2_ws 內 test 7 支、scripts/*.py 約 9 支用 `ros2_ws/src/shopping_*` 直接拼路徑
- **執行期程式碼**：`shopping_zone_manager/map_tour.py`（2 處 `/ws/src/shopping_...`）、scripts/dev.sh（`/ws/src/shopping_robot_gazebo/worlds/supermarket.sdf`）、check_nav2_contract/linkage.py → 建議順便改用 `get_package_share_directory()`/`ros2 pkg prefix`，以後就不怕搬
- README（7 處）、docs/SCENARIOS(8)、SLAM(4)、METRICS(2)、CONTAINERS(1)、maps/README(2)
- 已存在的 colcon `build/ install/` volume 需清掉重建（`dev.sh` 的 clean build 一次）

## 5. 風險
1. **與進行中分支衝突**：`codex/follow-los`、`codex/dynamic-scenarios` 尚未合併，會改到 scripts/、tests/、ros2_ws/src/shopping_uwb_follow、zone_manager。→ **等這兩支合進 develop 後再搬**，或搬完後立刻 rebase；git 的 rename 偵測一般能處理，但它們新加在舊資料夾的檔案（例如新腳本）會留在舊路徑，要手動補搬。
2. Python 腳本 `parents[1]` 找 repo 根 → 搬進子資料夾會靜默算錯路徑（只在執行時爆）。每支要改並靠 tests 驗。
3. compose 內的容器路徑（`/repo/scripts/...`、`/ws/src/...`）只有真的起容器才會發現錯 → 每階段都要跑一次 sim smoke。
4. docs 相對連結斷掉不會讓測試失敗 → 用簡單 link checker（grep `](` 後檢查檔案存在）驗。
5. docs/PROGRESS.md 裡的歷史路徑**不要改**（是紀錄），只在檔頭註明重整日期。
6. 使用者 Mac arm64 本地若有舊 colcon volume，第 4 階段後要清 volume 重建。
7. 外部筆記/例行任務提示（例如 /workspace/wt/logs、routine 指令）若寫死 `scripts/xxx.py` 路徑需同步更新；`scripts/dev.sh`、`docs/PROGRESS.md` 保持不動就是為了降低這個風險。

## 6. 建議執行順序（每步一個 commit，保持 build/test 綠燈）
0. 前置：follow-los、dynamic-scenarios 合進 develop（或確定放棄）；開 `codex/restructure` 分支從 develop。記錄基準：`scripts/run_unit_tests.sh` 全綠、`pytest` 全綠、`dev.sh` build OK。
1. **docs**：`git mkdir`+`git mv` 到 design/hardware/research；新增 docs/README.md 索引、docs/assets/；改 README 與 docs 內相對連結、test_comm_integration_contract.py:74、eval_report.py:91、uwb_study.py 預設輸出。→ 跑 pytest + link check。commit。
2. **hardware/** 佔位 README（UWB 錨點、lidar 位置待定、MCU 型號待確認）。commit。
3. **scripts + tools**：`git mv` 進 container/nav2/eval/checks，刪 tools/；修 `parents[N]`、dev.sh、compose、tests。→ pytest、`bash -n scripts/dev.sh`、`docker compose config`（不起容器、不吃 CPU）、一次 sim smoke（split profile 起得來、sim_health 綠）。commit。
4. **docker**：`git mv` 到 compute/display/microros-client/common；改 compose `dockerfile:`、volume、Dockerfile COPY、tests。→ `docker compose config`、build compute + microros-client image、comm-e2e-check、nav-check 一輪。commit。
5. **README 瘦身**：把細節段落移到 docs/（例如 Quick start 留在 README，指令大全移 docs/design/CONTAINERS.md 或新 docs/USAGE.md）。commit。
6. （可選）**ros2_ws/src 分組**：先把寫死 `/ws/src/shopping_*` 的執行期程式改成 package share 查詢（獨立 commit、驗證），再 `git mv` 分組 + 改 pytest.ini/.gitignore/Dockerfile/tests。→ 清 colcon volume、全量 build、完整回歸（nav-check 6/6 default+alt_start、loc-check、obstacle/follow fault-check）。commit。
7. PR → develop，自審 + CI 結果，合併後刪分支；PROGRESS.md 記錄。

工作量估計：階段 1–5 約半天（Codex 一輪 + 驗證）；階段 6 再加一輪完整回歸。
