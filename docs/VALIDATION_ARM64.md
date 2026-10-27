# 2026-10-26 Mac arm64 驗收

環境為 macOS Apple Silicon、Docker Desktop 4.40.0、Linux arm64 引擎（8 CPU／約 23.4 GiB RAM），Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 6.18.0。以 `develop` 的 `a929dc6` 為起點，修改整理於 `codex/mac-arm64-slam`；Nav2 私有子模組為 `bf60ee0`。所有映像原生建置，沒有指定 platform。

本機使用根目錄 `.env`：noVNC 6081、VNC 5970；另一專案的 6080 保留。`dev.sh novnc` 顯示實際發布位址。

## 自行驗證結果

| 功能 | 結果與證據 |
| --- | --- |
| 原生映像 | compute、display、micro-ROS C client 均為 linux/arm64；`output/container-arm64/build-final.log`、`images.txt` |
| ROS 套件 | 9 套件建置、colcon 721 tests／0 errors／0 failures／5 skipped；`colcon-test-delivery.log` |
| Nav2 私有 fork | 18 套件建置；110 項動態連結檢查 PASS；尚須下方功能驗收 |
| 純 Python | 1035 passed；`pytest-delivery.log` |
| 靜態檢查 | YAML、XML、Python、Bash、七種情境及兩／三錨點 xacro 展開 PASS；`static-delivery.log` |
| 桌面 | 雙容器 healthy；HTTP 200、noVNC 密碼登入、RViz 軟體 OpenGL 4.5；`graphics.log`、`novnc-rviz-sim.png` |
| 即時資料／TF | 時鐘、雷射、IMU、EKF／真值里程計、TF、三個 Nav2 lifecycle 與唯一 watchdog 發布者共 14 項 PASS；`guide-runtime.json` |
| apt Nav2 | nav-check 6/6 PASS；終點真值誤差 0.141 m；`nav-check.log` |
| AMCL 格網定位 | 全新模擬六區 PASS；定位 RMSE 0.0498 m，最大 0.1029 m／2.0044°；`amcl-localization.json` |
| 建圖 | map-tour 17/17 航點成功，無跳過；格網與位姿圖四檔保存為 `arm64_supermarket.*`；`map-tour-retry.log` |
| slam_toolbox 位姿圖定位 | 冷啟動重測六區 PASS，最大 0.0184 m／0.459°；首輪 lifecycle 未就緒失敗保留於 `slam-localization.json`，成功重測為 `slam-localization-retry.json` |
| 訊號收尾 | 獨立 DDS domain 中，watchdog 收到 SIGINT／SIGTERM 都先送零速、結束碼 0；`shutdown-check.log` |
| 錄製／轉換／重跑／彙整 | `arm64_amcl_01` 全流程成功，`eval-verify` Report matches；真值最小靜態淨空 0.349 m。loc-check 直接送 action，未發布購物狀態事件，故此 bag 不含導航 episode，不能宣稱成功率已驗證 |
| MCU 通訊 | Python 10/10、PTY serial 10/10、C／XRCE 經 agent 7/7（重測兩輪）；comm 啟動修正後 compute ID 保持不變、noVNC HTTP 200 |
| 人物同步 | 舊 CLI 控制 FAIL，位置差平均 0.089 m、延遲 0.25 s；改用持續服務連線後全部 PASS，平均 0.0096 m、延遲 0.03 s（`person-sync-retry.json`） |
| 跟隨（原 NavFn） | 最終 120 模擬秒：直線／轉彎距離誤差 RMSE 1.121／1.200 m，皆低於 1.5 m，濾波延遲 0.08 s；未出現過近／重取得事件，整體標示 NOT_COVERED，不能宣稱全部項目 PASS。`follow-eval-tf-cache-fix.json`；Smac2D 結果另列 |
| 故障與模式（原 NavFn） | guard＋collision_monitor：obstacle 4/4、follow 7/7；新增平移基線與平面真值速度檢查。急停／重設／模式切換 15/15；急停命令／真值停止 0.004／0.081 壁鐘秒。`fault-obstacle-collision-retry.log`、`fault-follow-collision.log`、`mode-check.log` |
| 新圖重載定位 | 新圖 `arm64_smac_slam` 在全新 slam_toolbox localization 六區 PASS，位置 RMSE 0.02049 m、最大 0.03130 m／0.41884°；`slam-live-localization.json`。即時感測／TF／Nav2／唯一 watchdog 14 項 PASS，`slam-runtime.json` |
| Smac2D 建圖 | Humble apt 插件成功載入；新圖 `arm64_smac_slam` 完整巡航 17/17，四檔保存，`slam-live-tour.log`；幾何量測 `slam-live-geometry.json`，不以缺少外牆面推論全圖準確 |
| Smac2D 情境 | alt_start 兩個 action 成功；入口真值 yaw 誤差 0.558 rad 未達情境建議。動態行人的時間戳、實際移動及雷射可見性三種情境都 PASS。aisle_crossing 導航仍逾時；narrow_pass 兩個 action 成功，但 household 真值誤差 1.878 m，判為未通過；checkout_queue 兩個位置誤差 0.135／0.129 m，入口 yaw 0.439 rad 未達建議。`guide-scenarios-smac-verified.json`，不能以 action 成功取代真值驗收 |
| SLAM 完善 | 專用俯視 RViz、可選設定路徑；定位前檢查位姿圖兩檔及有限起點；save-map 與巡航共用結果碼／四個有更新的非空檔案檢查；先寫入同目錄暫存區，整組檢查通過後才替換目的檔，失敗不以舊圖冒充成功。localization 服務空回覆成功卻未寫圖的負例被正確拒絕，`slam-manual-save.log`；不可在該模式另存位姿圖。實際缺圖啟動以結束碼 1 拒絕，`slam-missing-graph.log`。暫存儲存實測成功；localization 負例結束碼 1，原四檔 SHA-256 全部不變且無暫存殘留，`slam-staged-negative.json` |
| Smac2D 跟隨情境 | 新增過近解除遲滯與大角度原地對準後，stop-go 直線 RMSE 0.341 m，五次過近停止 PASS；turns 直線／轉彎 RMSE 1.227／1.366 m，皆低於 1.5 m。兩種人物同步重測 PASS，平均差 0.0047／0.0087 m、延遲 0.03 s。turns 仍有腳本人接觸停止車體後的真值微動 FAIL；命令維持零，不能宣稱無碰撞或整體通過。`smac-aligned-*-follow-eval.json`；缺少事件的項目保持 NOT_COVERED |
| 角色拆分 | sim／localization／navigation／app／display 實際啟動；14 項即時檢查及 nav-check 6/6 PASS，終點真值誤差 0.139 m。`split-guide-runtime.json`、`split-guide-nav-check.log` |
| fork 導航與底盤旋轉 | 本輪未重做 fork 導航與 yaw-check，不以建置／pytest 取代功能驗收 |
| Linux amd64 | 使用者將另外執行 Linux 回歸；本次沒有新的 Linux amd64 原生結果。既有 2026-10-24 結果不代表本次修改已驗證 |

日誌與 bag 位於忽略版控的 `output/container-arm64/`、`bags/`。表格是自行測試結果，審查者驗證欄仍由審查者填寫。

## 已修正問題與重現方式

- `dev.sh` 原先未載入儲存庫根目錄 `.env`，noVNC 固定輸出預設連接埠；現在明確載入並查詢 compute 第 1 個服務實例，避免誤選臨時建置容器。
- 情境 pytest 寫死另一環境的 Python 路徑；改用目前直譯器。Nav2 remap 測試明確隔離 fork underlay 環境。
- Bash 3.2 的 nounset 不接受空陣列展開，`eval-summary` 不帶 `--output` 時會退出；已加入分支，Mac 內建 Bash 與容器測試皆通過。
- apt 隔離子程序傳入空 `map:=`，使已安裝 fork 的 mapping 無法啟動；略過空 CLI 值後，完整建圖巡航成功。
- 正常中斷時 rclpy 先關閉 context，使零速清理與第二次 shutdown 拋錯；長駐節點改為自行處理 SIGINT／SIGTERM，清理完成才關閉 context。
- 首輪 slam_toolbox 定位啟動卡在 behavior_server configure 回應，其他感測／TF 正常且 watchdog 維持零速；訊息與 [rmw_fastrtps #842](https://github.com/ros2/rmw_fastrtps/issues/842) 相符，這是根因推論。增加 `service` writer 的 `max_blocking_time` 為 3 秒作為緩解，不宣稱已修正上游競態。[Humble service profile 載入實作](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_cpp/src/rmw_service.cpp)、[回應等待實作](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)；Fast DDS API 實測服務為 3 秒，一般 topic 仍為 0.1 秒（`fastdds-qos.log`）。

動態行人 bag `arm64_smac_verified_narrow_pass_01` 成功轉換，3390 筆人物淨空樣本、2 個完整導航 episode；人物最小淨空 0（接觸），household 終點誤差 1.894 m。雖 action 成功率欄位為 1.0，仍判為未通過情境驗收；`report.json` 及 `observations.jsonl.meta.json` 保留。

第一筆 `arm64_nav_01` 在 nav-check 開始後才錄製，轉換器拒絕不完整的導航開頭；保留該負例，重新於測試前開始錄製。

## 重跑

```bash
cp .env.example .env  # Only for a new checkout without .env.
./scripts/dev.sh build compute display
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh test
./scripts/dev.sh up
./scripts/dev.sh novnc
docker compose --env-file .env -f docker/docker-compose.yaml run --rm --no-deps compute python3 /repo/scripts/check_ros_shutdown.py
```

模擬與故障驗收命令見 CONTAINERS、SCENARIOS 與 README。每次僅有一個控制工作階段；建置期間先停止模擬。micro-ROS 的 C client 以 `comm-e2e-up` 明確選用，不開啟任何硬體裝置。

SLAM 的建圖、檢視、儲存與重載步驟見 [SLAM 操作說明](SLAM.md)。依使用者建議，apt 規劃器已改為 Humble 的 `nav2_smac_planner/SmacPlanner2D`，保留原 0.8 m 終點搜尋容差；fork 現亦改用其原始碼建置的 Smac2D，避免混用不同 underlay 的 C++ 外掛；此變更的建置與模擬驗收待重新執行，上方既有建置結果不涵蓋此次切換。[Humble Smac2D 實作](https://api.nav2.org/nav2-humble/html/smac__planner__2d_8cpp_source.html)。

使用者已授權提交及推送，並明確要求不開 PR。本次僅推送功能分支，不合併 `develop`。所有審查者欄位保留待驗證。
