# 工作進度

## 2026-10-23：建立模擬開發骨架

### 本次完成

- 保留專案名稱與畢業專題說明，重寫繁體中文快速開始、Mac／fish 操作、架構與時程。
- 建立七個 Humble 套件、套件描述與安裝設定，分離純 Python 邏輯與 ROS 節點。
- 建立 Fortress 四輪模型、雷射、IMU、輪式里程計、錨點座標與超市碰撞幾何。
- 建立共用人物路徑、離線圓柱人物、位姿服務更新及真值橋接。
- 建立建圖／定位／Nav2／RViz 啟動流程、區域設定、動作節點與命令列工具。
- 建立 UWB 噪聲、NLOS、掉包、兩／三錨點估測、濾波、安全停止與控制限制。
- 建立雙容器、Xvfb／noVNC、UDPv4 DDS、操作腳本及純邏輯測試。
- 建立 pytest、xacro、YAML、Python 與 XML 檢查流程；本紀錄的驗證結果由審查者另填。

### 已知限制與下一步

- 尚未執行 Docker、ROS 或 Gazebo；優先於 Docker Desktop 驗證映像、外掛、DDS、感測器、模型真值主題與生命週期。
- 指定虛擬環境的 `python -m xacro` 缺少套件入口；可用 `python -c 'import xacro; xacro.main()'` 執行展開，仍須由審查者確認。
- UWB 為平面模型；Float32MultiArray 尚無來源時間戳，硬體前必須升級介面。
- 人物服務更新約 10 Hz，視覺位置與分析路徑可能有短暫更新延遲。
- 無既有地圖；導航預設即時建圖，保存並重新載入後需核對區域座標。
- 尚無模式管理器、速度仲裁、獨立命令逾時看門狗或跟隨繞障；目前須互斥執行控制流程。
- 所有硬體仍未實作，預設模擬。下一階段先完成模擬啟動與 SLAM 建圖。

### 驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python pytest | 見下方「盒內整合驗證」 |
| xacro 展開（預設三錨點及兩錨點） | 見下方「盒內整合驗證」 |
| 所有 YAML 解析 | 見下方「盒內整合驗證」 |
| launch 與節點 Python 語法 | 見下方「盒內整合驗證」 |
| SDF／URDF／套件 XML | 見下方「盒內整合驗證」 |
| Bash 語法 | 見下方「盒內整合驗證」 |
| Docker 映像建置與 Compose | 見下方「盒內整合驗證」 |
| Fortress／DDS／RViz／雷射 | 見下方「盒內整合驗證」 |
| SLAM／Nav2／區域導航 | 見下方「盒內整合驗證」 |
| 人物真值與 UWB 跟隨安全 | 見下方「盒內整合驗證」 |

## 2026-10-23：重啟後續作與小型商店補齊

### 本次完成

- 先檢查既有 WIP 提交與檔案，保留七個套件及已完成的模擬／導航／跟隨骨架，未建立新提交或推送。
- 將展示世界縮為 12 × 10 公尺的小型模擬商店，配置恰好四排貨架、三條寬 1.6 公尺的走道、入口與結帳櫃臺。
- 調整唯一人物路徑來源，循環走過三條走道；六個區域目標移至自由空間。明確設定出生姿態 `(0, 0, 0.02)`、yaw = 0，文件說明世界／地圖原點與重新定位的對齊要求。
- 新增直接解析 SDF 碰撞盒的測試，檢查貨架數量、走道寬度、出生點、所有區域目標，以及人物航點、連線與閉合路段的淨空。
- 兩個 Dockerfile 新增空值預設的 `ROS_APT_MIRROR`，Compose 傳遞建置參數；共用程式在 apt 更新前處理 deb822 與舊版來源，保留簽章設定，並以暫存來源檔離線測試改寫。
- 統一純邏輯與 ROS 節點的前方雷射判讀，涵蓋無效角度、空掃描、NaN、負值、無限遠、量測上下限及零距離；無有效前方資料時停止。控制時間倒退或無效時歸零，停用服務同步清除控制器速度。
- 加強區域、測距、估測與濾波輸入檢查；擴充別名正規化、噪聲統計、掉包、NLOS、解算歧義、品質、逾時、立即停止與速度／加速度邊界測試。
- `goto_zone` 使用獨立 ROS 節點名稱，避免與常駐區域節點重名。
- 補齊 README 的「2027 年 1 月在 Mac 上執行」、clone、fish、Docker Desktop、鏡像與 noVNC 操作；新增 `mapping`／`guide` 指令，保留原有別名。建置與測試可在 `up` 前透過暫時運算容器執行。
- 更新架構及時程，新增可重跑的 `scripts/check_static.py`，以 `import xacro` 展開並檢查兩／三錨點與設定一致。

### 已執行檢查與方法（執行紀錄，非審查驗證）

以下皆在指定沙盒虛擬環境執行，沒有執行 Docker：

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 147 個測試通過，包含場景淨空及來源改寫 |
| `./scripts/run_unit_tests.sh /workspace/scratch/venv-shop/bin/python` | 相同測試集合通過 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | 9 個 YAML／RViz 檔案解析、10 個 XML／SDF／xacro 檔案解析、31 個 Python 檔案以 `py_compile` 檢查、4 個 Bash 檔案逐一 `bash -n`，均通過 |
| 同上，呼叫 `xacro.process_file` | 預設三錨點與兩錨點均展開成功；展開 URDF 以 XML 解析並核對四輪及錨點座標 |
| `git diff --check` | 無空白格式錯誤 |

### 尚未驗證、限制與下一步

- 尚未建置或執行容器；上述結果僅屬純邏輯與靜態檢查，ROS launch 接線不等於 Gazebo／Nav2 執行成功。
- 下一步在 Mac Docker Desktop 依 README 建置映像，確認實際 apt 鏡像可達、amd64／arm64 套件、DDS、Fortress 外掛、TF、雷射與 RViz，再進行三條走道建圖與六區導航。
- 世界、`odom` 與 `map` 的平面對齊仍須以模擬建圖及地圖重新載入確認；外部地圖不可直接假定原點相同。
- 人物圓柱每 0.1 秒透過服務移動，UWB 採同一路徑解析值；服務延遲下的畫面／碰撞位置同步仍待整合量測。
- 保留既有平面 UWB、無來源時間戳訊息、兩錨點前後歧義及 NLOS 限制；尚無模式管理器、命令仲裁、獨立看門狗與跟隨繞障，操作流程須互斥。
- 所有預設值仍為模擬，沒有新增硬體驅動或裝置連線。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純邏輯、安全行為與場景淨空 | 見下方「盒內整合驗證」 |
| xacro／XML／YAML／RViz／Python／Bash | 見下方「盒內整合驗證」 |
| ROS apt 鏡像、Compose 與雙架構映像 | 見下方「盒內整合驗證」 |
| Mac／fish 操作、noVNC、DDS 與 RViz | 見下方「盒內整合驗證」 |
| Fortress、SLAM、Nav2、人物真值與跟隨 | 見下方「盒內整合驗證」 |

### 同日追加：Docker 審查回饋修正

- 審查者回報：Humble／Fortress 無畫面環境七套件建置成功，建圖可產生 `/map`，`/scan` 約 7.4 Hz、`/odom` 約 37 Hz，底盤可由 `/cmd_vel` 驅動；引導模式 Nav2 生命週期可啟用。保留審查者已修正的 `use_composition: 'False'`，本輪未改動。
- `sim.launch.py` 新增預設為 false 的 `person` 參數，只有 true 才啟動人物路徑節點。人物初始停在店外 `(20, 20)`；建圖與引導預設不含人物活動，可傳入 `person:=true` 測試動態障礙；跟隨固定傳入 true。
- 人物唯一 YAML 路徑改從 `(1.5, 0)` 沿自由橫向通道向 +x 開始，再循環三條走道。場景測試改驗證店外停放、完整閉合路徑淨空，以及出生點至少 0.7 公尺淨空與雷射前方 ±30 度至 0.7 公尺的自由空間。
- 依本輪明確要求調整障礙行為：`OBSTACLE_STOP` 遇有效障礙時立即停止前進，目標有效且不過近時可原地轉向，保留角速度及角加速度限制；雷射失效、TOO_CLOSE、LOST、停用與無效控制間隔仍立即零 Twist。這是本輪對既有「前方障礙一律零 Twist」規範的指定例外。
- 新增不依賴 ROS 的 `follow_metrics.py`，對時間序列計算轉換後估測誤差平均值／RMSE／最大值及真實跟隨距離平均／最小／最大值。新增 `uwb_eval_node` 並接入跟隨 launch，以 `/odom` 轉換估測座標，真值可用且三筆時間戳差不超過 0.15 秒時發布 `/uwb/estimate_error`（Float32，公尺）。
- 兩個 ament_python 套件的 `setup.py` 加入 `tests_require=['pytest']`；新增測試確認設定、人物 launch 接線與 pytest 檔案未匯入 rclpy。擴充障礙轉向、故障立即停止、恢復、座標旋轉平移、時間差與統計測試。
- 更新 README 及架構文件的人物參數、狀態表、評估介面與限制；未提交或推送。

#### 本輪執行紀錄（非審查驗證）

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q`（專案根目錄） | 169 個測試通過，無需 ROS／rclpy |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | 9 個 YAML／RViz、10 個 XML／SDF／xacro、35 個 Python 的 py_compile、4 個 Bash 的 bash -n 均通過；兩／三錨點 xacro 展開與設定一致性通過 |
| `git diff --check` | 無空白格式錯誤 |

#### 限制與下一步

- 本輪沒有執行 Docker 或 ROS。需由審查者重新建置，確認 `colcon test` 實際使用 pytest 收集套件測試，再重跑預設無人物的建圖及 `goto_zone snacks`。
- 請在跟隨模式確認起始前進、障礙原地轉向、雷射失效完全停止及 `/uwb/estimate_error` 發布；靜態接線與純測試不能視為容器驗收。
- 評估使用最新資料而不插值，假設 sim_world 與 odom 初始座標一致；誤差包含里程計漂移、時間差與濾波延遲。人物真值仍為解析路徑，不是 set_pose 實際到位回報；服務延遲仍須整合量測。
- 原地轉向仍無側向／後方掃掠檢查或自主繞障，控制流程仍需互斥。硬體預設與範圍維持不變。

#### 本輪審查驗證

| 項目 | 結果 |
| --- | --- |
| 純邏輯、障礙轉向與故障停止 | 見下方「盒內整合驗證」 |
| 人物停放、參數接線與路徑淨空 | 見下方「盒內整合驗證」 |
| 座標轉換、評估統計與誤差主題 | 見下方「盒內整合驗證」 |
| colcon test 的 pytest 收集與執行 | 見下方「盒內整合驗證」 |
| 無人物建圖、snacks 區域導航與跟隨整合 | 見下方「盒內整合驗證」 |

## 2026-10-23：盒內整合驗證與導航修正

在 Linux amd64（Debian 13 VM、Docker Engine、無 GPU）上實際建置並執行兩個容器，結果如下。

### 本次完成

- 導航控制器由 DWB 改為 Regulated Pure Pursuit（先原地轉向再前進），全域／區域膨脹半徑 0.8 公尺、cost_scaling_factor 1.5，目標位置容差 0.15 公尺。DWB 在走道入口切角後「No valid trajectories」卡住。
- 模擬車體：DiffDrive 等效輪距乘上 `skid_track_factor`（2.0），車輪摩擦改為 0.5／0.5。原設定下小角速度（≤ 0.2 rad/s）命令完全無法轉向，RPP 的角加速度回授因此停在低速，車體只會側滑。
- `/odom` 改用 OdometryPublisher（模擬真值）；DiffDrive 輪式里程計改發布 `/wheel_odom`。
- 世界加入四個不對稱地標，slam_toolbox 限制迴圈搜尋範圍（`loop_search_space_dimension` 1.5、`loop_search_maximum_distance` 2.0）。修正前地圖曾整體偏移一個走道（map→odom 平移 2.41 公尺）。
- 顯示容器共用運算容器網路命名空間，連接埠可用 `NOVNC_HOST_PORT`／`VNC_HOST_PORT` 調整。
- 文件補上設計決策、Mac arm64／Linux 支援說明，移除對車載電腦的假設。

### 已驗證（方法）

| 項目 | 結果與方法 |
| --- | --- |
| 映像建置 | `./scripts/dev.sh build` 成功（amd64；ROS 套件使用 `ROS_APT_MIRROR` 鏡像） |
| colcon 建置 | `./scripts/dev.sh colcon-build`：7 個套件成功 |
| colcon 測試 | `./scripts/dev.sh colcon-test`：159 個測試，0 錯誤、0 失敗 |
| 純 Python 測試 | `./scripts/dev.sh test`：169 個通過；容器外 Python 3.10 虛擬環境相同結果 |
| 靜態檢查 | `scripts/check_static.py`：YAML／RViz、XML／SDF／xacro、Python、Bash 與兩／三錨點展開全部通過 |
| 感測器與 TF | 無畫面建圖時 `/scan` 約 7.4 Hz、`/odom` 約 50 Hz，`/map` 產生；`/cmd_vel` 可驅動，開迴路角速度 0.1–0.3 rad/s 均能轉向 |
| RViz／noVNC | noVNC HTTP 200；以 VNC 截圖確認顯示容器 RViz 呈現地圖、雷射與車體 |
| 區域導航 | `guide`（即時建圖）下以 `./scripts/dev.sh goto` 連續送出 10 個目標（入口、零食區、生鮮區、日用品區、飲料區、結帳區、零食區、日用品區、`飲料區` 中文別名、入口），bt_navigator 記錄 10 次成功、0 次失敗；以模擬真值核對最終位置距目標約 0.1–0.16 公尺，結束時 map→odom 平移約 0.003 公尺 |
| 跟隨流程 | `follow` 下監看 150 秒：狀態在 FOLLOWING／OBSTACLE_STOP／TOO_CLOSE 間轉換；`/uwb/estimate_error` 1102 筆，平均 0.11 公尺、RMSE 0.16 公尺、最大 0.88 公尺；車體跟進第一條走道至走道末端 |

### 尚未驗證／已知問題

- Mac arm64（Docker Desktop）建置與執行；Gazebo GUI（`gz-gui`）；任何真實硬體。
- 跟隨：人物在走道末端轉彎時，車體因前方 0.65 公尺內的貨架／端架停在 OBSTACLE_STOP，之後大多原地轉向；人物腳本路徑不避讓，曾走到距車體 0.19 公尺（TOO_CLOSE 正確停車）。需要跟隨繞障或改由 Nav2 追隨（時程階段 4）。
- 導航取消／拒絕、AMCL 與已存地圖定位模式、有人物時的動態障礙。
- 里程計為模擬真值；真實噪聲、IMU／輪速 EKF 與 `skid_track_factor` 需在實車校正。
- 啟動後立刻送目標，偶爾遇到 DDS 動作回應遺失（「Failed to send goal response」），`goto` 等到逾時；建議 Nav2 啟動後等待約 60 秒再送目標。

## 2026-10-23：UWB 錨點與噪聲離線研究

### 本次完成

- 新增不依賴 ROS／numpy 的 `uwb_study.py` 純邏輯與命令入口；僅使用標準函式庫及 PyYAML。重用既有測距、估測與 EMA，掃描兩／三錨點幾何、噪聲、偏差、NLOS、掉訊、品質門檻、更新率及濾波係數。
- 以種子 42、每靜態點 40 次試驗執行完整研究；120 個靜態點、26 組靜態條件、18 組完整路徑重播條件，輸出 3164 列 CSV（含每點及彙總）。完整運算約 4–5 秒，實測值與重現指令見 [研究報告](UWB_STUDY.md)，原始統計見 [CSV](uwb_study.csv)。README 加入報告連結。
- 未初始化兩錨點翻側率 49.74%，預設三錨點 0.07%；三錨點靜態 RMSE 0.1981 公尺，擴大三角形 0.1437 公尺，但擴大配置部分超出預估車體範圍。兩錨點精確先前位置僅作樂觀上界。
- NLOS 機率 10%、偏差 0.5 公尺時 RMSE 1.5554 公尺；掉訊 30% 時 RMSE 1.7926 公尺、無效率 25.29%。三錨點掉訊可能回退兩錨點解，品質門檻不能保證排除翻側。
- 合成軌跡沿唯一 `person_path.yaml` 路徑，車體落後人物 1 公尺路程，直線相距 1 公尺；轉角朝向瞬間改變。alpha=0.6、無掉訊下 5／10／20 Hz 的 RMSE 為 0.0901／0.0713／0.0596 公尺。暫保留預設參數，建議以至少 10 Hz 為後續整合起點。
- 新增種子重現、無噪聲幾何、兩錨點歧義、掉訊／NLOS、品質拒收、軌跡延遲、CSV、報告與 CLI 測試。不修改執行期設定、xacro 或 ROADMAP，不提交或推送。

### 限制與下一步

- 僅二維獨立高斯噪聲、固定正 NLOS 偏差與獨立掉訊，未納入標籤高度、人體遮蔽、貨架多重路徑、相關性、封包時間差及控制動力學。單因子掃描未涵蓋完整交互作用。
- 未執行 Docker、ROS／Gazebo 整合或任何硬體驗收。2027 年 2 月後須量測實際安裝座標、各距離／方位測距分布、貨架／人體 NLOS、連續掉訊、品質可信度及控制停止距離，再校正研究模型與參數。
- 延遲依更新週期離散搜尋，零值不代表無延遲；誤差僅計算新發布的有效估測，未涵蓋失效期間保持舊目標的安全行為。靜態檢查與離線測試不能取代整合驗收。

### 本輪執行紀錄（非審查驗證）

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python scripts/uwb_study.py` | 完整研究 4.24 秒；種子 42、每點 40 次，輸出報告與 3164 列 CSV |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 179 個測試通過（含新增 10 個），2.97 秒 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：9 個 YAML／RViz、10 個 XML／SDF／xacro、38 個 Python、4 個 Bash；兩／三錨點展開與設定一致性通過 |
| `git diff --check` 與新檔空白檢查 | 無空白格式錯誤；新模組沒有 rclpy／numpy 匯入 |

### 本輪審查驗證（2026-10-23 08:25 臺北時間，盒內 Linux amd64）

| 項目 | 結果 |
| --- | --- |
| 純 Python 測試 | `pytest -q` 179 個通過；修正測試路徑，使 colcon 掛載的 `/ws/src`（無儲存庫根目錄）也能找到人物路徑 |
| colcon 建置與測試 | `./scripts/dev.sh colcon-build` 7 套件成功；`colcon-test` 169 個測試、0 錯誤、0 失敗、1 略過（CLI 測試需要儲存庫的 `scripts/`，容器內略過，由 repo 層 pytest 覆蓋） |
| 容器內純 Python | `./scripts/dev.sh test` 179 個通過 |
| 離線數據結論 | 已審閱表格與結論一致：兩錨點未初始化約一半翻側，支持三錨點決策 |
| 未驗證 | Gazebo 整合、實際安裝座標、貨架／人體 NLOS、真實 UWB 模組 |

另一發現（尚未處理）：三錨點掉一筆時回退兩錨點解，在無先前位置時固定選前方解，掉訊 10% 即使 RMSE 由 0.20 升到 1.23 公尺。節點會傳入先前位置，實際影響較小，但可列為後續改進（例如掉訊時保持上一筆或要求三筆有效）。

## 2026-10-23：跟隨走道末端繞行（Nav2 交接）

### 本次修改

- 新增不匯入 ROS 的 `follow_handoff.py`：雷射走廊視線檢查、持續受阻延遲、DIRECT／NAV2 交接、目標更新節流、恢復延遲與失敗冷卻。參數拒絕非有限值或非正值，時間倒退取消交接。
- 跟隨節點加入 Humble NavigateToPose ActionClient、量測時間的 map／雷射 TF 轉換、map 目標與朝向、`/follow/handoff_goal`；保留直接控制狀態名稱，新增 NAV2_HANDOFF。過近、遺失／過期、停用或掃描失效立即取消並輸出零命令。
- 導航期間停止直接速度發布；目標更新直接送出 NavigateToPose，使用 Nav2 搶占，不取消或排空等待。新增純邏輯 GoalBook 世代編號，舊目標的回覆、結果及取消確認全部忽略，延遲接受也不取消目前目標。更新距離／週期預設改為 0.4 公尺／0.5 秒，YAML 說明搶占可降低更新成本；以實際送出位置及時間重新記錄。僅 NAV2→DIRECT 取消時排空：等終止確認且外部 `/cmd_vel` 安靜 0.3 秒或最新命令近零（線速度絕對值 <0.02、角速度絕對值 <0.05）；終止確認後最多等 2 秒便恢復，不略過終止確認。Humble 的 rclpy 回呼不提供發布端 GID，改為只在 NAV2／排空期間觀察 `/cmd_vel`，並略過本節點 0.2 秒內自行發布的零速，純邏輯 VelocityDrain 決定恢復時機；第一筆直接命令為零並重設內部速度。取消涵蓋尚未接受的請求，收到取消確認後不在每個 tick 重送，仍等待終止結果。傳輸例外無法確認遠端執行狀態時保留控制權，須停止流程並人工檢查。
- `follow.launch.py` 納入 SLAM／Nav2，傳遞 `slam_mode`／`map`／`use_sim_time`，新增 `route_around`（預設 true），保留 `person:=true`，不啟動 zone_navigator。false 時不建立動作客戶端，恢復直接控制；launch 仍啟動導航服務。補齊 nav2_msgs、tf2_ros、action_msgs 執行依賴，入口點未改。
- 目標採人物 map 座標，NavFn 共用容差由 0.3 提高為 0.8 公尺，允許人物旁自由終點；引導模式受阻區域目標也可能在較遠位置結束，需核對實際終點。LOS 目標端容許距離 0.35 公尺，涵蓋人物 0.25 公尺半徑及估測餘量。README 與架構文件補上參數、資料流、互斥及限制；未勾選 ROADMAP。
- 新增純邏輯、實際回呼函式的假動作／訊息測試及 launch 接線檢查。情境測試直接解析 SDF：貨架止於 y=5.0，車體恰在該位置已無遮擋，因此採 `(2.4, 4.5)` 朝 +y、人物 `(1.0, 5.9)` 驗證車寬走廊遮擋，並保留 y=5.0 清空對照。

### 本次沙盒驗證

維護者回報前一版已在 Docker／Gazebo 執行 240 秒並完成繞行與人物迴圈，但有 26 次目標、52 次取消，交接時距離約增至 6 公尺，另有約 20 秒停滯；本輪修正依此結果處理，尚未重跑模擬。

| 檢查 | 結果 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 277 項通過；無 ROS／rclpy，涵蓋搶占回呼亂序、舊回呼隔離、排空條件／上限、自身零速排除及導航／排空中的安全停止 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：9 個 YAML／RViz、10 個 XML／SDF／xacro、38 個 Python、4 個 Bash；兩／三錨點 xacro 展開與設定一致性通過 |
| `git diff --check` | 通過 |

### 尚未驗證與下一步

- 未執行 Docker、容器建置、colcon build／test、ROS 動作傳輸或 Gazebo 模擬；本次靜態檢查及假動作回呼測試不能視為 Humble 執行驗收。沒有安裝套件、建立虛擬環境或連接硬體／裝置／網路，也未提交或推送。
- 維護者須在容器檢查 ActionClient 接受／拒絕、取消與終止回覆順序，TF 在量測時間是否可取得，及重新建置後的 launch 參數型別與依賴。
- 模擬須確認走道末端繞行、人物移動時更新、清空後恢復、失敗冷卻，以及導航中 UWB 過期／過近／停用／雷射失效的停車。尤其核對搶占時是否持續移動、目標與取消數量是否降低、人物距離是否改善，排空條件與 2 秒上限是否合適，以及是否出現非零命令互相覆蓋。
- 必須重測人物占據目標格子的 NavFn 0.8 公尺容差與 RPP 終點判定，也要重測 guide 的區域導航與定位模式。線上建圖可能將動態人物寫入地圖，尚待觀察；兩錨點歧義與 NLOS 限制仍在。
- 尚無全域模式管理器或獨立速度逾時看門狗；follow 執行時不可另送 Nav2 目標或啟動 guide／鍵盤。非同步取消不能保證遠端立即停止，零命令也不是獨立仲裁器；傳輸失聯及 shutdown 的停止保證仍待整合驗證。若改動平滑器設定，須重新確認排空判斷及 2 秒上限。真實硬體、arm64 與本次修正後的容器模擬均未驗證。

### 審查者修正（2026-10-23 08:45 台北時間）

- 第二次模擬時 `follow_controller` 崩潰：`on_cmd_vel()` 使用 Iron 才有的 `(msg, info)` 回呼簽名，Humble 只傳 `msg`。改為單一參數並以 `last_own_publish` 時間略過自身零速，更新對應測試。
- 第三次模擬（240 秒）發現車體直接朝被貨架遮住的人物前進，斜靠在 shelf_2 東北角（約 (2.0, 4.8)）；RPP 回報 collision ahead、backup 也失敗，停滯約 60 秒（10 個 20 秒停滯窗）。新增 `BLOCKED_WAIT`：route_around 啟用且視線受阻時，直接模式改為原地零速等待交接，不再沿直線貼近貨架；`handoff_delay` 由 1.0 改為 0.5 秒。新增回呼測試。

### 本輪審查驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python pytest（沙盒） | 278 項通過 |
| Docker colcon build／test | 7 個套件建置成功；267 項測試，0 失敗 |
| Gazebo 跟隨 240 秒（修正後，route_around:=true） | 無 traceback；人物三條走道迴圈全程跟上，20 秒停滯窗 0 個；狀態：NAV2_HANDOFF 1824、FOLLOWING 1140、BLOCKED_WAIT 351、OBSTACLE_STOP 20、TOO_CLOSE 68；UWB 估測 RMSE 0.186 公尺；真實距離平均 3.30、最大 5.85 公尺 |
| 修正前對照 | 未改簽名前崩潰；未加 BLOCKED_WAIT 時距離平均 2.59 公尺但停滯 60 秒 |
| 已知限制 | 視線在走道邊緣抖動造成 FOLLOWING／BLOCKED_WAIT 頻繁切換（走走停停），跟隨距離偏大；需加入遲滯或依側向間距減速。最小距離 0.15 公尺為腳本人物不避讓、迎面走向車體所致，TOO_CLOSE 正確停車 |
| 未驗證 | 故障注入（UWB 過期／雷射失效於導航中）、guide 模式在 0.8 公尺容差下的迴歸（由後續 zone actions 分支驗證）、Mac arm64、Gazebo GUI、真實硬體 |

## 2026-10-23：里程計噪聲與 IMU／輪速 EKF

### 本次完成

- compute 映像新增 `ros-humble-robot-localization`，Gazebo 套件宣告執行相依；未安裝套件、建立虛擬環境、執行 Docker、提交或推送。
- `sim`、`mapping`、`guide`、`follow` 與 description launch 串接 `odom_source`，限定 `ekf`（預設）／`ground_truth`。EKF 模式僅由 robot_localization 發布 `/odom` 及 `odom → base_footprint`；真值模式僅由舊 OdometryPublisher 提供，並更新 AGENTS.md 的 TF 規範。
- 保留獨立 `/sim/ground_truth_odom`（`sim_world → base_footprint_gt`），真值 TF 與 `wheel_tf` 不橋接。舊 `/odom`／`/tf` 拆至只在真值模式啟用的橋接設定，`odom → sim_world` 單位轉換也只限真值模式。
- 新增 40 Hz 平面 EKF 設定，僅融合加噪輪速 vx／vyaw 及 IMU vyaw，不使用六軸 IMU 的模擬絕對朝向或加速度；Nav2 平滑器的 `/odom` 與 SLAM 的 `odom` 框架維持原設定。
- IMU 三軸角速度／加速度加入 Gaussian 噪聲及小偏差。新增可設定種子的輪速噪聲與比例誤差，以及明確非零的輪速／IMU covariance；噪聲純邏輯不匯入 ROS。
- UWB 評估改讀世界車體真值，排除 EKF 漂移混入測距估測誤差；確認 `uwb_sim_node` 仍讀 `/simulation/poses`。RViz 既有標記使用 `base_link`，未依賴 `sim_world`，不需更動設定。
- 新增 ROS-free `odom_metrics.py` 與 `odom_drift_monitor`，以真值時間插值比較平移／航向誤差與每公尺漂移；兩種模式都每 5 秒發布 `/sim/odom_drift`。補上噪聲、角度跨界、路程、時間對齊及無效資料測試。
- README 與架構文件補齊兩種模式、TF 歸屬、covariance、噪聲種子範圍及漂移比較步驟。ROADMAP 未勾選。

### 本機檢查紀錄與限制

- 指定既有虛擬環境執行 pytest：183 passed（xacro 相依套件有一項 Python 版本棄用警告）。
- `scripts/check_static.py` 通過 YAML／RViz 解析、XML／SDF／xacro 解析、Python／launch 與 Bash 語法、兩／三錨點與兩種里程計模式展開；新增 YAML 的序列由既有遞迴 YAML 掃描解析。
- `git diff --check` 通過。上述為純 Python 與靜態結果，沒有 ROS 執行環境，不能代表映像、colcon 或 Gazebo 驗收。
- 目前 covariance 與滑移等效輪距為模擬初始設定，沒有漂移實測數據。輪速噪聲種子不控制 Gazebo IMU 或物理引擎；加噪主題的 pose 仍是原始輪式積分，EKF 僅取 twist。
- 漂移比較假設共同出生原點；監測路程從第一個配對樣本起算，不做事後軌跡對齊。未移動時每公尺統計為 NaN，純旋轉須看航向誤差。

### 尚未驗證與下一步

- **未驗證：** compute 映像重建、Humble robot_localization 3.5.x 安裝、七套件 colcon build／test。
- **未驗證：** Fortress 實際載入 IMU 噪聲、真值模式兩個 OdometryPublisher、兩種模式橋接與 EKF 啟動；須確認 `/odom` 頻率、IMU 框架／時間戳、covariance、`/diagnostics` 與輸入無 TF 錯誤。
- **未驗證：** TF 樹每種模式僅一個 `odom → base_footprint` 來源；EKF 模式無 `odom → sim_world`，且輪速 TF／診斷真值 TF 不出現在 `/tf`。
- **未驗證：** 直線、轉彎與回到起點的模擬漂移數字；應同路線分別比較 EKF 與真值模式，真值應接近零、EKF 應有非零漂移，陀螺儀預期減輕輪式 yaw 誤差，但未設定或聲稱已達數值門檻。
- **未驗證：** 建圖品質、guide 六區導航與 follow 安全停止回歸、UWB 世界真值評估及 RViz 顯示。各移動流程仍須互斥執行。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| pytest／靜態檢查／差異空白檢查重驗 | 與 develop（PR #3）合併後 302 項通過；check_static PASS（含 ekf／ground_truth 兩模式 xacro 展開）；`git diff --check` 通過 |
| compute 映像重建與 colcon build／test | 映像含 `ros-humble-robot-localization`；colcon build 7 套件成功；colcon test 286 項、0 錯誤、0 失敗、1 略過 |
| EKF 模式啟動與 TF 唯一性 | `follow` 預設 ekf：`/tf` 發布者僅 slam_toolbox（map→odom）、robot_state_publisher、ekf_filter_node；`/odom` 實測約 29 Hz（牆鐘，模擬即時率 < 1）；無 traceback |
| 漂移數字 | `/sim/odom_drift`：3.25 公尺時平移 0.18 公尺（5.6%／m）、航向 0.021 rad；30.9 公尺時平移 0.86 公尺（2.8%／m）、航向 0.119 rad（0.004 rad／m） |
| guide 回歸（ekf） | 即時建圖下 10 個區域目標（含中文別名）10／10 成功、0 失敗；到點後 map TF 與真值差 ≤ 0.03 公尺，真值距區域目標約 0.12–0.15 公尺；`/odom` 漂移最多約 0.9 公尺，由 SLAM 的 map→odom 修正 |
| follow 回歸（240 秒 A／B） | ekf：無 traceback，UWB 估測 RMSE 0.202 公尺，真實距離平均 3.10、最大 6.04 公尺，20 秒停滯窗 7 個（在 shelf_1 西南角 (−1.6, 0.62) 約 50 秒 BLOCKED_WAIT／RPP collision ahead）；ground_truth：RMSE 0.237，距離平均 4.02、最大 6.30，停滯窗 0。單次樣本不足以歸因於 EKF，屬既有跟隨 LOS 抖動問題，由後續遲滯分支重測 |
| 未驗證 | ground_truth 模式的 TF 唯一性（僅跑 follow，未逐一列發布者）、covariance 數值檢視、RViz 顯示、Mac arm64 |

## 2026-10-23：階段 5 離線評估指標與報表

### 本次完成

- 新增 `shopping_zone_manager/eval_metrics.py`，以純 Python 字典時間序列計算導航成功率、成功耗時分布、每回合路長／效率與最終位置／朝向誤差；計算可旋轉矩形足跡到靜態盒及人物圓形的最小間距。靜態碰撞盒直接讀取既有 `supermarket.sdf`，不複製貨架或牆座標。
- 新增跟隨距離有號／絕對誤差、RMSE、p95、時間帶內比例、過近事件與持續時間，以及 LOST／FOLLOWING 遺失、恢復次數與重新取得時間；停止延遲同時檢查線速度及角速度，未觀測到停止或恢復的事件保留未完成數。
- 新增 ROS 無關的 CSV／JSONL 載入器與 `eval_report` console_scripts 入口，輸出 JSON 及 Markdown。空資料保留 null，拒絕非有限數值、倒退及同流重複時間；不自動排序或補值。
- 新增 [指標定義](METRICS.md)，記錄精確公式、單位、輸入欄位、預期主題對應、模擬商店驗收門檻提案及後續情境映射；README 與架構文件加入連結。ROADMAP 保持原狀，交由審查者勾選。
- 新增 54 個 pytest 案例，涵蓋手算導航、旋轉矩形與碰撞／包含、人物半徑、實際 SDF 載入、時間加權與邊界、未完成事件、無效資料及兩種格式的完整報表輸出。

### 限制與下一步

- 僅離線離散取樣，不執行 ROS 錄製、rosbag 轉換、TF 轉換、同步或掃掠碰撞檢查；後續需建立情境與錄製介面，核對導航起訖邊界、座標與真值覆蓋率。停止只判定首次符合樣本，未要求持續停止；多次觸發可共用同一後續停止樣本。
- 目前 OBSTACLE_STOP 允許原地旋轉，可能無法通過零 Twist 的完整停止門檻；報表不改寫控制行為。所有門檻均為提案，未宣告模擬情境或硬體已通過。
- 本次未執行 Docker、ROS／Gazebo 整合或硬體驗收；沒有提交或推送。下一步由審查者核對定義與門檻，再接續人物路徑／故障注入及可重現的紀錄評估。

### 本輪執行紀錄（非審查驗證）

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 233 個測試通過（含新增 54 個） |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：9 個 YAML／RViz、10 個 XML／SDF／xacro、41 個 Python、4 個 Bash；兩／三錨點展開及設定一致性通過 |
| `git diff --check` | 無空白格式錯誤 |

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 本輪變更、指標定義與門檻提案 | 待驗證 |
| 情境整合與紀錄覆蓋率 | 待驗證 |

### 本輪審查驗證（2026-10-23 14:20 臺北時間，盒內 Linux amd64）

| 項目 | 結果 |
| --- | --- |
| 純 Python 測試 | 重新基於 develop（含 EKF，PR #4）後 `pytest -q` 356 項通過；check_static PASS |
| colcon 建置與測試 | 7 套件建置成功；colcon test 340 項、0 錯誤、0 失敗、1 略過；容器內 `dev.sh test` 356 項通過 |
| 命令入口 | 容器內 `ros2 run shopping_zone_manager eval_report --help` 正常 |
| 未驗證 | 以真實 Gazebo 記錄產生報表（尚無 rosbag／CSV 記錄流程，屬階段 5 第 3 項）、Mac arm64 |

## 2026-10-23：區域導航拒絕／失敗／取消

### 本次完成

- 新增不匯入 ROS 的 `goal_tracker.py`，涵蓋待接受、執行中、取消中、成功、失敗、取消、拒絕、接受回覆逾時、伺服器未就緒與未知區域；以結構化狀態及世代計數器隔離舊目標事件。
- 保留預設 `BUSY` 政策，提供預設關閉的 `preempt` 參數與逐次呼叫旗標；取消舊目標並收到最終結果後才送最新排隊目標。
- 區域節點新增 Empty 取消主題、Trigger 取消服務、穩定時鐘計時器、約 1 Hz 回饋節流；保留逾時 future，延遲接受時立刻取消，結果或取消回覆遺失時不自行宣稱已停止。重複取消可重送要求。
- `goto_zone` 改由常駐區域節點統一送 action，保留 15 秒伺服器等待、新增預設 300 秒的 `--timeout`，Ctrl-C／期限到達時要求取消並有限等待；新增 `cancel_zone` 與 `dev.sh cancel`。
- 新增 `nav_action_check`／`dev.sh nav-check`，改以 map 座標 `(50, 50)` 的不可達目標（`unreachable goal`）取代貨架中心，讀取安裝版 NavFn 容許值，送出前確認與 SDF 牆面商店外框、實際全域成本地圖外框皆相隔超過容許值加 1 公尺，避免共用 0.8 公尺容許值使失敗案例意外成功；依序檢查未知區域、不可達目標、取消及停止、成功與忙碌拒絕，保留單項逾時與取消清理，輸出 PASS／FAIL 表與非零失敗碼。
- 新增預設關閉的 `--tolerance-check`，另送正常區域目標並要求成功後的新鮮 `/odom`；以 `--max-final-error`（預設 0.35 公尺）判定抵達誤差，通過或超標的表格均列出量測值。
- 補上生命週期、世代、逾時隔離、延遲接受取消、忙碌／搶先、閒置取消、待接受取消與入口靜態測試，新增不匯入 ROS 的幾何／量測邏輯與回歸測試，涵蓋容許值變更、成本地圖外框、外框距離門檻、牆體假設失效及抵達誤差超標，更新 README、架構及套件相依。
- 核對 Humble 官方原始碼：[`ClientGoalHandle.cancel_goal_async`／`get_result_async`](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/action/client.py)、[`CancelGoal.goals_canceling`](https://github.com/ros2/rcl_interfaces/blob/humble/action_msgs/srv/CancelGoal.srv)、[服務回呼的 request／response 雙參數簽章](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/service.py)、[`rclpy.init(signal_handler_options=...)`](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/__init__.py) 均已支援，無須 Iron API 或回退分支；[`remove_ros_args`](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/utilities.py) 改以明確匯入。
- 未提交、未推送、未安裝套件、未建立虛擬環境、未執行 Docker，未修改 ROADMAP 勾選。

### 已執行檢查（執行紀錄，非審查驗證）

- 指定沙盒虛擬環境執行 `/workspace/scratch/venv-shop/bin/python -m pytest -q`：334 個測試通過。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS，涵蓋 YAML／RViz、XML／SDF／xacro、Python／launch 與 Bash 語法，以及兩／三錨點展開一致性。
- `git diff --check`：無空白錯誤。

### 限制與下一步（尚未驗證）

- 此沙盒沒有 ROS；Humble API 已核對官方分支原始碼，但套件安裝入口、colcon 建置／測試與 DDS 執行行為仍需維護者在容器驗證。
- `guide.launch.py` 就緒且地圖涵蓋全店後，以 `./scripts/dev.sh nav-check --tolerance-check` 實測五項基本檢查及第六項正常區域抵達誤差；工具會移動機器人，常駐管理器必須閒置，且不得同時執行鍵盤、跟隨或其他 action client。工具使用相同 adapter 的獨立實例，不重複建立指令／服務端點；不代表已驗證常駐主題與服務的端到端通訊。
- 另需手動確認 `goto`、`cancel`、Ctrl-C、`--timeout`、搶先目標，以及人工延遲／遺失接受回覆時的自動取消。離線測試只驗證狀態機，不能冒充真實 DDS 故障注入。
- 若目標接受回覆永遠遺失，無法取得 handle，會保持忙碌並拒絕新目標；`TIMEOUT` 不保證機器人已停，須檢查 Nav2 與模擬狀態。不得以重啟區域節點取代停止確認。
- 目前共用狀態主題未加入呼叫者識別碼，互動操作限定單一 `goto` 呼叫者。尚無跨控制器仲裁，直接操作 Nav2 的外部客戶端不受區域狀態機管理。
- 不可達目標依安裝版設定及全域成本地圖檢查；若牆面幾何不再符合預期、成本地圖涵蓋 `(50, 50)` 或容許值過大，案例會失敗而非跳過。正常區域誤差直接比較 `/odom` 與 map 座標，假設模擬初始原點／朝向對齊且漂移可忽略，未補償 AMCL／SLAM 的 map→odom 修正；定位有偏移時須另以 TF 驗證。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python 測試與靜態檢查審查 | 待驗證 |
| Humble 容器 colcon 建置／測試與安裝入口 | 待驗證 |
| 常駐節點 goto／cancel 主題與服務、Ctrl-C／操作期限 | 待驗證 |
| Nav2 拒絕／不可達／取消／成功／忙碌五項模擬檢查及正常區域抵達誤差 | 待驗證 |
| 接受回覆遺失／延遲、舊世代隔離與取消重試 | 待驗證 |
| 搶先流程與取消後兩秒內停止 | 待驗證 |

### 本輪審查驗證（2026-10-23 14:25 臺北時間，盒內 Linux amd64，EKF 預設）

| 項目 | 結果 |
| --- | --- |
| 審查修正 | 重新基於 develop（含 EKF、評估指標）；`--tolerance-check` 改為優先使用 `/sim/ground_truth_odom`（`/odom` 在 EKF 下會漂移），無真值時才退回 `/odom` 並在結果標註；`test_action_entries` 在容器內（未掛載 `scripts/`）略過 dev.sh 檢查 |
| 純 Python 測試 | `pytest -q` 402 項通過；check_static PASS |
| colcon | 7 套件建置成功；colcon test 386 項、0 錯誤、0 失敗、1 略過 |
| `dev.sh nav-check --tolerance-check --max-final-error 0.35` | 6/6 PASS：未知區域、不可達目標（NavFn 容差 0.800 m，最小間距 61.4 m）、取消並停止、有效目標、BUSY 策略、最終誤差 0.112 m（真值） |
| 引導回歸 | 6/6 區域（drinks、snacks、produce、household、checkout、entrance）SUCCEEDED；地圖 TF 與真值差 ≤0.03 m；啟動紀錄無 Traceback |
| 未驗證 | Mac arm64、Gazebo GUI、搶占（preempt=True）的模擬測試 |

## 2026-10-23：自動建圖巡航與離線定位檢查

### 本次完成

- 新增 `shopping_zone_manager` 的 `map_tour` 與 `localization_check` 命令入口，以及 `dev.sh map-tour [NAME]`（預設 `supermarket`）與 `dev.sh loc-check [ROS_ARGS...]`。
- 巡航航點集中於 `config/map_tour.yaml`，蛇行走遍三條走道全長、南北橫向通道、入口及結帳區，最後返回原點。北側橫向路段由原規劃 y=5.9 移到 y=6.1，避免端架侵入車體外接圓加 0.15 公尺餘裕；測試重用場景碰撞盒工具並檢查整條連線。
- 分離無 ROS 的巡航排序／重試／跳過／整體逾時與定位統計／門檻邏輯。導航前等待 bt_navigator 生命週期啟用；目標逾時先取消並確認終止，無法確認時中止流程，避免並行目標。Ctrl-C 保留 ROS context 供取消目標後結束。
- 巡航完成後呼叫 `nav2_map_server map_saver_cli` 與 Humble `SerializePoseGraph` 服務，確認服務結果及四檔存在；跳過航點仍儲存但回傳失敗，整趟導航逾時或中斷不儲存。標準 `supermarket.{yaml,pgm,posegraph,data}` 開放版本管理，其他名稱繼續忽略，未建立假地圖。
- 定位檢查依區域 YAML 走訪六區，在目標後以真值時間戳查詢 map TF，檢查資料新鮮度與至少五筆有效採樣；輸出每區最後地圖／真值距目標誤差、定位位置與角度的平均值／RMSE／最大值，以及整體統計，可另存 JSON。
- PASS 門檻為每區所有採樣定位誤差 < 0.15 公尺且 < 5 度、最後真值距區域目標 ≤ 0.3 公尺，並要求六區皆導航成功。預設真值改為兩種里程計模式皆發布的 `/sim/ground_truth_odom`，保留 `truth_topic` 覆寫並移除探索／回退。純函式驗證 `(sim_world, base_footprint_gt)` 與舊版 `(odom|world|map, base_footprint)`，錯誤配對會明確報錯；強制 `/odom` 會警告並拒絕，除非另以 `odom_source:=ground_truth` 明確宣告模擬已採真值模式。AMCL 可用 `publish_initial_pose:=true` 在移動前發布出生姿態。
- 檢視 mapper／slam_toolbox 定位設定，保留一致的解析度、雷射上限、框架及 `use_sim_time`，說明 `map_start_at_dock` 對應出生點首筆掃描與 `map_start_pose` 優先權；AMCL 明確設定 12 公尺雷射上限與 TF 發布，未調整規劃器／控制器。
- 檢視 `map_tour` 與 `experiment_node`，兩者僅透過 Nav2 action 送目標，未讀取 `/odom` 作為真值；README 與地圖說明指定標準地圖用預設 EKF 生成，另存 ground_truth 比較地圖，建圖品質依賴 EKF 與 slam_toolbox。
- 更新 README 與地圖說明的重新產生、兩種離線定位、互斥操作與真值限制；補齊套件相依與命令／忽略規則測試。未提交、未推送、未執行 Docker、未安裝套件或建立虛擬環境，未修改 ROADMAP 勾選。

### 已執行檢查（執行紀錄，非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：351 個測試通過（含新增 13 個真值框架配對測試）；僅有既有 xacro 的 Python 棄用警告。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；12 個 YAML／RViz、10 個 XML／SDF／xacro、53 個 Python、4 個 Bash，以及 EKF／ground_truth 兩模式的兩／三錨點 xacro 展開與錨點一致性檢查通過。
- `git diff --check`：通過。

### 尚未驗證與下一步

- 本環境沒有 ROS；尚未執行 colcon 建置／套件測試，也尚未在 Gazebo 執行巡航、Nav2 生命週期等待、目標重試／取消、儲圖子程序或序列化服務。
- 尚未產生標準四檔地圖，未驗證格網覆蓋、閉環效果、檔案可攜性或離線重載；須由維護者在容器執行巡航並檢視摘要與地圖後，將同一次產生的四檔一起納入後續提交。
- 尚未驗證 slam_toolbox 與 AMCL 的六區誤差門檻、初始姿態發布、獨立真值訂閱、框架拒絕、`/odom` 保護及時間戳 TF 採樣。維護者需在每次全新模擬中各執行一次 `loc-check`，保留不同的 JSON 報告並核對六區座標。
- 預設 `/odom` 是會漂移的 EKF 估測；兩種模式都以獨立 `/sim/ground_truth_odom` 評估定位，須與地圖共用出生原點／朝向，不以 map 校正轉換真值。尚未執行 EKF 與 ground_truth 的巡航及兩種定位比較，也未涵蓋真實硬體。
- 幾何淨空測試無法保證 Nav2 實際路徑或完整探索；尚無模式管理器，巡航、定位檢查、區域目標、跟隨與鍵盤控制仍須互斥。ROADMAP 第 1 階段第 5–6 項待模擬審查後再決定完成狀態。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純邏輯、淨空、命令與忽略規則測試 | 待驗證 |
| YAML／XML／launch／Bash／xacro 靜態檢查 | 待驗證 |
| Humble colcon 建置、套件相依與命令入口 | 待驗證 |
| Fortress 巡航覆蓋、重試／取消與四檔儲存 | 待驗證 |
| slam_toolbox 位姿圖重新載入與六區定位 | 待驗證 |
| AMCL 格網重新載入、初始姿態與六區定位 | 待驗證 |
| 標準四檔地圖與區域座標人工檢視 | 待驗證 |

### 本輪審查驗證（2026-10-23 20:05 臺北時間，盒內 Linux amd64，EKF 預設）

| 項目 | 結果 |
| --- | --- |
| 審查修正 | 容器內 colcon test 只掛載 `ros2_ws/src`，`test_tour_logic.py` 改以套件路徑讀檔並在缺少 repo 檔案時略過 dev.sh／.gitignore 檢查；地圖版本政策改為只提交格網 `supermarket.{yaml,pgm}`（約 47 KB），位姿圖 `.posegraph`／`.data`（約 27 MB）維持忽略並於地圖說明記錄重建方式 |
| 純 Python 測試 | `pytest -q` 451 項通過；check_static PASS |
| colcon | 7 套件建置成功；colcon test 435 項、0 錯誤、0 失敗、5 略過 |
| `dev.sh map-tour supermarket`（14:28） | 17/17 航點到達、0 跳過、未逾時；格網與位姿圖儲存成功；格網目視含外牆、四排貨架與三條走道 |
| slam_toolbox 位姿圖定位（14:37） | `loc-check` 六區全部 PASS；定位誤差最大 0.024 m／0.39°，最後真值距區域 ≤ 0.133 m |
| AMCL＋map_server（提交格網，20:03） | 六區導航皆成功，但整體 **FAIL**：snacks 定位誤差最大 0.168 m（門檻 0.15）、最後真值距區域 0.309 m（門檻 0.3）；其餘五區 PASS（最大 0.122 m、2.0°）。AMCL 調參列入階段 2 第 4 項 |
| 未驗證 | Mac arm64、Gazebo GUI、ground_truth 里程計比較地圖、AMCL 調參 |

## 2026-10-23：跟隨視線遲滯

### 本次修改

- 依前次 240 秒 Gazebo 的平均距離 3.30 公尺、最大距離 5.85 公尺與走道邊緣抖動，新增 ROS-free `LosDebouncer`。原始 LOS 連續受阻 `los_block_time=0.3` 秒才鎖定，恢復須在額外加寬 `los_clear_extra_width=0.10` 公尺的走廊連續暢通 `los_clear_time=0.6` 秒；短暫受阻不累加，時間倒退重設。安全事件也重設確認狀態。新參數拒絕非有限值與負值，倍率與側向門檻另有範圍／順序檢查。
- `BLOCKED_WAIT` 僅使用已確認受阻的 LOS；原始受阻但尚未確認時維持 FOLLOWING，線速度乘 `los_uncertain_speed_factor=0.6`。側向 30°～100° 扇區取最小橫向淨空，從 `side_slow_clearance=0.35` 公尺的 `side_min_speed_factor=0.4` 線性增加到 `side_free_clearance=0.75` 公尺的 1.0；無側向涵蓋採最低倍率。兩倍率相乘並套用原有線速度／加速度限制。前方障礙停止與既有受限原地轉向維持原行為，不等待 LOS 確認；雷射無效、目標過近／遺失／過期與停用仍立即零 Twist。
- NAV2 返回 DIRECT 必須使用已確認暢通的 LOS，新增 `min_nav_hold=1.5` 秒；目標更新不重設保持時間，成功／失敗／取消與安全事件略過保持時間。DIRECT 返回後以 `min_direct_hold=1.0` 秒抑制再交接；若已確認的 LOS 受阻連續達該時間，可提前交接，仍須符合 handoff_delay。搶占、世代編號、舊回呼隔離與取消／速度排空邏輯未修改，訂閱回呼維持 Humble 的單一 msg 參數。
- 近距離（`follow_distance + handoff_margin=1.3` 公尺）恢復仍需 LOS 確認與 NAV2 最短保持，但略過原有額外 `resume_delay=0.5` 秒；導航成功直接返回。一般恢復約需 0.6 秒暢通確認再加 0.5 秒恢復等待，與 NAV2 保持時間並行；加寬走廊仍有障礙時可能更久。這是減少抖動與延後追近的取捨，不能保證距離改善。人物 map 目標及 NavFn 0.8 公尺容差未改。直接 max_linear=0.5 m/s 已高於人物 0.35 m/s，且等於 Nav2 velocity_smoother 上限，因此不提高；kp_distance=0.7 的比例控制本來就有約 0.5 公尺等速誤差。
- `follow_metrics.state_statistics` 新增每分鐘切換數與每狀態停留統計；`scripts/follow_state_stats.py LOGFILE` 可分析 `/follow/state` 的 echo 文字或狀態紀錄。支援同行時間戳，無時間戳預設每筆 0.05 秒（可調）；混用時鐘拒絕，首尾區間只計觀測範圍。僅記錄切換事件或有掉樣時須使用時間戳，否則時間統計只是估算。
- README 補齊所有新參數、限制與 CLI 用法；ARCHITECTURE 更新交接規則、側向減速與 Humble 自身零速排除說明。未勾選 ROADMAP。

### 本次沙盒驗證

| 檢查 | 執行結果 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 371 項通過；涵蓋時間／空間遲滯、短脈衝不累加、時間倒退、不確定期間減速、側向倍率／扇區、保持時間與例外、安全立即停止、實際 tick 的擦邊抖動、統計解析與 CLI；既有搶占／回呼測試持續通過 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：9 個 YAML／RViz、10 個 XML／SDF／xacro、44 個 Python、4 個 Bash；兩／三錨點 xacro 展開與錨點一致性通過 |
| `git diff --check` | 通過 |

### 限制與下一步

本輪未執行 Docker、colcon、ROS 傳輸或 Gazebo；未安裝套件、建立虛擬環境或新增硬體／裝置／網路程式，也未提交或推送。靜態及假訊息驗證不等於 Humble 容器或硬體驗收。維護者下一步須完成 colcon 建置／測試及 240 秒 Gazebo 跟隨，與前版比較每分鐘切換數、各狀態停留、目標／取消數、真實距離平均／最大值及 20 秒停滯窗；特別確認 0.35 公尺恢復走廊是否過嚴、減速是否拉大距離、近距離恢復是否及時，以及 NAV2 保持期間 UWB 過期／雷射失效／停用的立即取消與零速。維持既有流程互斥與兩錨點／NLOS 限制；真實硬體、arm64 與整合效果仍未驗證。

### 本輪審查驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python 與靜態檢查審查 | 待驗證 |
| Docker colcon 建置／測試 | 待驗證 |
| Gazebo 跟隨 240 秒：切換率／停留／距離／停滯窗 | 待驗證 |
| 保持時間內故障注入與安全停止 | 待驗證 |

審查者調整（2026-10-23 14:20 臺北時間）：側向減速門檻由 0.45／0.9 公尺改為 0.35／0.75 公尺。橫向淨空由雷射中心起算，走道中央約 0.8 公尺；原門檻會讓走道內線速度降到約 0.87 倍、偏離中心時更低，可能慢於人物 0.35 m/s 而拉大跟隨距離。
