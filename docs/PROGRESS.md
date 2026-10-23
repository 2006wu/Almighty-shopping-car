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
