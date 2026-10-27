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

### 本輪審查驗證（2026-10-23 20:15 臺北時間，盒內 Linux amd64，EKF 預設）

| 項目 | 結果 |
| --- | --- |
| 重新基於 develop | 含地圖巡航（PR #7）；PROGRESS 衝突保留兩側 |
| 純 Python 測試 | `pytest -q` 534 項通過；check_static PASS |
| colcon | 7 套件建置成功；colcon test 517 項、0 錯誤、0 失敗、5 略過 |
| Gazebo 跟隨 240 秒（側向門檻 0.35／0.75 m） | 真實距離平均 **2.81 m**（前版 3.30）、最大 6.03 m（前版 5.85）、最小 0.14 m（人物路徑經過停止中的車體，TOO_CLOSE 有觸發）；20 秒停滯窗 0；狀態切換 54 次（約 13.5 次／分）；狀態樣本 NAV2_HANDOFF 1818、FOLLOWING 814、BLOCKED_WAIT 661、TOO_CLOSE 68、OBSTACLE_STOP 46；UWB 估測誤差平均 0.135 m |
| 啟動紀錄 | 無 Traceback；僅啟動期 TF 等待、Gazebo 服務回應 Host unreachable 與 Nav2 搶占 halt 訊息 |
| 未驗證 | 保持時間內的故障注入（待故障注入分支）、Mac arm64、Gazebo GUI；BLOCKED_WAIT 停留比前版多，距離仍遠大於 1.0 m 目標，後續可再調整 |

| pytest／靜態檢查／差異空白檢查重驗 | 待驗證 |
| compute 映像重建與 colcon build／test | 待驗證 |
| EKF／真值模式啟動、TF 唯一性與 covariance | 待驗證 |
| Gazebo 漂移數字與比較路線 | 待驗證 |
| 建圖／guide／follow／UWB 評估回歸 | 待驗證 |

## 2026-10-23：模式管理與速度仲裁

### 本次完成

- 在既有 `shopping_zone_manager` 加入不匯入 ROS 的 `ModeManager`、`VelocityMux`、`VelocityLimiter`、`OutputWatchdog`，維持七個套件；只使用標準函式庫，未引入 numpy。
- 支援 IDLE／GUIDE／FOLLOW／TELEOP／ESTOP；切換立即零速、清除快取並停用跟隨，急停鎖定且 reset 只回待機。切換會保留尚未完成的取消責任，舊世代回覆不會解除新模式的停止條件。
- 加入 Humble `cmd_vel_mux` 節點與獨立 `cmd_vel_watchdog` 程序；後者為預設唯一的 `/cmd_vel` 發布端。來源逾時、限速、非有限值與非平面輸入保護，以及單調時鐘的輸出逾時停止均有純 Python 測試。
- NavigateToPose 取消使用 `/navigate_to_pose/_action/cancel_goal` 的零 UUID／零時間戳，未使用 Humble 不支援的 ActionClient cancel-all。等待跟隨停用成功、取消服務成功、被取消目標的相符終止狀態、無活動目標及 nav 非零命令安靜後才啟用新模式；無法確認時保持零速。
- 保留跟隨 `/follow/enable` 服務，新增可靠且保留最新值的 `/follow/enabled` Bool。停用立即歸零，若目標仍待接受或執行中，服務暫以 `success=false` 回報等待終止，由管理器重試；停用且已無目標的控制器不持續以零命令遮蔽導航。
- Nav2 只重新映射 velocity_smoother 的 `cmd_vel_smoothed` 至 `/cmd_vel/nav`，保留內部 `cmd_vel_nav`；跟隨改送 `/cmd_vel/follow`，排空觀察參數改為 `/cmd_vel/nav`，舊接線仍使用 `/cmd_vel`。
- guide／follow 新增預設 true 的 `use_mux` 及預設 guide／follow 的 `initial_mode`；false 恢復原接線。單獨仲裁節點及共用 mode_mux launch 預設 idle，未設定自動 respawn。mapping 不啟動 Nav2，維持既有接線，文件另列如何選用仲裁保護。
- `dev.sh` 加入 mode／estop／reset；README 與架構文件新增模式許可、主題、參數、Mermaid 接線、重啟取捨與容器驗收程序。未勾選 ROADMAP。

### 自行檢查與限制

- 已在指定既有虛擬環境執行完整 pytest：334 passed；`scripts/check_static.py` 回報 PASS，涵蓋 11 個 YAML／RViz、10 個 XML／SDF／xacro、53 個 Python、4 個 Bash 檔案，以及兩／三錨點、ekf／ground_truth 的 xacro 展開。現有 xacro 在 Python 3.10 的棄用警告仍存在。
- 已檢查 `git diff --check`；本次未提交、推送、執行 Docker、安裝套件或建立虛擬環境，未加入硬體驅動、裝置或自訂網路連線。
- 無 ROS 測試只驗證純邏輯、以替身執行的實際回呼與 launch 接線，不等於 Humble 執行期、colcon 或 Fortress 驗收。跟隨既有繞障及 TF 所有權保持原設計；容器互動尚待驗證。
- 來源逾時與獨立輸出逾時都於下一個 50 毫秒更新生效，另有傳輸排程延遲。看門狗若也失效，無法保證底盤停止；未新增 Fortress 的底盤逾時外掛或硬體保護。
- 單獨重啟仲裁節點預設待機；若保留 launch 的非 idle `initial_mode`，重啟相同參數會重新請求移動，此為保留 goto／follow 工作流程的明確例外。一次只啟動一個 bringup，跟隨時仍不得外送無關導航目標。

### 下一步與審查驗證

由維護者在 Docker 執行 colcon build／test，分別驗證 guide、follow 的 DIRECT／NAV2_HANDOFF、模式切換取消、急停／reset、來源停止、仲裁程序暫停／重啟及 `use_mux:=false`。核對底盤唯一發布端與逾時停止時間；本次審查欄保留待驗證。

| 項目 | 審查驗證 |
| --- | --- |
| 純邏輯、回呼及 launch 接線 pytest | 待驗證 |
| YAML／XML／xacro／launch／Bash 靜態檢查與差異空白檢查 | 待驗證 |
| Humble colcon build／test | 待驗證 |
| guide／follow 仲裁接線與舊接線回復 | 待驗證 |
| 模式切換、取消全部目標與跟隨停用確認 | 待驗證 |
| 急停、來源／輸出逾時與重啟待機 | 待驗證 |

### 本輪審查驗證（2026-10-23 20:30 臺北時間，盒內 Linux amd64，EKF 預設）

| 項目 | 結果 |
| --- | --- |
| 重新基於 develop | 含 EKF、評估指標、區域動作、地圖巡航、跟隨遲滯（PR #4–#8）；ARCHITECTURE／setup.py／dev.sh／跟隨回呼測試衝突手動合併，保留兩側功能 |
| 審查修正 | Nav2 behavior_server 的恢復行為原本直接發布 `/cmd_vel`（4 個發布者），會繞過仲裁、急停與看門狗；新增節點範圍重新映射 `behavior_server:cmd_vel → /cmd_vel/nav`，launch 測試同步更新 |
| 純 Python 測試 | `pytest -q` 566 項通過；check_static PASS |
| colcon | 7 套件建置成功；colcon test 520 項、0 錯誤、0 失敗、5 略過 |
| `/cmd_vel` 發布者（guide） | 修正後只有 `cmd_vel_watchdog`；`/cmd_vel/nav` 由 velocity_smoother 與 behavior_server 發布、`cmd_vel_mux` 訂閱 |
| `dev.sh nav-check --tolerance-check` | 6/6 PASS（取消並停止 PASS；最終誤差 0.108 m） |
| 模式切換模擬檢查（盒內腳本，兩次） | 14/14 PASS：初始 GUIDE、導航中 estop 後真值速度 ≤0.32 s 內歸零並維持、goto 因取消結束、ESTOP 期間拒絕 guide、reset→IDLE、teleop 可旋轉／前進、teleop 停送 0.54 s 內停止、idle→guide 後 goto entrance 成功 |
| 輸出看門狗 | teleop 旋轉中以 SIGSTOP 凍結 `cmd_vel_mux`，真值角速度 0.36 s 內歸零（門檻 1.0 s） |
| Gazebo 跟隨 240 秒（use_mux） | 平均距離 3.02 m、最大 5.99 m、20 秒停滯窗 0、切換 61 次；無 Traceback |
| 未驗證 | Mac arm64、Gazebo GUI、鍵盤 teleop_twist_keyboard 實際接線（以腳本發布 `/cmd_vel/teleop` 代替）、`use_mux:=false` 舊接線的模擬回歸 |


## 2026-10-23：階段 4 模擬故障注入與停止驗證

### 本次完成

- 在 `shopping_robot_gazebo` 以 ament_cmake_python 安裝 `shopping_robot_sim`，新增純邏輯故障引擎與 `fault_injector` ROS 入口。支援 UWB stale／定時 dropout／高斯 noise、雷射 drop／freeze、里程計 freeze 及 clear；訊息複製不改原始資料，凍結保留舊時間戳。
- sim／mapping／guide／follow 新增預設 false 的 `fault_injection` 與 YAML 字串清單 `fault_schedule`。只在啟用時將生產端 remap 到 `/sim/raw/*`，由中繼接回既有消費 topic；車體與人物真值、TF 唯一來源維持原規範。`/sim/fault_status` 以 transient_local 發布目前故障名稱陣列。
- 確認既有 UWB lost_timeout 與雷射 0.5 秒逾時使用來源時間戳，沿用其停止／導航取消行為。新增預設 false 的 `monitor_odom`，僅在故障模式 follow 自動啟用，里程計缺失／過期走同一安全零速與取消路徑；未改一般跟隨參數。
- 新增 `scripts/fault_check.py` 與 `dev.sh fault-check`，對執行中的故障模式 follow 依序注入六種故障，確認移動基準、命令／真值停止、持續停止、資料覆蓋及 clear 後恢復，列印 PASS／FAIL 表並以結束碼反映失敗。噪聲情境檢查速度上限與恢復，不假設所有噪聲都能偵測並停車。
- 純 Python 測試新增命令解析、排程到期／重設、訊息凍結與複製、固定種子噪聲、保守驗收判斷與 launch 接線；假回呼涵蓋直接／導航中的 UWB、掃描與里程計過期停止及里程計恢復。README 與架構文件補齊操作、門檻及限制；ROADMAP 未變更。

### 本輪執行紀錄（非審查驗證）

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 400 項通過；既有 xacro Python 棄用警告 1 項 |
| `PATH=/workspace/scratch/venv-shop/bin:$PATH python3 scripts/check_static.py` | PASS：11 個 YAML／RViz、10 個 XML／SDF／xacro、55 個 Python、4 個 Bash；兩／三錨點與兩種里程計模式展開通過 |
| `git diff --check` | 通過 |

系統預設 python3 缺少 PyYAML，故靜態檢查用 PATH 選用指定既有虛擬環境；沒有安裝套件。以上為純 Python／語法與配置驗證，不代表 ROS、colcon 或 Gazebo 整合驗收。

### 限制與下一步

- 本次未執行 Docker、colcon、ROS／Gazebo 或硬體驗收，未提交或推送。需在既有容器重新建置，分別以 `route_around:=false` 與 true 執行 fault-check，另於 NAV2_HANDOFF 中手動注入，確認非同步取消期間沒有非零命令覆寫，以及真值停止延遲符合門檻。
- 里程計凍結僅作用於 `/odom` 訊息，不凍結 TF、EKF 內部狀態或真值；僅故障模式 follow 自動啟用里程計保護，不宣稱 guide 已具此保護。仍沒有全域速度仲裁與獨立命令逾時看門狗。
- 驗收門檻為預設參數下的提案：命令 0.8 秒、真值 1.5 秒內停止，持續至少 0.3 秒，覆蓋缺口至多 0.25 秒。以模擬時鐘的本機接收時間量測，不是精確傳輸延遲；自然路徑沒有恢復移動時回報 FAIL。腳本未固定人物相位，也不保證每次在導航狀態注入，仍需固定情境與紀錄流程。
- 排程與手動命令可疊加；clear 不刪除未來排程，自動驗收必須使用空排程且只有一個注入者。UWB 兩錨點歧義、NLOS 與高斯噪聲下錯誤目標不一定可辨識，速度限制檢查不能當作跟隨精度或碰撞安全保證。

### 審查驗證

2026-10-23 20:50（台北時間）於 box（Linux amd64、EKF、headless）審查。已 rebase 到含模式仲裁（PR #9）的 develop，並合併衝突：follow 控制器同時保留 `/cmd_vel/follow` 輸出、mux 的 `enabled` 和 `observed_cmd_vel_topic`，以及故障模式的 `monitor_odom` 和 `/sim/raw/*` remap。guide 同時傳入 `use_mux` 和 `fault_injection`。另外，故障分支的 `valid = fresh` 會蓋過 develop 新增的 `scan_valid` 檢查，因此刪除該行。

| 項目 | 結果 |
| --- | --- |
| pytest（rebase 後） | 610 項通過 |
| check_static | PASS |
| colcon build＋test（容器） | 7 個套件；531 項測試，0 失敗，5 跳過 |
| `follow.launch.py fault_injection:=true`＋`dev.sh fault-check`（預設 `route_around:=true`、`use_mux:=true`） | 6/6 PASS。命令停止／真值停止：uwb_stale 0.60／0.70 s，uwb_dropout 0.12／0.15 s，lidar_drop 0.49／0.56 s，lidar_freeze 0.08／0.12 s，odom_freeze 0.61／0.66 s；uwb_noise 通過速度上限與恢復檢查。命令在看門狗輸出的 `/cmd_vel` 量測，代表經過 mux 後的實際命令 |
| 預設 follow（`fault_injection` 未設） | 沒有 fault 節點，也沒有 `/sim/raw/*`、`/sim/fault*` topic；`/uwb/ranges` 只有一個發布者；`/cmd_vel` 只有 `cmd_vel_watchdog` |
| NAV2_HANDOFF 中注入故障 | 未專門驗證，因為 fault-check 不固定人物相位 |
| `route_around:=false` 執行 fault-check | 未驗證 |
| 排程、時間倒退與清除行為 | 只有純 Python 測試，沒有在模擬中驗證 |

紀錄：`/workspace/wt/logs/shop/t9-faultcheck1.txt`、`t9-default.txt`（box 本機）。

## 2026-10-23 20:00 場次總結（台北時間）

- 已合併進 develop：PR #7（自動建圖巡航、地圖格網、定位檢查）、#8（跟隨視線遲滯）、#9（模式管理、速度仲裁、看門狗）、#10（模擬故障注入）。各 PR 的分支和 worktree 都已刪除。
- 已勾選 ROADMAP：階段 1 第 5、6 項，階段 4 第 1、2、4 項。
- 待追蹤：用已提交的格網跑 AMCL loc-check，結果 5/6，零食區超出門檻（定位誤差 0.168 m，最終距離 0.309 m），歸到階段 2 第 4 項的 AMCL 調參。另外，teleop 角速度命令 0.6 rad/s 時，真值偏航率約 0.855 rad/s，尚未調查。
- 已排隊、未執行：Codex 5 小時額度到 2026-10-24 00:30（台北時間）才重置，因此下列兩項還沒開始，提示詞放在 box 的 `/workspace/wt/logs/shop/`：
  - `t10-prompt.md`：階段 5 第 3 項，rosbag 紀錄與 `bag_to_eval`。
  - `t11-prompt.md`：階段 3 第 3、4 項，人物同步檢查與 `follow_eval`。
  這兩項沒有建立遠端分支。
- 未完成分支：無。

## 2026-10-24：rosbag 記錄與評估報告

### 本次完成

- 新增 `dev.sh record NAME [--duration S] [額外 topic...]`，透過純 Python helper 載入唯一 `shopping_zone_manager/config/record_topics.yaml`，在 compute 使用 Humble sqlite3、模擬接收時間與正常 SIGINT 結束錄製。輸出 `bags/NAME/` 共用回主機並加入 Git 忽略；既有名稱不覆寫，時限使用牆鐘秒數。
- 新增 `bag_to_eval` console entry 與 Humble ROS 讀取薄層，使用 SequentialReader／StorageOptions／ConverterOptions、訊息型別查詢與 CDR 反序列化。純 `bag_converter.py` 不匯入 ROS，處理來源時間排序、同 tick 狀態、真值有界內插、地圖目標剛體對齊、區域導航生命週期、跟隨距離、狀態、保護／急停事件、最終命令與真值停止速度。
- 新增 `dev.sh eval-bag NAME [eval_report 參數...]`，依序輸出觀測 JSONL、來源與覆蓋中繼資料及 `report.json`／`report.md`。導航截止仍納入失敗；真值重複、導航邊界缺樣、部分導航及跟隨內部缺口明確拒絕，避免無聲補零或略過失敗。
- 新增 21 項合成資料／假 ROS 讀取介面測試，涵蓋成功、拒絕、取消、失敗、逾時、未完成、多回合、最短 yaw 內插、跟隨同步、停止 pending／延遲、空輸入、亂序／缺樣、時鐘倒退、topic 清單來源、錄製正常結束與轉換結果載入評估器的往返測試。
- README、METRICS 與架構文件補上固定情境、時間規則、可推導列、來源限制、獨立 domain 回放與兩次報告比較程序。ROADMAP 未勾選；未提交或推送，未執行 Docker、安裝套件或建立虛擬環境。

### 本輪執行紀錄（非審查驗證）

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 631 項通過；既有 xacro 棄用警告 1 項 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：YAML、XML／SDF／xacro、Python、Bash 與兩／三錨點、兩種里程計模式展開 |
| `git diff --check` | 通過 |

### 限制與下一步

- 本環境沒有 ROS，只有純 Python、假 Humble API 與靜態驗證；維護者須在既有容器執行 colcon build／test、短 guide／follow 錄製、轉換、報告與隔離回放，核對真實型別、QoS、錄製關閉及時間覆蓋。這不是容器、跨架構或硬體驗收。
- `eval-bag` 僅適用目前預設 map 與世界原點、朝向一致的程序；其他地圖要手動提供經驗證的 map→世界轉換。導航只還原區域管理器已送出的回合，無法涵蓋 RViz 目標、跟隨 handoff 或未送出前的未知區域／伺服器不可用；孤立終態與截止回合需連同中繼資料檢視。
- 人物真值是唯一人物路徑模型的位置，非 Gazebo 人物模型實際到位量測；跟隨位置採有界內插，不外推或跨內部缺口。長紀錄目前在記憶體排序，沒有串流或自動情境執行器。
- 無 header 的資料須由 `--use-sim-time` bag 接收時間對齊；同 topic、同 tick 的命令／跟隨狀態取最後一筆並揭露合併數，無法解析 tick 內延遲。LOST／OBSTACLE_STOP 不能區分 UWB 過期、掉訊、雷射／里程計失效或障礙；停止延遲從狀態公布算起，看門狗缺少獨立事件來源。固定種子仍不能消除 Gazebo／DDS／人物相位非決定性，應多次重複保存所有失敗。
## 2026-10-24：偏航率差異與 AMCL 定位調查

### 偏航率根因與量測

- 已讀 xacro、world SDF、bridge／EKF、輪速加噪、sim／guide 接線與歷史紀錄。純 `sim.launch.py` 沒有模式仲裁或輸出看門狗；`/cmd_vel` 經 bridge 直接進 DiffDrive。預設 EKF 的 `odometry/filtered` 重新映射成 `/odom`，真值固定為獨立的 `/sim/ground_truth_odom`，未修改 TF 所有權。
- 新增 `scripts/yaw_rate_check.py`、`dev.sh yaw-check`。預設六列為線速 0 的角速 0.2／0.4／0.6／0.8，以及線速 0.2／0.4 的角速 0.3。每列先停 1 秒、命令暫態 2 秒、採樣 3 秒；使用來源時間戳的解纏航向直線擬合及平面累積路徑速率，同列顯示真值、EKF、角速比與線速比。資料覆蓋不足、間隔過大、過期、時鐘倒退／停止與前方或側方雷射過近均中止。正常結束、例外與 Ctrl-C 皆重複送零速；有替身入口測試覆蓋中斷與例外清理。這不是底盤程序失效時的看門狗。
- 重要公式方向：已核對 [Fortress DiffDrive 的 UpdateVelocity](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/diff_drive/DiffDrive.cc)。固定輪速時，解算 yaw 的確與 separation 成反比，故里程計輪距校正是 `old × ratio`；但此外掛在輸入側使用 `(v ± w × separation / 2) / radius`。因此本 xacro 增大 separation 會增加左右輪速差，近似固定滑移時應以 `old / ratio` 試調。工具保留並清楚命名兩種公式，不能把 `fixed_wheel_rate_separation` 套至 xacro。單點 0.855／0.6 = 1.425，兩種候選分別為 2.85 與 **1.404**；後者才是命令側的試調方向，但單點不足以校正整個速度範圍。
- 幾何輪距為 `0.50 − 0.04 = 0.46 m`，目前命令側等效輪距 `0.92 m`。世界步長 0.001 秒；四輪前後位置 ±0.198 m，轉向必須橫向擦滑。車輪碰撞是轉軸沿 y 的圓柱，URDF gazebo 標記 `mu1=mu2=0.5`；world 地板未明訂 surface／摩擦，沒有 `fdir1`、方向性摩擦或 slip 參數。兩方向相同的摩擦值不提供低側向阻力，實際接觸行為仍依 URDF→SDF 轉換與 Fortress 物理後端。沒有容器轉換結果，不能宣稱某個摩擦標記已被忽略或確實生效。
- 既有紀錄曾在幾何輪距下 ≤0.2 rad/s 無法起轉，故 2.0 是克服起轉阻力的補償，並非已校正的固定滑移常數。較高角速的過量補償是目前最合理假說，但摩擦起轉門檻可能使低速與高速無法共用倍率。有線速時左右輪滑移及橫向擦滑不同，不能以原地平均值保證弧線正確。
- 外掛限速 0.6 m/s／1.2 rad/s，未設定加速度限制；仲裁的一般加速度限制僅適用 guide／follow，純 sim 不經過它。六列命令都在限速內；最大輪速需求約 5.38 rad/s，低於 joint 標記的 15 rad/s（不據此假定物理引擎必然執行 joint limit）。限幅不能解釋穩態 1.425 倍增益；暫態仍須排除。輪速加噪為線速 ×1.01、角速 ×1.02 加高斯噪聲；EKF 融合輪速 twist 與 IMU yaw rate，不回饋控制底盤，也不會改變獨立真值。因此 EKF 不是車體真值偏航增益的根因，但其輸出會與真值不同，工具會一併量測。
- **保留 factor=2.0**，未新增未經證實的新預設；只修正原註解的 0.44 m 與實車校正措辭，明訂此為模擬待校正參數。維護者須先取得四列原地與兩列弧線的完整表，再決定是否以 xacro argument 提供校正值；若調整，重跑小角速起轉、Nav2 與人物跟隨，調校可能位移。

### 格網幾何實測與 AMCL 修正假說

- 新增 `scripts/map_geometry_check.py`，只用標準函式庫＋PyYAML，不匯入 ROS／numpy。支援 P2／P5、註解、16 位元大端序，保留二進位首像素的空白／井字號，拒絕截斷或超出範圍的影像。依 map YAML 的旋轉原點、解析度與占用門檻解析格子；未知格不當成占用。
- 幾何直接讀淨空測試同一份 `worlds/supermarket.sdf` 的碰撞盒；遇到未支援的旋轉／link 或 collision 位姿會拒絕，避免靜默比錯。各面排除角落後，每個切向格取最近占用中心，搜尋預設 ±0.3 m 並限制在半厚度內，避免把內牆面重複認成外牆面；輸出覆蓋、偏移中位數、直線斜率／RMS。兩面覆蓋皆至少 50% 才估中心平移與尺寸比例。區域附近一公尺長走道另估橫向偏移、旋轉與寬度比例。
- 本環境實際執行 `python scripts/map_geometry_check.py --json /tmp/shop-map-geometry.json`（使用指定既有 venv）。提交地圖解析度 0.05 m、原點 `[-5.92, -3.12, 0]`。以下偏移皆為「格中心 − 世界表面」，單位 m；約有半格量化誤差。此表是執行結果，非審查驗證。

| 區域／物件 | 邊緣偏移與覆蓋 | 平移／旋轉／尺寸估計 |
| --- | --- | --- |
| north 內面、east 內面 | +0.005；99%、84% | 外面缺資料，無法估牆中心／厚度 |
| west 內面 | +0.005；61% | 面旋轉約 −0.024°；外面缺資料 |
| south_left／south_right 內面 | +0.005；100% | 外面缺資料，不將單面誤報為零厚度 |
| shelf_0 | 右／下／上 +0.005；100% | y 平移 +0.005、y 倍率 1.000；左面缺資料 |
| shelf_1 | 四面 +0.005；100% | dx/dy +0.005/+0.005、旋轉 0°、x/y 倍率 1.000/1.000 |
| shelf_2 | 四面中位數 +0.005；100% | dx/dy +0.011/+0.005、面旋轉中位數 0°、x/y 倍率 0.985/1.000；局部邊緣有變化 |
| shelf_3 | 左／下／上 +0.005；100% | y 平移 +0.005、y 倍率 1.000；右面缺資料 |
| landmark_endcap | 四面中位數 +0.005；100% | dx/dy +0.005/−0.002、旋轉 0°、x/y 倍率 1.000/1.033 |
| landmark_pillar | 右 +0.005；100%；其餘不足 | 無完整平移／尺寸估計 |
| landmark_fridge | 右 +0.005、下 +0.055；100% | 可見面旋轉中位數約 2.450°；其餘缺面，不能視為完整剛體旋轉 |
| landmark_display | 左／上 +0.005；50%／90% | 其餘不足，無完整平移／尺寸估計 |
| checkout | 左／上 +0.005；100% | 其餘不足，無完整平移／尺寸估計 |
| drinks／snacks／produce 目標附近 | 左右各 +0.005；100% | 橫向平移 +0.005、旋轉 0°、走道寬倍率 1.000 |
| household 目標附近 | 左 +0.005、右 +0.030；100% | 橫向平移 +0.017、旋轉 +1.196°、走道寬倍率 1.015 |

- snacks 的局部結果不支持 **0.166 m 的橫向格網變形**。長平行貨架對沿走道位置約束弱，少量局部厚邊、未知面及最近面對應仍可能漏掉問題；不能由此斷言地圖完全正確。區域座標與實體走道一致，未更改 goals、影像或 YAML 原點／解析度，未重建地圖。
- 近乎恆定的採樣誤差也可能是停車後 AMCL 不再更新，而非必然是格網偏移。Humble 原始碼預設 `update_min_d=0.25`、`update_min_a=0.2`；在到站前最後一次更新後的短距離運動與 EKF 漂移，可能保留固定校正誤差。稀疏取樣與長直走道的縱向歧義也是候選。這些是待測假說，尚無誤差向量、粒子分布或逐次更新紀錄可證實因果。
- 在 `nav2_params.yaml` 實作 AMCL 試調：更新門檻改為 0.05 m／0.05 rad；max_beams 60→180，提高非對稱走道端點的取樣機會；z_hit/z_rand 0.5/0.5→0.9/0.1、sigma_hit 0.2→0.1 m、likelihood 最大距離 2.0→0.5 m。顯式保留既有預設 likelihood_field、alpha1–5=0.2（diff model 不用 alpha5）、recovery alpha=0，沿用 set_initial_pose=true 與出生原點。參照 [Humble AMCL 預設與更新條件](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp) 及 [likelihood field 計分](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp)。
- 調整目的是在停止前更頻繁修正、讓較精準的格網邊緣及更多方向光束提供約束；較窄分布可能降低錯誤初始位置的恢復範圍，較多光束及更新頻率會提高 CPU 用量，因此不宣稱已解決定位問題。沒有證據支持更改 motion noise、重設每區初始姿態、提高容差或調整 slam_toolbox；mapper 保留 0.05 m、scan matching／loop closure 與 0.1 m 最小移動距離。

### 維護者重跑與限制

先 `./scripts/dev.sh colcon-build`／`colcon-test`。每個 bringup 都包含模擬，以下各組須先以 Ctrl-C 結束上一組；每組第二行在另一終端機執行。

```bash
./scripts/dev.sh sim headless:=true odom_source:=ekf person:=false
./scripts/dev.sh yaw-check --settle 2 --sample 3 --old-factor 2.0

# Stop sim before starting guide.
./scripts/dev.sh guide headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml odom_source:=ekf person:=false
./scripts/dev.sh loc-check -p publish_initial_pose:=true -p report_file:=/tmp/loc_amcl_yaw_calib.json

# Optional comparison map only if further evidence calls for remapping; stop guide first.
./scripts/dev.sh guide headless:=true slam_mode:=mapping odom_source:=ekf person:=false
./scripts/dev.sh map-tour loc_yaw_compare
```

- 第一優先是同一提交格網的 AMCL 重測，建議每次全新模擬至少三次；預期六區皆成功、每筆定位位置誤差 <0.15 m／角度 <5°、最後真值距目標 ≤0.3 m，並確認其他五區沒有退步。這些是驗收目標，未在本環境觀測到。若仍失敗，需回報各區 JSON、誤差方向與 AMCL 更新／EKF 殘差，不以單次 PASS 宣告根因已證實。
- 若偏航倍率四列差異大或低速不起轉，先檢視容器產生的 SDF／物理接觸，再做多點非線性校正；不要直接以單點 1.404 取代 2.0。弧線因雷射淨空中止須重新選擇模擬空曠姿態，不能降低安全門檻來掩蓋碰撞造成的速率偏差。
- 選用比較建圖後，可對 `/ws/src/shopping_robot_navigation/maps/loc_yaw_compare.yaml` 執行離線幾何工具，再以相同 map_server／loc-check 步驟比較；不要覆寫提交格網來混淆 AMCL 調參效果。只有證據支持後才重新產生正式地圖。此次不需先 map-tour。
- 未執行 Docker、ROS、colcon 或 Gazebo；未安裝套件、建立 venv、加入硬體／裝置程式、提交或推送。未勾選 ROADMAP。README 補齊兩工具操作與限制；靜態檢查及替身測試不代表容器整合驗收。

### 本次執行檢查（非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：**640 passed**，包含本次新增 30 項測試；僅有既有 xacro Python 棄用警告。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：**PASS**，13 個 YAML／RViz、10 個 XML／SDF／xacro、81 個 Python、4 個 Bash；兩／三錨點與 ekf／ground_truth 的 xacro 展開及錨點一致性通過。
- `git diff --check`：通過。離線格網工具已對提交地圖實際執行；ROS 傳輸、六列動態量測、AMCL 改善與 colcon 仍待維護者驗證。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python 測試與轉換／評估往返 | 待驗證 |
| YAML／XML／xacro／launch 與 Bash 靜態檢查 | 待驗證 |
| Humble colcon build／test 與執行期依賴 | PASS：colcon test 647 tests、0 failures（t31s） |
| guide 短錄製、成功／失敗回合與真值邊界 | 待驗證 |
| follow 短錄製、同步距離與停止事件 | 待驗證 |
| report JSON／Markdown 與中繼資料完整性 | 待驗證 |
| 停止所有控制來源後的隔離 bag 回放 | 待驗證 |

### 本輪審查驗證（2026-10-24 08:50 臺北時間，盒內 Linux amd64，EKF 預設）

| 項目 | 結果 |
| --- | --- |
| 審查修正 | 實際 bag 轉換時 `bag_to_eval` 失敗：`numpy.float64` 也有 `tolist()`，原本會被當成序列走訪。已改為先呼叫 `tolist()` 再判斷是否為序列，並新增 numpy 類純量／陣列測試 |
| 純 Python 測試 | `pytest -q` 632 項通過；check_static PASS；`git diff --check` 通過 |
| colcon | 7 套件建置成功；colcon test 531 項、0 錯誤、0 失敗、5 略過 |
| guide 短錄製（70 秒，`goto snacks`→`goto entrance`） | 錄到 /clock、/odom、真值、/cmd_vel、模式與區域狀態；`eval-bag` 產生 report.json／md：2/2 成功，耗時 7.2／16.3 s，最終誤差 0.143／0.127 m，最小靜態間距 0.41 m |
| follow 短錄製（60 秒） | 跟隨 1916 筆樣本、38.3 s；平均絕對誤差 1.46 m（目標 1.0±0.2 m 帶內 5.7%）；擷取到 obstacle_stop／too_close 停止事件，命令延遲 0.005–0.056 s |
| 未驗證 | 隔離 bag 回放（`ros2 bag play`）、失敗／取消回合的實際錄製、Mac arm64 |

## 2026-10-24：人物同步與跟隨評估

### 本次完成

- 新增 `shopping_uwb_follow` 的 `person_sync_check`、`follow_eval` 命令入口及 `dev.sh person-sync-check`／`follow-eval`。工具唯讀訂閱，預設使用模擬時鐘，暖機 2 秒後記錄指定模擬時間；具牆鐘逾時、時鐘倒退／框架／有限數值檢查、來源時間對齊、資料覆蓋、表格與 JSON 輸出。結束碼為全部通過 0、有失敗 1、僅未涵蓋 2。
- 確認人物移動節點與 UWB 模擬都從 Gazebo 套件的唯一 `person_path.yaml` 依絕對模擬時間插值。既有 SceneBroadcaster `/simulation/poses` 雖有模型名稱，但 Humble 的 Pose_V→TFMessage 轉換不保留外層時間戳與父框架，不能可靠用於來源時間延遲量測。故在人物 SDF 加入 Fortress 唯讀 PosePublisher，提供各筆時間戳與 `supermarket → person`；只有 follow launch 透過 `bridge_person.yaml` 單向接至 `/simulation/person_pose`，不進入 `/tf`。此為觀察資料的補足，沒有修改人物位移、UWB 生成或既有橋接消費端；尚未發現需修改移動演算法的同步錯誤。
- 同步檢查比較人物模型、`/uwb/person_ground_truth` 與解析路徑，列出平面偏差統計和最佳時間位移。跟隨評估使用獨立 `/sim/ground_truth_odom` 與人物真值，轉出車體相對位置，量測估測延遲、直線／轉彎距離與估測誤差、近停違規及 stale／LOST 後新鮮估測至 FOLLOWING 的時間。
- 指標集中於不匯入 ROS 的 `evaluation_metrics.py`，沿用 `person_path.path_position`。測試涵蓋帶噪已知正負位移、共同搜尋樣本／插值缺口、運動不足／邊界解、轉彎分類、近停持續／旋轉／紀錄截斷、恢復時間／缺資料、框架驗證、純模組匯入、入口／dev.sh 接線，以及假 Humble 單參數回呼的完整同步紀錄與 JSON 輸出。
- README 與架構文件補齊操作、來源實作連結、門檻與限制。跟隨控制器、調參、TF 所有權與 ROADMAP 均未修改；未提交或推送，未執行 Docker、安裝套件或建立虛擬環境。

### 門檻與判定

| 項目 | 預設門檻／方法 |
| --- | --- |
| 人物同步 | 三組平面平均偏差各 <0.05 m；各組最佳延遲絕對值 <0.1 s |
| 延遲搜尋 | 整段紀錄窗、±0.5 s、0.01 s 格距、相同共同樣本、至少 10 筆與 0.02 m 運動激發；最佳值貼邊不通過 |
| 濾波延遲 | 相對位置向量最佳位移絕對值 <0.3 s；正值表示估測落後真值 |
| 直線／轉彎 | 路徑前後各 0.5 s 的航向變化率 >0.3 rad/s 為轉彎；各類至少 10 筆，真實距離減 YAML 期望距離的 RMSE <1.5 m |
| 近停 | 真實距離 <YAML `stop_distance` 後 0.5 s 起，平面速度 ≤0.03 m/s、角速度絕對值 ≤0.05 rad/s，維持至離開近距區間 |
| 重新取得 | 估測來源時間缺口 >YAML `lost_timeout` 或 LOST 後，下一筆新鮮估測至 FOLLOWING ≤2 s；另外列出整體遺失至恢復時間 |
| 資料覆蓋 | 真值／模型／狀態時間窗兩端與中間缺口至多 0.25 s；插值不外插、不跨較大缺口 |

### 本輪執行紀錄（非審查驗證）

| 指令 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 631 項通過；既有 xacro 棄用警告 1 項 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：14 個 YAML／RViz、10 個 XML／SDF／xacro、81 個 Python、4 個 Bash；兩／三錨點與兩種里程計模式 xacro 展開通過 |
| `git diff --check` | 通過 |

### 限制與下一步

- 尚未執行 ROS、colcon 或 Gazebo。維護者須在既有容器建置／測試，再啟動單一 follow，確認人物 PosePublisher 的 Fortress 輸出、來源時間戳與父子框架，執行兩個檢查；純 Python 與靜態檢查不能代替此驗收。
- 一般循環路徑不保證出現近距停止；無轉彎、近停、UWB 缺口或 FOLLOWING 恢復事件時標為 `NOT_COVERED`。近停起訖碰到紀錄邊界、區間太短或有缺口，也不會通過；觀察到門檻後仍移動則記錄違規。恢復事件若估測未恢復或狀態缺資料則未涵蓋，估測恢復且充分觀察後仍不跟隨則失敗。下一步由維護者安排可重現的近停情境，並在 UWB 缺口前啟動量測，補齊所有事件覆蓋。
- 延遲統計包含取樣、橋接、服務呼叫排程、位置偏差與模擬時間誤差，並非濾波函式內部耗時；搜尋格距不代表精度。轉彎分類依解析路徑，應先確認同步；角落為折線瞬間轉向，以有限時間窗分類。統計依樣本加權，沒有額外濾波或變更控制器。
- `/follow/state` 沒有時間戳，使用接收時模擬時間；NAV2_HANDOFF 不視為 FOLLOWING。車體相對位置沿用目前 base_link 與 base_footprint 平面原點相同的模型假設。自訂模擬參數／路徑必須用 `--follow-params`／`--person-path` 提供相同檔案；讀取 YAML 不代表查詢執行中節點的即時參數。
- 僅模擬、無硬體或裝置連線。兩錨點仍有前後解歧義，NLOS 仍可能產生錯誤位置；量測工具不會改善估測或保證碰撞安全。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python 指標與單參數回呼測試複核 | 待驗證 |
| YAML／XML／launch 語法與 xacro 靜態檢查複核 | 待驗證 |
| Humble 容器 colcon build／test | 待驗證 |
| Fortress 人物來源時間、位姿橋接及框架 | 待驗證 |
| `person-sync-check` 三組位置與延遲 | 待驗證 |
| `follow-eval` 濾波延遲與直線／轉彎 | 待驗證 |
| 近距停止、持續停止與違規情境 | 待驗證 |
| UWB 缺口後重新取得與未涵蓋判定 | 待驗證 |
| 預設 EKF／ground_truth 模式與 TF 唯一來源回歸 | 待驗證 |

### 本輪審查驗證（2026-10-24 08:30 臺北時間，盒內 Linux amd64）

| 項目 | 結果 |
| --- | --- |
| 盒內 pytest | 653 passed |
| `check_static.py` | 通過 |
| 容器 colcon build + test | 552 tests，0 errors，0 failures，5 skipped（修正測試路徑：容器內 repo 根目錄不在 `parents[4]`，改以 `src` 推算，dev.sh 不存在時 skip） |
| 修正：人物模型位姿主題 | Fortress `PosePublisher` 發布於 `/model/person/pose`（非 `/world/supermarket/model/person/pose`），已改 bridge_person.yaml、測試與文件；修正後 model_coverage 由 0 筆變 1765 筆 |
| `person-sync-check --duration 60`（follow 模擬） | truth_vs_path PASS（誤差 0）、person/model coverage PASS；**model_vs_path / model_vs_truth FAIL**：Gazebo 模型平均落後 0.105 m、延遲 0.30 s（補償延遲後 RMSE 0.029 m）。這是模擬中模型驅動的真實延遲，工具判定正確，列為待修項目（見 ROADMAP） |
| `follow-eval` | 濾波延遲 0.07 s PASS、重新取得 0.049 s PASS、近距停止 NOT_COVERED、**tracking FAIL**（平均距離 3.35 m，目標 1.0 m，與 10/23 的 3.02 m 一致）：是跟隨控制器現況，非工具錯誤 |
| 未驗證 | Mac arm64、Gazebo GUI、實車（只做模擬） |
| 純邏輯、PGM／邊緣估計與中斷零速測試 | 待驗證 |
| YAML／XML／xacro／launch／Bash 靜態檢查及差異空白檢查 | 待驗證 |
| Humble colcon build／test | 待驗證 |
| Fortress 六列偏航／線速比、低速起轉與中斷停止 | 待驗證 |
| 容器 SDF 摩擦設定與實際接觸模型 | 待驗證 |
| 提交格網的六區 AMCL 定位重測與跨次穩定性 | 待驗證 |
| Nav2／跟隨回歸及選用比較建圖 | 待驗證 |

### 審查驗證（2026-10-24 14:30 臺北時間，盒內 Linux amd64 Docker）

| 項目 | 結果 |
| --- | --- |
| 盒內 pytest／`check_static.py` | 683 passed／PASS（rebase 至 develop 496685d 後） |
| 容器 colcon build + test | 552 tests，0 errors，0 failures，5 skipped |
| `yaw-check`（plain sim、EKF） | 原地旋轉比值：0.2→1.187、0.4→1.441、0.6→1.382、0.8→1.385 rad/s（平均 1.349）；EKF 與真值一致。弧線兩列因淨空不足中止（工具正確停車並發布零速）。擬合 `fortress_command_factor` ≈ 1.48，**未套用**，xacro 仍為 2.0（低速起轉非線性，需多速度確認） |
| AMCL 六區 `loc-check`（map_server、新 likelihood_field 參數） | **6/6 PASS**：drinks 0.031 m、snacks **0.096 m**（原 0.168 m 失敗）、produce 0.033 m、household 0.057 m、checkout 0.002 m、entrance 0.008 m；航向誤差最大 2.07° |
| 未驗證 | Mac arm64、Gazebo GUI、`map-geometry-check` 實際格網比較、yaw 弧線列、AMCL 重複多次的穩定度 |
## 2026-10-24：micro-ROS 通訊層與模擬韌體

### 修改內容

- 新增 `shopping_mcu_bridge` ament_python 套件與唯一介面來源 `config/interface.yaml`；以標準 Twist、JointState、Imu、Bool、UInt32、Int64MultiArray、String 定義命令、四輪狀態／encoder、IMU、急停、雙向 heartbeat 與狀態。MCU 遙測維持 `/mcu/*`，沒有取代 Gazebo `/odom`、`/imu` 或新增 TF。
- 新增 `sim_mcu_firmware` 與純 Python 安全／輪模型：直接讀 description xacro 的 0.10 m 輪徑與 0.92 m 有效輪距，具一階響應、加速度限制、不倒車、無效命令拒絕與飽和。命令逾時、PC heartbeat 遺失與急停皆立即零輪速；失聯須三筆新 heartbeat，急停須放開按鈕再明確 reset。所有安全期限採單調壁鐘。
- 新增 `mcu_link_monitor`，選用後透過既有 `/mode_manager/request` 請求 estop。速度唯一出口仍是既有 watchdog；通訊節點沒有發布 `/cmd_vel`，也不自動解除模式急停。
- 新增固定二進位 serial codec／CRC16-CCITT-FALSE、可重同步的分段解碼器與 PTY 模擬 peer。`serial_bridge` 預設只建立程序內 PTY；實體串列必須同時 `use_hardware:=true` 與明確 port，才延遲匯入 pyserial，不掃描裝置或自動重連。包含真實 PTY 雙向收送與斷線／恢復測試。
- 新增 comm launch，支援 sim_ros／serial_loopback；follow、guide 以預設 false 的 `mcu_comm` 引入。獨立 `dev.sh comm-sim` 啟動 mux／watchdog、模擬韌體與 monitor，無 Gazebo 可執行。`comm-check` 檢查隔離 graph 後才經 teleop／mux 注入速度與故障，列 PASS／FAIL，最後保持急停。
- compose 新增 `comm` profile 的 UDP4 agent（官方 Humble image 命名、內部 8888、無裝置／host port／固定平台），提供獨立 serial opt-in override 範本，預設 devices 仍為註解。`dev.sh up` 仍只啟動 compute／display；新增 comm-up／comm-down／comm-sim／comm-check。
- 新增 MCU_INTERFACE、MICROROS_GUIDE，更新 README／ARCHITECTURE，對照舊競賽 repo 的 PC bridge 與 rosserial 經驗；說明 CubeIDE／PlatformIO、FreeRTOS／記憶體預算、QoS、重連與備援。H7 完整板型、F446 是否 F446RE、「451d」均保留待確認。ROADMAP 未勾選或修改。

### 本機自行檢查

指定既有環境 `/workspace/scratch/venv-shop/bin/python -m pytest -q`：682 項通過（本次新增 72 項），保留既有 xacro Python 棄用警告 1 項。`/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS，包含 YAML／XML／Python／Bash 與兩／三錨點、ekf／ground_truth 的 xacro 展開。`git diff --check`：通過。未安裝套件、建立虛擬環境、執行 Docker、提交或推送。

### 限制與下一步

- 這些是純 Python、PTY 與靜態檢查，不能冒充 Humble DDS、colcon、容器、Gazebo 或硬體驗收。維護者須依 MICROROS_GUIDE 重新建置並分別執行兩種 transport 的 comm-check，再測 follow／guide 的選用啟動。
- 模擬韌體是一般 rclpy DDS 節點；serial loopback 走自訂 codec；兩者都不經 micro-ROS agent。目前沒有真正 XRCE client 或 MCU 韌體。下一步可另建 micro_ros_setup host／Linux client 或確認板型後再接板；ROADMAP 對 agent 的端到端項目尚未達成。
- 官方映像名稱與發行版 tag 依官方文件對照，但未 pull 或檢查 registry manifest；amd64／arm64 原生啟動與 DDS discovery 仍待驗證。預設 UDP 僅 compose 內部可達，外部 client 的 port mapping 必須另外明確設定。
- serial 版本 1 只攜帶 yaw rate，不是完整實體 IMU 資料；stamp 由 PC 接收時重建，未作 MCU 時鐘同步。輪模型未辨識實車滑移；4096 ticks/rev、速度／加速度與 IMU covariance 皆為模擬值。實車 EKF 轉接、馬達／encoder／IMU 驅動與電氣急停均未實作。
- 實體 serial 分支是未驗證的明確選用備援，預設容器沒有裝置映射，也不保證含 pyserial。連線失敗保持 LINK_LOST，不自動重開；必須先由維護者確認依賴、韌體協定與雙邊安全期限。不要對同一埠同時啟動 agent 與 binary bridge。
- 通訊注入驗收只能用獨立 comm-sim，不得與 Gazebo／follow／guide 同時執行；整合模擬時沿用原有單一 mux／watchdog 與一致 use_sim_time。

### 審查驗證

以下全部由審查者執行與填寫，本次自行檢查不代填審查結果。

| 項目 | 結果 |
| --- | --- |
| 純邏輯／kinematics／heartbeat／急停 pytest 審查 | 待驗證 |
| codec 分段、損毀、PTY 與硬體雙重啟用條件 | 待驗證 |
| check_static、xacro、介面／文件／QoS 一致性 | 待驗證 |
| 容器 colcon build／test（Humble） | 待驗證 |
| sim_ros 獨立 comm-sim＋comm-check | 待驗證 |
| serial_loopback 獨立 comm-sim＋comm-check | 待驗證 |
| agent profile、UDP 啟動與 amd64／arm64 原生映像 | 待驗證 |
| 真正 micro-ROS client 經 agent 端到端連線 | 待驗證 |
| follow／guide mcu_comm 預設關閉與明確啟用 | 待驗證 |
| 模式急停請求、唯一 cmd_vel 發布者及現有 EKF／TF 不受影響 | 待驗證 |
| 實體 serial 與 MCU 韌體／硬體安全 | 待驗證 |

### 審查驗證結果（2026-10-24 14:40 臺北時間，盒內 Linux amd64 Docker）

| 項目 | 結果 |
| --- | --- |
| 盒內 pytest／`check_static.py` | 755 passed／PASS（rebase 至 develop 3346718 後） |
| 容器 colcon build + test | 624 tests，0 errors，0 failures，5 skipped。修正：套件測試在容器（`/ws/src`）找不到 repo 根目錄，37 項失敗 → 改為找不到 `docs/` 時使用 `/repo` |
| sim_ros `comm-sim` + `comm-check` | 10/10 PASS。修正：首次執行因 DDS discovery 未完成，隔離檢查誤判 FAIL；改為最多輪詢 10 秒並在失敗時記錄發布／訂閱者 |
| serial_loopback `comm-sim` + `comm-check` | 10/10 PASS |
| `comm-up`／`comm-down`（`microros/micro-ros-agent:humble`，UDP 8888） | 映像可取得，agent 啟動並顯示 running；**沒有** XRCE client 連線（模擬韌體不經 agent） |
| 未驗證 | 真正 micro-ROS client 經 agent 端到端、Mac arm64、guide／follow `mcu_comm:=true` 與 Gazebo 同時執行、實體 serial／MCU 韌體 |

## 2026-10-24：跟隨追蹤誤差修正

### 根因分析與修改

- 新增 `scripts/follow_loop_sim.py`：直接讀取 Gazebo 套件唯一 `config/person_path.yaml` 與 `uwb_follow.yaml`，使用既有 TargetFilter、控制器、LOS 遲滯、VelocityMux、VelocityLimiter 與 OutputWatchdog。保留修改前完整 `step` 公式作為固定對照。單輪車模型從原點開始，控制／仲裁／看門狗 20 Hz、UWB／雷射 10 Hz；`--world` 由 SDF 碰撞盒與人物圓柱產生 181 條近似雷射，使用車前 0.18 m 的雷射原點。完整一圈 94.857 s，1898 筆樣本，沒有省略啟動暫態或重設車體追上人物。
- 人物各段都是同一個 0.35 m/s，低於直接控制上限 0.5 m/s 與仲裁上限 0.6 m/s；不是預設路徑整體超速。原本比例控制在正前方等速時需要 `0.35/0.7=0.5 m` 距離誤差才產生維持速度。側向／LOS 倍率原先乘在飽和後的整個命令，再乘朝向餘弦，會同時縮小未飽和修正量及實際速度上限。倍率 0.4 時上限僅 0.2 m/s；LOS 不確定倍率 0.6 時上限也僅 0.3 m/s，均無法追上 0.35 m/s 人物。彎道落後增加後，LOS 鎖定等待會進一步放大距離。
- 無障礙舊版平均距離只有 1.397 m；加入 SDF、側向減速、朝向閘控與 LOS 保持後，全程平均距離為 2.766 m，接近容器 3.35 m 的量級，但不是精確重播。無障礙情境移除控制器加速度限制後為 1.396 m，顯示該情境中加速度不是主因；障礙情境則會因轉角進入不同阻塞狀態而改變結果。主要證據支持比例控制落後與幾何／安全等待的放大效應，不能只把 3.35 m 歸因於增益不足，也不能從此近似模型斷言 Nav2 執行時的單一根因。
- 人物圓柱本身也會被雷射掃到：雷射前移 0.18 m＋人物半徑 0.25 m＋前方停止門檻 0.65 m，使正前方距離約 1.08 m 就可能觸發安全停止。這解釋修正版仍有部分安全停止與正距離誤差，不能透過提高增益消除；保留原安全幾何與門檻。
- 新增純邏輯 TargetVelocityEstimator：以 UWB 估測位置及里程計位姿轉至 odom 後差分，EMA alpha=0.2、向量限幅 0.6 m/s，再轉回車體座標；資料缺口／時間倒退會清除歷史。跟隨節點持續訂閱 `/odom`，量測時間差與控制時里程計年齡限制為 0.15 s；超過門檻停用前饋，故障模式原有里程計停止規則不變。
- 控制改為目標絕對速度在視線方向的前饋＋距離比例修正，再以原側向／LOS 速度上限限幅，保留朝向閘控及原加速度限制。沒有提高線／角速度、加速度或增益。新增距離誤差 0.04／0.08 m 保持遲滯，保持時抑制比例修正、保留前饋。近距、目標遺失／過期、雷射失效及前方障礙直接輸出零 Twist；前方有效障礙停止改為包含角速度歸零，不再原地轉向。既有 Nav2 獨占交接流程與管線保持原狀，看門狗仍是預設模式唯一 `/cmd_vel` 發布者。
- 人物模型原先每 0.1 s 取樣當下路徑後啟動 `ign service set_pose` 子程序，前一個請求未結束就跳過；命令取樣早於套用，加上位置保持會累積延遲。移動命令沒有經 ROS bridge，bridge 只量測位姿；`set_pose` 也沒有排程套用時間戳。本輪不能分離 0.30 s 中程序啟動、服務排程與保持時間的各自比例。新增純邏輯 `model_command_position` 與節點參數 `model_lag_compensation_s=0.30`，模型命令取樣改為 t+補償，嘗試週期縮至 0.05 s，仍只允許一個請求在途。服務完成觀察耗時寫入 DEBUG 紀錄，依模擬秒計算。
- 補償不修改 UWB 真值、人物觀測時間戳或同步檢查參考路徑。人物路徑、速度與碰撞模型均未修改，仍為模型與 UWB 的唯一來源；未修改 Nav2 設定／launch、Docker、通訊層或 ROADMAP，未提交／推送。

### 閉迴路結果（純 Python，非容器驗收）

誤差平均為 `mean(abs(distance-1.0))`，RMSE 使用相同誤差；距離與分類沿用 `follow_metrics`／`evaluation_metrics`。有效區間排除安全停止、原始 LOS 不通或 LOS 鎖定未解除的樣本；每次均保留全程統計及互斥排除計數。無雜訊、理想里程計，尚未模擬 Nav2、碰撞動力學或傳輸排程。

| 情境 | 全程平均距離 m | 有效區間平均距離 m | 有效區間平均誤差 m | 有效區間 RMSE m | 有效樣本比例 |
| --- | --- | --- | --- | --- | --- |
| 無障礙，舊版 | 1.397 | 1.397 | 0.397 | 0.412 | 100% |
| 無障礙，修正版 | 1.003 | 1.003 | 0.053 | 0.109 | 100% |
| SDF，舊版 | 2.766 | 1.673 | 0.673 | 0.734 | 51.7%（982／1898） |
| SDF，修正版 | 1.807 | 1.171 | 0.171 | 0.198 | 69.3%（1316／1898） |
| SDF，修正限幅但停用前饋 | 2.010 | 1.470 | 0.470 | 0.505 | 73.6%（1396／1898） |
| SDF，舊版移除控制器加速度限制 | 3.158 | 1.620 | 0.620 | 0.680 | 33.7%（640／1898） |

SDF 舊版排除安全停止 84 筆、LOS 受阻／鎖定 832 筆；修正版分別為 86／496 筆。修正版有效區間已達平均誤差 <0.5 m、RMSE <1.0 m，但全程平均距離仍大於目標，不能視為完整跟隨驗收通過。固定倍率 0.4 的壓力情境仍會失敗，證明前饋沒有繞過實際速度上限。預設走道中央約 0.8 m 側向淨空可使用完整速度，因此沒有全面放慢人物路徑。

### 測試、限制與下一步

- 新增 pytest 覆蓋舊版閉迴路量級、修正版全路徑有效區間門檻、移除前饋／加速度對照、持續低速上限的失敗、速度估測的自車平移／旋轉補償與缺口重設、節點新鮮里程計接線、距離遲滯、前饋不得越過安全停止、模型路徑補償與循環邊界。既有前方障礙測試更新為完整零 Twist；其餘安全與路徑淨空回歸一併執行。
- 人物補償的合成測試明確假設服務延遲為 0.20／0.25／0.30 s，加上命令保持；以原 `sync_report` 檢查修正後平均誤差 <0.05 m、延遲絕對值 <0.1 s。這不是 Gazebo 量測。0.30 s 補償依維護者先前實測校準，主機負載、實際模擬倍率與轉角外推都可能改變結果；必要時用 `ros2 param set /person_path model_lag_compensation_s 值` 重校，0 關閉。
- 尚未執行 ROS、colcon 或 Gazebo；沒有安裝套件、執行 Docker、加入硬體／裝置連線。里程計時間只用最新量測，未補間；EKF 雜訊、漂移、UWB 雜訊／掉包與兩錨點前後解歧義／NLOS 限制仍存在。模型命令補償不能修正真實 UWB 估測錯誤。
- 近似閉迴路沒有 Nav2，因此不能證明貨架末端繞行已修復。維護者須在既有容器重新建置／測試，停止其他移動流程後只啟動單一 follow；同時量測人物同步與全程追蹤，檢查交接卡住、轉角與停止事件。線上 `follow-eval` 沒有套用離線有效區間排除，應保留原始全程結果。
- 建議命令：`./scripts/dev.sh colcon-build`、`./scripts/dev.sh colcon-test`；啟動 `./scripts/dev.sh follow fault_injection:=true 'fault_schedule:=[{time: 120.0, fault: uwb_stale, duration: 3.0}]'`，再於不同終端機執行 `./scripts/dev.sh person-sync-check --duration 60 --output /tmp/person-sync-follow-tracking.json` 與 `./scripts/dev.sh follow-eval --duration 180 --tracking-limit 1.0 --output /tmp/follow-tracking.json`。兩個量測都須在 120 s 故障前啟動。近距停止可能仍為 NOT_COVERED，需另外安排可重現的近停測試。

### 本輪執行紀錄（非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：682 項通過，既有 xacro 棄用警告 1 項。
- `PATH=/workspace/scratch/venv-shop/bin:$PATH python3 scripts/check_static.py`：PASS；15 個 YAML／RViz、10 個 XML／SDF／xacro、87 個 Python、4 個 Bash，兩／三錨點與 ekf／ground_truth 的 xacro 展開及錨點一致性通過。
- `git diff --check`：通過。以上未包含 ROS、colcon、Gazebo 或硬體驗收。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純 Python 舊版重現、前饋、遲滯與安全回歸複核 | 待驗證 |
| 人物模型補償合成測試與路徑淨空複核 | 待驗證 |
| pytest、YAML／XML／launch 語法與 xacro 靜態檢查複核 | 待驗證 |
| Humble 容器 colcon build／test | 待驗證 |
| Fortress `person-sync-check` 平均誤差與延遲 | 待驗證 |
| `follow-eval` 全程直線／轉彎追蹤與前饋雜訊 | 待驗證 |
| 貨架末端 LOS／Nav2 繞行與獨占控制 | 待驗證 |
| 近距停止、前方障礙零 Twist、UWB 缺口恢復與急停 | 待驗證 |
| 預設 EKF／ground_truth 與唯一 TF／cmd_vel 發布者回歸 | 待驗證 |

### t29：180 秒評估失敗後的控制與繞行修正（2026-10-24）

#### 證據與根因判讀

- 維護者本次 Fortress 結果：直線平均距離 2.895 m、最大 6.465 m，轉彎平均 3.368 m；估測平均誤差約 0.13 m、延遲 0.07 s，近停通過。這支持優先調查車體控制與交接，而不是再次調整感知。完整啟動紀錄有 96 次目標 preemption、16 次控制器目標取消，以及 100 次 RPP 預測碰撞訊息；這些計數涵蓋完整啟動紀錄，不等同評估區間的事件計數。
- 程式原先已對移動人物使用 `send_goal_async` preemption，並非每次更新都先取消。問題在另一條返回條件：LOS 恢復後，不論是否還落後數公尺或朝向轉角，都能在短暫淨空後取消 Nav2、等待終止／速度排空，再從零速直接追人。直接追逐切過貨架轉角，又會重新進入繞行，與紀錄的週期取消相符；舊紀錄缺少跟隨狀態，因此無法把每次取消唯一對應至某一個分支。
- 人物軌跡原速 0.35 m/s，路徑沒有停留、轉角沒有減速；`person_path_node` 與 UWB 都依同一份路徑取樣，模型命令保留原 0.30 s 補償與單一在途服務請求。0.35 m/s 雖小於直接跟隨／smoother 的 0.5 m/s 上限，卻大於側向最低限速 0.2 m/s；舊側向與 LOS 倍率相乘時甚至只有 0.12 m/s。Nav2 原巡航 0.4 m/s，曲率限速最低 0.25 m/s，追趕餘裕不足。mux 上限 0.6 m/s、加速度 0.6 m/s²，以及 Gazebo DiffDrive 的 0.6 m/s 上限並不是此處較低的限速來源；直接控制與 smoother 仍先限制在 0.5 m/s、0.4 m/s²。
- 原 Nav2 目標在人物中心，終點與人物碰撞模型重合，依賴 planner 的 0.8 m 容許範圍找附近終點。另有安全缺口：直接控制器的 `OBSTACLE_STOP` 雖已產生零 v/w，Nav2 擁有控制時卻沒有持續發布該零速，可能讓較低優先序的 Nav2 命令通過。
- 評估距離是相同世界座標系中車體原點與人物中心的平面歐氏距離，不是人物估測誤差，也不是沿路徑距離；直線／轉彎依人物路徑分類，包含安全停止與交接等待，沒有排除難追蹤區間。這些定義及驗收門檻均保留。

#### 修改與安全界線

- Nav2 返回直接控制改為：最少持有時間已滿、人物中心距離不超過 1.3 m、視線角度不超過 0.25 rad、LOS 經較寬走廊確認、前方淨空超過停止值加餘裕，且上述返回條件持續 `resume_delay=0.5 s`。遠處可見或尚未對齊不再取消繞行；目標成功／失敗、停用、近停、遺失、過期與失效資料仍走原本終止／安全流程。
- 移動目標繼續以 Nav2 preemption 更新；同一時間最多等待一個接受回覆，等待期間合併新目標，回覆後使用最新位姿更新。正常更新不呼叫取消、不啟動速度排空。Nav2 終點改為人物前方保留 1.0 m 的位置，距離不足時不把終點放到車後；LOS 與安全判斷仍使用人物本身，沒有遮掉雷射回波。共用 planner 容許範圍仍為 0.8 m，實際終點與轉角路徑仍須在容器確認。
- LOS 安全走廊半寬由 0.25 m 改為 0.4 m，涵蓋 0.60 × 0.50 m 車體旋轉時約 0.391 m 的外接半徑，提早攔截轉角切入；解除阻塞仍額外增加 0.10 m。此值是控制走廊，不是修改實際車體 footprint。
- 側向與 LOS 現在各自作為速度上限，合併取較小值，避免重複相乘。保留側向最低 0.4 倍、LOS 0.6 倍與所有速度／加速度限制。人物唯一設定改為 0.18 m/s（約 0.65 km/h）的緩慢購物瀏覽速度，低於窄處 0.2 m/s 限速與 RPP 曲率限速，留有少量追趕餘裕；這不是一般成人自由步行速度，也不保證任意急彎都能等速跟上。Nav2 巡航改為既有安全上限 0.5 m/s；沒有提高直接控制、smoother、mux 或 Gazebo 的上限。
- Nav2／排空期間若前方障礙停止，持續發布高優先序跟隨零命令，同時遮蔽 v/w，不為每筆障礙回波取消再重送目標；障礙解除後才允許 Nav2 命令通過。近停、遺失、過期、雷射失效與急停仍立即歸零。前方停止門檻保留 0.65 m，因此雷射前移 0.18 m、人物半徑 0.25 m 的正前方幾何仍會在人物中心距離約 1.08 m 觸發；安全值不能用比例增益消除。若車已停在永久貨架的停止區內，本輪仍不允許為脫困而帶速旋轉，必須重設模擬並確認繞行是否及早介入。
- 跟隨節點新增每模擬秒最多一次 INFO，記錄 state／mode、距離、視線角、上游命令 v/w、速度倍率、前方距離、目標年齡、Nav2 狀態／待回覆、LOS 鎖定、排空狀態及限制原因；時間倒退後可重新記錄。直接控制限制原因包含速度上限、朝向、線／角加速度與距離保持。Nav2 命令採最近觀測的 smoother 輸出，安全遮蔽時顯示零；這不是 `/cmd_vel` 最終實測值，mux 與 watchdog 仍可能再限速或停止。
- shutdown 捕捉 `KeyboardInterrupt`／`ExternalShutdownException`，只在 context 有效時嘗試取消與發布最後零速，並捕捉檢查後 context 失效造成的 `RCLError`；已關閉時不再重複 shutdown。最終輸出仍由獨立 watchdog 逾時保護。預設 launch 的 `/cmd_vel/follow`、`/cmd_vel/nav`、mux、watchdog 管線維持原狀，watchdog 仍是唯一最終 `/cmd_vel` 發布者。

#### 本機檢查與限制

- 新增／調整純 Python 測試：遠處 LOS 恢復不取消、轉角未對齊不交接、持續近距淨空才返回、更新接受等待與合併、舊回呼隔離、人物前方終點與實際節點座標轉換、獨立速度上限合併、Nav2 前方障礙雙軸零速與解除、診斷節流／時間重設／Nav2 命令、有效／失效／競態 shutdown、旋轉安全走廊與跨設定速度預算。需要整個 repo 設定的套件測試先檢查根目錄 `docs/`，否則使用 `/repo`，皆不存在才 skip。
- 離線腳本同步新的 LOS 走廊與速度上限合併；舊版重現測試明確固定舊人物速度 0.35 m/s，避免慢速設定掩蓋歷史問題。最終 0.18 m/s 全圈 184.4 s、3689 筆的純 Python 單輪車模型：無障礙全程平均距離 0.976 m、誤差 RMSE 0.059 m；持續 0.2 m/s 低速上限時平均距離 1.000 m、誤差 RMSE 0.094 m。包含啟動與所有轉角，沒有重設車體追上人物。
- **SDF 直接控制模型仍未通過全程追蹤**：平均距離 1.760 m、誤差 RMSE 1.551 m、最大距離 6.379 m；1341 筆安全停止，正常追蹤樣本比例約 63.6%。該模型沒有 Nav2、碰撞動力學、EKF 漂移或排程，會因保守障礙停止停在轉角，不能拿正常追蹤子集合冒充完整驗收。本次 Nav2 交接改善由狀態／回呼測試驗證，仍需維護者實跑 Fortress 才能確認全程追蹤改善。離線完整結果暫存於 `/tmp/follow-t29-offline.json`。
- 未安裝套件、建立環境、執行 Docker、colcon、ROS 或 Gazebo；未新增硬體／裝置連線，未提交或推送，ROADMAP 未勾選。靜態與純邏輯結果不代表容器或硬體驗收。
- 下一步：維護者重新建置與 colcon test，重跑相同 180 s `follow-eval` 與人物同步量測，保留完整 INFO 與 `/follow/state`、`/cmd_vel_mux/active`、最終 `/cmd_vel`。對照每次取消的近距／角度條件、障礙停止是否及早繞行、速度是否受側向限速，以及直線／轉角全程誤差。近停必須維持 PASS；若變成 NOT_COVERED，需另外安排近距事件，不能視為通過。人物完整一圈現在約 184.44 s，故障測試仍須另外明確注入 UWB 過期事件。

#### t29 本輪自行檢查（非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：802 項通過，保留既有 xacro 棄用警告 1 項。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；17 個 YAML／RViz、11 個 XML／SDF／xacro、104 個 Python、4 個 Bash，以及兩／三錨點、ekf／ground_truth 的 xacro 展開與錨點一致性皆通過。
- `git diff --check`：通過。沒有執行容器或實際模擬驗收。

#### t29 審查驗證

| 項目 | 結果 |
| --- | --- |
| 純邏輯、回呼競態、速度預算與安全停止測試複核 | 待驗證 |
| pytest、check_static、YAML／XML／launch／xacro 複核 | 待驗證 |
| Humble 容器 colcon build／test 與 shutdown 無 RCLError | 待驗證 |
| Fortress 人物同步、0.18 m/s 瀏覽速度與完整路徑 | 待驗證 |
| 180 秒全程直線／轉彎追蹤、Nav2 取消頻率與早期繞行 | 待驗證 |
| 近停 PASS、前方障礙雙軸停止、雷射失效、過期與急停 | 待驗證 |
| INFO 診斷與最終 cmd_vel／仲裁來源對照 | 待驗證 |
| 唯一 cmd_vel 發布者、EKF／ground_truth TF 與模擬限定回歸 | 待驗證 |

### t32：貨架前永久停止、Nav2 交接與速度恢復（2026-10-24）

#### 診斷與修改

- 先讀取 AGENTS.md，檢視重開機前留下的四份修改，保留人物速度恢復、旋轉淨空檢查、導航速度閘控與成功狀態不直接退出的方向，完成接線、失效停止、測試及文件。未提交或推送。
- 本輪依據 `/workspace/wt/logs/shop/t29s-check1.txt`、`t29s-follow-diag.log` 與 `t29s-launch.log`：180 秒評估直線平均距離 2.883 m、轉彎平均 3.544 m，near_stop PASS；完整啟動紀錄有 23 筆 `Goal was canceled`。初期 DIRECT 約 1.1 m，後續前方約 0.606～0.608 m 長期不變，OBSTACLE_STOP 同時遮蔽線／角速度，使車體面對靜態貨架時無法轉向；此時人物走遠不能藉增益或估測修正解除。取消計數涵蓋完整啟動，不將每筆唯一歸因於跟隨狀態機。
- DIRECT 遇前方障礙仍立即完整零 Twist；人物遠離且受阻持續後交給 Nav2。NAV2 改為由純邏輯 `NavigationGuard` 篩選既有規劃／控制命令：前方不超過 0.65 m 時強制 v=0，但可在旋轉淨空確認後放行 w，朝規劃路徑轉向。前方清空後恢復前進。不自行產生盲轉、倒車或逃離人物的速度。
- 旋轉淨空要求完整 360° 雷射、有效範圍與 TF；以 scan←base_link 的偏移，逐射線檢查 base_link 周圍旋轉圓盤的遠交點。0.60 × 0.50 m footprint 外接半徑約 0.391 m，預設圓盤半徑為 max(0.391, robot_half_width=0.4)+0.05=0.45 m，側方及後方均須清空。空／部分／過疏掃描、NaN／負值／低於最小量程、量程不足、圓盤內遮擋與 TF 缺失均不放行旋轉。平面雷射不能證明所有高度皆無障礙，適用本專案既有模擬場景。
- 保留前方停止值 0.65 m：量測原點是前移 0.18 m 的雷射，正前方對應障礙距 base_link 約 0.83 m、距車體前緣約 0.53 m；0.607 m 回波對應前緣約 0.487 m，仍有空間原地轉動。1.6 m 走道中心至側架約 0.8 m，0.45 m 旋轉圓盤可容納；不以降低障礙門檻換取通過。真實回波方向、雷射離散化、EKF 與貨架轉角仍須 Fortress 驗證。
- NAV2 命令年齡超過 0.2 s 即歸零；目標過近、遺失／UWB 過期、雷射失效、停用、無 map 轉換、非活動中導航或控制時間無效時雙軸停止，安全停止略過加速度斜坡。Nav2 轉送限制在原配置的 0.5 m/s、1.0 rad/s、0.4 m/s²、1.5 rad/s² 以內，禁止倒車；底層仲裁與急停仍可更嚴格限制。
- Nav2 速度固定從 `/cmd_vel/nav` 輸入，安全檢查後持續轉送 `/cmd_vel/follow`。FOLLOW 仲裁不再把原始 nav 當作 follow 逾時備援，避免安全節點中斷後繞過目標與障礙保護；GUIDE 仍使用 nav。follow launch 僅接受 `use_mux:=true`，移除該流程的舊共用主題模式，避免轉送回授與多個最終發布者。最終 `/cmd_vel` 維持只由 watchdog 發布。未增加硬體、裝置存取、依賴套件或其他移動來源。
- Nav2 目標成功不再直接切 DIRECT 或啟動取消／排空。人物仍遠離時按既有週期刷新保留 1.0 m 距離的終點；正常移動更新保持 preemption，接受等待時合併新位置。返回仍要求至少持有 1.5 s、人物 ≤1.3 m、視線角 ≤0.25 rad、加寬 LOS 確認、前方 >0.8 m，並持續 0.5 s；只在離開 NAV2 時取消。安全事件與失敗仍能立即退出，舊回呼不能改動目前世代。
- 人物唯一來源 `person_path.yaml` 恢復 **0.35 m/s**。巡航上限 0.5 m/s，比人物快 0.15 m/s；直接控制側向最低 0.5×0.4=0.2 m/s，比人物慢 0.15 m/s，持續 10 s 約多落後 1.5 m；Nav2 曲率最低 0.25 m/s 也比人物慢 0.10 m/s。保留安全限速，明確保留持續窄處限速時無法等速追人的失敗回歸；沒有再以慢人物速度掩蓋追蹤落後。README 與架構文件同步目前接線、終點、遲滯、速度與旋轉例外。

#### 測試、限制與下一步

- 新增純邏輯回歸：0.607 m 固定前方回波、人物以 0.35 m/s 遠離 120 s，確認受阻後只交接一次、Nav2 可旋轉、沒有取消循環；前方清空後能前進。另以固定貨架射線與單輪車積分建立閉迴路，初始約 0.607 m，確認依合成規劃轉向命令旋轉後前進，旋轉圓盤始終不碰貨架。此測試使用合成 Nav2 命令，不能替代真正的成本地圖與規劃器。
- 覆蓋偏移雷射的後方／側方旋轉範圍、無效／部分掃描、命令過期、速度與加速度上限、禁止倒車、成功目標刷新及返回遲滯；節點回呼測試驗證實際轉送、近停／遺失／失效時即使可旋轉且有導航命令仍全停、舊回呼隔離與取消排空。仲裁測試確認 follow 逾時後原始 nav 不能通過，而 GUIDE 維持可用。套件新測試不依賴儲存庫外檔；需整個 repo 的既有套件設定測試維持 docs 根目錄→`/repo`→skip。
- 沒有 ROS 環境，未執行 Docker、colcon 或 Fortress；未安裝套件或建立 venv，ROADMAP 未勾選。純邏輯通過不表示 180 秒追蹤已 PASS；未知旋轉淨空或實際困在障礙內仍保持停止，不保證任意幾何都可自主脫困。
- 下一步由維護者建置／colcon test，重跑 0.35 m/s 的相同 180 秒評估，保留完整 launch／1 Hz 診斷與最終 `/cmd_vel`。對照 `rotation_clear`、前方距離與實際 w：貨架前 v=0 時應可受控轉向，轉開後恢復 v；確認 near_stop 持續 PASS、目標失效／雷射過期／急停完全停止、Nav2 更新沒有反覆取消排空、watchdog 為唯一最終發布者。同步檢查人物 0.30 s 模型補償在較快速度下的誤差；完整路徑約 94.86 s，180 秒涵蓋超過一圈。若仍失敗，須區分真正規劃失敗、旋轉淨空拒絕與持續限速落後。

#### t32 自行檢查（非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：825 項通過，保留既有 xacro 棄用警告 1 項。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；17 個 YAML／RViz、11 個 XML／SDF／xacro、105 個 Python、4 個 Bash；兩／三錨點、ekf／ground_truth xacro 展開與錨點一致性通過。
- `git diff --check`：通過。以上未包含 ROS、colcon 或模擬驗收。

#### t32 審查驗證

| 項目 | 結果 |
| --- | --- |
| 0.607 m 貨架死鎖重現、旋轉圓盤與閉迴路測試複核 | 待驗證 |
| 近停、遺失／過期、雷射失效、命令逾時與急停 | 待驗證 |
| preemption、成功狀態刷新、返回遲滯與取消頻率 | 待驗證 |
| pytest 與 check_static、YAML／XML／launch／xacro 複核 | 待驗證 |
| Humble colcon build／test 與 FOLLOW 轉送接線 | 待驗證 |
| Fortress 0.35 m/s 人物同步與 180 秒直線／轉彎追蹤 | 待驗證 |
| 貨架前轉向恢復、真實成本地圖繞行與 near_stop PASS | 待驗證 |
| 唯一 cmd_vel 發布者、仲裁逾時拒絕原始 nav 與 TF 回歸 | 待驗證 |

## 2026-10-24：修正跟隨模式遮擋交接震盪

### 修改與動機

- 維護者提供的 t32 後無畫面 Gazebo／follow-eval 模擬（人物速度 0.35 m/s）顯示：直線平均距離 3.03 m、轉彎 3.47 m，相對 1.0 m 目標皆未通過；濾波延遲 0.08 s 與過近停止通過。直接跟隨約維持 1.15～1.3 m，主要落差出現在遮擋交接。
- 原紀錄約每次 NAV2 維持 5～10 秒後退回 DIRECT／BLOCKED_WAIT 3～4 秒，約重複 15 次；4 分鐘內取消目標 167 次，約 380 筆診斷中 54 筆為 BLOCKED_WAIT，估計閒置約 80 秒，距離增至 5～6.6 m。
- 純邏輯 supervisor 現在於 LOS 遮擋完成去抖後立即保留 NAV2 控制權，不再額外等待直接模式停留時間或交接延遲。一般僅前方障礙、LOS 尚未確認遮擋的情況保留原交接等待策略。
- 遮擋期間 Nav2 成功、中止或其他終止結果不再觸發 DIRECT／取消／排空循環；依既有 `goal_update_period`（預設及設定均為 0.5 秒）刷新人物前方停靠目標，包含已接近但仍被遮擋的人物。目標更新沿用直接送新目標搶占與等待接受時合併更新的機制。
- 地圖轉換缺失時維持 NAV2 控制權並停止轉送速度；伺服器不可用時明確回報 `Nav2 unavailable` 並限頻重試。首次目標尚未接受時也維持 NAV2，正常遮擋流程不再進入 BLOCKED_WAIT；沒有新增逾時後直行的途徑。
- 回到 DIRECT 仍須 LOS 持續暢通達 `los_clear_time`，並符合近距離（預設不超過 1.3 m）、朝向、前方淨空、最短導航停留及恢復等待條件；只在退出時取消一次，等終止與速度排空後第一筆直接命令歸零。保留過近、遺失／過期、雷射失效、停用及關閉保護。
- 節點明確區分 Nav2 ABORTED 與伺服器不可用；送出新目標時清除前一筆導航速度，等待新速度輸入。未修改 `/cmd_vel` 看門狗與主題重映射，未新增硬體連線、依賴、提交或推送；未勾選 ROADMAP。

### 自行檢查（執行紀錄，非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：832 個測試通過；有 1 筆既有 xacro 相依套件棄用警告。
- 新增不匯入 ROS 的 20 秒遮擋重播，分別每數秒回報成功或中止，驗證遮擋期間沒有 DIRECT、取消不超過一次、BLOCKED_WAIT 累積不超過 1 秒及刷新間隔至少 0.5 秒；恢復近距離暢通後恰取消一次。
- 以假訊息與 future 執行實際節點方法，涵蓋連續終止結果、不可用伺服器的限頻與零速度、無地圖轉換的控制權保留，以及返回直接跟隨時取消一次與第一筆零命令；更新舊測試中失敗退回 DIRECT 與遮擋近距離不刷新的預期。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS，含 17 個 YAML／RViz、11 個 XML／SDF／xacro、105 個 Python、4 個 Bash，以及兩／三錨點在 ekf／ground_truth 模式的 xacro 展開與設定一致性。
- `git diff --check`：通過。

### 限制與下一步

- 本環境沒有 ROS，未執行 Docker、colcon、Gazebo 或硬體；上述測試不代表實際 Nav2 動作傳輸、搶占與跟隨距離驗收通過。
- Nav2 伺服器或地圖轉換持續不可用、目標接受回呼失聯時仍保留導航控制權並停止，不以 DIRECT 追逐隱藏人物；等待接受的時間取決於外部服務，不保證此時能移動。
- 請維護者執行 Humble 容器建置／測試及相同人物速度的直線、轉彎與過近停止模擬，比對平均距離、模式停留、目標取消次數、速度過期與停用／關閉行為。開發基準維持純模擬。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 遮擋期間持續 NAV2、終止後限頻刷新與無取消排空震盪 | 待驗證 |
| BLOCKED_WAIT 上限、伺服器／地圖不可用時停止與診斷 | 待驗證 |
| 持續暢通且近距離才返回 DIRECT、取消一次與首筆零速度 | 待驗證 |
| 過近／遺失／過期／雷射失效／前方障礙／停用／關閉安全 | 待驗證 |
| 看門狗唯一發布 `/cmd_vel`、Humble colcon 建置與測試 | 待驗證 |
| Fortress 直線／轉彎跟隨距離、延遲與過近停止重測 | 待驗證 |
## 2026-10-24：Nav2 fork 與 TEB 整合

### 本次完成

- 沿用已加入的私人 `third_party/eurobot_nav2` submodule，沒有修改其原始碼、gitlink 或 `.gitmodules`。公開主專案只新增整合層、參數及指定的 BT 複本；Docker context 排除 third_party，映像不 clone、不需要 GitHub token。
- compute 新增唯讀 `/nav2_ws/src` 與 build／install／log 具名 volumes，入口 source apt → fork → 應用 workspace。新增唯一套件清單、`dev.sh nav2-build`／`nav2-status`、Release／關閉 fork 測試／可調低平行度，以及所選套件需要的原生 apt 編譯相依。顯示容器維持 apt RViz。
- guide／follow／navigation 接上 `nav2_stack:=auto|fork|apt` 與 ROS-free 選擇 helper，明確 fork 缺套件會報錯。apt 使用原本 RPP／YAML；已有 fork 時透過清理環境的子程序執行 apt 導航，避免只換參數卻仍用 fork binaries。
- 新增 fork YAML：Slow／Fast 的非全向 gentle TEB、預設 Loose 及 Precise／NonStopGoalChecker profiles；未變更原 apt YAML。BT 保留 selector／StopController／ResetRecovery／RecoveryNode，僅將 recovery 設為 GoalUpdated 或 Shrink→Wait，無 Escape。controller／behavior／velocity smoother 與內部 BT client 的速度路由維持模式仲裁／看門狗出口。
- 比較工具重用區域清單與 nav-check 的 ZoneNavigator，分別記錄 apt／fork 全部六區的 action 結果、模擬秒數、真值軌跡長度及終點平面誤差，保存 JSON 並列印並列表。拒絕缺失／倒退／有大缺口的真值，逾時等待取消確認；README 提供每輪重啟同一格網與起點的操作順序。
- 新增選擇、apt 環境隔離與重啟參數、BT／TEB／goal checker／速度出口、掛載／source 順序／套件範圍、合約前檢與比較量測測試。既有 AST launch 測試更新注入的 helper；主專案的測試與靜態掃描排除 third_party 自己維護的 ROS 整合測試與舊工具。ROADMAP 未勾選。

### 本輪執行紀錄（非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：630 項通過，既有 xacro Python 棄用警告 1 項。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；14 個 YAML／RViz、11 個 XML／SDF／xacro、85 個 Python、5 個 Bash，兩／三錨點及 ekf／ground_truth xacro 展開通過。
- `git diff --check`：通過。未執行 Docker、colcon、ROS／Gazebo、網路存取、安裝套件、建立虛擬環境、提交或推送。

### 限制與下一步

- **固定的 `31c3ef1` 尚缺平行工作中的 TEB 合約。** 新增建置前檢會拒絕缺少八個新參數名稱的版本，避免 `holonomic: false` 被忽略。維護者須先完成／審查 fork 的非全向與 gentle 實作，再更新主機 checkout／主專案固定版本並建置；本輪沒有自行變更 pin。名稱前檢不等於 C++ 行為驗收。
- **Shrink 尚有上游差異。** XML 提案 0.40 m 大於車體半對角線約 0.391 m，但 fork 忽略 shrink_to，inflation service 寫死 0.15 m 且 inscribed radius 為 0；同一絕對 service 名稱用於兩個 costmap。缺 rival／object layer services 會增加延遲；action goals 沒有同步到其 `/move_base_simple/goal`，costmap 索引缺範圍保護。這些必須由 fork 維護者修正／驗證，不能宣稱 recovery 已符合購物車的碰撞安全需求。
- apt AMCL／RPP／collision_monitor 等會與 overlaid util／msgs／costmap 混用。建置後執行 `ldd -r` 攔截缺 library／symbol，通過才記錄 build commit；仍須驗證 ABI layout、DDS 型別、lifecycle、action 與取消行為。失敗建置可能留下部分 install 檔，不能以 setup 存在當作驗收；再次建置會先移除成功 marker。
- 維護者下一步依 README 重建 compute、nav2-build、重建／測試應用 workspace，確認 `ros2 pkg prefix nav2_controller` 在 fork 場次解析至 `/nav2_ws/install/nav2_controller`，並執行兩種 stack 的 guide／AMCL、nav-check、比較、mapping／map-tour、loc-check、follow NAV2_HANDOFF 與故障注入。比較須獨占導航並從相同出生點重新開始，真值為平面位置，不含 yaw 誤差；action 成功不能取代終點誤差判斷。
- 本輪只完成 Python／配置與語法驗證；容器映像與原生 amd64／arm64 建置、Gazebo recovery、行人 gentle 避障及停止行為全部待維護者驗證，沒有加入任何硬體連線。


### 審查驗證結果（2026-10-24 15:10 臺北時間，盒內 Linux amd64 Docker）

| 項目 | 結果 |
| --- | --- |
| fork 固定版本 | submodule 改指 fork `shopping-car` 分支 `bf60ee0`（含 opt-in `shrink.safe_recovery`：數值半徑、0.40 m 下限、local／global 分開 service、optional layer 逾時、結束時還原）；`31c3ef1` 缺少的 TEB 合約已在此分支 |
| `nav2-build`（18 套件） | 約 145 s 完成；`check_nav2_linkage.py` 110 個目標 PASS（修正：被 fork 遮蔽的 apt 函式庫與未使用的 Route BT 節點不再誤報）；舊 colcon 不支援 `--allow-overriding` 時自動略過該旗標 |
| fork gtest | custom_controller（TEB）與 nav2_behaviors（含 test_shrink_utils）13/13 PASS |
| 盒內 pytest／`check_static.py` | 775 passed／PASS |
| 容器 colcon build + test（fork underlay） | 8 套件；624 tests，0 errors，0 failures，5 skipped |
| 審查修正 | (1) fork YAML 的 AMCL 段同步 develop 新的 likelihood_field 參數；(2) bt_navigator 預設 through-poses 樹引用 Spin 而 behavior_server 只有 Shrink／Wait，導致 lifecycle 啟動失敗 → 新增 `navigate_through_poses_shopping.xml`（僅 Shrink＋Wait）並設為預設；(3) 開啟 Shrink safe 模式並給兩個 costmap 各自的 inflation service 與 0.40 m 下限，BT `shrink_to` 改 0.45；(4) gentle TEB 的 `min_obstacle_dist`／`gentle_stop_dist` 0.45 → 0.15／0.12：clearance 已扣除外接圓半徑 0.391 m，原值要求 0.84 m 淨空，出生點即被判阻擋、1.6 m 走道無法通行；(5) launch 預設 `nav2_stack` 由 `auto` 改為 `apt`，避免 fork 建置後預設導航改用尚未調好的 TEB |
| fork guide（map_server，headless） | 兩個 lifecycle manager 全部 active；Shrink 實際執行並成功還原（optional rival／object layer 0.05 s 略過） |
| fork `nav-check --tolerance-check --max-final-error 0.35` | 5/6：未知區域、不可達目標、取消並停止、有效目標、BUSY 皆 PASS；**最終誤差 FAIL 0.604 m**（apt 為 0.108 m） |
| fork `loc-check` | AMCL 定位誤差 1–2 cm、航向 <3.6°（定位正常），但 snacks／produce／household 導航皆未抵達（`TEB: path blocked` 約 260 次）；超過 740 s 逾時中止 |
| 未驗證 | apt／fork 六區 `nav-compare`、mapping／map-tour、follow NAV2_HANDOFF 於 fork、Mac arm64、Gazebo GUI、實車（只做模擬） |

### 結論與下一步

- fork 整合（建置、連結、lifecycle、BT、Shrink safe 模式）可用，但 **gentle TEB 在 1.6 m 走道與貨架轉角仍常判定阻擋**：clearance 以外接圓（0.391 m）計算，0.60×0.50 m 矩形車體在走道中央只剩約 0.37 m。已交給 fork 任務 t23（opt-in 依朝向的矩形 footprint clearance＋恢復遲滯，Eurobot 預設不變）。完成後以 `nav2_stack:=fork` 重跑 nav-check、loc-check 與 nav-compare，通過再考慮把預設改回 `auto`。
| 項目 | 結果 |
| --- | --- |
| 盒內 pytest／`check_static.py` | 755 passed／PASS（rebase 至 develop 3346718 後） |
| 容器 colcon build + test | 624 tests，0 errors，0 failures，5 skipped。修正：套件測試在容器（`/ws/src`）找不到 repo 根目錄，37 項失敗 → 改為找不到 `docs/` 時使用 `/repo` |
| sim_ros `comm-sim` + `comm-check` | 10/10 PASS。修正：首次執行因 DDS discovery 未完成，隔離檢查誤判 FAIL；改為最多輪詢 10 秒並在失敗時記錄發布／訂閱者 |
| serial_loopback `comm-sim` + `comm-check` | 10/10 PASS |
| `comm-up`／`comm-down`（`microros/micro-ros-agent:humble`，UDP 8888） | 映像可取得，agent 啟動並顯示 running；**沒有** XRCE client 連線（模擬韌體不經 agent） |
| 未驗證 | 真正 micro-ROS client 經 agent 端到端、Mac arm64、guide／follow `mcu_comm:=true` 與 Gazebo 同時執行、實體 serial／MCU 韌體 |

## 2026-10-24：Linux rclc 客戶端與 micro-ROS agent 端到端驗收流程

### 本次修改

- 新增 `firmware/sim_client`，以 C／rclc、`rmw_microxrcedds` 實作 Linux 模擬客戶端，透過 UDP4 agent 發布 heartbeat、wheel 與 status，訂閱 command 與 PC heartbeat；沒有實體裝置、馬達或板卡 SDK。
- 純 C 模型與 ROS 邊界分離，包含三筆遞增心跳恢復、UInt32 回繞、命令 0.3 秒逾時、PC 心跳 0.5 秒逾時立即零速、拒絕非有限／非平面輸入、預設不倒車、一階輪速與加速度限制。由 `interface.yaml` 與 description xacro 產生契約 header，Docker 建置時重新產生。
- 新增基於 `ros:humble-ros-base-jammy` 的客戶端 Dockerfile，使用官方 `micro_ros_setup` humble 分支的 host 建立／建置流程，將獨立 ament C 套件複製到工具工作區 `src/uros`。新增 opt-in `comm` profile 客戶端服務，以 agent hostname 解析 IPv4，無 host port、devices、privileged 或固定架構。
- 新增 `comm-e2e-up`／`comm-e2e-check`；前者建置／啟動 agent、client 後，在前景執行 standalone `transport:=agent` 的既有 mux／watchdog／monitor，不啟動 Python 韌體。watchdog 維持唯一 `/cmd_vel` 發布者。`comm-down` 同時停止 client 與 agent。
- 新增 ROS 驗收器，保留最長 10 秒 DDS discovery 等待，檢查 graph 隔離、兩向 heartbeat、輪速頻率、teleop 經 mux 抵達、命令逾時及恢復、PC 心跳真正停止發布後的 LINK_LOST／零速，以及心跳恢復仍維持 ESTOP。執行中持續重查隔離；失敗後請求 ESTOP 並嘗試清除心跳故障。
- PC monitor 的心跳故障參數僅在 standalone agent 模式明確啟用模擬故障時生效，一般預設不啟用。新增純邏輯、實際 C 編譯執行、安全清理與 wiring 測試；repo 檔案測試依祖先 repo、`/repo`、skip 的順序處理 colcon 掛載位置。
- 更新 README、micro-ROS 入門、MCU 介面子集與客戶端說明，補齊 rosserial 對照、STM32 移植責任、啟停及維護者驗收指令。未勾選 ROADMAP，未提交或推送。

### 自行檢查（執行紀錄，非審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：764 項通過；1 筆既有 xacro／Python 棄用警告。測試包含以系統 C 編譯器實際建置／執行純模型，未編譯需要 ROS 的 `main.c`。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；YAML、XML／SDF／xacro、Python、Bash 與兩／三錨點、ekf／ground_truth 展開一致性檢查。
- `git diff --check`：通過。`dev.sh` 分派測試使用暫存假 Docker 指令記錄參數，沒有呼叫 Docker。

### 限制與下一步

- 本環境沒有 ROS，未執行 Docker、colcon、XRCE session 或 DDS rate 驗收；沒有安裝套件或建立虛擬環境。Docker 映像、Humble C API 完整編譯、agent 連線／重啟與兩種原生架構仍須維護者實測，靜態與純邏輯檢查不可視為通過容器驗收。
- Linux client 是有限子集，沒有 encoder、IMU、MCU 按鈕急停／reset；現有 Python 完整模擬仍保留。只支援獨立壁鐘時間，不接 Gazebo／EKF、不操作硬體，不能作為完整板端安全韌體。
- Linux 失敗重試依 Compose 程序重啟，重啟清除舊命令；STM32 仍需 RTOS transport、時鐘、記憶體與獨立安全控制／重連設計。heartbeat 故障若遇強制 kill，需重啟前景 launch 清除殘留參數。
- 下一步：依 `docs/MICROROS_GUIDE.md` 在容器執行 colcon build/test、`comm-e2e-up` 與 `comm-e2e-check`，回報完整建置、agent／client 日誌及七項驗收結果；再於 amd64／arm64 原生主機驗收。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 契約 header 與純 C 安全模型 | 待驗證 |
| pytest、靜態檢查及 colcon 掛載路徑 | 待驗證 |
| Humble host 映像與 C／rclc 建置 | 待驗證 |
| agent／client XRCE session、DDS 頻率及重連 | 待驗證 |
| mux／watchdog 唯一速度發布與 graph 拒絕注入 | 待驗證 |
| 命令逾時、PC heartbeat 失聯及 ESTOP 恢復 | 待驗證 |
| amd64／arm64 原生執行及無裝置預設 | 待驗證 |

## 2026-10-24：t24 依角色拆分模擬容器

### 本次修改

- 新增 `split` profile 的 sim、localization、navigation、app，以及 `record` profile；保留預設 compute／display 與既有 comm agent／XRCE 客戶端服務名稱。角色與工具容器透過 YAML anchors 共用原生架構運算映像、工作區與 Nav2 underlay 卷，補齊 rosbag2 工具依賴。
- 所有服務共用 compute 網路命名空間與相同 domain；DDS 參與者共用 UDPv4-only profile，XRCE 客戶端改以迴送位址連 agent。沒有新增 devices、privileged 或固定 platform；serial 範本額外要求明確設定 `MICROROS_SERIAL_DEVICE`，仍不掛載裝置。
- 新增 bringup 的四個角色入口與共用角色 launch，透過既有 guide／follow／Gazebo／navigation launch 的預設相容開關重用節點定義。sim 保留模擬感測與 MCU；EKF 與 SLAM／AMCL 移至 localization；Nav2 獨立；app 保留唯一 watchdog 與 mux。拆分固定模擬時間與 mux，維持 TF 唯一來源、apt／fork／auto 選擇及 apt 隔離重新啟動的角色參數。
- 新增 `split-up [guide|follow] [ARGS...]`、`split-down`、`split-status`、`split-logs SERVICE`；啟動前重建同一專案的服務，避免舊單容器 launch 殘留。參數不經 eval，支援空白並拒絕換行及未開放參數。Gazebo 健康檢查使用有逾時的 `ign topic -l`，確認世界時鐘主題後才啟動其他角色。
- 新增離線測試，涵蓋 Compose profiles／映像／DDS／無裝置限制、十二組模式與里程計來源的節點歸屬、定位與導航 include 開關、apt 重新啟動參數、命令派送、字面參數與健康檢查。同步調整既有 XRCE 網路契約測試；套件測試仍保留 repo root／`/repo`／skip 的既有處理。
- 新增 `docs/CONTAINERS.md` 的角色表、主題圖、Mac／Linux 操作、DDS 排錯、選用 profiles 與維護者驗收命令；更新 README、架構與 micro-ROS 網路／serial 說明。未勾選 ROADMAP，未提交或推送。

### 自行檢查紀錄（不代替審查驗證）

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：803 passed、1 skipped；另有既有 xacro DeprecationWarning。跳過項目為未初始化的私有 Nav2 fork 原始碼契約檢查。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；18 個 YAML／RViz、14 個 XML／SDF／xacro、120 個 Python、7 個 Bash，並完成兩／三錨點及 ekf／ground_truth xacro 展開與錨點設定比對。
- `git diff --check`：通過。命令派送測試使用假 Docker 執行檔，沒有執行 Docker、安裝套件或建立虛擬環境。

### 限制與下一步

- 本環境沒有 ROS，未執行 Compose 實際解析、映像建置、colcon、DDS、Gazebo、Nav2 或硬體驗收；靜態結果不代表容器整合通過。維護者請依 `docs/CONTAINERS.md` 的完整指令建置、執行 `split-up guide headless:=true`、對拆分堆疊執行 `nav-check`，最後 `split-down`，回報錯誤後再修正。
- Gazebo 健康僅代表 transport 已列出世界時鐘主題，不保證 ROS 時鐘、模型、感測器、TF、定位與 Nav2 lifecycle 已就緒。app 延用既有 steady timer 重試及零速等待機制；連線異常觸發的 ESTOP 仍須明確解除。
- XRCE C 模擬客戶端保留既有壁鐘 wheel telemetry，獨立 comm-e2e 流程仍使用壁鐘；不當作 Gazebo EKF 輸入。預設 split 使用 Python 模擬韌體；替換為 agent 的時間戳與整合尚待驗收，不能同時開啟兩個韌體發布者。
- `split-up`／`split-down` 會停止同一專案的選用通訊與錄製工作；先正常結束錄製，再切換模式。Mac 日常使用預設單容器；拆分驗收先保持 headless 並不啟用額外 profiles。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 離線 pytest、靜態語法與模型檢查 | 待驗證 |
| Mac arm64／Linux 原生映像、Compose 與 colcon | 待驗證 |
| split 啟動順序、Gazebo transport、ROS 時鐘與 DDS | 待驗證 |
| EKF／ground_truth、SLAM／AMCL TF 唯一來源 | 待驗證 |
| guide／follow、Nav2 apt／fork 與晚啟動容忍 | 待驗證 |
| watchdog 唯一發布者、急停與失聯零速 | 待驗證 |
| 單容器與 micro-ROS e2e 相容性 | 待驗證 |
| noVNC／RViz、record 與停止清理 | 待驗證 |

### 審查追加（2026-10-24 15:50 臺北時間）

- t23（fork 分支 `codex/teb-footprint-clearance` `80ef04f`）：opt-in `gentle_footprint_clearance`（依朝向矩形 footprint 計算淨空）與 `gentle_resume_margin`；容器 `nav2-build` 與 linkage PASS，custom_controller gtest 18/18 PASS。
- 以 `nav2_stack:=fork` 加上兩個參數重跑：nav-check 基本五項 PASS；loc-check 機器人停在 (0.11, 4.28) **完全不動**，drinks／snacks／produce 皆失敗（約 6 分鐘 151 次 `path blocked`）。footprint 淨空不是主因，已交給 fork 任務 t25 做根因分析（地圖邊界項、低速前向模擬時間、遲滯鎖死等假說）並加入阻擋原因診斷 log。
- 因此本 PR 只合併 fork 整合層，submodule 仍固定 `bf60ee0`，**預設 `nav2_stack:=apt` 不變**；TEB 修正驗證通過後另開 PR 更新固定版本與參數。
- 修正分支上 PROGRESS.md 誤刪 develop 既有段落（偏航率／AMCL 調查、micro-ROS 通訊層）的合併錯誤，已還原。


## 2026-10-24：t33 Shrink 退出還原與 fork 規劃診斷

### 修改與調查

- 已讀取兩個工作樹與 t25s 四份指定日誌。`(50,50)` 超出成本圖，安全 Shrink 的
  目標 cost 為無限大，判定失敗是預期結果；日誌沒有必要服務逾時或還原失敗訊息。
  optional rival／object 的兩輪訊息符合縮小與還原皆有呼叫，不能由 `shrink failed`
  推論全域成本圖未還原。有效 household 與 drinks 的失敗在全域規劃階段，未進入 TEB。
- 更正紀錄：提供的 `t25s-check2.txt` 中 household **成功**，map 目標誤差約 0.128 m；
  drinks 約 7 秒失敗，snacks／produce／checkout／entrance 皆成功。
- fork 程式僅修改 `/workspace/wt/navfork-t23`：新增 action 退出保護，安全 Shrink 在
  action 結果送出前還原，初始失敗、例外、shutdown 亦走還原；新增有限服務等待與
  還原重試、各服務 ACK、robot／goal cost 與還原結果診斷。相容模式不啟用退出 hook。
  新增無 ROS 的退出保護／dirty state 重試 gtest；實際 action 與服務仍需整合測試。
- 購物車 `nav2_params_fork.yaml`：必要服務探索及回覆等待各 3 秒、每次還原最多 2 輪；
  optional 仍為 0.05 秒。`costmap_tolerance` 從 50 改為 98，符合 OccupancyGrid 的
  可通行成本範圍，仍拒絕內切障礙 99、致命障礙 100、未知與超界格。
- apt／fork 的 NavFn YAML、footprint、正常 inflation radius／scaling 原本相同，但
  fork 額外指定 `inscribed_radius_floor=0.40`，成本圖不等同 apt 依 footprint 計算的
  約 0.26 m。本次保留保守下限。BT 的 Shrink／Wait 與 apt recovery 亦不同。
- fork NavFn 成本轉換與 Dijkstra 優先門檻不同於 apt；原搜尋預算可能提前耗盡。
  新增預設關閉的 `GridBased.dijkstra_full_expansion`，購物車明確啟用，將預算從
  `max(nx*ny/20,nx+ny)` 改為 `nx*ny`，到達端點／佇列清空仍提前結束。
  新增高成本蛇形走廊與致命牆 gtest，以及搜尋耗盡、goal cost／potential 診斷。
  **舊日誌沒有當時成本圖與搜尋統計，尚未證實搜尋預算就是本次規劃失敗根因。**
- Slow／Fast 的 `name_ + ".gentle_*"` 讀取路徑正確。原阻擋診斷是 WARN，沒有被
  INFO 靜音巨集移除，但必須進入控制器且判定阻擋才會出現；現在改為實際啟用的
  1 Hz INFO，並在 gentle configure 列出兩個 plugin 的安全參數。新增 pytest 防止
  Slow／Fast 的 t25 選項遺漏。
- fork 已更新 `docs/shopping_car_teb.md` 與 Shrink 說明。未修改 submodule 內容或
  固定版本，未變更 `/cmd_vel` 發布鏈、硬體選項或 ROADMAP 勾選。

### 自行檢查、限制與下一步

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：776 passed，1 個既有
  xacro DeprecationWarning。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS，包含
  YAML、XML／SDF／xacro、Python／Bash、兩／三錨點與兩種里程計模式的 xacro 展開。
- 兩個工作樹執行 `git diff --check`。沒有 ROS，因此未編譯 C++、執行 gtest、
  colcon、容器或 Gazebo；未安裝套件、建立 venv、提交、推送或操作真實硬體。
- 維護者需將 fork 變更形成版本並同步 submodule、完整重建 NavFn／behavior／TEB，
  跑 `test_shrink_utils`、`test_dijkstra_budget` 與既有 TEB gtests，再以單一 Gazebo
  重跑不可達目標 → household → drinks、nav-check 與六區 loc-check。
  優先收集兩個 plugin 的 configure INFO、Shrink 還原至 0.8 m 的 ACK、NavFn
  耗盡／goal cost 診斷；若非耗盡，需比對失敗當下 global costmap 與內切下限。
- 服務斷線或 ROS shutdown 時只能保證嘗試還原，不能保證遠端已套用；同步服務等待
  中收到取消需等待本次有限等待返回，多個服務及重試的總時間可能大於 3 秒。

| 審查驗證項目 | 狀態 |
| --- | --- |
| Humble C++ 編譯、新舊 gtest 與 lint | 待驗證 |
| Shrink 部分失敗／逾時／取消／清理的雙成本圖還原 | 待驗證 |
| Slow／Fast configure 與 1 Hz INFO | 待驗證 |
| 單一 Gazebo nav-check 與六區 loc-check | 待驗證 |
| 全域規劃根因與 apt／fork 成本圖比對 | 待驗證 |

## 2026-10-24：fork inscribed_radius_floor 0.25 模擬複驗（t36s，17:35 台北時間）

- 設定：`nav2_stack:=fork`，local／global costmap `inscribed_radius_floor: 0.25`（前次 0.40），symlink-install 已確認生效。
- nav-check（`--tolerance-check --max-final-error 0.35 --case-timeout 180`）：unknown zone、unreachable goal、busy policy PASS；cancel and stop「no navigation progress」、valid goal 與 zone tolerance 到 household (0, 4.5) ABORTED → **FAIL**。
- loc-check：snacks、produce 導航成功並 PASS（前次 0.40 時 drinks 起即失敗）；drinks 仍 navigation_success=false，household 未通過。
- planner log 仍為「NavFn has no reachable goal within tolerance 0.80 … goal_cost=253」（goal_cell (118,151)／(70,101)），代表 household／drinks 目標附近仍被 fork 的膨脹層視為致命；另有一次 worldToMap 超界（目標座標在 costmap 外）。
- 結論：0.25 改善部分走道，但 fork 仍未達 apt 的 nav-check 5/5；**不合併**，預設維持 `nav2_stack:=apt`。下一步：比對 fork InflationLayer 與 apt 在 household／drinks 目標格的 cost，檢查 min_inflation_radius≥0.4 與 cost_scaling 是否把目標格推到 253，以及 TEB 控制器在 cancel-stop 不出發的原因。

## 2026-10-24：fork 膨脹成本離線核對（待模擬驗證）

- 讀取子模組 `src/Navigation2/nav2_costmap_2d/plugins/inflation_layer.cpp` 與
  `include/nav2_costmap_2d/inflation_layer.hpp`：初始化及 footprint 更新皆直接令
  `inscribed_radius_ = inscribed_radius_floor`，**不採用 footprint 的內切半徑**。
  正常膨脹半徑為 `max(inflation_radius, min_inflation_radius, inscribed_radius_floor)`；
  模式服務縮小時將第一項換成 `shrink_to`，數值服務則換成請求半徑，還原時取回
  YAML 的 `inflation_radius`。目前正常為 0.80 m、縮小為 0.45 m，最小範圍仍為 0.40 m。
- 令 `r = inscribed_radius_floor`、`d` 為格中心到障礙格中心的距離：障礙格成本 254；
  `0 < d <= r` 時為 253；其他受膨脹格為 `floor(252 * exp(-1.5 * (d-r)))`。
  傳播只到 `ceil(膨脹半徑 / resolution)` 格；`min_inflation_radius` 與 `shrink_to`
  只限制傳播範圍，不會擴大 253 平臺。合併成本時取原格與膨脹成本的最大值，
  因此既有致命障礙亦不會因調低 floor 消失。這個 0.40 m 下限是膨脹範圍，
  **不是保證車體與障礙相距 0.40 m 的硬性淨空**。
- 新增 `tools/nav2_inflation_probe.py`，沿用既有 PGM 讀取與 trinary 門檻，將影像
  Y 軸反轉為成本圖索引。執行 `/workspace/scratch/venv-shop/bin/python tools/nav2_inflation_probe.py`
  可重現下表（原始成本圖數值，非 OccupancyGrid 的 0–100）：

| 目標 | 日誌格索引 | 最近佔用格 | 距離 | fork floor=0.25 | fork floor=0.40 | apt |
| --- | --- | --- | --- | --- | --- | --- |
| household | (118,151) | (103,151) | 0.75 m | 119 | 149 | 120 |
| drinks | (70,101) | (54,101) | 0.80 m | 110 | 138 | 112 |

- apt 的 0.60×0.50 m footprint 加上預設 `footprint_padding=0.01`，內切半徑為
  0.26 m；fork 的 0.25 不含此 padding，故成本略低。指定地圖的 origin 為
  (-5.92,-3.12)、解析度 0.05 m；NavFn 的四捨五入換算實際得到 (118,152) 與
  (70,102)，與日誌差一列。腳本也計算這兩格，距離與成本均相同。
- **目前原始碼、地圖與 YAML 無法重現目標成本 253，故不套用沒有證據的參數修正，
  `nav2_params_fork.yaml` 保持原值。** 現有設定已使兩格小於 253，連舊 floor=0.40
  亦如此。新增 pytest 鎖定數值、最小範圍與 253 平臺的差異，取代原先僅檢查
  floor 大小區間的斷言。縮小後這兩格皆在傳播範圍外，靜態膨脹貢獻為 0。
- 日誌的 `goal_cost` 取自 NavFn `costarr`，不是原始成本圖；fork `setCostmap`
  先判斷 `v >= 253`，將 253／254／255 全映為 253，連 `allow_unknown=true`
  的未知格分支也無法到達。因此不能由該日誌認定是 InflationLayer 的 INSCRIBED。
  若實測確認原值為 255，候選程式修正是先處理未知格再處理障礙門檻，讓
  `allow_unknown` 生效；尚無證據證明本次就是未知格，依要求僅記錄、不修改子模組。
- 自行檢查：指定 pytest 為 **778 passed**，1 個既有 xacro DeprecationWarning；
  指定 `scripts/check_static.py` PASS（18 YAML/RViz、13 XML/SDF/xacro、110 Python、
  5 Bash，含兩／三錨點及 ekf／ground_truth 的 xacro 展開）。未執行 git 指令、
  Docker、ROS 或 Gazebo；未修改子模組，這些檢查不代表模擬驗收。
- 下一步：在同一次失敗中擷取載入的地圖 origin／尺寸／解析度、兩層 InflationLayer
  執行期參數、global costmap 原始 0–255 資料及目標附近 scan 障礙，核對執行中
  函式庫與目前原始碼是否一致，再區分未知格、雷射標記、地圖或參數／建置版本差異。
  cancel-and-stop 未出發可能是規劃失敗的下游結果，仍須追蹤該次 action 與控制器日誌；
  目前不能宣稱已修復。維持單一移動流程，重跑 household／drinks、nav-check 與六區測試。

| 審查驗證項目 | 狀態 |
| --- | --- |
| 公式、地圖數值與離線測試審查 | 待驗證 |
| 執行期原始成本、參數與地圖差異定位 | 待驗證 |
| Gazebo 目標導航與取消停止（待模擬驗證） | 待驗證 |

- 補充（17:46 台北時間，審查）：t36s 的 global costmap StaticLayer 每 2 秒重新調整尺寸（236×199 → 237×200），代表 guide 使用即時 SLAM 地圖，household／drinks 附近在探索前多為未知格（255）。這與上方推論一致：fork NavFn `setCostmap` 先以 `v >= 253` 判為障礙，使未知格無法依 `allow_unknown` 通行，而 apt NavFn 可穿越未知格。下一步優先在 fork 修正 NavFn 未知格順序（先判 255 再判障礙門檻），重建 fork 後重跑 nav-check／loc-check。
## 2026-10-24：硬體介面設計、預設關閉入口與 UWB 訊息

### 修改內容

- 新增 [HARDWARE_INTERFACES.md](HARDWARE_INTERFACES.md)，整理雷射、IMU、UWB、MCU 輪狀態／encoder、輪式里程計、命令、急停、心跳的 topic、型別、frame、頻率、QoS、單位與模擬／實車責任；說明安裝位置、covariance、TF 唯一來源、時鐘同步、高度校正、有效性及維護者驗證指令。ARCHITECTURE 加入入口連結，ROADMAP 未修改或勾選。
- 新增 `hardware.launch.py`，四個 `use_real_lidar/imu/uwb/base` 選項皆預設 false。每項 true 只引入對應 placeholder，未明確給定 driver package 與 launch 時記錄錯誤，不啟動裝置、不探測、不自動回退。未來 launch 檔只接受指定套件 launch 目錄的單一檔名，並傳遞 use_sim_time。
- 將原模擬 bridge 契約分為 common／lidar／imu／base 四檔，選用真實來源時不建立對應 bridge；輪速噪聲節點分別關閉 wheel／IMU 的 publisher 與 subscription。選用真實 UWB 時不建立舊模擬及估測節點。硬體入口固定 EKF、關閉模擬故障、啟動待機 mux 與唯一 watchdog，沒有導航、跟隨或 teleop 移動來源；真實來源不得搭配 ground_truth 或故障發布者。
- 新增獨立 `shopping_interfaces` ament_cmake 套件，提供 UwbRange、UwbRangeArray、UwbAnchor、UwbAnchorArray；含量測 stamp、錨點／標籤 ID、距離、不確定度、品質、有效性原因、NLOS 與未知狀態、三維位置及校正高度。bringup 宣告介面套件依賴，compute Dockerfile 明列 rosidl 產生器供維護者重建；本次沒有安裝套件。
- 保留原 UWB ROS 訊息與估測流程，新增無 ROS 匯入的 dict adapter 與 pytest，檢查 ID 對應、NaN／Infinity／非正距離、stamp、未知 metadata、不確定度與 msg 契約一致性。adapter 僅轉接既有平面距離，不能宣稱是實體斜距或已完成時鐘同步。
- 擴充 `check_static.py`，驗證 msg 語法子集、欄位重複、型別引用、CMake／package.xml／rosidl 相依、映像建置依賴與 bridge 分檔完整性。launch AST 測試涵蓋四項選項的全部 16 組組合、預設值、轉送、placeholder 缺設定／不存在套件與檔案、明確 include、EKF／故障互斥及模擬處理的 channel guard。

### 本次自行檢查

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：794 passed，保留既有 xacro 棄用警告 1 項。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；4 個 msg、21 個 YAML／RViz、12 個 XML／SDF／xacro、110 個 Python、4 個 Bash，含兩／三錨點與 ekf／ground_truth 的 xacro 展開及錨點位置一致性。
- `git diff --check`：通過。未執行 Docker、colcon、ROS 或裝置連線，未安裝套件、建立虛擬環境、提交或推送。新增套件測試若讀取 repo 契約，依 docs 存在與否選擇原始碼根目錄、再退回 `/repo`，都不存在才 skip。

### 限制與下一步

- 沙盒沒有 ROS；以上為純 Python 與靜態檢查，不能代替 Humble rosidl／colcon 建置、DDS QoS、Gazebo 執行、TF 發布者數量或硬體驗收。維護者須依 HARDWARE_INTERFACES 的指令重建 compute 映像、colcon build／test，再驗證預設與缺驅動入口，以及 guide／follow 的模擬回歸。
- 入口仍是含 Gazebo 的介面測試，不是完整無模擬實車 bringup；未提供雷射、IMU、UWB、底盤驅動、輪式里程計轉接或電氣急停。混合測試的 Gazebo 車體與實車不會同步，模擬真值不得作為實車 EKF 漂移基準。
- guide／follow 暫不轉送硬體選項；待新 UWB 時間／高度感知估測器與實車輪速轉接完成後，再整合移動流程。新 UWB topic 本次僅定義，未有 ROS 發布者，原流程仍使用 Float32MultiArray。
- 後續須實作並測試模組時鐘 rollover／重啟與 offset／drift、量測同步不確定度、標籤與錨點高度校正、斜距轉平面距離及 covariance 傳播，再驗證兩錨點歧義、貨架 NLOS、掉訊恢復與保護停止。未來硬體選用仍須明確設定，不能靠裝置偵測自動啟用。

### 審查驗證

以下由審查者執行與填寫，自行檢查不代填審查結果。

| 項目 | 結果 |
| --- | --- |
| 硬體介面、MCU 契約、QoS、單位與 TF 所有權 | 待驗證 |
| 四項預設關閉選項、16 組切換與模擬來源互斥 | 待驗證 |
| placeholder 明確配對、無裝置探測與缺驅動缺資料行為 | 待驗證 |
| UWB msg／純 adapter、有效性、時鐘與高度遷移設計 | 待驗證 |
| check_static、xacro、YAML／XML／launch 與相依檢查 | 待驗證 |
| Humble 原生 amd64／arm64 映像及 colcon build／test | 待驗證 |
| Fortress 預設／缺驅動入口、publisher 數與唯一 cmd_vel watchdog | 待驗證 |
| guide／follow 純模擬回歸與 use_sim_time 一致性 | 待驗證 |
| 未來實車驅動、輪速轉接、時間同步、安裝校正與電氣急停 | 待驗證 |


## 2026-10-24：t28 合併 Nav2 fork 與硬體介面

### 修改內容

- 解決 ARCHITECTURE、PROGRESS 與 check_static 的合併衝突；保留兩側全部進度內容，依時序排列 Nav2 fork 整合及審查追加、硬體介面紀錄，再附加本次紀錄。逐行比對合併雙方的歷史內容，確認沒有遺失。
- check_static 同時保留四個 rosidl 訊息／相依圖／bridge 分檔檢查，以及 third_party 排除規則；README 補上硬體入口與 shopping_interfaces，ARCHITECTURE 保留兩項功能並明列 nav2_stack 的 apt 預設值。
- 自行檢查四個 placeholder：模組頂層僅匯入與定義函式，不開啟裝置；hardware 入口四項實體來源選項全部預設 false，placeholder 套件與 launch 預設空白，沒有裝置偵測。既有純 Python launch AST 測試涵蓋四個入口與 16 組來源互斥組合；擴充四個入口的缺套件測試及預設不查找／不啟動驅動測試。入口維持待機 mux 與唯一 cmd_vel watchdog。
- 保留 Dockerfile.compute 的 rosidl 產生器及 Nav2 原生編譯相依，確認 compose 的唯讀 /nav2_ws/src 與產物 volumes、nav2-build 的 apt 起始環境及入口 apt → fork → 應用層載入順序。未自行修改 ROADMAP 或勾選項目。

### 本次自行檢查

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：820 passed、1 skipped、1 warning。略過項目需要尚未初始化的私人 Nav2 submodule；警告為既有 xacro Python 棄用警告。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；4 個 rosidl msg、22 個 YAML／RViz、14 個 XML／SDF／xacro、117 個 Python、5 個 Bash，含兩／三錨點及 ekf／ground_truth 的 xacro 展開。
- `git diff --check` 與 `git diff --cached --check` 通過；未提交或推送，未執行 Docker、安裝套件或建立虛擬環境。嘗試 `git add` 時，實際 Git 目錄 `/workspace/repos/Almighty-shopping-car/.git/worktrees/shop-hw-interfaces` 為沙盒唯讀路徑，無法建立 `index.lock`，因此未能暫存；工作檔已移除衝突標記，但索引仍保留未解決狀態。

### 限制與下一步

- 本環境沒有 ROS；自行檢查僅涵蓋純 Python、設定與靜態契約，未執行容器映像建置、colcon、DDS、Gazebo 或實體裝置驗收，不能據此宣稱 Nav2 underlay 已成功建置。
- 維護者先在可寫入 Git 索引的環境執行 `git add docs/ARCHITECTURE.md docs/PROGRESS.md scripts/check_static.py README.md tests/test_hardware_launch.py` 完成暫存，再接續執行 Humble 容器重建、colcon build／test、Nav2 apt／fork 模擬回歸及硬體入口預設／缺驅動驗證。所有實體驅動仍須未來明確選用；目前沒有新增真實裝置連線。

## 2026-10-24：模擬情境與動態行人

### 修改

- 新增純 Python 情境載入器與七種 YAML：default、aisle_crossing、narrow_pass、checkout_queue、follow_turns、follow_stop_go、alt_start，驗證出生姿態、路徑、逐點停留與行人欄位。default 保留原 `config/person_path.yaml`，未複製或更動其內容。
- sim／mapping／guide／follow 新增 `scenario:=default`；每次 launch 只解析一份人物路徑，Gazebo 主要人物與 UWB 共用相同檔案，暫存世界與解析設定於關閉後清除。出生位置／朝向同步至 EKF 初始狀態、AMCL `/initialpose`、slam_toolbox 定位起點及 loc-check 的 scenario 參數。
- 新增 Fortress 原生 VelocityControl 圓柱行人、ROS `pedestrian_mover`、延遲／循環／速度上限純軌跡邏輯及真值過期停止。選用 Humble 已有的 Twist topic 橋接，避免依賴 set_pose 服務橋接修訂版差異；人物有 collision 與 visual，未使用 Classic 外掛或硬體。
- `/sim/pedestrians` 以 `sim_world` PoseArray 發布 Gazebo 回報姿態，納入錄製清單、有界內插及最小人物間距；缺樣數與半徑設定列於轉換中繼資料。既有跟隨目標與停止事件定義保留。
- 新增本機 `dev.sh scenario-list`、逐情境驗收程序與 README／ARCHITECTURE／METRICS 說明。擴充 schema 錯誤、原始路徑一致性、所有航點連線與出生點淨空／地圖邊界、軌跡停走／循環／延遲、launch 單次解析／參數、行人 watchdog、程式入口、SDF／Fortress 外掛及 bag 動態間距測試。check_static 納入情境 schema 與生成 SDF XML 檢查。
- 未修改 ROADMAP 勾選、Nav2 launch／參數或跟隨調校；未提交、推送、執行 Docker、安裝套件或建立虛擬環境。

### 本次執行驗證

| 指令／方法 | 執行觀察 |
| --- | --- |
| `/workspace/scratch/venv-shop/bin/python -m pytest -q` | 684 項通過；既有 xacro 棄用警告 1 項 |
| `/workspace/scratch/venv-shop/bin/python scripts/check_static.py` | PASS：21 YAML／RViz、10 XML／SDF／xacro、89 Python、4 Bash、7 情境，兩／三錨點及兩種里程計模式展開 |
| `./scripts/dev.sh scenario-list` | 列出七種情境與出生姿態、行人數，不啟動 Docker |
| `git diff --check` | 通過 |

### 限制與下一步

- 本環境未安裝 ROS，僅執行純邏輯、假 launch API 與靜態檢查。維護者依 `docs/SCENARIOS.md` 執行 colcon 與七種 headless 情境，核對外掛載入、橋接、AMCL 啟動時序、雷射／成本地圖可見性、實際相遇、讓行與恢復；本表不冒充容器、跨架構或硬體驗收。
- 行人依世界模擬秒開始移動，目標送出與啟動耗時會影響相遇；需記錄時刻及重複試驗。圓柱無避讓決策，物理接觸後可能偏離名義軌跡；強制終止 mover 時外掛無內建命令逾時，須結束整個模擬。主要人物仍用既有原生 set_pose 程序，解析人物真值與模型實際到位可能不同步。
- PoseArray 順序依 YAML，不包含半徑；內附情境皆為 0.25 公尺。自訂尺寸需用 bag_to_eval 的 `--pedestrian-radius` 指定保守最大值。缺 topic／缺樣不得判定動態安全通過；仍未實作連續碰撞監測、自動 scenario_check 執行器或純 Nav2 讓行事件偵測。
- eval-bag 與 loc-check 假設標準 map→world 為單位轉換，替代出生不是改變世界原點。自訂地圖需先核對座標，非單位轉換須手動轉換評估，loc-check 不支援自訂轉換。初始就停的 stop-go 不能代替行進煞停驗收，須另做既有 fault-check。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 合併衝突解決、雙方歷史完整性與文件一致性 | 待驗證 |
| pytest、check_static 與 xacro／YAML／XML／launch 契約 | 待驗證 |
| 四個 placeholder 預設關閉、明確選用與來源互斥 | 待驗證 |
| Dockerfile 原生相依、Nav2 underlay 建置與 colcon build／test | 待驗證 |
| Fortress apt／fork、硬體介面入口與唯一 cmd_vel watchdog | 待驗證 |

| 純 Python 測試與情境 schema／原始人物路徑一致性 | PASS：合併 develop 後 pytest 897 passed、1 skipped |
| 全情境路徑／行人／出生點淨空與地圖邊界 | PASS（pytest／check_static） |
| YAML／SDF／xacro／launch／Bash 靜態檢查 | PASS：check_static（7 情境） |
| Humble colcon build／test 與執行期依賴 | PASS：colcon test 647 tests、0 failures（t31s） |
| 七種情境 headless 啟動及 default 舊路徑重現 | 部分：aisle_crossing、alt_start、follow_stop_go、follow_turns 已啟動；narrow_pass、checkout_queue 未跑 |
| alt_start 的 EKF／AMCL／slam_toolbox／loc-check 初始姿態 | PASS：loc-check 6/6，平均誤差 0.04 m |
| Fortress 行人碰撞幾何、雷射與 Nav2 成本地圖可見性 | 待驗證 |
| 窄走道／結帳／穿越的實際相遇、讓行與恢復 | 部分：aisle_crossing nav-check 4/5，valid goal 到 household 120 秒內未收斂（ACTIVE）；窄走道、結帳未驗 |
| 急轉／迴轉／停走／過近完整零 Twist 與故障回歸 | 部分：follow_stop_go 追蹤 PASS（直線平均 0.95 m、轉彎 1.04 m）；follow_turns FAIL（平均 2.71 m，屬既有跟隨控制問題，非情境框架） |
| PoseArray 真值、錄製完整性、間距與 report 中繼資料 | 待驗證 |
| amd64／arm64 原生容器驗收 | 待驗證 |


## 2026-10-24：t31 情境分支合併 develop 衝突整合

### 修改內容

- 解決 README、ARCHITECTURE、PROGRESS、follow／guide launch、dev.sh 與模式 launch 測試的七處檔案衝突。保留雙方全部章節與 develop 歷史紀錄，依序接回情境紀錄，沒有刪除既有審查結果或勾選 ROADMAP。
- follow／guide 同時保留並轉送情境、Nav2 stack、故障注入、時間、里程計及模式參數；`nav2_stack` 預設 apt、`mcu_comm` 預設 false。保留人物位姿唯讀橋接，UWB 與 Gazebo 人物共用本次情境解析路徑。
- 修正無衝突標記的交互問題：fork underlay 存在而選 apt 時，導航會另開隔離程序，原先繼承的出生姿態會遺失。導航新增選用的 spawn 三參數並轉送，slam_toolbox 僅在三個值完整時覆寫定位起點；直接啟動未指定情境的定位仍保留原 dock 行為。
- 擴充 ROS-free launch 回歸測試，涵蓋兩種 bringup 與三種 stack、MCU 預設關閉／時鐘轉送、人物橋接／共用路徑、apt 隔離程序的出生姿態，以及有／無情境的 slam_toolbox 定位。
- 已檢視 `git diff origin/develop`，包含未衝突檔案的差異。develop 的容器、Nav2 apt／fork YAML、兩份 Shrink＋Wait BT、模式 mux、世界檔、MCU 與跟隨改進保留；額外差異為情境、出生姿態、動態行人、離線評估與測試整合。預設速度接線維持 mux → watchdog，未新增底盤 `/cmd_vel` 發布者，未更動 submodule 固定版本。

### 本次自行檢查

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：836 passed、1 skipped、1 個既有 xacro Python 棄用警告。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS；25 YAML／RViz、13 XML／SDF／xacro、116 Python、5 Bash、7 情境，包含兩／三錨點與 ekf／ground_truth 的 xacro 展開。
- `./scripts/dev.sh scenario-list`：七種情境列出成功，不啟動 Docker。`git diff --check` 通過；工作樹衝突標記已清除。另以 AST 比對兩邊 launch 參數，並比對雙方文件章節與 develop 關鍵設定。
- 未執行 git add、提交、推送、Docker、安裝套件或建立虛擬環境。index 仍保留 unmerged 狀態，需維護者確認工作樹後完成暫存與合併。

### 限制與下一步

- 此環境沒有 ROS；以上為純 Python 與靜態檢查，不能當作 Humble colcon、DDS、Gazebo 或硬體驗收。
- 維護者須執行容器 colcon build／test，再依 SCENARIOS 驗證七種 headless 情境；特別核對 alt_start 在 apt／fork、已有 fork underlay 的 apt 隔離程序、AMCL 與 slam_toolbox 定位起點，以及 EKF／ground_truth 的 TF 唯一來源。
- 核對人物位姿橋接、UWB 共用路徑、行人碰撞／雷射／成本地圖與錄製評估覆蓋；guide／follow 分開驗證預設 watchdog 為唯一 `/cmd_vel` 發布者、故障零速、NAV2_HANDOFF 與選用 `mcu_comm:=true`。fork 的既有 TEB 阻擋問題與人物模型延遲仍以原 develop 紀錄為準，本次合併未調校或宣稱修復。

### 審查驗證

以下由審查者執行與填寫，本次自行檢查不代填審查結果。

| 項目 | 結果 |
| --- | --- |
| 衝突解決、雙方文件歷史與 develop 行為保留 | 待驗證 |
| pytest、靜態檢查與 Humble colcon build／test | 待驗證 |
| 七種情境與預設情境回歸 | 待驗證 |
| apt／fork／auto 與情境參數共存、隔離 apt 出生姿態 | 待驗證 |
| alt_start 的 EKF／AMCL／slam_toolbox 與 TF 唯一來源 | 待驗證 |
| 人物橋接、共用路徑、動態行人與錄製評估 | 待驗證 |
| watchdog 唯一 cmd_vel 出口、急停／故障零速與跟隨交接 | 待驗證 |
| MCU 預設關閉、明確選用與 amd64／arm64 容器回歸 | 待驗證 |
## 2026-10-24：獨立障礙安全層、選用 collision_monitor 與故障觀測

### 修改內容

- 新增不匯入 ROS 的 obstacle_guard 幾何／判定模組與 ROS 節點：依要求速度檢查正向／反向矩形掃掠、曲線及原地旋轉外接圓，具速度相關停止／減速區、立即收緊及加寬／時間遲滯。整合啟動直接讀取已安裝 Nav2 的 local／global 足跡，拒絕不一致或不支援的形狀；pytest 與 description xacro 比對尺寸。
- 接線為 mux → guard → 選用 collision_monitor → watchdog → `/cmd_vel`；watchdog 仍為唯一最終速度發布者。guard 自行檢查 mux 逾時，雙重檢查 scan header／單調壁鐘、TF、完整掃描與無效讀值，20 Hz 壁鐘定時器使模擬暫停也會停車。最終 watchdog 的恢復加速度限制採共同比例保留曲率，安全減速／零速立即套用，不能超出上游限制。
- guide／follow／mode_mux 新增 obstacle_guard 預設 true、collision_monitor 預設 false。停用整合 launch 的 use_mux:=false 旁路，保留 Nav2 controller／BT 與 navigation.launch.py 原樣。collision_monitor 使用 Humble 的 points／max_points 參數、獨立 lifecycle manager；compute Dockerfile 宣告 apt 套件，未實際安裝或建置映像。
- `/safety/obstacle_state` 提供 CLEAR／SLOW／STOP／STALE、倍率、模擬時間及掃掠間距。follow_handoff 沿用既有交接監督器：STOP 可進入持續障礙判定，STALE 禁止繞行並歸零。加入 lidar_front 距離故障，沿用 lidar_drop／lidar_freeze；fault-check 增加 obstacle suite、移動基線／恢復要求與唯一 cmd_vel publisher 檢查。
- 錄製清單及 bag_converter 接上 guard STOP／STALE trigger，沿用 METRICS 的停止延遲與既有世界真值間距流程。新增 OBSTACLE_SAFETY，更新 README／ARCHITECTURE／METRICS。ROADMAP 未修改、未勾選。

### 本機自行檢查

使用指定既有環境執行 `/workspace/scratch/venv-shop/bin/python -m pytest -q`：781 項通過，既有 xacro 棄用警告 1 項。`/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS，涵蓋 18 個 YAML／RViz、11 個 XML／SDF／xacro、106 個 Python、4 個 Bash，以及兩／三錨點、ekf／ground_truth xacro 展開。`git diff --check` 通過。未安裝套件、建立 venv、執行 Docker、提交或推送。

### 限制與下一步

- 本次只有純 Python、回呼替身與靜態驗證，ROS／DDS／colcon／Gazebo／amd64／arm64 實際整合均未驗收。維護者須依 OBSTACLE_SAFETY 的命令重建容器並逐次執行 headless follow／guide／teleop，以及 collision_monitor 開／關。
- 等速掃掠與旋轉外接圓屬保守模型，尚未用真值辨識煞車能力或驗證停止距離；雷射平面外物體、漏測及真實滑移不在模型內。預設仍禁止倒車，倒車僅有純邏輯驗證。
- 減速試驗目前要求 SLOW 倍率與仍有移動命令，不等於真值減速度驗收。假障礙只有感測器回波，沒有 Gazebo collision；swept_clearance 不能冒充 METRICS 的真值最小足跡間距。時間以 guard 狀態與故障命令起點分別解讀，不能混合門檻。
- route_around=true 時的 STOP 可能隨上游停車要求重新評估，持續交接仍搭配原有 LOS 遲滯。維護者需在實際貨架情境驗證 NAV2_HANDOFF、繞行與返回 DIRECT；平行 Nav2 fork 的 TEB／Shrink+Wait recovery 尚未整合驗證，未新增另一套導航恢復策略。
- watchdog 程序、橋接或 Gazebo 本身故障的停止保證維持既有限制；本次沒有新增硬體控制或 Fortress 底盤命令逾時外掛。

### 審查驗證

以下由審查者執行與填寫，本次自行檢查不代填。

| 項目 | 結果 |
| --- | --- |
| 區域、速度、方向、曲線、旋轉、遲滯與過期 pytest | 待驗證 |
| 缺 TF、mux 失聯、雷射舊 header、時間倒退與恢復加速度 | 待驗證 |
| launch 四種 guard／collision_monitor 組合與唯一 cmd_vel 發布者 | 待驗證 |
| Humble colcon build／test 與 apt collision_monitor lifecycle／QoS | 待驗證 |
| headless follow 減速／停止／掉訊／凍結與解除恢復 | 待驗證 |
| guide／TELEOP 共用安全鏈路、急停／reset 與程序失聯 | 待驗證 |
| 原地旋轉淨空、側邊障礙與真值最小間距／停止距離 | 待驗證 |
| NAV2_HANDOFF 繞行、恢復 DIRECT 與平行 Nav2 fork 整合 | 待驗證 |
| bag 停止 trigger／真值延遲與 METRICS 報表 | 待驗證 |
| amd64／arm64 原生模擬回歸 | amd64 Gazebo 無頭模擬如上；arm64 未驗證 |

## 2026-10-24：t30 障礙安全層與 develop 合併衝突解決

### 修改內容

- 解決進行中 `git merge origin/develop` 的四個工作樹衝突：本文件、guide／follow launch 與 shopping_zone_manager/setup.py。guide／follow 同時保留並轉送 `nav2_stack:=apt|fork|auto`、obstacle_guard、collision_monitor 及雙方原有參數；預設 apt、guard 開啟、collision_monitor 關閉、mcu_comm 關閉，use_mux 仍只接受 true。
- setup 保留 obstacle_guard 與 nav_compare 及其餘命令入口；速度鏈路維持 mux → guard → 選用 collision_monitor → watchdog，watchdog 仍為整合流程唯一 `/cmd_vel` 發布者。跟隨對 `/safety/obstacle_state` 的訂閱及既有交接／失效保護均保留。
- PROGRESS 保留雙方完整紀錄，依同日 Nav2 整合／審查、障礙安全層、本次解衝順序排列。比對 HEAD 與 origin/develop，README／ARCHITECTURE／PROGRESS 的所有章節標題均保留；未刪除歷史或新增 ROADMAP 勾選。
- 逐一檢視 `git diff origin/develop` 的非衝突差異，均為障礙安全層及其文件／測試擴充；Nav2 navigation、參數、BT、stack helper、submodule 固定版本、micro-ROS 入口與容器設定保留 develop 內容，Dockerfile 僅額外宣告 collision_monitor 相依。
- 新增 24 組無 ROS 的 launch 回歸案例，涵蓋 guide／follow、apt／fork／auto、guard／collision_monitor 四種組合；檢查雙方參數宣告、預設值、安全旁路限制與 navigation／mode_mux／sim／comm 的轉送。

### 本機自行檢查

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：824 passed、1 skipped、1 warning。略過私人 submodule 未掛載時的來源驗證；警告為既有 xacro Python 棄用警告。
- `/workspace/scratch/venv-shop/bin/python scripts/check_static.py`：PASS，19 個 YAML／RViz、13 個 XML／SDF／xacro、113 個 Python、5 個 Bash；包含兩／三錨點、ekf／ground_truth xacro 展開與錨點一致性。
- 工作樹衝突標記掃描及 `git diff --check` 通過。未執行 git add、提交、推送、Docker、套件安裝或建立 venv；index 仍保留未合併狀態，由維護者檢視後標記已解決。

### 限制與下一步

- 本次只驗證純 Python、launch 替身與靜態配置；未執行 Humble、colcon、DDS、容器或 Gazebo，不能將上述檢查視為模擬驗收。
- 維護者須建置／測試應用 workspace，逐次驗證 apt／fork 的 guide／follow、guard／collision_monitor 四種組合、lifecycle／QoS／TF、唯一 `/cmd_vel` 發布者，以及急停、雷射掉訊／舊 header、程序失聯與恢復行為；fork 須依既有流程完成建置與連結檢查。
- follow 的 NAV2_HANDOFF、STOP 持續障礙繞行、STALE 取消與返回 DIRECT，以及 fork 的 Shrink＋Wait 和既有 TEB 阻擋問題仍待模擬驗證；保留前述審查限制，未新增硬體連線或更改預設 stack。

### 審查驗證

以下全部由審查者執行與填寫，本次自行檢查不代填。

| 項目 | 結果 |
| --- | --- |
| 雙方文件歷史與非衝突差異保留 | PASS（審查者 diff 檢視） |
| launch 參數宣告／轉送與純 Python 回歸 | PASS：pytest 824 passed、1 skipped |
| Humble colcon build／test 與靜態檢查 | PASS：Docker 映像建置、colcon test 640 tests、0 failures；check_static PASS |
| apt／fork guide／follow 與安全層四種組合 | 部分：apt guide＋guard nav-check 5/5 PASS；apt follow＋guard（fault_injection、route_around:=false）通過；fork 與 collision_monitor 組合未模擬 |
| 唯一 cmd_vel 發布者、急停／失聯／雷射過期停止 | PASS：fault-check --suite obstacle 4/4（lidar_front 0.2 m 停止 0.09 s、lidar_drop 0.37 s、lidar_freeze 0.71 s，門檻 cmd≤0.8 s／真值≤1.5 s）；--suite follow 7/7 |
| NAV2_HANDOFF、恢復 DIRECT 與 Shrink＋Wait 模擬整合 | 未驗證（route_around:=false；Shrink＋Wait 屬 fork） |
| amd64／arm64 原生模擬回歸 | amd64 Gazebo 無頭模擬如上；arm64 未驗證 |

審查時間：2026-10-24 16:30（台北時間）；結果紀錄於 Docker 專案 shopobs（t30s）。

## 2026-10-24：t35 撤回與跟隨模擬審查（17:10 台北時間）

- t35（遮蔽時持續由 Nav2 接手、不取消重送）在 Gazebo follow-eval 反而變差：直線平均距離 3.64 公尺、轉彎 4.17 公尺（t32 為 3.03／3.47，develop 基準 t18s 為 2.89／3.36，目標 1.0 公尺）。205 筆 1 Hz 診斷中 NAV2_HANDOFF 佔 205、其中 162 筆 `los_blocked=True`，Nav2 接手期間平均命令速度僅 0.10 m/s（人物 0.35 m/s），51 筆為「Forward blocked; rotation allowed」。因此以 `git revert` 撤回 t35 程式與測試，保留 t32 行為；本節上方 t35 紀錄僅作歷史參考。
- 推論：主要瓶頸是視線遮蔽判定過於保守（大量誤判遮蔽）與 Nav2 接手時速度過低，不是 BLOCKED_WAIT 本身；需另外調整 LOS 判定寬度／去彈跳與 Nav2 跟隨速度上限，再以 follow-eval 驗證。ROADMAP「評估濾波延遲、轉彎跟隨…」維持未勾選。
- 已驗證：filter latency 0.07–0.08 s PASS；near_stop PASS；pytest 893 passed。fault-check 結果見合併 PR 說明。

## 2026-10-24：可重現離線評估的錄製來源、重跑驗證與多次彙整

### 本次完成

- 錄製命令增加可重複的 `--meta KEY=VALUE` 與鍵名驗證；收尾時保存 UTC 起始時間、Git 提交／dirty、Nav2 子模組提交、topics、指定錄製秒數與使用者中繼資料至 bag 內的 `manifest.json`。Git 查詢失敗以 unknown 表示，不阻止錄製。
- 報告固定鍵排序與九位小數精度，CLI 參數先正規化再計算；新增純邏輯正規化與欄位差異比較，排除來源路徑及預留的獨立產生時間欄位。
- 新增 `eval_verify`／`dev.sh eval-verify NAME`，依保存的轉換設定與評估參數在暫存目錄重跑，回傳相同／差異狀態並列出差異欄位，不覆寫原報告。
- 新增純邏輯彙整與 `eval_summary`／`dev.sh eval-summary NAME... [--output DIR]`，輸出數值 count／mean／min／max／樣本標準差、既有 verdict 的通過／失敗次數及來源缺漏／版本與情境不一致警告。
- 補齊純邏輯、錄製收尾、命令列驗證、暫存清理及高精度參數重現測試；更新使用說明與 `docs/METRICS.md` 精確命令。未修改 bag_converter、ROADMAP 或控制流程，未提交或推送。

### 自行檢查

- 使用指定沙盒 Python 執行儲存庫根目錄 pytest：834 項通過、1 項略過，另有既有 xacro 相依套件的棄用警告。`scripts/check_static.py` 回報 PASS：19 份 YAML／RViz、13 份 XML／SDF／xacro、117 份 Python、5 份 Bash，並完成兩／三錨點在 ekf／ground_truth 的 xacro 展開。
- `git diff --check` 通過。未安裝套件、建立虛擬環境或執行 Docker。

### 已知限制與下一步

- 本環境沒有 ROS，尚未以 Humble sqlite3 真實 bag 驗證錄製與 SequentialReader；維護者需執行容器 colcon build/test 與模擬，驗證 console script、錄製收尾、重跑一致及差異回傳碼。
- manifest 的 duration 為指定上限而非實際經過時間；中繼資料由使用者填寫，dirty 旗標不保存修改內容。Git 不可用時只能標示 unknown。
- 驗證需原 bag、轉換中繼資料、世界檔與 zones 原路徑仍有效；不保證重新啟動 Gazebo 會產生相同資料。舊版未正規化參數的報告可能需先重新評估。
- 彙整警告不會禁止比較，仍需人工核對參數、資料覆蓋、回合順序與失敗事件；現有報告沒有 verdict 時不推論通過。下一步由維護者以同情境兩次錄製及刻意改動報告的案例驗收。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| manifest 欄位、Git 降級與 metadata 驗證 | PASS：模擬錄製 t34r1 產生 manifest（metadata scenario／nav2_stack 正確）；容器內 git 不可用時三欄降級為 unknown，錄製未受影響 |
| 正規化、九位精度與差異欄位 | PASS（pytest）；實際 bag 驗證見下列 |
| 暫存重跑、原報告保留與結束碼 | PASS：`eval-verify t34r1` 輸出 Report matches、結束碼 0 |
| 多次統計、verdict 與來源警告 | 部分：`eval-summary t34r1` 單一 bag 結束碼 0；多次重複彙整只有 pytest 覆蓋 |
| pytest 與靜態檢查 | PASS：pytest 834 passed、check_static PASS |
| Humble colcon 與模擬 bag 整合 | PASS：colcon test 650 tests、0 failures；guide 無頭模擬錄製 60 秒 → eval-bag → eval-verify → eval-summary 全部結束碼 0 |

審查時間：2026-10-24 17:20（台北時間），Docker 專案 shoprep（t34s）。限制：manifest 的 Git 欄位在容器內為 unknown（容器沒有可用 git），需在主機補記；機器人在錄製期間靜止，未涵蓋導航中的報表數值。

審查時間：2026-10-24 17:40（台北時間），Docker 專案 shopscen（t31s／t31f）。修正：整數 YAML 航點使 uwb_sim_node 崩潰（'y' 須為 float），person_path 改為一律回傳 float 並加測試。

## 2026-10-24：LOS 足跡檢查縮至跟隨停止點（待模擬驗證）

- `line_of_sight_clear` 新增可選的 `follow_distance` 關鍵字參數，舊呼叫維持原行為。目標距離大於跟隨距離時，足跡檢查長度改為 `distance - max(los_margin, follow_distance)`，排除停止點與人物間的回波；一般與加寬 LOS 均使用控制器的跟隨距離，保留原寬度、去彈跳、寬度遲滯與 NAV2_HANDOFF 狀態機。
- 目標距離小於或等於跟隨距離時，仍檢查至 `distance - los_margin`，範圍內有回波就回報阻擋；距離小於或等於 margin、掃描無效或目標方向無涵蓋時，保留不淨空判定。既有近距離停車與障礙安全邏輯維持原樣。
- 補上停止點前後障礙、側向 0.45／0.35 公尺貨架、近距離與端點、參數／掃描失效，以及節點兩種 LOS 與跟隨距離轉送的回歸測試。
- 本機自行檢查：根目錄 `python3 -m pytest -q` 因系統缺少 pytest 無法執行，改用允許的 `/workspace/scratch/venv-shop/bin/python -m pytest -q`，結果 912 passed、1 skipped、1 warning（既有 xacro 棄用警告）。儲存庫無 `tools/check_static.py`，使用現有 `scripts/check_static.py`；系統 Python 缺少 PyYAML，改用同一虛擬環境執行後 PASS，包含 YAML、XML、Python／launch 語法與 xacro 展開。
- 限制與下一步：本次未執行 Gazebo、Humble 或容器驗收；需重跑 follow-eval，比較平均跟隨距離、LOS 阻擋比例及交接行為，並確認貨架轉角與近距離停止安全。未更動 Nav2 設定、Docker 或其他套件，未執行 git 指令。

| 審查驗證項目 | 結果 |
| --- | --- |
| LOS 停止點與近距離回歸、節點參數轉送 | 待驗證 |
| Gazebo follow-eval、貨架避障與交接安全 | 待驗證 |

## 2026-10-24：障礙停車時安全原地轉向（待模擬驗證）

- `shopping_uwb_follow` 新增並宣告布林參數 `obstacle_stop_rotate: true`。直接跟隨且未啟用繞行或 Nav2 不可用時，`OBSTACLE_STOP` 的線速度立即固定為 0；僅在目標新鮮、掃描有效且既有旋轉足跡檢查淨空時，依 `kp_bearing` 與原有角速度／角加速度限制朝人物原地轉向。設為 false 即恢復雙軸停止。
- 直接跟隨也建立 TF listener；旋轉淨空只依賴雷射到車體的 TF，不要求 map TF 或 Nav2。目標遺失／過期、過近、掃描失效、旋轉不淨空、停用或安全 guard 的 STOP／STALE 均立即雙軸歸零；保留 Nav2 交接、LOS 與既有前方淨空恢復判斷。
- 補上正負方位、角速度與加速度限制、持續零線速度、故障立即停止、參數停用、缺少 map／雷射 TF、Nav2 不可用與前方清空恢復的純邏輯及節點替身測試。本機自行檢查：`/workspace/scratch/venv-shop/bin/python -m pytest -q` 為 946 passed、1 skipped、1 warning（既有 xacro 棄用警告）；同一 Python 執行 `scripts/check_static.py` 為 PASS，包含 YAML、XML、Python／launch 語法與兩／三錨點、ekf／ground_truth xacro 展開。
- 限制與下一步：未執行 Gazebo、Humble 或容器驗收；需以 `route_around:=false`、`fault_injection:=true` 重跑轉角跟隨及 fault-check，確認原地旋轉淨空、恢復移動與故障停止。未修改其他套件，未執行 git 指令。

| 審查驗證項目 | 結果 |
| --- | --- |
| 原地旋轉安全條件、加速度限制與停用參數回歸 | 待驗證 |
| Gazebo 轉角脫困、恢復跟隨與 fault-check 移動基準 | 待驗證 |

## 2026-10-24：follow-los 模擬審查（17:58 台北時間）

- t39s（含 t40 LOS 修正，route_around:=false）：fault-check obstacle 與 follow 套件全部「no moving baseline」→ **FAIL**。log 顯示機器人起步跟隨（v 0.18–0.5、w 至 0.72），轉彎切角後前方 0.593 m < 0.65 進入 OBSTACLE_STOP，v、w 皆 0 且永不恢復，人員離開至 5.6 m。
- 已由 t44 加入 `obstacle_stop_rotate`（預設 true）：OBSTACLE_STOP 時線速度維持 0，旋轉淨空且目標新鮮時原地轉向人員；pytest 946 passed，**待模擬驗證**。
- follow-eval 未跑完（為讓出模擬鎖給 hw／split 提前停止）。此分支**未合併**；下一步：重建後跑 fault-check --suite obstacle／follow 與 follow-eval，確認不再卡死且追蹤距離改善後再開 PR。
## 2026-10-24：硬體介面分支審查結果（18:03 台北時間）

| 項目 | 結果 |
| --- | --- |
| pytest／check_static（合併含情境的 develop 後） | PASS：943 passed、1 skipped；check_static PASS（4 rosidl 訊息） |
| Humble colcon build／test（t41） | PASS：672 tests、0 failures |
| guide nav-check（use_real_* 皆預設 false） | PASS 5/5 |
| follow fault-check --suite obstacle | PASS 4/4（lidar_front 0.2 停止 0.11 s） |
| follow fault-check --suite follow | PASS 7/7 |
| 真實驅動、實體感測器、arm64 | 未驗證（僅模擬；未連接任何硬體） |

- 合併 develop 時 sim.launch.py 保留分源 bridge（lidar／imu／base 各自受 `use_real_*` 控制）並接上情境出生姿態與行人；測試替身新增 `pedestrian_actions`。

## 2026-10-24：容器拆分審查（合併至 develop）

- t42（硬體介面整合前 cf88ae5）：colcon test 667／0；拆分 guide nav-check 5/5；XRCE comm-e2e 7/7。
- 硬體介面 #19 後再次合併 develop：sim.launch.py 保留拆分分支的 start_sim／start_localization GroupAction，加入硬體介面的 validate_sources、各來源 lidar／imu／base bridge 與 wheel_odom_noise simulate_* 旗標。tests/test_hardware_launch.py 改為展開 GroupAction；pytest 972 passed。
- 尚未驗證：硬體介面整合後版本（70046eb+）的 Gazebo 模擬；下次須再執行 guide nav-check 與 comm-e2e。
- 本次取代 codex/microros-agent-e2e 的工作。

## 2026-10-26：Mac Apple Silicon 原生容器、SLAM 與整合回歸

### 本次修改

- 切至 `develop`，以 macOS arm64／Docker Desktop 的 Linux arm64 引擎原生建置三個映像，沒有固定 platform。Nav2 私有子模組固定 `bf60ee0`，未修改私有原始碼。使用者後續授權提交／推送，並要求不開 PR；本次整理於 `codex/mac-arm64-slam`，不合併 develop。
- `dev.sh` 從任意目錄載入根 `.env`，依服務第 1 個實例查詢實際連接埠；本機使用 noVNC 6081／VNC 5970。6080 不由本專案接管。早先舊 display 曾短暫停止後恢復；晚間 Docker Desktop 已關閉，重啟後原容器已不存在，僅重建本專案 compute／display。
- 修正 Bash 3.2 空陣列 nounset、apt 隔離啟動的空 `map:=`、comm-e2e-up 意外重建 compute 網路命名空間，以及 15 個長駐 Python 節點中斷時 context 提早關閉問題。
- 服務專用 Fast DDS writer 的 max blocking time 改為 3 秒，緩解一次 lifecycle configure 回應遺失；一般 topic 仍為 0.1 秒。這是上游競態的緩解，不能保證根治。
- 跟隨遇前方障礙改為完整零 Twist；安全層完整旋轉圓盤通過後保留要求角速度，避免摩擦門檻造成停滯。平移／曲線的縮速與所有 STOP／STALE 保護保留。
- 人物模型改用持續非同步 SetEntityPose 服務連線。跟隨交接在遠距離時由 Nav2 持有，接近才回到 DIRECT；有新鮮 guard 狀態時由該層負責側向淨空縮速。apt follow 明確傳入 RPP 0.5 m/s，guide 維持 0.4；目標 map 轉換只容許重用不超過 0.2 秒且不刷新來源時間的配對結果，避免 TF 短暫未到導致取消重試。
- 故障檢查改用平面真值速度，0.9 m 減速案例要求平移基線與恢復，不能以純旋轉冒充。
- 依使用者建議，apt 全域規劃器改用 Humble Smac2D；保留原 0.8 m 搜尋容差。fork 保留原生建置的 NavFn。不可達目標檢查同步支援兩種格網規劃器。
- 動態情境發現世界 SceneBroadcaster 沒有個別時間戳，行人因此不動；補各模型的 Fortress PosePublisher、模型專屬 pose topic 橋接及有界時間差合併。初次誤用不支援的 topic override 已修正，Fortress 6.18 另須同時設定 `publish_nested_model_pose=true`，否則只收到空 Pose_V；修正後三種動態情境的時間戳、移動與雷射可見性實測通過。
- SLAM 增加 map 固定座標的俯視 RViz 設定、雷射／機器人／規劃路徑／位姿圖顯示及可選設定檔。重載前檢查位姿圖兩檔與有限出生姿態。
- save-map 改與 map-tour 共用服務結果碼及四個有更新的非空檔案檢查；先完成暫存整組再替換目的檔，負例會保留原圖。實測 localization 序列化服務回覆成功卻未寫圖，新檢查正確以失敗回報，並防止舊圖冒充新輸出。新增 [SLAM 操作說明](SLAM.md)。暫存儲存成功，localization 負例也確認原四檔 SHA-256 不變、暫存清除。

### 自行檢查

- 三種 arm64 原生映像；fork 18 套件建置及 110 項動態連結檢查通過。最後完整檢查為 pytest 1035 passed、colcon 721 tests／0 errors／0 failures／5 skipped；靜態檢查與 git diff --check PASS。
- noVNC／RViz 實際登入顯示；即時時鐘、雷射、IMU、里程計、TF、Nav2 lifecycle 與唯一 watchdog 14 項檢查通過。SIGINT／SIGTERM 都確認 watchdog 先送零速、結束碼 0。
- 原 NavFn／AMCL guide nav-check 6/6；AMCL 六區定位最大 0.1029 m／2.0044°。建圖巡航 17/17；第一份新位姿圖冷啟動重測六區定位最大 0.0184 m／0.459°。第一輪 lifecycle 未就緒的失敗紀錄保留。
- Smac2D 的第二份新圖 `arm64_smac_slam` 亦完成 17/17 航點，四檔保存；重載定位六區全部 PASS，位置 RMSE 0.02049 m，最大 0.03130 m／0.41884°。RViz 實際顯示建圖，並量測 SDF／格網幾何；缺少外牆面不能推論全圖誤差。
- 人物同步重測平均 0.0096 m／0.03 s；原 NavFn 跟隨最終 120 模擬秒直線／轉彎距離誤差 RMSE 1.121／1.200 m、濾波延遲 0.08 s。未出現過近／重取得事件，整體 NOT_COVERED，尚須對應情境及 Smac 重測。
- guard＋collision_monitor 的障礙故障 4/4、跟隨故障 7/7；模式切換及急停／重設 15/15。MCU Python 10/10、PTY serial 10/10、C／XRCE 7/7，均為模擬。
- bag 轉換／重跑／彙整成功；晚錄導航的負例遭拒。loc-check 直接送 action，該 bag 沒有購物狀態 episode，不能由它宣稱導航成功率。Smac alt_start 兩個 action 成功，但入口真值 yaw 0.558 rad 未達情境建議；動態情境初輪停滯，保留失敗紀錄。

### 限制與下一步

- Smac2D stop-go 直線 RMSE 0.341 m、五次過近停止 PASS；turns 直線／轉彎 RMSE 1.227／1.366 m，人物同步兩情境均 PASS。turns 接觸時真值微動仍 FAIL，命令確認為零；不放寬門檻或移除碰撞。過近解除遲滯及大角度原地對準已補純邏輯測試。
- 三種動態行人的實際來源／移動／雷射驗證通過；aisle_crossing 仍逾時，narrow_pass 的 household 雖 action 成功但真值誤差 1.878 m，checkout_queue 入口 yaw 0.439 rad 未達建議，均明列未通過。後續應分析 AMCL 動態遮擋與代價地圖／路徑／guard 的交互作用；不可用真值修正控制器來掩蓋定位問題。
- 拆分容器的 14 項即時檢查及 nav-check 6/6 PASS，終點真值誤差 0.139 m。fork 導航與 yaw-check 本輪未重做功能回歸，既有建置及純測試不能取代實測。
- 使用者會另外測 Linux；本次沒有新的 Linux amd64 原生結果，不能將歷史結果視為本次修改已通過。沒有連接硬體。完整證據索引見 [arm64 驗收](VALIDATION_ARM64.md)，日誌／bag 為忽略版控的本機產物。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| arm64 原生映像與套件建置 | 僅使用者 Mac 自行回報（映像、fork 建置、colcon 721／0 failures）；審查端無 arm64 主機，未重做 |
| 環境變數與實際連接埠 | 審查端檢視 `dev.sh`／`.env.example`／compose：`.env` 已列入 .gitignore，連接埠預設 6080／5910 只綁 127.0.0.1；在 Linux 以環境變數覆寫連接埠（16085／15915）實際啟動通過。Mac 上 6081／5970 為自行回報 |
| SLAM 建圖、儲存與重載定位 | 審查端只跑 pytest（含 `test_slam_save.py`）；mapping＋map-tour、save-map、重載定位為 Mac 自行回報，Linux 未重跑 |
| 模擬功能與安全回歸 | 審查端 Linux amd64（2026-10-27）：apt Nav2＋Smac2D guide nav-check 5/5、loc-check 六區全過（最大 0.027 m）、follow 故障 fault-check 7/7。跟隨轉彎、aisle_crossing、narrow_pass、checkout_queue 仍未通過（見上方限制）；fork 導航與 yaw-check 未重測 |
| Linux amd64 相容性 | 審查端 Linux amd64 Docker：映像建置、colcon build、colcon test 721 tests／0 errors／0 failures／5 skipped；pytest 1034 passed、1 skipped（私有 fork 子模組未取出）。修正 3 個 `setup.py` 的 glob，避免主機 pytest 留下的 `launch/__pycache__` 讓 colcon build 失敗 |

使用者 2026-10-27 授權測試通過後合併至 develop（取代上方「不開 PR／不合併」的原指示）。

## 2026-10-27：合併 develop 與 fork Smac2D 規劃器切換

### 修改與原因

- 解決本文件與 `tests/test_nav2_fork.py` 的內容衝突，保留雙方進度段落，依日期排列；測試同時保留 Shrink 的成本容忍、服務逾時及還原重試斷言。
- fork 的 NavFn 將未知格 255 先轉為障礙 253，使 `allow_unknown` 無法生效，household／drinks 目標可能失敗。fork GridBased 改為 `nav2_smac_planner/SmacPlanner2D`，與 apt 共用 0.8 m 容差、不降採樣、2 秒規劃時間、2.0 成本倍率及允許未知格；移除 NavFn 專用參數，保留 0.25 m 內切半徑下限與其他 fork 安全設定。apt 仍為預設 stack。
- 將 `nav2_smac_planner` 加入 fork 建置清單，必須由 checkout `12f06c4` 的原始碼在 `/nav2_ws` 建置。該 Smac 套件標示 1.1.20，fork core／costmap 為 1.1.16；直接載入 apt 1.1.20 Smac 外掛會混用 ABI。合約與 linkage 檢查共用安裝檢查，缺少可用的 fork `libnav2_smac_planner_2d.so` 即明確失敗，不接受 apt 回退。建置前用 `--source-only` 保留首次建置能力，建置後必跑安裝與 linkage 檢查。
- 查閱 fork Smac 的 CMake／package.xml 與呼叫符號，核對 core 規劃器介面、costmap／publisher／inflation API、util 參數、線段迭代及幾何工具，未發現缺少的 API。Smac 同時編譯 Hybrid／Lattice 共用來源，因此仍需要 OMPL、nlohmann JSON、Eigen、angles 與 OpenMP；Dockerfile 明列 OMPL／JSON／angles／Eigen CMake 相依，既有 Eigen 開發套件保留。未執行安裝或映像建置。
- 更新 ARCHITECTURE、README 與 ARM64 驗證文件，明確區分既有驗收結果與本次待驗證的 fork 規劃器。

### 自行檢查、限制與下一步

- 指定沙盒純 Python 完整測試：`/workspace/scratch/venv-shop/bin/python -m pytest -q`，結果為 **1041 passed、1 warning**（xacro 的既有 Python 棄用警告）；含私有子模組測試及缺少／失效 Smac 安裝、apt 回退拒絕的回歸案例。
- `check_static.py` 通過 YAML、XML／SDF／xacro、Python／launch、Bash 與情境檢查；本次未執行 Docker、C++ 編譯或模擬，原始碼 API 核對不能證明 ABI 與導航行為相容。
- 工作樹已移除衝突標記；嘗試只暫存兩個衝突檔時，Git 無法在唯讀的 `/workspace/repos/Almighty-shopping-car/.git/worktrees/shop-navteb` 建立 `index.lock`，因此索引仍維持未解衝突。維護者須在可寫入索引的環境執行 `git add docs/PROGRESS.md tests/test_nav2_fork.py`。本次未提交或推送。
- 下一步：原生重建 fork，確認 Smac 從 fork install 載入並通過 linkage，再分開執行 household／drinks、nav-check、loc-check、取消停止與 Shrink 還原模擬回歸。

| 審查驗證項目 | 狀態 |
| --- | --- |
| 衝突合併、設定一致性與純 Python 回歸 | 審查端（2026-10-27）：pytest 1041 passed；索引衝突已於提交 8d091d4 解決 |
| Humble colcon build／test 與執行期依賴 | 審查端 Linux amd64：fork nav2-build 含 Smac2D 1 分 5 秒完成，custom_controller gtest 28/0 |
| fork Smac 安裝來源、動態連結與 ABI | 審查端：linkage 檢查 110 PASS，Smac2D 從 `/nav2_ws` 載入 |
| Gazebo 導航、未知格、取消停止與 Shrink 還原 | 審查端：fork nav-check 6/6（區域容差 0.043 m）、loc-check 六區全過；取消停止與 Shrink 還原未單獨重測 |

## 2026-10-27：fork 成為預設 Nav2 stack（`nav2_stack:=auto`）

### 修改與原因

- 審查端 fork 模擬驗收：fork guide loc-check 六區全過（最大定位誤差 0.078 m，entrance；其餘 ≤ 0.030 m）；fork follow fault-check 7/7（命令停止 0.114–0.665 s）；fork 障礙物 fault-check 4/4（lidar_front:0.2 命令停止 0.070 s、lidar_drop 0.538 s、lidar_freeze 0.661 s）。同次 develop（apt）障礙物 fault-check 4/4 作為基準。
- 修正：fork 的 `StopController` BT 裝飾器從每個 navigator 的 client 節點發布絕對 `/cmd_vel`。原本只重映射 `bt_navigator_navigate_to_pose_rclcpp_node`，through-poses client 仍在 `/cmd_vel` 留下第二個發布者，fault-check 以「`/cmd_vel` 必須只有 cmd_vel_watchdog 一個發布者」失敗。新增 `bt_navigator_navigate_through_poses_rclcpp_node:/cmd_vel → /cmd_vel/nav`；重跑後 `ros2 topic info -v /cmd_vel` 只有 cmd_vel_watchdog，障礙物 fault-check 通過。
- guide／follow／navigation 與拆分容器 `runtime_role` 的 `nav2_stack` 預設由 `apt` 改為 `auto`：已建置 fork underlay 時使用 fork TEB＋Smac2D，沒有私有子模組或未建置時自動回退 apt（Mac／無權限使用者不受影響）。要比對 apt 須明確指定 `nav2_stack:=apt`。

### 限制與下一步

- `auto` 實際選到 fork 的 Gazebo 啟動以及動態情境（aisle_crossing／narrow_pass／checkout_queue）在 fork 下尚未重跑；Mac arm64 fork 建置未驗證。
- 跟隨 TEB 繞行與 Shrink 還原仍待調參與專門回歸。

| 審查驗證項目 | 狀態 |
| --- | --- |
| through-poses `/cmd_vel` 重映射與單一發布者 | 審查端 Linux amd64（2026-10-27）：fork 障礙物 fault-check 4/4，`/cmd_vel` 只有 cmd_vel_watchdog 一個發布者 |
| 預設 `auto` 選擇與回退 | 審查端：pytest（test_mode_launch 53 項、全套 1041）通過；`auto` 實際載入 fork 的 Gazebo 確認併入合併後動態情境重跑 |

## 2026-10-27：follow-los 與 develop 衝突整合

- 以 develop 為執行時基準，保留近停釋放遲滯、大角度先對準、guard 接手側向減速、短暫 map TF 快取、人物非同步服務、Smac2D 及原有 YAML 數值；只增加必要的 `obstacle_stop_rotate` 設定。歷史前饋／距離保持實驗保留於診斷用途，執行時預設不啟用；不引入控制器真值輸入。
- 疊加 t40：一般與加寬 LOS 均檢查至跟隨停止點，保留近距離 margin 與失效保護。疊加 t44：前方受阻立即固定零線速度，僅在有效目標、有效雷射／TF、至少 0.45 公尺旋轉圓盤淨空且未由 Nav2 接手時，允許受角速度／角加速度限制的原地轉向。近停／遺失／過期／guard STOP 或 STALE 均優先雙軸停止；前方清空後仍須遵守大角度對準遲滯。
- Nav2 交接遵循 develop 的成功返回 DIRECT、接近與 LOS 恢復規則，沒有重新套用已撤回 t35 的持續持有策略；保留分支既有的命令安全轉送及取消／排空生命週期。README、ARCHITECTURE、METRICS 保留雙方內容，舊設計明確列為歷史；本檔保留雙方全部工作紀錄，依日期與已知審查時間排列。
- 自行檢查：`/workspace/scratch/venv-shop/bin/python -m pytest -q` 為 1158 passed、1 skipped、1 warning（既有 xacro 棄用警告）；同一 Python 執行 `scripts/check_static.py` 為 PASS：4 個訊息介面、31 份 YAML／RViz、15 份 XML／SDF／xacro、151 份 Python、7 份 Bash、7 種情境，含兩／三錨點在 ekf／ground_truth 的 xacro 展開。`git diff --check` 與全儲存庫衝突標記檢查通過；這些為純 Python／靜態驗證，不能取代模擬驗收。
- 限制與下一步：本次未執行 Docker、Gazebo、ROS 或硬體驗收；需重跑轉角跟隨、follow-eval、fault-check obstacle／follow，確認恢復移動、追蹤距離與故障停止。未提交或推送，合併結果留待使用者審查。工作檔衝突已清除，但此沙盒將實際 Git 索引目錄 `/workspace/repos/Almighty-shopping-car/.git/worktrees/follow-los/` 設為唯讀，`git add` 因無法建立 `index.lock` 失敗；須在可寫入索引的環境執行 `git add -u` 才能將衝突標記為已解決，合併仍未提交。

| 審查驗證項目 | 結果 |
| --- | --- |
| develop 行為、t40／t44 與安全優先序回歸 | 待驗證 |
| 文件歷史、設定值與衝突清除 | 待驗證 |
| Gazebo 轉角恢復、follow-eval 與故障驗收 | 待驗證 |

## 2026-10-27：近距離鎖定跨 Nav2 路徑防護

- 依使用者提供的 `t65-los-turns-check.txt`：171.96 秒開始的過近事件在 174.88／174.90 秒違規，243.98 秒開始的事件在 246.86／246.88／246.90 秒違規，分別為事件後 2.92–2.94／2.88–2.92 秒；門檻仍為真值距離低於 0.6 公尺、寬限 0.5 秒後平面速度不超過 0.03 m/s 且角速度絕對值不超過 0.05 rad/s。另有 211.20 秒事件為 NOT_COVERED，不能算通過。develop 對照為五次過近事件全部 PASS。
- 程式確認的缺口：`FollowController` 原本就先檢查 `near_latched`，因此直接避障旋轉與 FOLLOWING 對準不會繞過鎖定；但節點的 guard STOP／STALE 會將 TOO_CLOSE 覆寫成 OBSTACLE_STOP，交接管理器與 `NavigationGuard` 原本只依狀態字串判斷過近，未直接檢查鎖定。這是本次修補的安全優先序缺口，尚不能認定為上述兩次真值違規的已證實根因：同一 tick 的 guard STOP／STALE 也會禁止 Nav2 輸出，且新交接另受距離大於 1.3 公尺限制。
- 日誌證據與限制：`t65-los-turns-launch.log` 全檔未出現 `Forward blocked; rotating toward target`；七筆 `Forward blocked; rotation allowed` 的估測距離為 1.041–1.810 公尺。兩段穿越附近，牆鐘 1793036623.316712 記錄距離 0.081 公尺／方位 2.888 rad，1793036624.358174 為 0.389 公尺／3.068 rad；另一段 1793036698.776129 為 0.211 公尺／3.030 rad，1793036699.812090 為 0.500 公尺／3.037 rad。這些均為 DIRECT、取消完成、雙軸零命令。可見的重新對準分別在 1793036626.407060（估測距離 0.982 公尺、角命令 0.825 rad/s）與 1793036700.835890（0.800 公尺、0.074 rad/s）。舊診斷僅 1 Hz 且沒有模擬時間；錄製起點牆鐘 1793036521.76 對應模擬 82.517 秒與約 0.92 即時倍率不足以精確定位 0.04–0.06 秒違規尾段，不能據此斷言當時有非零旋轉命令或排除接觸物理微動。
- 修改：guard 保留 TOO_CLOSE；交接管理器直接接收 `near_latched`，鎖定期間不得送出／更新目標，既有 Nav2 交接立即取消；`NavigationGuard` 直接接收鎖定並立即清零雙軸與內部殘速。直接跟隨沿用既有解除距離與 0.3 秒持續淨空判斷，不改控制參數、評估指標或門檻。診斷新增模擬時間、鎖定狀態及解除累計秒數。
- 新增純 Python 回歸：人物由前方穿越原點至後方，旋轉淨空為真時，0.59／0.70 公尺仍保持零輸出；分別覆蓋前方受阻旋轉、前方清空對準、Nav2 既有旋轉立即停止、被 guard 遮蔽狀態的禁止交接／取消，以及實際節點 callback 在 STOP／STALE／CLEAR 下的取消期間零輸出。亦確認解除計時不足及淨空中斷時不釋放，持續淨空後才恢復受限旋轉。
- 追蹤退化：直線／轉彎距離誤差 RMSE 由 develop 的 0.6355／0.6724 公尺變為 0.7680／0.8054 公尺。比對確認預設前饋與距離保持實驗均未啟用，預設比例控制律相同；LOS 改查至跟隨停止點、Nav2 旋轉轉送及目標更新生命週期改變可能影響軌跡，但現有摘要不足以單獨歸因，本次不調參。
- 自行檢查：指定虛擬環境執行根目錄 `python -m pytest -q`，1168 passed、1 skipped、1 warning（既有 xacro 棄用警告）；`scripts/check_static.py` PASS，涵蓋 YAML、XML、launch／Python 語法及兩／三錨點兩種里程計模式的 xacro 展開；`git diff --check` 通過。未執行 Docker、ROS、Gazebo、硬體、提交或推送。
- 限制與下一步：本次完成近距離鎖定的明確跨路徑防護，尚未證明真值 near_stop 回歸已消失。需重跑相同 supermarket 的 `follow_turns`、`follow_stop_go` 與 `fault-check`（跟隨及障礙），保留既有門檻；針對兩段穿越同步記錄模擬時間的逐 tick 鎖定／命令、底盤最終 Twist、真值速度與碰撞接觸，區分命令旋轉與接觸微動。1 Hz 診斷仍不能取代逐 tick 證據。

| 審查驗證項目 | 結果 |
| --- | --- |
| 近距離鎖定跨直接跟隨、Nav2 交接與轉送回歸 | 待驗證 |
| follow_turns／follow_stop_go 真值過近停止與追蹤誤差 | 待驗證 |
| fault-check 跟隨及障礙停止 | 待驗證 |


## 2026-10-27：TRACKING 貼架退化分析與繞行恢復修補

### 離線證據

- 僅讀取既有 `bags/t65b_dump.jsonl`、`bags/t65b_turns` 對應資料及 `/workspace/wt/logs/shop/t65-*` 日誌；拋棄式分析程式與輸出為 `/tmp/analyze_follow_los.py`、`/tmp/analyze_follow_los.txt`，未加入儲存庫。真值僅用於此次離線分析，未進入控制器。
- 原評估門檻及樣本保留：develop 直線／轉彎距離誤差 RMSE 為 0.6355／0.6724 公尺，最大距離 2.392 公尺；LOS run1 為 0.7680／0.8054；`0bdb4c1` run2 為 1.0516／1.2379，最大距離 4.573 公尺。stop-go run2 約 1.04／0.98，直線／轉彎／near_stop 均 PASS。turns run2 的 near_stop 仍為 FAIL，原違規時間不刪除；依審查者提供的逐筆調查，為人物圓柱穿越、推動零命令且 TOO_CLOSE 的車體之模擬接觸現象，本次不處理該物理問題。
- 以下採 turns run2 原評估區間 `[82.097, 262.097]` 秒，以各 topic 最新已收到樣本做分段常值時間積分；距離誤差為真值距離減 1 公尺。這是歸因統計，與正式評估的插值／轉彎分類樣本統計不同，不修改正式評估器，也不排除安全停止。

| 模式 | 時間（秒） | guard SLOW（秒） | STOP（秒） | CLEAR（秒） | STALE（秒） | 距離誤差 RMSE（公尺） |
| --- | --- | --- | --- | --- | --- | --- |
| NAV2_HANDOFF | 99.354 | 92.701 | 1.587 | 5.066 | 0 | 0.6996 |
| DIRECT（含所有停止與等待） | 80.646 | 51.411 | 16.611 | 8.291 | 4.333 | 1.3838 |

- 全段時間加權 RMSE 為 1.0621 公尺；153–177 秒的 24 秒為 2.3757 公尺，占全段平方誤差積分 **66.7%**（135.454／203.065 公尺平方・秒）。其餘 156 秒為 0.6583 公尺，接近 develop 摘要的誤差量級，但沒有 develop 原始 bag 可作相同方法的逐段比較。develop launch 日誌沒有跟隨診斷樣本，不能量化其 DIRECT／NAV2 時間、guard 占比與交接位置；不可把日誌中的導航開始／取消數當作精確模式時間。
- 關鍵 24 秒有 23.241 秒 SLOW、0.759 秒 CLEAR；最終 `/cmd_vel` 平均線速度 0.0981 m/s、guard 平均倍率 0.3117、平均 swept clearance 0.2757 公尺。159–167 秒診斷持續要求約 0.50 m/s，160／166 秒最終命令約 0.0964 m/s，guard 倍率約 0.1928。160 秒車體 x=2.0565，SDF 貨架右緣 x=1.6，側向中心間距約 0.4565 公尺，扣車體半寬 0.25 後僅約 0.2065 公尺；guard 掃掠淨空約 0.2287 公尺，符合側邊貨架限制的量級。前方診斷仍約 1.2 公尺，LOS 已清楚，直接追蹤只對準遠方人物，未能迅速回到走道中央。

### 交接位置與觸發限制

下表採 `/follow/state` 第一次觀察到模式變更的時間及最近一筆車體真值。主要縱向走道中心 x=2.4；入口橫向通道中心 y=0，接近入口的位置應比較 y，不能將 x 接近 0 誤判為縱向走道偏移。首尾交接區間受錄製邊界截斷。

| 模擬時間（秒） | 事件 | 車體位置（公尺） | 相對通道中心 |
| --- | --- | --- | --- |
| 97.551 | NAV2 → DIRECT | (2.412, 4.704) | x 偏移 +0.012 |
| 107.052 | DIRECT → NAV2 | (2.412, 4.705) | x 偏移 +0.012 |
| 133.854 | NAV2 → DIRECT | (0.116, 0.603) | 入口 y 偏移 +0.603 |
| 144.007 | DIRECT → NAV2 | (0.116, 0.604) | 入口 y 偏移 +0.604 |
| 152.910 | NAV2 → DIRECT | (1.965, 0.245) | 轉入 x=2.4 走道，x 偏移 −0.435 |
| 194.507 | DIRECT → NAV2 | (2.263, 1.679) | x 偏移 −0.137 |
| 205.855 | NAV2 → DIRECT | (0.108, 0.594) | 入口 y 偏移 +0.594 |
| 215.955 | DIRECT → NAV2 | (0.107, 0.595) | 入口 y 偏移 +0.595 |
| 241.661 | NAV2 → DIRECT | (2.417, 4.703) | x 偏移 +0.017 |
| 250.956 | DIRECT → NAV2 | (2.417, 4.704) | x 偏移 +0.017 |

- 152.801 秒仍為 NAV2；152.910 秒為 `OBSTACLE_STOP: Obstacle or unavailable scan; waiting for navigation stop`，152.937 秒恢復 FOLLOWING。152.150 秒診斷人物距離 1.539 公尺／方位 0.774 rad，153.150 秒為 1.609 公尺／0.636 rad；同期 Nav2 日誌為 client requested cancellation，沒有成功到達或規劃失敗。這支持「單次安全資料／TF 缺口取消未完成的繞行」；現有 bag 未錄 `/scan`、估測目標、TF 或逐 tick supervisor 原因，不能確定是哪一項失效，也不能將此事件當成已證實的正常近距離出口。程式的失效路徑會取消 Nav2 並清掉受阻計時，之後 LOS 清楚就不再交接，正好留下貼架 DIRECT 的長尾。

### 修改與理由

- 純邏輯 `follow_handoff.py` 新增未完成繞行意圖：安全資料失效立即取消、保持零輸出；資料恢復後，如果直接路徑的側向／朝向仍不合適，沿用原有距離、交接延遲及失敗冷卻重新交接，即使 LOS 已清楚。近距離鎖定、LOST、停用及時間倒退會清除此意圖；安全鎖定不被恢復程序繞過。
- 人物超過跟隨距離時，正常出口要求接近、LOS 遲滯、前方淨空、側向空間及朝向持續成立，導航成功也不能跳過遠距離出口保護。審查修正補回已追上人物的出口：LOS 經遲滯確認、人物距離不超過 `follow_distance`，且已滿足 `min_nav_hold` 或導航成功，即交回 DIRECT，不受 `direct_clear` 限制。Nav2 目標為人物自身的 map 位置，窄走道不能因側向空間不足而持續導航至過近安全停止；直接跟隨應在跟隨距離停止／保持。人物移動時，成功目標仍沿用既有 0.4 公尺／0.5 秒更新機制。Nav2 接手期間維持獨占命令、既有取消／終止／排空流程與線角加速度限制。
- **未改 YAML 或任何參數值**。新出口檢查重用 `side_free_clearance=0.75`：相較貼架約 0.4–0.5 公尺側距，提供明確離架條件；0.8 公尺半寬走道仍可滿足。按 guard 的半寬 0.25、margin 0.10、slow_extra 0.50，筆直近側距 0.75 時理想倍率約 0.8，0.5 m/s 命令可保留約 0.4 m/s，實際仍受掃掠與遲滯限制。因此使用側向距離判定，沒有要求 guard 一定 CLEAR（人物本身也可能造成 SLOW）。朝向重用 `rotate_to_target_release_angle=0.25`，讓 Nav2 完成轉入走道再交回直接控制。這些是新增用途的幾何依據，改善幅度待模擬確認。
- 節點在恢復交接前以 BLOCKED_WAIT 保持零命令；每次取消額外記錄模擬時間、supervisor 出口原因、側向距離、恢復意圖、雷射／map 有效性、前方距離、guard 狀態及目標年齡，避免 1 Hz 診斷漏掉單次取消來源。README 跟隨段落新增目前生效規則，保留舊分支歷史。
- pytest 新增單次雷射失效取消後的真實節點 callback 回歸、LOS 清楚仍須恢復繞行、側距／朝向出口邊界、近停與遺失等清除恢復意圖、成功後持續更新人物、遠距離成功不可跳過持有與淨空遲滯；補上窄走道側向空間不足時，已追上人物仍可返回 DIRECT、導航成功可略過最短持有時間但不能略過 LOS 遲滯的回歸，並保留遠距離低側距維持 NAV2 的測試。未放寬任何測試或評估門檻。

### 自行檢查、限制與下一步

- 指定沙盒執行 `/workspace/scratch/venv-shop/bin/python -m pytest -q`：**1192 passed、1 skipped、1 warning**（既有 xacro 棄用警告），25.60 秒；`scripts/check_static.py` PASS，涵蓋 31 份 YAML／RViz、15 份 XML／SDF／xacro、152 份 Python／launch、兩／三錨點的 ekf／ground_truth xacro 展開及設定一致性。`git diff --check` 通過。全程未執行 Docker、ROS、Gazebo、其他模擬、硬體、提交或推送；離線積分與純 Python／靜態驗證不是閉迴路驗收。
- 維護者需以同一 supermarket 與既有參數分別重跑 `follow_turns`、`follow_stop_go`、跟隨及障礙 `fault-check`，保留評估門檻與 near_stop 原始違規。同步錄製 `/scan`、TF、`/uwb/target`、`/follow/state`、`/safety/obstacle_state`、最終 `/cmd_vel` 及離線真值；核對每次退出原因、恢復意圖、側距與朝向，確認 153–177 秒式的貼架限速長尾消失，且新條件沒有造成 Nav2 長期無法返回或故障恢復延遲。人物穿越推車現象另外保留接觸證據，不能隱藏或當作本次已修好。

| 審查驗證項目 | 結果 |
| --- | --- |
| 離線時間／誤差歸因與交接位置 | 待驗證 |
| 繞行恢復、側向出口及近停安全回歸 | 待驗證 |
| follow_turns／follow_stop_go TRACKING 與 near_stop | 待驗證 |
| 跟隨及障礙 fault-check | 待驗證 |

### 同日審查修正：窄走道追上人物的出口

- 保留 t72 的 `recovery_pending` 與遠距離 `direct_clear` 門檻，補回跟隨距離內的 LOS 清楚出口；README 同步區分近距離停止／保持與遠距離貼架保護。
- 本次僅執行指定沙盒 pytest 與差異檢查；未執行 Docker 或模擬，亦未提交或推送。指定 `/workspace/scratch/venv-shop/bin/python -m pytest -q` 結果為 **1196 passed、1 skipped、1 warning**（既有 xacro 棄用警告），25.50 秒；`git diff --check` 通過。窄走道閉迴路行為仍待模擬驗收。下一步由審查者核對出口條件並執行既有模擬驗收。

| 審查驗證項目 | 結果 |
| --- | --- |
| 窄走道近距離退出與遠距離側距限制 | 待驗證 |

## 2026-10-27：修正 map TF 暫失造成跟隨繞障與等待死鎖

### 本次完成

- 比對 `develop`，確認分支新增的繞障恢復旗標與無期限 `BLOCKED_WAIT`，會在 map TF 暫失、側向淨空不足時阻止直接跟隨。
- 雷射 LOS 與 map 目標有效性分離；精確時間 map TF 失敗時，改查最新 TF，僅接受相對節點時鐘不超過 0.3 秒且非未來的資料。無有效 map TF 時回傳空目標，仍保留實際雷射 LOS，不再沿用舊 map 目標快取。
- 新增 `map_invalid_grace: 0.5`，僅於 map 目標連續無效達寬限時間後退出 NAV2；導航失敗或取消仍立即退出並保留既有重試冷卻。寬限期間不提交空目標，也不放行缺少有效 map 目標的導航速度。
- 新增 `max_blocked_wait: 4.0`；supervisor 統一決定等待，僅在 map 目標有效、Nav2 action server 就緒且冷卻結束時允許。單次阻擋等待到期後維持 DIRECT，直到阻擋條件解除才重設到期旗標，避免立即重啟相同等待。
- 兩個參數經節點既有 `HandoffParams` 欄位迴圈宣告，並明列於 YAML。未更動安全淨空、速度／加速度或評估門檻，控制器未新增 ground truth 來源。
- 補上純邏輯與實際回呼測試，涵蓋無 map TF 的真實 LOS、最新 TF 時效邊界、寬限計時重設、等待到期、導航不可用、失敗冷卻，以及各種退回直接跟隨路徑的安全停止與側向減速。

### 已執行檢查與方法（執行紀錄，非審查驗證）

- 系統 `python3` 缺少 pytest；透過 `PATH=/workspace/scratch/venv-shop/bin:$PATH` 選用既有指定虛擬環境。
- 執行 `nice -n 19 python3 -m pytest -q ros2_ws/src/shopping_uwb_follow/test tests/test_mode_launch.py tests/test_follow_loop_sim.py`：672 項通過。
- 新增最後一項 map 寬限回呼測試後，僅重跑 `nice -n 19 python3 -m pytest -q ros2_ws/src/shopping_uwb_follow/test/test_follow_handoff_callbacks.py`：101 項通過。
- 修改的節點／純邏輯 Python AST 語法檢查、跟隨 YAML 解析及參數值檢查、`git diff --check` 通過。未修改 launch、xacro 或 XML。

### 限制與下一步

- 本次僅執行指定離線測試，未執行完整測試套件、colcon、Docker、模擬或硬體驗收，未干擾共用主機既有 Gazebo。
- 導航取消仍須等待既有終止確認與速度排空才能交回直接控制，等待期限不會略過速度所有權保護。
- 後續由審查者在既有 AMCL／Nav2 跟隨情境重做 TF 暫失故障檢查，確認實際移動與狀態轉移。

## 2026-10-27：Linux 動態情境離線根因分析與修正

### 修改與證據

- 遵照本次限制，僅以 `/tmp` 的純 Python 分析腳本唯讀既有 t64 bag／日誌，未啟動 Docker、模擬或重型程序，未提交／推送。完整數據、方法、參數理由與 PR 說明見 [動態情境分析](DYNAMIC_SCENARIOS_REVIEW.md)。
- 找到 apt／fork 全域 costmap 的 `scan.max_obstacle_height` 缺漏：Humble 觀測來源預設 0.0，會排除高度 0.36 m 的雷射。t≈99 局部地圖有行人致命格，全域行人附近卻無致命格，路徑仍沿 x=0.005 穿越 y=3。兩份設定補為 2.0，與局部相同；不放寬測試或安全門檻。
- aisle_crossing：household 真值終點誤差 2.346 m、僅移動 2.157 m；goto 120 秒逾時，bag episode 為 107.786 模擬秒。AMCL 最大誤差僅 0.068 m；92 次路徑輸出的間隔中位數 0.953 秒，持續走原中線。2,401 個 guard 樣本有 1,443 個 STOP，全部仍收到非零 mux 命令，最終輸出零；progress checker／清圖／旋轉／後退反覆失敗。主要問題是全域漏標與局部／安全層阻擋的交互作用。
- narrow_pass：household 35.079 秒回報成功但真值誤差 1.804 m。t≈102.802 接觸時最終命令已為零，真值速度仍約 0.296 m/s；行人將底盤從 y≈3.13 推至 y≈1.20，AMCL 卻維持 y≈3.02，最大誤差 1.827 m。接觸後雷射出現 `-inf`，guard 保守停車並有 178 個 STALE 樣本。最後路徑端點距原目標僅 0.007 m，因此不能將 1.8 m 誤差單歸咎 Smac 0.8 m 搜尋容差；原 zone_navigator 完全未驗證終點。
- checkout_queue：checkout 誤差 0.135 m／0.168 rad；入口耗時 79.745 秒、誤差 0.129 m。AMCL 兩段最大誤差 0.043／0.056 m。返回時 guard 將 0.4 m/s 縮至約 0.011 或零，最低人物淨空仍為 0.077 m；最近接近點已靜止，仍未達建議 0.10 m。
- 區域導航新增 ROS-free `arrived()`，Nav2 成功後另檢查區域座標系的定位 TF（標準區域為 map）：0.15 m／0.25 rad、年齡 0…0.5 秒；缺資料、非有限值、過期或超差則 ABORTED，已完成 action 可重新發目標。生命週期預設不接受未驗證成功，保留取消、舊 generation 與逾時隔離；新增 pytest。控制／導航沒有使用真值。
- SCENARIOS 錄製範例增加 AMCL／plan，補終點驗證的失敗處理及真值驗收限制。

### 自行檢查

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：1049 passed、1 skipped（另有既有 xacro DeprecationWarning）。
- `scripts/check_static.py`：PASS（34 YAML/RViz、15 XML/SDF/xacro、148 Python、7 Bash、7 情境，含 ekf／ground_truth、2／3 錨點 xacro 展開）；`git diff --check` 通過。以上不視為容器／模擬驗收。

### 限制與下一步

- 未盲調 AMCL：資料顯示先接觸推移、再定位落後，沒有證據支持 beam skipping／量測權重能解決未被輪式里程計量到的滑移。
- 本輪先修全域標記，未新增停等協定或放寬 progress checker；人物持續占道時等待也不保證通過。既有 guard／watchdog 的零命令無法阻止行人推撞。新的終點檢查也無法偵測所有 AMCL 錯位，不能宣稱 narrow_pass 已修復或無接觸。
- 必須重新獨立錄製三情境，確認兩份 costmap 都標記／清除行人、路徑安全繞行或有界失敗、離開後恢復；保留原 0.20 m／0.20 rad 真值門檻、淨空與停止延遲要求。另驗證終點超差／缺失或過期 TF 回報 ABORTED，以及 default／alt_start／follow 障礙故障回歸。fork 功能也須另測，這批 bag 僅為 apt。

- 審查端調整：`zone_navigator` 的終點驗證容差改為參數 `arrival_xy_tolerance`（預設 0.25 m）、`arrival_yaw_tolerance`（0.35 rad）、`arrival_max_tf_age`（0.5 s）。Nav2 goal checker 為 0.15 m／0.25 rad，留餘量避免成功後的滑行與 AMCL 抖動把真正抵達判成 ABORTED，仍可擋下 narrow_pass 1.8 m 外的假成功。已合併 develop（PR #22，fork 成為預設 stack），pytest 1055 passed、1 skipped。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| map TF 暫失、LOS 獨立性與最新 TF 時效 | 待驗證 |
| NAV2 寬限、失敗冷卻與等待上限 | 待驗證 |
| 直接跟隨安全停止與側向減速 | 待驗證 |
| AMCL／Nav2 模擬故障檢查 | 待驗證 |

## 2026-10-27（t74／t76）：follow-los 模擬驗收與未合併原因

| 項目 | 結果（b6afb07） | develop 基準 |
| --- | --- | --- |
| follow_turns 距離誤差 RMSE 直線／轉彎 | 0.628／0.638 m（不再退步） | 0.636／0.672 m |
| follow_turns near_stop | FAIL @103.16 s：腳本人物路徑穿過機器人中心（真值距離 0.00 m @102.1 s），命令為 0、機器人被推（vx 0.011）→ 腳本人物假象，門檻不放寬 | PASS 5/5 |
| follow_stop_go | PASS（0.343／0.352） | PASS |
| fault-check（t76） | **FAIL 3/7**：uwb_stale／dropout／noise PASS；lidar_drop、lidar_freeze、odom_freeze、lidar_front 無移動基準 | 7/7 |

fault-check 根因：map→base_link TF 精確時間查詢暫時失敗（map_valid=False）即退出 NAV2，`recovery_pending` 因側向淨空 0.43–0.47 m < 0.75 m 永不解除，節點無期限保持 BLOCKED_WAIT，機器人不動；map TF 失敗時 LOS 也被誤判為受阻。

c627756（Codex t77）修正：LOS 與 map TF 分離、`map_invalid_grace` 0.5 s、`max_blocked_wait` 4.0 s，安全停止不變；指定離線測試 672 項通過。模擬驗證（c627756）：

| 項目 | 結果 | develop 基準 |
| --- | --- | --- |
| fault-check（t77） | **7/7 PASS**（與 t76 shop-dyn follow fault 同時執行，負載較高仍通過） | 7/7 |
| follow_turns（t78）距離誤差 RMSE 直線／轉彎 | 0.590／0.593 m，near_stop PASS 5/5 | 0.636／0.672，5/5 |
| follow_stop_go（t78） | PASS（near_stop PASS） | PASS |
| reacquire | NOT_COVERED（情境未涵蓋，與 develop 相同） | NOT_COVERED |

本輪 near_stop 未再出現腳本人物穿過機器人的假象，人物路徑未修改。驗證通過後合併進 develop。

下一步：加入涵蓋 reacquire 的跟隨情境；near_stop 若再出現人物穿過機器人中心，改腳本人物路徑（不放寬門檻）。
| 離線數字與根因交叉檢查 | 待驗證 |
| 全域雷射障礙標記與清除 | 待驗證 |
| 區域終點定位驗證與失敗處理 | 待驗證 |
| 三種動態情境、無人物與安全回歸 | 待驗證 |

## 2026-10-27：t71 fork 動態情境根因、行人讓行與定位信心檢查

### 修改與證據

- 依限制唯讀 t71 原始 SQLite／observations／report 與啟動日誌，分析腳本僅放 `/tmp`；沒有執行 ROS、Docker、Gazebo、容器建置、提交或推送。完整時序與參數依據見 [動態情境分析](DYNAMIC_SCENARIOS_REVIEW.md) 的 t71 節。
- narrow_pass：額外行人是 Fortress VelocityControl 實體圓柱，不是 set_pose。三次淨空小於 0.3 m 的區間約 106.84–112.08、129.58–136.22、152.84–156.98 秒；其中零命令樣本分別 241/263、304/333、192/208，零命令時真值速度仍達約 0.297 m/s。受推後 AMCL 落後約 0.92 m，再累積至 2.20 m；household 假成功誤差 2.396 m。入口回程駛過有限地板南界 y=−3 後墜落，最後平面誤差 13.114 m、z 約 −7565 m，AMCL 仍在目標附近。
- aisle_crossing：人物全程留在 x≈±0.45、y=3，TEB band 提前受阻；42 次 replan、8 次 progress failure，household 真值誤差 2.768 m，最低人物淨空 0.637 m。guard 沒有 STOP，AMCL 最大誤差僅 0.022 m。兩份地圖抽查有移動標記與清除；局部地圖轉回 map 後未見持續橫向殘影，不以清圖或放寬 progress checker 掩蓋占道。
- checkout_queue：t≈126–200 的 1,576 筆命令全零、guard 沒有 STOP。卡在距路徑終點約 0.13 m 的最後平移，尚未進 TEB 0.10 m 純旋轉階段；band 淨空約 0.173／0.190 m，無法越過 0.20 m 恢復門檻。不是旋轉命令振盪或 guard 阻轉；終點誤差 0.135 m／0.334 rad，38 次 replan、7 次 progress failure。
- 新增 `shopping_robot_sim/pedestrian_yield.py` 純邏輯；`pedestrian_mover.py` 使用帶時間戳的既有機器人模擬里程真值，行人預測線段遇機器人外接圓便停等、凍結排程後續走，缺資料也停止。真值只進模擬演員，不進導航或機器人控制。`scenario_launch.py`、sim／guide launch 提供預設 true 的 `pedestrian_yield`，false 還原舊不避讓行為。
- `scenarios.py` 允許行人 pauses；aisle_crossing 保留原穿越段，增加 y=5.8 的北側段及各 8 秒停留，使相遇段確實清空。自由循環約 40.7 秒，讓行會延長；這是情境版本變更，舊 t71 與新結果須分開比較，單改旗標不還原舊路徑。
- （已撤回）原提案把 `nav2_params_fork.yaml` 的 Slow／Fast 恢復遲滯改為 0.02 m；因使用者確認 checkout 在機器人上進行、`checkout_queue` 不在範圍內，維持 0.05 m。原提案內容：0.15 m STOP＋0.02=0.17 m，容許 t71 的 0.173 m 淨空解除停等；保留停止距離、footprint／前推／旋轉掃掠、速度／加速度、goal checker、guard 與評估門檻。未修改 fork 子模組。較小遲滯可能增加切換，必須模擬驗證。
- 新增 `shopping_zone_manager/localization_health.py` 純邏輯；`node.py` 抵達時檢查 AMCL 平面最大主軸標準差 ≤0.15 m、yaw 標準差 ≤0.35 rad、年齡 0…5 秒及 covariance 有效性。guide 的 map_server 模式要求 AMCL，SLAM-only 不強制；已有 AMCL 樣本的呼叫者同樣檢查。t71 household y 方差 0.02761 m²（標準差至少 0.166 m）、入口 x 方差約 1.70 m²，皆拒絕成功並回報 ABORTED。5 秒容許 AMCL 運動觸發更新，原 t71 household 最近樣本距結果 1.963 秒；不修改既有抵達 TF 容差 0.25 m／0.35 rad。
- 更新 [情境程序](SCENARIOS.md) 的行人規則、旗標、情境版本、錄製覆蓋與定位失敗處理；新增／擴充純邏輯、結果回呼、真值時間配對、launch 轉送、設定合約等 pytest。

### 自行檢查

- `/workspace/scratch/venv-shop/bin/python -m pytest -q`：**1091 passed、1 skipped、1 warning**，8.29 秒；警告為既有 xacro DeprecationWarning。
- `scripts/check_static.py` PASS：4 rosidl、37 YAML／RViz、15 XML／SDF／xacro、154 Python、7 Bash、7 情境；含 2／3 錨點與 ekf／ground_truth 的 xacro 展開。`git diff --check` 通過。
- 靜態檢查與 pytest 不代表容器、模擬或硬體驗收；本次沒有新增執行期模擬結果。

### 限制與下一步

- covariance 只能拒絕不確定的抵達；第一次受推時 y 方差僅約 0.00816 m²，仍可能低信心指標卻錯位，不能宣稱完整 kidnap 偵測。檢查只在結果成功時執行，沒有行進中自動取消、重定位或重試。
- 讓行不保證通行；迎面雙方停等可能仍有界逾時，沒有自動側移。新的穿越窗口與 TEB 恢復調整也尚未證明可通過。人物碰撞與真值評估保留；不得將不相遇、零命令或 action 成功視為無接觸／真值抵達。
- 維護者須另錄新 bag，驗證三情境、清空後恢復與 checkout yaw、AMCL 超限 ABORTED、人物缺真值停止、暫停後續走與 aggressive 模式，並獨立做 default／alt_start／無人物 nav-check／follow 障礙故障回歸。保留所有舊失敗與原位置、角度、淨空及停止延遲門檻。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| t71 時序、行人推撞、定位與墜落根因 | 待驗證 |
| 行人讓行、時間暫停／恢復與 aggressive 旗標 | 待驗證 |
| 穿越窗口、TEB 恢復與 checkout 最終旋轉 | 待驗證 |
| AMCL 信心檢查、假成功拒絕與生命週期 | 待驗證 |
| 新 bag 三情境與無人物／安全回歸 | 待驗證 |

## 2026-10-27（t76）：dynamic-scenarios f396c2d 模擬驗收

範圍調整（使用者 15:07）：機器人在較寬敞區域只做跟隨／帶路，結帳在機上完成，**checkout_queue 不在範圍內**（不再追 yaw 問題），narrow_pass 降為低優先；aisle_crossing 行人讓行與跟隨／帶路穩定為優先。fork `gentle_resume_margin` 維持 0.05。

環境：Linux amd64 Docker（shopnav2，ROS_DOMAIN_ID 31），fork TEB＋Smac2D（`nav2_stack:=auto`），AMCL＋`supermarket.yaml`，headless。bag：`bags/t76_{aisle,aisle_noyield,narrow}`（不入版控）。

| 項目 | t76 結果 | 對照 |
| --- | --- | --- |
| aisle_crossing（讓行開）household | ABORTED，真值誤差 0.584 m，人物最小淨空 0.321 m（無接觸） | t71 TIMEOUT 2.77 m |
| aisle_crossing（讓行開）entrance | SUCCEEDED 0.094 m | — |
| aisle_crossing（`pedestrian_yield:=false`）household | 回報 SUCCEEDED 但真值誤差 1.43 m、人物淨空 0（接觸、推移、AMCL 滑移；covariance 檢查未攔下）→ 假成功 | 證明讓行有效 |
| narrow_pass（低優先）household | ABORTED，2.78 m，人物淨空 0.342 m（無接觸、誠實失敗） | t71 假成功 2.40 m、接觸 |
| narrow_pass entrance | SUCCEEDED 0.074 m | t71 13.1 m |
| nav-check default | 6/6 PASS（區域容差 0.145 m） | develop 6/6 |
| nav-check alt_start | 6/6 PASS（0.119 m） | develop 6/6 |
| loc-check | PASS（六區，定位 RMSE ≤ 0.072 m） | develop PASS |
| obstacle fault-check | 4/4 PASS | develop 4/4 |
| follow fault-check | 7/7 PASS | develop 7/7 |

結論：無人物／安全回歸全部通過；動態情境沒有退步且安全性改善（無接觸、無假成功），但 aisle_crossing household 仍失敗。

根因（aisle_crossing）：互相等待死鎖。機器人停在 (0, 3.92)，行人在 (-0.45, 4.77) 讓行凍結約 124–160 s；雙方都等，Nav2 Shrink 後 ABORTED，行人在機器人離開後才繼續。

下一步：行人模擬加入耐心逾時（受阻超過數秒即沿路徑倒退讓出走道，真值仍只用於模擬行人），重跑 aisle_crossing；AMCL covariance 無法偵測推移後的錯位（noyield 假成功），需另加真值以外的一致性檢查（例如雷射與地圖匹配分數）。

## 2026-10-27（16:40 收尾）：本日合併與狀態

合併進 develop（均僅模擬驗證，未碰 main、未碰硬體）：

| PR | 內容 | 驗證 |
| --- | --- | --- |
| #23 → 45a17a2 | dynamic-scenarios：行人讓行（真值僅用於模擬行人）、aisle_crossing 清空窗口、AMCL 抵達信心檢查 | nav-check default／alt_start 6/6、loc-check PASS、obstacle 4/4、follow fault 7/7；aisle_crossing 無接觸但 household 仍 ABORTED（互等死鎖） |
| #24 → c983149 | follow-los：LOS 感知交接、map TF 暫失寬限、有界 BLOCKED_WAIT | fault-check 7/7、follow_turns RMSE 0.590／0.593 m、near_stop 5/5、stop_go PASS |
| #25 → 8f4b4f2 | 資料夾重整第 1–3、5 步（docs 分類、hardware/ 佔位、scripts 分類、README 瘦身→docs/USAGE.md） | pytest 1284 passed／1 skipped、check_static、bash -n、compose config、連結檢查、模擬 smoke（sim_health＋guide drinks SUCCEEDED） |

**自本節起路徑已變更**：文件在 `docs/design|hardware|research/`，腳本在 `scripts/container|nav2|eval|checks/`（`scripts/dev.sh` 不變）；本檔以上的舊路徑是歷史紀錄。

未做／下一步：
1. 重整第 4 步（docker/ 子資料夾）需要重建映像與 comm-e2e／nav-check，尚未做；第 6 步（ROS 套件分組）依使用者決定不做。
2. aisle_crossing 互等死鎖：模擬行人加入耐心逾時（受阻數秒後沿路徑倒退讓出），重跑情境。
3. AMCL covariance 抓不到推移後錯位（noyield 假成功），需加雷射／地圖匹配一致性檢查。
4. 跟隨的重新取得目標（reacquire）情境尚未涵蓋。
5. checkout_queue 不在範圍（結帳在機上完成）；narrow_pass 低優先。
6. 硬體 BOM、UWB 錨點與 MCU 型號待定（hardware/README.md）。

## 2026-10-27：依使用者要求關閉開發環境

- 已關閉本專案全部 Compose profiles（split／comm／record）的容器與網路，模擬、ROS 節點、RViz 及 noVNC 一併結束；另關閉本專案的瀏覽器分頁。
- 保留原始碼、映像、建置磁碟區、地圖、bag 與驗收紀錄；未操作其他專案或 6080。
- 自行檢查：Compose ps -a 無本專案容器。重新使用時執行 `./scripts/dev.sh up`，再依 SLAM 文件啟動流程。
- 審查驗證：待驗證。

- 使用者進一步要求關閉 Docker：已執行 `docker desktop stop`，並確認 Docker Desktop 為 stopped；重新使用容器前須先啟動 Docker Desktop。審查驗證：待驗證。

## 2026-10-27：Mac 上驗證 develop 707403e

### 範圍與環境

- 依使用者要求切回 develop，`git pull --ff-only` 更新至 `707403e`；Nav2 子模組同步至 `dd063030b9c4bee8beee1af0c9f572870c41550b`。保留本機 `.env` 的 6081／5970 與未追蹤 `.vscode/`；上輪關機筆記已保存並帶回。
- Docker Desktop 重新啟動，確認引擎為 linux/arm64；全程不固定 platform、不啟動硬體。
- 本次不建立 PR、不合併 main；驗證完成後，使用者進一步授權將 README 修正與驗證紀錄直接提交、推送至 develop。

### 自行檢查

| 項目 | 本次結果 | 本機證據（不入版控） |
| --- | --- | --- |
| Compose config | 本機 .env、新 checkout 的 .env.example 均解析通過 | 本輪工具紀錄 |
| Docker 完整建置 | compute／display／micro-ROS client 三種映像全部成功，均為 linux/arm64 | docker-build.log、images.json |
| Nav2 fork | 19 套件完成；110 項動態連結檢查 PASS；fork_commit 與 dd06303 相符 | nav2-build.log |
| ROS 工作區 | 清除 CMake cache 後，9 套件建置完成 | colcon-build.log |
| dev.sh test | 1285 passed、2 warnings；無失敗／略過 | pytest.log |
| colcon test | 923 tests、0 errors、0 failures、5 skipped | colcon-test.log |
| 靜態檢查 | YAML／XML／Python／Bash／launch、7 情境、2／3 錨點及 ekf／ground_truth xacro 展開 PASS | static.log |
| README 快速開始 | fish 4.0.2／Mac Bash 3.2 執行；mapping 15 項即時檢查 PASS、RViz OpenGL 4.5、noVNC HTTP 200 且以 README 密碼實際登入、圖形目視正常 | mapping-runtime.json、rviz.log、readme-rviz.png |
| 帶路模擬 | 預設 auto 實際選到新 fork（TEB＋Smac2D），線上建圖；18 項感測／TF／lifecycle／唯一 watchdog 檢查 PASS | guide-runtime.json、guide.log |
| 中文區域目標 | 300 秒上限下，飲料區／生鮮區皆 SUCCEEDED；獨立真值位置誤差 0.118／0.125 m、yaw 0.024／0.022 rad；壁鐘約 56／105 秒；閒置 cancel 回覆 IDLE | quickstart-guide-result.json、goto-*.log、truth-*.json、cancel-idle.log |
| 文件索引 | docs/README.md 的 17 個連結均開啟讀取；連同根 README 共 21 個本機目標有效 | docs-links.json、readme-links.json |

上述檔案位於 `output/develop-mac-20261027/`。本次帶路真值僅用於測試後量測，不送回定位或控制；位置與 yaw 檢查上限維持各 0.35 m／0.35 rad。

### 文件修正與限制

- README 將前景 RViz 與 novnc 查詢分成第二／第三個終端機，改以查詢所得位址為準；帶路明確等待 `Managed nodes are active`，生鮮區範例採原指令預設的 300 秒壁鐘上限。
- 原 120 秒生鮮區輪次仍有進度但逾時取消，紀錄保存為 deadline120-*。重測的第二輪約 105 秒成功；有路徑／時序變異，不能推論加長期限是唯一原因，也不保證每次都在 120 秒完成。
- 初次 HTTP 探針在圖形服務就緒前連線中斷，另一次 guide 探針在 lifecycle 啟用中讀到 bt_navigator 尚未 active；正式流程改為等待完整啟用後再測。早期探針與該次收尾紀錄保留，未以降低檢查條件換取通過。
- 退出仍有異常：mapping 收尾時 Gazebo 經 SIGINT／SIGTERM 未退出，launch 最後以 SIGKILL 結束（-9）；RViz 收尾出現 -11。畫面曾有 Mesa shader 初次連結錯誤，但之後實際顯示正常。這些尚未修復，不能宣稱圖形流程退出完全正常。正式帶路輪次沒有相同錯誤，容器最後已移除。
- 本次是指定的 Mac 原生建置／快速開始／基本帶路回歸，不是所有動態行人、跟隨、硬體或跨平台驗收。既有 aisle_crossing 互等死鎖等限制仍見上方既有紀錄。
- 僅修改 README 與本進度紀錄，保留 .vscode/；依後續授權提交並推送 develop，不開 PR、不合併 main。測試後已關閉測試環境與 Docker Desktop，保留映像、建置卷與紀錄。
- 下一步：釐清 Mac Mesa／RViz 退出 -11、Fortress 關閉卡住及長距離帶路時間變異；由使用者決定何時開 develop 到 main 的 PR。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| Mac arm64 原生完整建置 | 審查者已讀 7729cd1 差異與上表紀錄，內容一致；未在 Mac 重跑，結果以使用者 Mac 實測為準 |
| README 快速開始與單元測試 | 審查者已讀 README 差異，指令分終端機與等待 `Managed nodes are active` 合理；未重跑 Mac 單元測試 |
| 帶路模擬與文件索引 | 審查者未重跑 Mac 帶路與連結檢查；以使用者 Mac 實測紀錄為準。退出 -9／-11 列入待辦 |

待辦（2026-10-27 審查加入）：

- Mac mapping 收尾時 Gazebo 未回應 SIGINT／SIGTERM，最後被 SIGKILL（-9）；RViz 結束時 -11，曾出現 Mesa shader 初次連結錯誤。
- 長距離帶路壁鐘時間有變異（生鮮區 120 秒逾時、重測約 105 秒），需要釐清原因。

## 2026-10-27：codex/faster-follow 高速跟隨與速度前饋

- 基準為 develop `7729cd1`。依要求不使用網路、Docker、Gazebo 或硬體。
- 跟隨節點以 UWB 量測時間查詢 `odom → base_link`，接上 TargetVelocityEstimator；估測上限由 0.6 提高至 1.5 m/s。速度保持於 odom 座標，輸出時轉至目前車體座標，以車頭方向分量做前饋。TF 缺失／過期、無效量測、時間倒退及估速過期時前饋歸零；只有新鮮 CLEAR guard 狀態允許前饋，SLOW／STOP／STALE 均不允許。停用時重設估速，不讀取任何模擬真值主題。
- 預設前饋 0→0.9；跟隨距離 1.0→1.2 m、過近停止 0.6→0.7 m、kp_distance 0.7→1.0、線速上限 0.5→1.2 m/s、線加速度 0.4→0.8 m/s²、角速上限 1.0→1.5 rad/s。小於設定跟隨距離時禁止前進，避免前饋或原本減速斜坡持續逼近。過近鎖定、遺失、雷射失效、guard STOP／STALE 的停止流程保留；障礙停止的既有安全旋轉逃脫條件不變，不允許前進。
- mux／獨立 watchdog／Gazebo／MCU 模擬模型上限同步為 1.3 m/s、1.8 rad/s；一般加速度 0.8 m/s²、1.5 rad/s²。watchdog 對正負速度大小對稱夾限，倒車政策由 mux 管理；MCU 的產生式 `firmware/sim_client/contract.h` 同步重新產生。Gazebo 增加正加速度 0.8／1.5、負加速度 -2.0／-3.0，理想命令煞停由 1.2 m/s 需 0.6 秒，實際動力學尚未驗證。
- 前方固定障礙停止距離保留 0.65 m；下游 ObstacleGuard 保留 0.25 秒反應、停止與解除遲滯；本輪煞停掃掠及減速度調整如下。原有命令停止 ≤0.8 秒、真值停止 ≤1.5 秒等驗收門檻不變；提高速度後必須實測。
- fork 兩棵 BT 的預設 ControllerSelector 改為 Fast，保留 Slow 與原本主題選擇介面。Fast 線速 0.4→1.0 m/s、線加速度 0.4→0.6 m/s²、角速 0.8→1.5 rad/s；Slow 維持 0.3 m/s，兩者近人上限維持 0.15 m/s。apt RPP 0.4→0.8 m/s，follow／runtime role 傳入值同步 0.8；launch 覆寫上限 0.5→1.3。兩套 smoother 線／角速 0.5／1.0→1.2／1.5、線加減速度 ±0.4→±0.8；auto 選擇及 apt 隔離流程保留。
- 新增 `follow_fast_1_0`／`follow_fast_1_2`：南側開放區域、最長 7.5 m 直線、人物 1.0／1.2 m/s，無額外行人；沿用情境框架產生本輪唯一 person_path.yaml。SDF 每線段取 101 點，最小碰撞盒淨空 0.80 m；標準地圖佔用格中心最小距離 0.795 m。新增情境註冊、幾何回歸與繁中操作文件。
- follow_eval 增加 INFO 指標：設定人物速度、讀自跟隨設定的目標距離、初始 5 秒暖機後排除人物暫停的距離絕對誤差平均／RMSE，以及整輪最大真值間距。INFO 不影響既有 PASS／FAIL／NOT_COVERED 判定，未增加或放寬評估門檻。

- 本輪改善：guard 改為反應期間定速、之後線／角速度各自減速至零的煞停軌跡掃掠，保留足跡、取樣保守補償、旋轉圓盤、margin、STOP／STALE 與解除遲滯；線減速度 0.6→1.2 m/s²，仍小於 Fortress DiffDrive 的 2.0 m/s²，真實硬體必須重新量測。跟隨增加 `follow_time_headway=0.4 s`，有效距離為基礎 1.2 m 加上正向目標估速乘時距；另以 0.25 秒反應及 1.2 m/s² 減速度，將命令限制在前方雷射／目標距離扣除 0.7 m 後可煞停的速度，淨空縮小時立即夾限。SLOW 保留估速供時距計算，但仍停用速度前饋。基礎跟隨距離內零前進、過近／障礙／遺失停止、0.15 m/s 近人上限與不可繞過的下游 guard 保留。
- `follow_eval` 與 `person_sync_check` 共用新增 `--scenario NAME`，從情境 YAML 的 `person_path` 讀取速度與路徑；未指定時沿用共用預設路徑，與 `--person-path` 同時指定時拒絕歧義。INFO 列增加基礎跟隨距離、時距及按情境速率推算的移動目標間距；此推算值假設目標朝車頭方向行走，實際控制使用新鮮估測前向分量。原有誤差定義及所有 PASS／FAIL 門檻不變。
- 低優先轉角調整暫緩：保留 45° 進入／0.25 rad 解除原地轉向的遲滯與 `max(0, cos(bearing))`，避免同時改變貨架轉角安全行為；小於進入角度且未鎖定時仍可邊轉邊走。
- 模擬結果：待填

### 自行檢查與分析結果

- 本輪 `nice -n 10 /workspace/scratch/venv-shop/bin/python -m pytest -q`：1348 passed、1 skipped、1 個既有 xacro 棄用警告；新增正／反向煞停邊界、遠障礙全速、線角獨立減速的密集積分包覆、時距估速、淨空突然縮小、SLOW 時距但無前饋、情境路徑與 INFO、跨節點設定一致性測試。既有前饋／遲滯實驗固定時距 0，另測目前預設；高速比較扣除新增的預期間距，未修改驗收門檻。
- 本輪 `/workspace/scratch/venv-shop/bin/python scripts/checks/check_static.py`：PASS，33 YAML/RViz、15 XML/SDF/xacro、158 Python、7 Bash、9 情境，四種 xacro 展開與 TF／錨點契約通過；`git diff --check` 通過。未執行 Docker、Gazebo、網路或硬體驗收。

- 前輪 `nice -n 10 /workspace/scratch/venv-shop/bin/python -m pytest -q`：1305 passed、1 skipped、1 個既有 xacro 棄用警告。新增估速過期／無效／TF 失敗、目前朝向投影、距離內不前進、guard 零命令、高速閉迴路、資訊指標與 watchdog 獨立夾限測試。原有參數斷言同步更新。
- 前輪 `scripts/checks/check_static.py`：PASS，33 YAML/RViz、15 XML/SDF/xacro、157 Python、7 Bash、9 情境；2／3 錨點 × ekf／ground_truth 共四種 xacro 展開、TF 所有權及錨點一致性通過。新增跨套件速度、加速度、MCU／Gazebo／smoother／BT 與 0.15 m/s 近人上限契約檢查。
- `scripts/checks/map_geometry_check.py` 與 `git diff --check` 通過。`scripts/nav2/check_nav2_contract.py --source-only` 未完成：沙盒缺少 `/nav2_ws/src/custom_controller/src/teb_controller.cpp`，工作樹的 fork 子模組也沒有原始碼；未下載或假稱完成 fork 安裝／ABI 檢查。
- 前輪 ROS-free 60 秒、初始人物在車前 1.5 m、理想里程計、10 Hz UWB 濾波、20 Hz 控制，使用 `scripts/eval/follow_loop_sim.py --straight --person-speed SPEED --duration 60`。比較原有參數／mux 限速與目前預設；以下誤差從第 5 秒起計，最大間距包含起步：

| 人物速度 | 距離誤差平均：修改前→後 | 誤差 RMSE：修改前→後 | 最大間距：修改前→後 |
| --- | --- | --- | --- |
| 1.0 m/s | 17.052→0.102 m | 18.812→0.103 m | 31.802→2.150 m |
| 1.2 m/s | 23.552→1.230 m | 26.046→1.230 m | 43.802→2.430 m |

- 同一新參數但關閉前饋時，1.0 m/s 平均誤差約 1.000 m，加入前饋降至 0.102 m。1.2 m/s 已達機器人線速上限，前饋無法追回起步落差，開關前饋均約 1.230 m；未額外提高要求外的速度上限。
- 未解限制：舊 0.35 m/s 貨架路徑的簡化 `--world` 模型沒有 Nav2 繞行，新預設有效追蹤比例約 0.369，低於舊控制約 0.517；有效樣本誤差平均約 0.146 m／RMSE 0.205 m 不能抵銷低覆蓋率。原有低速實驗測試明確固定其歷史參數（含 1.0 m 跟隨距離、0.5 m/s 上限與實驗遲滯），保留原誤差／覆蓋率斷言；另測目前高速預設，未以放寬門檻隱藏此限制。
- 分析直線不在超市中，模型不含 Gazebo 動力學、下游障礙守門器、雜訊或 Nav2；新超市情境也含循環轉角，不能由分析直線推論其實測結果。yaw_rate_check 主程式需要 ROS 訂閱與已啟動的模擬，依本輪禁用 Gazebo 的要求未執行多速度掃描。
- 下一步：Fortress 多速度偏航掃描（包含提高後的工作速度）、由 1.2 m/s 執行既有故障停止驗收、兩個高速情境、貨架跟隨覆蓋率回歸，以及 apt／fork auto 的 guide／handoff 整合與原生 amd64／arm64 建置。保留所有失敗，不調低安全或評估門檻。

### 審查驗證

| 項目 | 結果 |
| --- | --- |
| 煞停掃掠、時距與情境評估修正 | 待驗證 |
| 估速前饋、停止狀態與參數契約 | 待驗證 |
| 高速情境、資訊指標與純 Python 比較 | 待驗證 |
| Fortress 偏航、煞停與 apt／fork 整合 | 待驗證 |
