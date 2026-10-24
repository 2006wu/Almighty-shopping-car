# 模擬評估指標

本文件定義 ROADMAP 階段 5 第 1 項的離線量測方式。實作位於 `shopping_zone_manager/eval_metrics.py`（純 Python、標準函式庫、不匯入 ROS），`eval_report.py` 僅負責檔案載入及報表。適用於 ROS 2 Humble、Gazebo Fortress 的模擬商店；rosbag 記錄與轉換流程見文末，仍沒有自動情境執行器。

## 共通規則

- 時間 `t` 為同一次模擬的模擬秒，所有資料使用相同 `/clock`／`use_sim_time`；重設模擬時間後另開紀錄。位置以公尺、yaw 以弧度、線速度以公尺／秒、角速度以弧度／秒表示。
- 同一資料流的時間嚴格遞增，重複或倒退時間一律 `ValueError`，不自動排序。混合紀錄整體可有相同時間（不同流），但不可倒退；導航每個 episode 各自檢查。
- 必填數值拒絕 NaN、正負無限大、布林及非數字；不丟棄壞樣本或自動補值。缺資料使用空資料流；人物座標可成對省略，不可以 NaN 表示缺值。
- 空輸入回傳樣本／事件數 0、無法定義的統計為 `None`，JSON 為 `null`，Markdown 為「無資料」。單一樣本的觀測期間為 0，時間比例未定義。這些結果不表示通過。
- 平均、RMSE、中位數與 p95 均按樣本加權。排序後的 p95 在索引 `0.95 × (n−1)` 的兩側值間線性內插；不採最近名次法。時間比例與持續時間則採左端值保持：樣本 i 適用於 `[t_i, t_(i+1))`，末筆沒有額外持續時間，不延伸到紀錄之外。
- 不做座標轉換、時間同步或插值。車體位姿、人物位姿與目標必須事先對齊座標，人物座標須與該筆車體位姿同時。靜態碰撞盒使用 SDF 世界座標，因此 `pose` 流須轉入模擬世界；不得直接混用漂移的 map／odom 座標。

## 導航回合

`navigation_metrics(episodes)` 接受字典序列，每個字典有 `id`、布林 `success`、`goal=(x,y,yaw)`、`samples`；每筆樣本為 `{t,x,y,yaw}`。回合第一筆必須對應目標送出、末筆對應終態（成功、失敗、取消、拒絕或逾時），由後續錄製／轉換工具保證邊界取樣。未結束的回合應先補記測試截止逾時後再評估，不可只匯入成功回合。

| 輸出 | 精確定義 | 單位 |
| --- | --- | --- |
| `success_rate` | `success=True` 回合數／全部回合數；以 NavigateToPose 的 SUCCEEDED 結果標記成功，失敗、取消、拒絕及逾時均為 false | 0–1 |
| `time_to_goal` | 僅成功且有位姿的回合，末筆 t − 首筆 t；彙整 count、mean、median、p95、max | 秒 |
| 每回合 `duration` | 所有結果均回報末筆 t − 首筆 t；失敗耗時不混入成功抵達時間 | 秒 |
| `path_length` | 相鄰車體中心平面位置距離的總和，不計原地旋轉 | 公尺 |
| `path_efficiency` | 首筆位置到要求目標的直線距離／`path_length`；路長為 0 時 null | 無因次 |
| `final_position_error` | 末筆車體中心與要求目標的平面歐氏距離 | 公尺 |
| `final_yaw_error` | `abs(atan2(sin(goal_yaw−yaw), cos(goal_yaw−yaw)))`，範圍 0–π | 弧度 |

路徑效率不是最短可行路徑效率，繞貨架本來就可能偏低；尚未抵達或提早宣告成功時也可能大於 1，保留原值以揭露問題，不截斷。空位姿回合仍納入成功率分母及標記成功數，但幾何量與耗時為 null；錄製完整性須另外檢查，不得以此宣告驗收。

## 足跡最小間距

`clearance_metrics(records, boxes, length=0.6, width=0.5, person_radius=0.25)` 的車體為以 base_footprint 為中心、長軸沿車體 x 的矩形，隨 yaw 旋轉。碰撞盒格式為 `(xmin,ymin,xmax,ymax)`；長、寬、人物半徑均可調整，必須大於 0。

`load_store_boxes(world_path)` 直接讀取既有 `shopping_robot_gazebo/worlds/supermarket.sdf` 的實際 collision，來源與 `test_store_layout.py` 一致；包含貨架、牆、結帳櫃臺及不對稱地標，排除地板和人物。不另存障礙座標。只接受目前場景的一個 link／一個軸對齊靜態碰撞盒；未支援的旋轉、link／collision pose、相對框架、巢狀模型或 include 會拒絕，避免場景改動後低估風險。

- `static_min`：所有車體樣本與靜態碰撞盒之間的最小歐氏邊界間距。以分離軸檢查相交，再取頂點到邊線段的最短距離，支援旋轉、邊交叉及包含。
- `person_min`：人物中心轉至車體局部框架後，到矩形的距離減去人物半徑；小於 0 截為 0。
- `minimum`：以上兩者可用資料的最小值。所有接觸與重疊均為 0，不以負值表達穿透深度。
- `pose_count`、`static_sample_count`、`person_sample_count` 揭露覆蓋程度；缺人物資料時只計靜態障礙，不能宣稱已驗證人物安全間距。

以上均為公尺及離散樣本最小值；未檢查相鄰位姿間的掃掠碰撞，也未計入雷射誤差、定位誤差、車體高度或未提供真值的動態障礙。後續情境測試須提高取樣率並另做連續碰撞檢查。

## 跟隨距離與狀態

`follow_metrics(records, target=1.0, band=0.2, too_close=0.6)` 接受 `{t,distance}`。`distance` 為同時刻人物真值中心與車體中心的平面距離，必須非負；它不是足跡邊界間距，也不是 UWB 位置估測誤差。預設目標對應目前 `uwb_follow.yaml` 的 `follow_distance`，變更控制參數時需同步傳入 CLI。

| 輸出 | 精確定義 | 單位 |
| --- | --- | --- |
| `mean_error` | `mean(distance−target)`，有號偏差，正值表示落後較遠 | 公尺 |
| `mean_absolute_error` | `mean(abs(distance−target))` | 公尺 |
| `rmse` | `sqrt(mean((distance−target)²))` | 公尺 |
| `p95_absolute_error` | `abs(distance−target)` 的 p95 | 公尺 |
| `time_fraction_in_band` | `target−band ≤ distance ≤ target+band` 的左端保持區間總秒數／首末樣本期間 | 0–1 |
| `too_close_count` | 進入 `distance ≤ too_close` 的連續區段次數；首筆已過近也計一次，末筆新進入也計一次 | 次 |
| `too_close_duration` | 過近區段的左端保持總秒數；紀錄尾端截斷，不外推 | 秒 |

`state_metrics(records)` 另接收 `{t,state}`，依精確大寫字串判定：首次 `LOST` 開啟事件，後續重複 `LOST` 不重設起點；首次 `FOLLOWING` 結束事件。中間的 IDLE、TOO_CLOSE 或 OBSTACLE_STOP 等其他狀態不算恢復。輸出 `lost_count`、`reacquire_count`、`pending_lost_count` 與完成事件的 `reacquire_time`（mean／median／p95／max，秒）。紀錄開始即 LOST 算一次，結尾未恢復則 pending=1，不混入完成時間。遺失時間描述本次可見區間，無法推知紀錄開始之前的真正遺失起點。

距離與狀態為獨立資料流。LOST 期間若無真值距離，距離統計僅代表實際輸入的有效觀測，不可跨缺資料長時間保留舊距離來美化比例；後續情境需完整記錄真值並另核對資料覆蓋率。

## 停止延遲

`stop_metrics(triggers, commands, odometry, linear_threshold=0.02, angular_threshold=0.05, zero_tolerance=1e-9)` 接受觸發 `{t,event}`（event 在純邏輯 API 可省略）及速度 `{t,linear,angular}`。`linear` 為平面線速度大小或差速底盤的有號前進速度，判定時取絕對值；`angular` 為有號 yaw 角速度。

- 命令停止：從觸發時刻起，首次觀測到 `abs(linear) ≤ zero_tolerance` **且** `abs(angular) ≤ zero_tolerance` 的 `/cmd_vel` 樣本，時間減觸發時間。
- 實際停止：從觸發時刻起，首次觀測到 `abs(linear) < linear_threshold` **且** `abs(angular) < angular_threshold` 的 `/odom` 樣本，時間減觸發時間。
- 觸發同時已停止則為 0；不沿用觸發之前的零速樣本、不內插、不要求持續停止時間。不等同於煞停距離或穩定停止保證。
- 每事件輸出 `commanded_latency`、`measured_latency`（秒），另有完成事件統計及 `commanded_pending`／`measured_pending`。紀錄結束前未看到符合值則 null，不當成 0，也不納入完成平均。
- 多次觸發各自尋找後續首個停止，可共用同一停止樣本，不推論因果；建議情境中每次注入後等待完成停止再啟動下一次。整筆紀錄結束是觀測截止，不自動以另一觸發截斷。

`OBSTACLE_STOP` 目前允許原地旋轉，單靠線速度為 0 不算零 Twist。需要完整停止的障礙情境應檢查兩軸及雷射失效路徑；只停止前進的既有行為可能無法通過完整停止門檻，應如實列出，不能靠放寬角速度欄位隱藏。

## 建議驗收門檻與後續情境

下列為模擬商店的**提案**，不是既有實測通過結果；CLI 輸出數值與缺資料狀態，不自動宣告通過。所有情境須附完整紀錄與事件數、取樣頻率、參數、世界版本。失敗及 pending 事件不可省略。

| 指標 | 提案門檻 | 後續情境與判讀 |
| --- | --- | --- |
| 導航成功率 | ≥ 95%；至少 100 回合 | 六區域往返、不同起始位姿、SLAM／AMCL 分別統計；拒絕／取消另設預期結果情境，不混入正常路線效能批次 |
| 導航耗時 | 每回合 ≤ 60 秒；成功回合 p95 ≤ 45 秒 | 三走道、入口、結帳區路線分組比較；失敗逾時也須列出 duration |
| 路長／效率 | 路長逐回合留存；正常成功回合效率 ≥ 0.5 作初步檢視 | 直行、跨貨架繞行分組；效率偏低可能是必要繞行，非獨立安全通過條件 |
| 最終位置／朝向誤差 | ≤ 0.20 公尺／≤ 0.20 弧度 | 六區域抵達，以對齊目標框架的真值核對，不能只看 action 成功 |
| 最小間距 | 靜態及人物均 > 0.10 公尺，接觸次數為 0 | 窄走道、轉角、結帳區、人物穿越；人物缺資料不可驗收動態間距 |
| 跟隨誤差 | RMSE ≤ 0.25 公尺；p95 絕對誤差 ≤ 0.40 公尺；±0.20 公尺帶內時間 ≥ 80% | 沿唯一 `person_path.yaml` 路徑直行／轉彎分組，噪聲、NLOS、掉訊條件分開報告 |
| 過近事件 | 正常路徑 0 次；故障注入逐次列出次數及持續秒數 | 人物突然靠近，另以觸發與停止延遲驗證保護；人物持續主動接近造成的過近不等於車體仍在移動 |
| 遺失／恢復 | 受控重新出現後須恢復；總重新取得時間暫定 ≤ 2 秒，pending=0 | 掉訊、UWB 過期、重新出現；目前時間從 LOST 開始，包含注入的不可見時間，測試設計須限制該時間並另保存重新出現時刻 |
| 命令／實際停止延遲 | 每事件 ≤ 0.30／≤ 0.80 秒，兩種 pending 均為 0 | 過近、遺失、UWB 過期、障礙、雷射失效、急停、命令逾時分開注入；後兩項待模式管理器／看門狗實作 |

停止驗收建議命令與里程計至少 20 Hz，記錄觸發前移動及觸發後至少 2 秒；離散結果仍可能有一個取樣週期的不確定性。重新取得及安全停止應跨正常追蹤、轉彎與前方障礙場景，所有控制流程維持互斥。本文件不修改控制器，也不代表 ROS／Gazebo、跨架構或硬體驗收。

## CSV／JSONL 格式與命令

每列是單一資料流的一筆觀測，CSV 共用欄位 `type,t`，其餘使用下表欄位的聯集；不適用欄位留空。JSONL 每行一個物件，可以省略不適用欄位。UTF-8 編碼、CSV 使用標準引號規則；未知 type 拒絕，額外欄位不參與計算。

| type | 必填欄位（除 type,t） | 可省略欄位／用途 |
| --- | --- | --- |
| `navigation` | `episode,success,goal_x,goal_y,goal_yaw,x,y,yaw` | 同回合每列重複相同目標及最終結果；episode 為非空字串，success 為 JSON 布林或精確 `true`／`false` 字串，CSV 用小寫字串 |
| `pose` | `x,y,yaw` | `person_x,person_y` 成對提供，僅這個資料流計算間距；導航樣本不自動合併 |
| `distance` | `distance` | 真值中心跟隨距離 |
| `state` | `state` | 精確大寫狀態字串 |
| `cmd` | `linear,angular` | 最終底盤命令，包含旋轉 |
| `odom` | `linear,angular` | 實測速度，包含旋轉 |
| `trigger` | `event` | 觸發原因字串，例如 estop、too_close、obstacle、lost、uwb_stale、laser_stale、command_timeout |

最小 CSV 範例（完整導航資料還須依上表增加欄位）：

```csv
type,t,distance,state,linear,angular,event
distance,0,1.2,,,,
state,0,,FOLLOWING,,,
trigger,1,,,,,estop
cmd,1.2,,,0,0,
odom,1.6,,,0.01,0.02,
```

JSONL 導航範例（兩列同回合，離線補記最終結果）：

```json
{"type":"navigation","t":0,"episode":"trip-1","success":true,"goal_x":1,"goal_y":0,"goal_yaw":0,"x":0,"y":0,"yaw":0}
{"type":"navigation","t":5,"episode":"trip-1","success":true,"goal_x":1,"goal_y":0,"goal_yaw":0,"x":1,"y":0,"yaw":0}
```

在儲存庫根目錄可直接使用沙盒 Python，不需 ROS 或 Docker：

```bash
PYTHONPATH=ros2_ws/src/shopping_zone_manager \
  /workspace/scratch/venv-shop/bin/python -m shopping_zone_manager.eval_report \
  /tmp/run.jsonl \
  --world ros2_ws/src/shopping_robot_gazebo/worlds/supermarket.sdf \
  --output /tmp/evaluation
```

套件安裝後同樣參數可用 `eval_report`（console_scripts）執行；世界檔使用明確路徑，不依賴 ament 或 ROS 套件搜尋。輸出前綴會附加 `.json` 及 `.md`（上例為 `/tmp/evaluation.json`、`/tmp/evaluation.md`），目錄須已存在，同名前綴會覆寫。JSON 包含完整每回合／每事件資料、參數及來源路徑；Markdown 以欄位路徑列出相同資料。

可調參數：`--target`、`--band`、`--too-close`、`--length`、`--width`、`--person-radius`、`--linear-threshold`、`--angular-threshold`，預設值如上。錯誤輸入回傳非零結束碼；輸入驗證在寫入報表前完成。

## 後續錄製介面的預期對應

此表為來源介面契約；目前 bag 轉換器採用的子集與限制見下一節。使用不同框架時，轉換器必須先用 TF 對齊；不得由本模組猜測座標。

| 預期來源 | 紀錄欄位 | 注意事項 |
| --- | --- | --- |
| NavigateToPose 目標、結果與對齊的車體位姿 | `navigation` 的 episode、success、goal_*、x/y/yaw | 一次動作一回合，保留取消／拒絕／失敗／逾時；以目標送出到終態的姿態邊界取樣，座標須一致 |
| `/odom` 的 pose，或 `/simulation/poses` 的車體真值 | `pose` 的 x/y/yaw；導航另轉入目標框架 | 預設 `/odom` 為 EKF 估測；真值須使用 `/sim/ground_truth_odom` |
| `/uwb/person_ground_truth` 或 `/simulation/poses` 的人物真值 | `pose` 的 person_x/person_y，並計算 `distance` | 轉成與車體相同的世界框架並同步；解析人物路徑真值與 Gazebo 模型實際到位可能有時間差 |
| `/follow/state` | `state` | 使用接收的模擬時間，保留狀態變化與取樣終點 |
| 最終 `/cmd_vel` 的 linear.x／angular.z | `cmd` 的 linear/angular | 輸入為底盤最終命令；Nav2 上游 `/cmd_vel_nav` 不代表底盤實際收到的命令 |
| `/odom` 的 twist | `odom` 的 linear/angular | planar linear 使用 hypot(vx,vy)，angular 使用 wz；不要把 cmd 當量測速度 |
| 故障注入器、急停、看門狗及狀態轉換 | `trigger` 的 event | 需獨立標記實際觸發時刻；若從狀態字串產生，延遲只代表狀態公布之後的反應，不包含狀態偵測之前的延遲 |
| `/clock`、訊息 header、`/tf` | t 及離線框架對齊 | 無 header 的狀態／命令使用模擬接收時間並記錄其限制；重啟與倒退時間必須分檔 |

UWB 估測位置誤差仍使用既有 `shopping_uwb_follow/follow_metrics.py` 與 `/uwb/estimate_error`；本指標的跟隨距離誤差評估控制結果，兩者應分開呈現。後續情境變化須由 Gazebo 套件的情境載入器解析，本次執行人物模型與 UWB 共用一份路徑檔（default 仍為 `config/person_path.yaml`）。

## rosbag 記錄、轉換與可重現流程

`dev.sh record NAME [--duration S] [額外 topic...]` 在 compute 使用 Humble `ros2 bag record --storage sqlite3 --use-sim-time`。`S` 是牆鐘秒數，省略時以 Ctrl-C 結束；到期送 SIGINT 並等待 SQLite 與 `metadata.yaml` 寫完。不要在同一次錄製重設模擬時間。資料位於主機 `bags/NAME/`，既有名稱拒絕覆寫。清單唯一來源為 `shopping_zone_manager/config/record_topics.yaml`，由 `scripts/record_bag.py` 載入；測試另外核對各 topic 在實際程式或 bridge 中存在。

| 固定 topic | 實際型別 | 轉換用途 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 模擬接收時間、偵測時鐘倒退 |
| `/sim/ground_truth_odom` | `nav_msgs/msg/Odometry` | 世界位姿、真值速度、導航路徑與邊界 |
| `/odom` | `nav_msgs/msg/Odometry` | 保留估測里程計供診斷；不代替真值 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 最終命令 `cmd`，非上游 Nav2 命令 |
| `/uwb/person_ground_truth` | `geometry_msgs/msg/PointStamped` | 依唯一 `person_path.yaml` 產生的人物路徑真值 |
| `/uwb/target` | `geometry_msgs/msg/PointStamped` | 保留車體座標估測供診斷；不代替人物真值 |
| `/follow/state` | `std_msgs/msg/String` | `state` 與保護狀態進入事件 |
| `/shopping/zone_status` | `std_msgs/msg/String` | 區域導航回合生命週期 |
| `/mode_manager/state` | `std_msgs/msg/String` | ESTOP 進入事件 |
| `/sim/fault_status` | `std_msgs/msg/String` | 僅故障模式存在；保留注入狀態供診斷 |

這些流程不需 `/scan`、`/tf`、`/tf_static` 或 action feedback；額外錄製的 topic 僅保留於 bag，不自動成為指標。`/sim/fault_status` 不是保護偵測事件，不直接拿來算停止觸發。此工具沒有新增控制、硬體連線或故障注入。

### 時間與同步

薄層 `bag_to_eval.py` 使用 Humble `SequentialReader`、sqlite3 與 CDR，將訊息轉成普通字典；有頂層 header 時採 header stamp（含零秒），否則採 bag 接收時間。因此外部 bag 也必須用 `--use-sim-time` 錄製，不能混入系統時間。`/clock` 倒退立即拒絕；不含 `/clock` 的外部檔案無法保證偵測重設，須由使用者確保單次模擬。轉換不修改 TF 或 ROS 時鐘。

純模組 `bag_converter.py` 先依來源時間排序，處理接收順序與 header 順序不同；同 topic 的真值時間重複拒絕。沒有 header 的命令／狀態可能共用 recorder 的同一個時鐘 tick，因此其他 topic 同時刻採最後一筆，合併數量列於 `.meta.json`；無法量測 tick 內的先後延遲。區域導航生命週期保留同 tick 的原始先後順序。狀態來源同時觸發時合併原因為一筆 `trigger`，以符合既有指標的嚴格時間規則。

人物與車體都須為 `sim_world`。車體真值取樣時，人物位置以左右兩筆線性內插對齊；導航起訖邊界同樣內插車體真值，yaw 使用最短角差。每側時間差不得超過 `--max-skew`（預設 0.15 秒），不外推。`pose` 永遠使用車體真值；人物無法配對時省略成對欄位並統計缺樣數。有 follow 狀態才產生 `distance`；開頭／結尾未配對區間不延伸，內部配對缺口或距離取樣間隔超過兩倍同步上限會拒絕轉換，須分割評估區間，避免距離指標跨缺口保留舊值。這是對原始列格式要求「先同步」的前處理，原有評估器仍不自行插值。

### 可還原與不可還原的觀測

- `navigation`：使用 `PENDING: Requested NAME` 建立回合，目標讀取當次保存的 zones YAML；SUCCEEDED 為成功，ABORTED／CANCELED／REJECTED／TIMEOUT 為失敗。ACTIVE 的進度文字不另開回合。沒有終態時，在最後車體真值時刻補記失敗截止，識別碼列入 `cutoff_episodes`；不是宣稱 Nav2 回傳失敗。缺少邊界真值、未知區域或重疊回合立即報錯。錄製中途才看到 ACTIVE／CANCELING 也會報錯；孤立終態只列入 `ignored_zone_statuses`，不可拿這類 bag 計算完整成功率。UNKNOWN_ZONE／UNAVAILABLE 在目標送出前即失敗，沒有可對應目標，不產生回合。RViz 直接目標、跟隨的 Nav2 handoff 及其他 action client 沒有區域狀態，不能從此清單還原其導航回合。
- 地圖目標要先以明確 `--map-world-transform X Y YAW` 轉入真值世界。轉換器沒有 TF 推算功能；`eval-bag` 固定傳入 `(0,0,0)`，僅適用目前世界起點、初始 yaw 與 map 原點一致的固定程序。自訂地圖、重新定位或變更起點必須自行提供經驗證的剛體對齊，並使用手動轉換命令。參數保存在轉換中繼資料；估測漂移不會拿來修正世界真值。
- `pose`／`distance`：車體矩形間距與真值中心跟隨距離；人物資料是路徑模型真值，仍非 Gazebo 人物模型實際到位量測。`/uwb/target` 是 base_link 估測，這裡不計 UWB 位置估測誤差。
- `state`：取冒號前的大寫狀態，保留每次取樣。`trigger` 由進入 LOST／TOO_CLOSE／OBSTACLE_STOP 或 ESTOP 產生；重複同狀態不重開事件。LOST 無法區分 UWB 過期或掉訊；OBSTACLE_STOP 無法區分障礙、雷射失效或里程計保護。延遲從狀態發布後開始，不含故障注入到偵測的時間；首次觀測已在保護狀態也計一次。
- `cmd` 為最終 `/cmd_vel`；`odom` 列刻意使用 `/sim/ground_truth_odom` 的平面線速度與 yaw 角速度，以免 EKF 或故障凍結影響實際停止量測。無命令或真值時保留 pending，不補零。獨立看門狗目前沒有逾時狀態 topic，因此無法還原 `command_timeout`；沒有獨立事件的雷射／UWB 過期也不能聲稱量到各自觸發延遲。

空 bag 可輸出空 JSONL，報告為無資料，不表示通過。`.meta.json` 保存來源、輸入 topic 數量、同步上限、最大車體真值取樣間隔、缺樣、同時間合併、截止與孤立狀態數；必須和 JSON／Markdown 報告一起保存。工具目前會在記憶體載入及排序一次紀錄，適合固定時長評估，不適合無上限長期錄製。

### 固定程序、回放與比較

1. 保存 Git 版本、使用的 zones YAML、人物唯一 `shopping_robot_gazebo/config/person_path.yaml`、控制參數、世界檔及 launch 參數。每次重新啟動模擬，使用相同預設起點與 `odom_source:=ekf`、`use_mux:=true`；不可在錄製中重設模擬時間。
2. 導航：終端機 A 執行 `./scripts/dev.sh guide`，等待定位及 Nav2 就緒；B 執行 `./scripts/dev.sh record guide1 --duration 120`。確認錄製就緒後，C 依序執行 `./scripts/dev.sh goto drinks --timeout 30`、`goto snacks --timeout 30`、`goto entrance --timeout 30`，每次等前一筆結束。若到期仍未結束會列為失敗截止，不能排除重跑以美化成功率。
3. 停止 guide 後重新開模擬：A 執行 `./scripts/dev.sh follow`，B 執行 `./scripts/dev.sh record follow1 --duration 120`。人物僅沿既有 `person_path.yaml`，不另建路徑來源。保存錄製起始模擬秒數；人工啟動不能保證人物相位完全相同。自然情境可能沒有保護事件，事件數 0 不是停止驗收。故障比較需另固定既有注入排程並保存參數。
4. 等 recorder 結束，再執行 `./scripts/dev.sh eval-bag guide1` 與 `./scripts/dev.sh eval-bag follow1 --target 1.0`。報表位於對應 bag 目錄。也可直接在 compute 執行：

   ```bash
   ros2 run shopping_zone_manager bag_to_eval /repo/bags/guide1 \
     --zones /ws/src/shopping_zone_manager/config/zones.yaml \
     --map-world-transform 0 0 0 --output /repo/bags/guide1/observations.jsonl
   ros2 run shopping_zone_manager eval_report /repo/bags/guide1/observations.jsonl \
     --world /ws/src/shopping_robot_gazebo/worlds/supermarket.sdf \
     --output /repo/bags/guide1/report
   ```

5. 回放僅在停止所有模擬、Nav2、跟隨及其他底盤命令來源後，於獨立 ROS domain 使用 `ros2 bag play /repo/bags/guide1`。bag 已錄 `/clock`，不要另加 `--clock` 產生第二個時鐘；也不要與現場 Gazebo 並播。此回放用於觀察已錄資料，離線轉換不需啟動任何 ROS 節點，也不會重送導航目標。
6. 以相同程序產生 guide2／follow2，先比對 `.meta.json` 的涵蓋程度、缺樣、截止回合及報告 parameters，再按相同區域順序比較 success_rate、逐回合 duration／path_length／final_position_error，以及跟隨 RMSE、帶內時間比例與停止 pending／每事件延遲。可用 `diff -u bags/guide1/report.json bags/guide2/report.json` 檢視完整差異（source 路徑必然不同）。不同資料覆蓋率不可只比平均數。

Gazebo 排程、DDS 傳遞、Nav2 規劃、錄製起點與人物相位仍有非決定性；固定種子不保證逐位元相同。應重複多次並保留失敗，這份程序不是自動通過判準，也不代表容器或硬體已驗收。

## 模擬情境與動態行人

七種情境與逐步操作見 [模擬情境驗收程序](SCENARIOS.md)。每個情境獨立錄製，保存名稱、YAML、人物路徑、程式版本、啟動參數、目標及送出時的 `/clock`。行人延遲與主要人物軌跡採世界模擬秒；DDS 啟動或人工送目標時機仍可能影響相遇，沒有相遇的紀錄不能算穿越／讓行驗收通過。

固定錄製清單新增 `/sim/pedestrians`（`geometry_msgs/PoseArray`）。轉換器要求 `sim_world` 與固定非空人數／順序，拒絕重複真值時間；各行人以最多每側 0.15 秒的有界內插配對車體真值，不外推。JSONL `pose` 新增 `pedestrians: [{x,y,radius}, ...]`，同時計算主要人物及所有行人的邊界間距，`person_min` 取其中最小值；`person_sample_count` 計算具有至少一個人物的車體樣本數。主要人物跟隨距離與停止事件維持原定義，動態行人不成為跟隨目標。PoseArray 不帶半徑，提供的七種情境皆為 0.25 公尺；自訂尺寸要以 `bag_to_eval --pedestrian-radius R` 指定保守的最大半徑，不可直接沿用固定包裝指令。

轉換器 `.meta.json` 的 `missing_pedestrian_samples` 記錄有行人 topic 但無法完整同步的車體樣本。需確認 `input_counts` 有該 topic、人數符合情境，且驗收區間沒有缺樣；完全缺 topic 時數字仍可能是 0，不能當作有覆蓋。首尾缺樣應明確裁切與註記，內部缺樣不得宣告安全通過。仍採離散幾何間距，未增加接觸感測器或連續掃掠碰撞偵測；PASS 至少要求靜態及人物最小間距皆 > 0，建議沿用 > 0.10 公尺的較嚴門檻。

`eval-bag` 的固定 `--map-world-transform 0 0 0` 與 `loc-check` 直接比較真值，都假設標準地圖的 map 與 Gazebo `sim_world` 原點、朝向一致。出生姿態不是 map→world 轉換：`alt_start` 仍使用同一標準地圖，EKF 初始化在世界中的 `(-2.4,0,π/2)`，AMCL 及定位工具也使用此 map 姿態。新建圖或自訂地圖須先核對座標；不一致時手動呼叫 `bag_to_eval --map-world-transform X Y YAW`，`loc-check` 目前沒有自訂轉換功能，不可直接用於該地圖。

導航案例每目標須在 90 秒內 SUCCEEDED，且有真值核對最終位置；遭行人擋住時需觀察減速、等待或安全繞行與行人離開後恢復。`follow_stop_go` 起始人物中心距離 0.58 公尺，低於既有 0.60 公尺過近門檻且高於矩形前緣加人物半徑的 0.55 公尺接觸距離；應記錄 TOO_CLOSE 與零 Twist，再觀察人物離開後恢復。初始已停不能代替行進中的煞停驗收，須另執行既有 `fault-check`；障礙、過近及急停驗收均須檢查 linear 與 angular。未調整控制器，既有行為若未達門檻應記為失敗。

## 障礙防護觀測（階段 4）

預設錄製清單新增 `/safety/obstacle_state` 與 mux／guard／collision_monitor 的中間速度。bag_converter 取 guard JSON 內的模擬時間 `t`，在進入 STOP／STALE 時產生 `obstacle_guard_stop`／`obstacle_guard_stale` trigger，重複狀態不重開事件，沿用本文件 `stop_metrics` 的首次零命令／真值停止定義。同時出現的其他保護事件沿用既有合併規則。啟動即 STALE 也會產生事件；須保留它並另判讀是否有移動基線，不能拿啟動零速作為障礙停止驗收。

狀態事件的延遲起點是 guard 偵測並發布狀態的模擬時間，不包含故障命令傳送到偵測的延遲。`scripts/fault_check.py --suite obstacle` 則以送出故障命令前的模擬時間開始計算，且要求移動基線、持續停止與恢復；與本文件「首次停止」統計不可混稱同一判準。減速列檢查 guard SLOW 比例與命令仍在移動，沒有自動辨識真值減速度。

`/safety/obstacle_state.swept_clearance` 是雷射回波到要求速度掃掠範圍的保守距離，用於診斷；不取代本文件的真值矩形最小間距。既有 `clearance_metrics` 仍以世界 collision 與真值位姿計算，沿用原有錄製／轉換鏈路。`lidar_front` 假障礙沒有新增 Gazebo collision，因此真值最小間距不會包含它；不能用假障礙試驗宣稱通過實際障礙距離驗收。完整啟動、故障與 headless 程序見 [OBSTACLE_SAFETY.md](OBSTACLE_SAFETY.md)。

## 可重現評估（manifest、驗證、彙整）

在已啟動的 compute 容器中依序執行（此處未代為啟動容器）：

```bash
./scripts/dev.sh record guide1 --duration 120 \
  --meta scenario=guide --meta nav2_stack=stock \
  --meta 'launch_args=odom_source:=ekf use_mux:=true'
./scripts/dev.sh eval-bag guide1 --target 1.0
./scripts/dev.sh eval-verify guide1
./scripts/dev.sh record guide2 --duration 120 \
  --meta scenario=guide --meta nav2_stack=stock \
  --meta 'launch_args=odom_source:=ekf use_mux:=true'
./scripts/dev.sh eval-bag guide2 --target 1.0
./scripts/dev.sh eval-summary guide1 guide2 --output /repo/bags/guide-summary
```

錄製結束（含 Ctrl-C／到期正常收尾）後，`bags/NAME/manifest.json` 保存 UTC 開始時間 `start_time_utc`、`git.commit`、`git.dirty`、Nav2 子模組的 `git.submodule_commit`、去重後的 `topics`、指定的牆鐘錄製上限 `duration`（未指定為 null，並非實際經過秒數），以及 `metadata`。Git 使用 `git -C /repo` 盡力查詢，未安裝、查詢失敗或子模組未初始化時對應欄位為 `unknown`，不阻止錄製。`--meta` 可重複，鍵限英文字母或底線開頭、後續為英數或底線；同鍵最後一個值優先，值可含空白與等號。情境、launch 參數與 Nav2 stack 是使用者提供的描述，不會自動驗證是否與執行環境相符；仍須另保存設定與地圖，dirty 旗標不包含修改內容。

`eval_report` 的 JSON 鍵固定排序、浮點數四捨五入至小數點後九位，CLI 評估參數也先套用相同精度再計算，沒有牆鐘產生時間。驗證採相同正規化，排除 `source`（來源路徑）與預留的 `generated_at_utc` 欄位，其他參數、指標、逐回合及逐事件資料均參與比較；陣列順序保留。這也是對正規內容計算雜湊時的規則。JSON 仍保留來源路徑供追溯；不以原始 JSON 位元組直接比較。

`eval-verify NAME` 從原 bag 重新轉換並評估，使用 `observations.jsonl.meta.json` 中的 zones 路徑、座標轉換與同步上限，以及 `report.json` 中的世界檔路徑和評估參數。暫存產物會清除，既有報告不覆寫；相同回傳 0，不同回傳 1 並列出至多 20 個差異欄位路徑，輸入錯誤回傳 2。原始 bag、世界檔、zones 與這兩份報告必須仍可讀，來源路徑須在目前容器有效。它驗證目前程式能否重現既有指標，不保證重新執行 Gazebo 會得到相同 bag，也不驗證未納入指標的原始資料是否改變。

`eval-summary NAME... [--output DIR]` 讀取各 bag 的 `report.json` 與可選的 `manifest.json`；名稱沿用 `eval-bag` 的英數、底線與連字號限制，重複名稱拒絕。預設輸出 `/repo/bags/summary/summary.json` 與 `summary.md`，指定的 DIR 是容器內路徑，既有同名彙整檔會覆寫。以欄位路徑（陣列含索引）彙整每個數值的 count／mean／min／max／stdev；stdev 為樣本標準差，僅一筆時為 0，null 與布林不當作數值，parameters 與 source 不混入指標。不同回合索引的統計僅適用相同順序的情境，不能視為同一區域的自動配對。

報告若有 `verdict` 或 `verdicts.*` 的 pass／fail 字串（不分大小寫）或布林，另列 pass_count／fail_count；現有評估器沒有自動判定，因此通常沒有判定計數，不會由數值猜測通過。缺少 manifest、commit／scenario／nav2_stack 資訊，或這三者跨次不同時會列出警告；警告不會阻止彙整，也不代表資料可直接比較，仍須核對評估參數、缺樣及失敗回合。

安裝套件後，在 compute 也可直接使用以下等價命令；轉換與驗證只讀 bag，不發布或回放速度：

```bash
ros2 run shopping_zone_manager eval_verify guide1
ros2 run shopping_zone_manager eval_summary guide1 guide2 --output /repo/bags/guide-summary
```
