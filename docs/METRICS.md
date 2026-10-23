# 模擬評估指標

本文件定義 ROADMAP 階段 5 第 1 項的離線量測方式。實作位於 `shopping_zone_manager/eval_metrics.py`（純 Python、標準函式庫、不匯入 ROS），`eval_report.py` 僅負責檔案載入及報表。適用於 ROS 2 Humble、Gazebo Fortress 的模擬商店；本次沒有新增錄製節點、rosbag 轉換或情境執行器。

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

以上均為公尺及離散樣本最小值；未檢查相鄰位姿間的掃掠碰撞，也未計入雷射誤差、定位誤差、車體高度或其他動態障礙。後續情境測試須提高取樣率並另做連續碰撞檢查。

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

此表是後續 rosbag／轉換器的介面契約，不是目前已實作的錄製功能。使用不同框架時，轉換器必須先用 TF 對齊；不得由本模組猜測座標。

| 預期來源 | 紀錄欄位 | 注意事項 |
| --- | --- | --- |
| NavigateToPose 目標、結果與對齊的車體位姿 | `navigation` 的 episode、success、goal_*、x/y/yaw | 一次動作一回合，保留取消／拒絕／失敗／逾時；以目標送出到終態的姿態邊界取樣，座標須一致 |
| `/odom` 的 pose，或 `/simulation/poses` 的車體真值 | `pose` 的 x/y/yaw；導航另轉入目標框架 | 現有 `/odom` 是模擬真值；未來輪式里程計不能直接當真值 |
| `/uwb/person_ground_truth` 或 `/simulation/poses` 的人物真值 | `pose` 的 person_x/person_y，並計算 `distance` | 轉成與車體相同的世界框架並同步；解析人物路徑真值與 Gazebo 模型實際到位可能有時間差 |
| `/follow/state` | `state` | 使用接收的模擬時間，保留狀態變化與取樣終點 |
| 最終 `/cmd_vel` 的 linear.x／angular.z | `cmd` 的 linear/angular | 輸入為底盤最終命令；Nav2 上游 `/cmd_vel_nav` 不代表底盤實際收到的命令 |
| `/odom` 的 twist | `odom` 的 linear/angular | planar linear 使用 hypot(vx,vy)，angular 使用 wz；不要把 cmd 當量測速度 |
| 故障注入器、急停、看門狗及狀態轉換 | `trigger` 的 event | 需獨立標記實際觸發時刻；若從狀態字串產生，延遲只代表狀態公布之後的反應，不包含狀態偵測之前的延遲 |
| `/clock`、訊息 header、`/tf` | t 及離線框架對齊 | 無 header 的狀態／命令使用模擬接收時間並記錄其限制；重啟與倒退時間必須分檔 |

UWB 估測位置誤差仍使用既有 `shopping_uwb_follow/follow_metrics.py` 與 `/uwb/estimate_error`；本指標的跟隨距離誤差評估控制結果，兩者應分開呈現。後續情境變化須由 Gazebo 套件的 `config/person_path.yaml` 產生，不另建執行期人物路徑來源。
