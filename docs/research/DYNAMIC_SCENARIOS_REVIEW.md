# 動態情境失敗分析與修正說明（2026-10-27）

證據為 Linux amd64、apt Nav2／Smac2D、AMCL、標準 `supermarket.yaml` 的 `bags/t64_{aisle_crossing,narrow_pass,checkout_queue}`，以及 `/workspace/wt/logs/shop/t64-*-{launch.log,goto.txt}`。bag 與暫存分析腳本不納入版控。本輪只讀取既有資料、修改程式及執行純 Python／靜態檢查，未啟動 Docker 或模擬。

## 方法與可重現的數字

`/tmp/dyn_numbers.py`、`dyn_detail.py` 比對 `observations.jsonl` 的真值與 `extra.jsonl` 的 AMCL／路徑／模型位姿；AMCL 以訊息時間對最近真值樣本，排除開始錄製前的初始訊息。`/tmp/dyn_cdr.py`、`dyn_grid.py`、`dyn_scan.py` 以唯讀 SQLite 及 Python `struct` 解讀原始 bag 的 String、Twist、OccupancyGrid、LaserScan，避免只從最終 `/cmd_vel` 猜測上游行為。時間皆為模擬秒；guard／mux／最終命令依最近 bag 接收時間配對，並非同一回呼的原子快照。`simposes` 約 5 Hz；報告中的最低淨空採原評估器較密集的真值樣本，不能用抽樣結果取代它。

| 情境／目標 | 結果／耗時 | 真值終點位置誤差 | AMCL 對真值最大位置誤差 | 全域路徑數／相鄰中位間隔 |
| --- | --- | --- | --- | --- |
| aisle_crossing／household | goto 120 秒逾時；bag episode 107.786 秒 | 2.346 m | 0.068 m | 92／0.953 s |
| aisle_crossing／entrance | 成功／18.013 s | 約 0.135 m | 0.081 m | 19／0.975 s |
| narrow_pass／household | 成功／35.079 s | 1.804 m | 1.827 m | 21／0.974 s |
| narrow_pass／entrance | 成功／23.299 s | 0.134 m | 1.588 m，返回途中降至約 0.002 m | 24／0.987 s |
| checkout_queue／checkout | 成功／12.825 s | 0.135 m，yaw 0.168 rad | 0.043 m | 15／0.911 s |
| checkout_queue／entrance | 成功／79.745 s | 0.129 m | 0.056 m | 76／0.961 s |

路徑數是錄得的 `/plan`；擷取腳本有 0.5 秒篩選，但本批資料與原始 bag 的總數相同（111／45／91）。不能由它宣稱規劃器設定頻率等於實際成功輸出頻率。goto 的牆鐘逾時與 episode 的模擬時間不能混用。

## 全域地圖漏掉行人，局部控制與安全層反覆擋住直行

兩份 Nav2 設定的全域 `obstacle_layer.scan` 都漏設 `max_obstacle_height`，局部則明確為 2.0。Humble 的**觀測來源**預設是 0.0，與 obstacle layer 本身預設 2.0 不同，會在 ObservationBuffer 濾掉高度約 0.36 m 的雷射點。來源：[Humble ObstacleLayer 原始碼](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)（觀測來源參數宣告與緩衝區建立）。這是本次參數修正的依據。

- `aisle_crossing` 在 t≈99，全域地圖的行人周圍 ±0.25 m 方框沒有任何值 100 的致命格；同時局部地圖在 x≈−0.30…0.15、y≈2.75…2.95 有行人表面致命格。全域路徑仍沿 x=0.005 經 y=2.755／3.005／3.255 直穿相遇區。t≈120、140 全域行人附近同樣無致命格；中心格代價分別約 54、92，是既有膨脹成本，不能當作成功標記人物。
- household 的 2,401 個 guard 樣本為 STOP 1,443、SLOW 812、CLEAR 146；全部 1,443 個 STOP 的最近 mux 命令仍非零。例如 t≈120：mux 0.4 m/s、0.035 rad/s，guard scale=0，最終命令完整為零。t≈140 即使 swept clearance=0.129 m 仍未滿足 0.15 m 解除遲滯，因此維持 STOP。
- 原評估人物最低淨空為 0.208 m；真值在 t≈99…140 幾乎停於 y≈1.98…2.00，最後僅前進約 2.16 m。啟動紀錄有多次 `Failed to make progress`、清除局部地圖、旋轉／後退失敗及再次控制；最後由 goto 取消。持續重規劃同一條直線，並沒有建立繞行路徑。人物只在 x=±0.45 間循環，仍可能持續占用通道；等待本身不保證能通過。
- `checkout_queue` 的全域路徑終點都在目標約 0.007 m 處，但返回時接近排隊者而緩行／停止。例如 t≈140：mux 0.4 m/s，guard scale≈0.0275，最終約 0.011 m/s；t≈160：mux 0.136 m/s，最終為零。這與 79.745 秒返回時間一致，並非 AMCL 大幅漂移。最低人物淨空 0.077 m；t≈149.98 真值與最終命令均已停止，guard 在阻止旋轉（mux 約 −0.165 rad/s，旋轉圓盤 swept clearance≈−0.007 m）。單一靜態點雲的安全層無法保證移動行人不再靠近。

因此先修復全域標記，維持既有 clearing／marking 與安全 STOP。沒有放寬 progress checker、停車距離或解除遲滯，也沒有增加無限重試、清掉障礙後強行通過。先重跑確認全域路徑會回避真實障礙，再評估是否需要額外的有界停等交握；目前資料不能證明只改等待時間即可解開通道。

## narrow_pass：接觸推移、定位落後與未驗證的成功

- t≈100：真值 y=2.818，AMCL y=2.839，誤差僅約 0.014 m。t=101.251 guard STOP，最近 mux 仍要求 0.4 m/s；最終零命令延遲約 0.046 s。
- t≈102：機器人 y=3.131，人物 y=3.903，命令已零。t≈102.802 機器人 y=3.111、人物 y=3.660，圓柱與矩形底盤淨空達零；最近最終命令仍為完整零 Twist，真值平移速度卻達約 0.296 m/s。這是接觸推動，不能把零命令當作機器人已靜止。
- t≈103／108 雷射分別有 2／74 個無效近距回波（`-inf`）；guard 因拒絕無效掃描而進入 STALE，並非 watchdog 放行。household 734 個 guard 樣本中有 178 個 STALE、55 個 STOP，其中 50 個 STOP 同時收到 mux 移動要求；t≈108 mux 甚至要求旋轉 1 rad/s，最終仍為零。
- 受速度命令驅動的行人持續向南走，將真值由 y≈3.13 推至 t≈114 的 y≈1.203；AMCL 最後接觸前更新為 t=100.9、y=3.083，下一筆到 t=110.6 仍是 y≈3.021。這段沒有約 1.8 m 的 AMCL 跳躍，而是真值被推走、定位未追上。輪式里程計不能可靠量出受推的滑移；僅憑此資料不能將誤差歸因為雷射靜態地圖不匹配。增大隨機量測權重、改 `z_max` 或啟用 beam skipping 無法據此保證修復，因此本輪不調 AMCL。
- 成功發生在 t=126.157：真值約 (−0.004, 2.696)，離 household 1.804 m；最後一筆 AMCL t=126.0 約 (0.001, 4.279)，與同期真值差約 1.598 m，距區域中心仍有 0.221 m，y 方差約 0.0448 m²。回入口後定位重新貼近真值；返回途中 t=147.0→147.1 的 AMCL 位移約 0.324 m。
- Smac2D 的 0.8 m 是找不到精確終點時的規劃搜尋容差，**不是**區域抵達判定。household 曾有距區域 0.345／0.245／0.095 m 的路徑端點；但最後 t=125.392 的端點為 (0.005, 4.505)，離原目標僅 0.007 m，不能說本次 1.804 m 真值誤差是 0.8 m 容差單獨造成。Nav2 使用定位後的 TF 與路徑 goal checker，原 `zone_navigator` 只把 action 狀態 4 映成 SUCCEEDED，完全不查區域距離。

## 實作與參數理由（供 PR 說明使用）

1. apt／fork 全域 `scan.max_obstacle_height: 2.0`：接納 0.36 m 雷射平面，修正 Humble 來源預設 0.0 排除動態人物的錯誤；與局部設定一致。既有 clearing／marking、範圍及膨脹半徑保留。fork 未在本批 bag 驗證，僅修正相同設定缺漏。
2. 區域導航在 action 成功時，以該區域座標系的 `base_footprint` 最新定位 TF 驗證位置／朝向；標準區域皆為 `map`。使用 ROS-free `arrived()`，位置 ≤0.15 m、yaw ≤0.25 rad，沿用既有 Nav2 goal checker 值；TF 年齡須在 0…0.5 s，與現有安全層的資料逾時尺度一致，需模擬確認。缺 TF、非有限值、過期、未來時間或超出容差皆回報 ABORTED，釋放已完成的 action，供使用者重新發目標。沒有自動重送或讀取真值。
3. 生命週期預設拒絕「未驗證的成功」，保留舊 generation 忽略、取消與逾時隔離；pytest 涵蓋成功／失敗、失效位姿、位置／朝向容差、角度包覆及逾時後的遲到成功。

終點檢查只能驗證**定位**是否抵達，無法辨識一個錯誤但看似抵達的 AMCL。全域標記修正也尚未證明能避免迎面人物推撞；不修改行人軌跡、不取消碰撞、不使用真值修正導航，也不宣稱三種情境已通過。

## 必須重跑

在新的獨立 bag 依 [情境程序](../design/SCENARIOS.md) 重跑三情境，以及 default／alt_start 無人物回歸與 follow 障礙故障。固定原有位置、yaw、淨空與停止延遲門檻。確認全域／局部都有同一雷射人物的致命格，人物離開後會清除、路徑會繞行或安全有界失敗；保存所有失敗與逾時。另測 action 成功但終點超出區域容差、TF 缺失／過期的 ABORTED，以及 AMCL 尚未恢復時不得宣稱真值抵達。必要時再設計感測器支持的動態障礙預測／停等協定與滑移定位恢復，不以參數猜測替代證據。

## t71：fork TEB 重跑與第二輪修正（2026-10-27）

本節以 `bags/t71_{aisle_crossing,narrow_pass,checkout_queue}` 的原始 SQLite、observations／report，以及 `/workspace/wt/logs/shop/t71-*-{launch.log,goto.txt}` 為證據。啟動紀錄確認為 fork gentle TEB＋Smac2D、AMCL、標準地圖。僅使用 `/tmp/t71.py`、`t71_detail.py`、`t71_contact.py`、`t71_grid.py`、`t71_tfgrid.py`、`t71_summary.py` 與 CDR 解碼器，沒有 ROS、Docker 或模擬執行。SQLite 以唯讀 URI 開啟；訊息接收時間用於命令／costmap 配對，AMCL 使用 header 時間，真值速度以相隔約 0.1 秒的位置差估計。最近樣本配對不是同步快照；區間端點有約一筆真值取樣誤差。舊 t64 結果保留，不以本節覆寫。

| 情境／目標 | bag episode 模擬時間 | 終點真值誤差 | 定位與主要證據 |
| --- | --- | --- | --- |
| aisle_crossing／household | 85.407–198.476，113.069 秒；goto 牆鐘 120 秒逾時 | 2.768 m | AMCL 對真值最大誤差 0.022 m，非定位失敗；路程 1.73 m，最低人物淨空 0.637 m |
| narrow_pass／household | 91.474–154.363，62.889 秒，回報成功 | 2.396 m | 終點前最後 AMCL y=4.272，真值 y=2.104；AMCL y 方差 0.02761 m² |
| narrow_pass／entrance | 156.562–225.990，69.428 秒，回報成功 | 13.114 m | AMCL 仍在入口附近；最後協方差已達 x≈1.70、y≈0.56、yaw≈1.37 |
| checkout_queue／checkout | 91.684–204.426，112.742 秒；goto 牆鐘 120 秒逾時 | 0.135 m／0.334 rad | Nav2 yaw 容差仍是 0.25 rad；AMCL 最大位置誤差 0.029 m |

入口回程：aisle_crossing 成功、位置誤差約 0.100 m；checkout_queue 成功、位置約 0.046 m／yaw 約 0.199 rad。無人物 nav-check 由使用者提供 PASS；本次未重跑。

### narrow_pass：行人主動推撞，之後定位錯誤與墜落

這三種情境的額外行人不是 set_pose 瞬移：`scenarios.py` 建立質量 60 kg、保留碰撞的 Fortress `VelocityControl` 圓柱，`pedestrian_mover` 以有界速度追蹤世界時間軌跡。主要跟隨人物的 set_pose 是另一條流程。

- 第一次淨空小於 0.3 m：t=106.84–112.08。機器人 y=1.605→0.673；263 個真值樣本中 241 個最近最終 Twist 完整為零，零命令時真值速度最大約 0.297 m/s。t=106 時機器人已停，行人仍以約 −0.30 m/s 向南靠近；t=108 機器人 vy≈−0.296 m/s、行人 y≈2.103，最終命令為零。是行人推動停住的機器人，並非機器人追撞；區間開頭仍有短暫零平移的轉向命令。
- 第二次為 t=129.58–136.22：y=1.979→0.698；333 筆中 304 筆零命令，零命令時速度最大 0.297 m/s。第三次為 t=152.84–156.98：208 筆中 192 筆零命令，最大約 0.297 m/s，涵蓋 household 成功時刻。評估報告的最低淨空仍為 0（接觸）；上述幾何重算的負值代表重疊，不替代報告指標。
- t=107.2 AMCL y=1.601；到 t≈111 真值已為 0.684，AMCL 沒有隨受推滑移更新。t=112.9 AMCL 仍為 1.599，方差僅 0.00816 m²。t=139.4 AMCL y=2.920、真值 y=0.721，誤差 2.199 m；t=152.4 方差才增至 0.02761 m²。因此「低協方差」不能證明未遭綁架或滑移。
- household 回報成功時，AMCL 距目標約 0.228 m，能通過 t68 審查後的 0.25 m 抵達檢查；同一個錯誤 AMCL 的 TF 無法提供獨立驗證。返回入口時定位仍落後約 3 m，機器人實際駛過地板南界 y=−3。t≈188 真值 z≈−7.93，t≈190 z≈−52.72，t≈226 已為 z≈−7565；最終 13.1 m 是平面誤差，還包含離開有限地板後墜落，不能解讀為正常地面導航。

### aisle_crossing：沒有清空的通道與提前路徑檢查

- t=110–190 行人 x≈−0.445…0.446、y恆為3，每約 6 秒循環，並未走出走道。機器人最後停在 y≈1.73，最低人物淨空提高到 0.637 m；t=110–190 的 1,698 個最終命令僅 74 筆非零。household guard 為 SLOW 2,326／CLEAR 75，沒有 STOP，代表這次主要是 TEB 自己停等。
- TEB 日誌有 42 次「path blocked or obstacle too close → replan」、8 次 progress failure。停滯時 `pose` 淨空約 0.43 m、`forward-sim` 約 0.28–0.36 m，但 `band` 會到 −0.035…0.16 m；0.8 m 前視路徑與 0.20 m 恢復門檻讓控制器反覆停等／重規劃。全域路徑已有嘗試繞向 x≈−0.445…0.505，483 筆輸出間隔中位數 0.228 秒，但人物持續回到相遇段。BT 三次 Retry、Recovery／Shrink／Wait 又重新嘗試；不是單純延長 progress allowance 就能保證通過。
- 本次全域確實有行人致命格。例如 t=110.43 的行人 x≈−0.322，致命格 x≈−0.545…−0.095；t=113.30 行人 x≈0.361，致命格改為 x≈0.155…0.555。局部地圖先以當時 `map→odom` TF 轉回 map，再排除貨架；抽查 110／112／114／140／142／144／180 秒，行人區致命格沒有距當時人物中心橫向超過 0.33 m 的樣本。不支持「持續殘影橫封走道」是主因，不能直接用 odom 格網座標比世界 x，也不能據有限抽查保證所有格都已清除。

### checkout_queue：卡在平移與最終旋轉的交界

- t≈126–200 機器人維持 (2.892, −0.918, −0.334)，1,576 筆最終命令全部為零；mux 也為零。checkout episode guard 為 SLOW 2,019／CLEAR 381，沒有 STOP。這不是 guard 持續阻擋旋轉，也沒有命令正負振盪的證據。
- 路徑終點約 (3.005, −0.995)，定位到該點約 0.13 m：已進 Nav2 Loose 0.15 m 位置容差，但尚未到 TEB `goal_xy_stop_dist=0.10` 的純旋轉距離。TEB 仍要求少量平移，因此仍檢查平移 band。最初 band=0.148 m 觸發 0.15 m STOP，之後 band≈0.173／0.190 m，大於 STOP 卻小於原 0.20 m 恢復門檻；`pose` 約 0.27 m、前推約 0.21–0.22 m，卡住原因明列 `resume-hysteresis`。此輪 38 次 replan、7 次 progress failure。不可把它描述為已經進入原地轉向而遭圓盤檢查拒絕。

### 本輪實作、參數依據與限制

1. **行人讓行（僅模擬演員）**：新增 ROS-free `YieldingTrajectory`。預設以機器人外接圓半徑 `hypot(0.30,0.25)`、行人半徑、0.20 m 餘量檢查行人未來 0.5 秒線段；遇阻停下，脫離後再繼續，解除餘量另加 0.05 m。0.20 m 用於演員回呼與停止餘量，0.5 秒涵蓋約 0.15 m 行人行程；不是修改機器人安全門檻。停等期間凍結行人路徑時間，避免恢復時追趕已移動的排程。機器人真值只進入模擬行人節點，導航與控制器不讀取真值。使用既有帶時間戳的 `/sim/ground_truth_odom`，從最近 32 筆選不晚於行人時刻的樣本；缺資料、超過 0.5 秒或未來資料皆停止演員。`pedestrian_yield:=false` 還原舊有不避讓的速度追蹤；不取消碰撞、不移走人物、不改評估器。
2. **穿越情境確實離開相遇區**：原 x=±0.45、y=3 永久往返無法提供「等人離開再走」的驗收條件。新增到 y=5.8 的北側段，在貨架末端 y=5 之外各停 8 秒，再循環回來；允許每位行人使用已驗證的 `pauses`。人物速度／半徑、出生點與原穿越段不變。8 秒停留用於形成可觀察的清空窗口；完整自由循環約 40.7 秒，讓行會延長。這是明確的情境版本變更，不能把舊／新輪次直接當成單一控制器比較。
3. **（已撤回，未採用）fork TEB 恢復遲滯**：2026-10-27 使用者確認結帳在機器人上進行、`checkout_queue` 不在範圍內，因此維持 `gentle_resume_margin` 0.05 m 不變。原提案：Slow／Fast 的 `gentle_resume_margin` 0.05→0.02 m，恢復門檻由 0.20 降至 0.17 m，使 t71 實測 0.173 m 的安全 band 不再僅因遲滯無限卡住。0.15 m STOP、footprint、前推與旋轉掃掠、guard、速度／加速度、Nav2 goal checker 與評估門檻均保留；不改 fork 子模組、清圖、AMCL 模型或 progress allowance。較小遲滯可能增加停止／恢復切換，須新 bag 驗證。這是針對最後接近的參數修正，不能保證剩餘接近及旋轉已通過。
4. **抵達增加定位信心檢查**：`guide slam_mode:=map_server` 必須收到 map frame 的 AMCL covariance；沒有資料不接受成功。曾收到 AMCL 的其他呼叫者同樣檢查，SLAM-only 無 AMCL 可繼續使用原 TF 驗證。平面最大主軸標準差上限 `arrival_max_xy_std=0.15 m`、yaw 標準差 `arrival_max_yaw_std=0.35 rad`；採主軸特徵值避免漏掉相關性。門檻與抵達位置／角度尺度相稱，並非以真值誤差當輸入。樣本最長 `arrival_max_amcl_age=5.0 s`，容許 AMCL 運動觸發更新（t71 household 最近樣本距結果 1.963 秒），仍拒絕無限沿用舊信心。缺失、過期、未來、非有限、不對稱或無效 covariance 會把 Nav2 成功轉為 ABORTED，釋放 action。t71 household 主軸標準差至少 0.166 m，入口至少 1.30 m，均遭拒絕；不自動重定位或自動重試。這不是完整的 kidnap 偵測，也不會在行進中自動取消；第一次受推仍可低協方差，必須保留真值驗收與碰撞檢查。

新增重要邏輯與節點結果回呼的 pytest；最終完整測試為 1091 passed、1 skipped（另有既有 xacro 棄用警告），詳見 PROGRESS。未啟動容器或模擬，所有行為修正仍需新 bag。必須檢查行人會停／續走且無排程追趕、缺真值會停、舊 aggressive 模式可重現、通道清空後導航恢復、checkout 完成 yaw、covariance 超限時回報 ABORTED，以及 default／alt_start／無人物 nav-check／follow 故障回歸。迎面雙方都停仍可能造成有界失敗；沒有加入自動側移或保證通行。

| 審查驗證項目 | 狀態 |
| --- | --- |
| t71 時序、推撞與墜落根因數字 | 待驗證 |
| 行人讓行、路徑暫停／恢復與 aggressive 旗標 | 待驗證 |
| 新穿越窗口、TEB 恢復與 checkout yaw | 待驗證 |
| AMCL 信心不足拒絕成功與無人物回歸 | 待驗證 |
