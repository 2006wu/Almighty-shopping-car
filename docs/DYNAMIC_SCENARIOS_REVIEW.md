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

在新的獨立 bag 依 [情境程序](SCENARIOS.md) 重跑三情境，以及 default／alt_start 無人物回歸與 follow 障礙故障。固定原有位置、yaw、淨空與停止延遲門檻。確認全域／局部都有同一雷射人物的致命格，人物離開後會清除、路徑會繞行或安全有界失敗；保存所有失敗與逾時。另測 action 成功但終點超出區域容差、TF 缺失／過期的 ABORTED，以及 AMCL 尚未恢復時不得宣稱真值抵達。必要時再設計感測器支持的動態障礙預測／停等協定與滑移定位恢復，不以參數猜測替代證據。
