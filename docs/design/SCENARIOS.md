# 模擬情境驗收程序

適用 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress；本輪只完成 ROS-free 與靜態驗證，以下容器操作由維護者執行。每次啟動一個情境，不得同時執行不同 guide／follow／mapping、鍵盤、nav-check 或 goto 控制來源。不得因有模型或裝置就啟動硬體。

## 建置與列出

```bash
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh scenario-list
```

設定位於 `shopping_robot_gazebo/config/scenarios`。`robot_pose` 為世界 x、y、yaw，`person_path` 為字串 `person_path.yaml` 或 `{speed, waypoints, pauses?}`；pauses 與航點等長，缺省皆為 0 秒。`pedestrians` 缺省為空；每人必填唯一 `ped_` 名稱、waypoints、speed、radius、start_delay，可選 pauses（與航點等長、秒、非負）。速度單位公尺／秒、半徑公尺、延遲秒。軌跡循環包含末點回首點；速度為 0 可只有一點。延遲期間行人已存在並停在首點。未知欄位、非有限數、布林數值及非法名稱皆拒絕。

額外行人預設會讓行：行人未來 0.5 秒線段若靠近機器人外接圓＋人物半徑＋0.20 公尺，便停下並凍結自己的路徑時間；離開後續走，解除另有 0.05 公尺遲滯，不追趕舊排程。這是模擬情境真實度規則：行人不能持續頂著已停住的機器人前進。只有行人節點使用帶時間戳的模擬真值；導航與機器人控制器均不使用。資料缺失、過期或只有未來樣本時，行人也停止。這不能保證機器人不撞上已停行人，迎面互相停等仍可能導航逾時，須如實記錄。

`guide`／`sim` 可加 `pedestrian_yield:=false` 還原不避讓的舊速度追蹤，用獨立 bag 保存接觸壓力測試；預設為 true。旗標不會還原已更新的 scenario YAML，重現 t71 永久橫越時須另保存並使用該輪舊設定，不可只改旗標便宣稱與 t71 同一情境。人物仍有碰撞與質量，沒有瞬移或接觸後自動重設。

## 每個情境的操作

導航使用既有標準地圖；地圖不存在或座標不一致時，先在無人物的 `default` 情境建圖並驗證地圖，再跑動態測試，避免把行人寫入標準靜態地圖。表中 launch 皆加 `headless:=true`；導航另加 `slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml`。

| scenario | launch | 目標／觀察 |
| --- | --- | --- |
| default | guide | `goto snacks` → `goto entrance`，確認舊出生位置與無額外行人；另以 follow 核對舊人物路徑 |
| aisle_crossing | guide | `goto household`，行人在 x=−0.45…0.45、y=3 橫越後，沿側邊到 y=5.8，在北側兩航點各停 8 秒再返回；回 `goto entrance`，核對穿越、清空窗口與恢復 |
| narrow_pass | guide | `goto household` → `goto entrance`，行人在中間 1.6 公尺走道沿 y=4.8…1.2 迎面走動；確認等待／安全繞行及恢復 |
| checkout_queue | guide | `goto checkout` → `goto entrance`，櫃臺南西側一靜止、一低速排隊者；確認抵達與間距 |
| follow_turns | follow | 不送人工目標，至少觀察一個完整人物循環，含 x=2.4 走道末端急轉及迴轉 |
| follow_stop_go | follow | 不送人工目標，起始 0.58 公尺 TOO_CLOSE、停留、離開、走道停走；至少錄製 120 秒並保留失敗 |
| follow_fast_1_0 | follow | 人物 1.0 m/s，南側開放區域循環；核對直線誤差、轉彎與最大間距 |
| follow_fast_1_2 | follow | 人物 1.2 m/s，同一路徑；記錄速度飽和後無法追回的起步落差 |
| alt_start | guide | 出生 `(-2.4,0,π/2)`，`goto drinks` → `goto entrance`，核對 AMCL／EKF 與真值起點 |

逐條啟動，上一條 Ctrl-C 完全結束再執行下一條：

```bash
./scripts/dev.sh guide scenario:=default headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh guide scenario:=aisle_crossing headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh guide scenario:=narrow_pass headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh guide scenario:=checkout_queue headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh follow scenario:=follow_turns headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh follow scenario:=follow_stop_go headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
./scripts/dev.sh guide scenario:=alt_start headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
```

導航目標需等 AMCL active、`scenario_initial_pose` 印出發布起點且 map TF／Nav2 就緒後送出。`sim scenario:=NAME` 可只觀察模型；`mapping scenario:=NAME` 可檢查建圖，但不要把含行人的地圖當作標準驗收地圖。follow 保留既有 `route_around:=true`；若要隔離直接跟隨安全行為，可另開一輪加 `route_around:=false`，不得覆蓋原輪次紀錄。

## 錄製與檢查

以穿越情境為例，第二個終端機先錄製，第三個終端機送目標：

```bash
# Terminal 2: wall duration; use a new bag name on each repetition.
./scripts/dev.sh record aisle_crossing_01 --duration 300 /scan /simulation/poses /sim/pedestrians /tf /tf_static /global_costmap/costmap /local_costmap/costmap /amcl_pose /plan
# Terminal 3: one command at a time, after navigation is ready.
./scripts/dev.sh goto household --timeout 90
./scripts/dev.sh goto entrance --timeout 90
# After recording finishes:
./scripts/dev.sh eval-bag aisle_crossing_01
```

其他情境替換名稱並依表送目標。錄製須覆蓋兩個目標及停止後狀態；範例 300 秒是錄製上限，不改導航與評估門檻。follow 至少錄製 120 秒，完整循環的最低時間可用路徑總長／speed 加 pauses 總和計算；不足一圈應延長。動態行人的 `start_delay` 是世界秒，不是相對送目標；啟用讓行後路徑時鐘會因停等延後，不能再單憑世界時間推算位置。請保存目標送出時刻，若錯過相遇則重跑或依明確記錄的時機送目標，不得將未相遇當作通過。錄製完成後，將本次 scenario YAML、canonical person_path.yaml、zones.yaml、Git 版本、`pedestrian_yield` 值與啟動命令保存到 bag 目錄旁；執行期展開檔會於 launch 結束刪除。新 aisle_crossing 自由循環約 40.7 秒，停等會延長；與 t71 的永久橫越版本分開比較。

在另一個獨立無導航目標的輪次執行一般回歸；`nav-check` 自己會發目標，不能與 goto 同時執行。定位檢查建議使用相同標準地圖與 `alt_start`，並在任何移動前選對情境：

```bash
./scripts/dev.sh nav-check --case-timeout 120
./scripts/dev.sh loc-check -p scenario:=alt_start -p publish_initial_pose:=true
```

以上兩行也須分開輪次執行。故障回歸請停止原情境，以 `./scripts/dev.sh follow scenario:=default fault_injection:=true headless:=true` 啟動，另執行 `./scripts/dev.sh fault-check`，依既有故障驗收程序保存記錄；不可將初始就停的情境視為煞停通過。

## 判定與限制

- `goto` 的 SUCCEEDED 另驗證區域座標系的定位 TF：`arrival_xy_tolerance` 預設 0.25 公尺、`arrival_yaw_tolerance` 預設 0.35 弧度、`arrival_max_tf_age` 預設 0.5 秒，沿用 t68 審查值；Nav2 Loose goal checker 仍為 0.15 公尺／0.25 弧度。缺 TF、過期或超差會回報 ABORTED。
- `guide slam_mode:=map_server` 另外要求 AMCL 樣本：`arrival_max_xy_std` 預設 0.15 公尺（平面最大主軸標準差）、`arrival_max_yaw_std` 0.35 弧度、`arrival_max_amcl_age` 5 秒。缺失、非有限、無效或過期信心皆拒絕成功；SLAM-only 不強制 AMCL。請保存 AMCL 警告，檢查定位／通道後再發目標，不會自動重試或重定位。此檢查只在成功結果時執行，不會自動中止行進；低協方差仍可能錯位，不能取代真值驗收。t71 第一次受推即是低協方差假定位，後續兩次假成功才有足以拒絕的協方差。
- 分析動態失敗時，除既有安全狀態與 mux／guard／最終命令，須保存 `/amcl_pose`、`/plan` 及兩份 costmap；先比對雷射人物是否同時被全域與局部地圖標為障礙，再檢查定位、重規劃與 STOP 的時間關係。新 bag 不覆寫舊失敗；本批根因與重跑項目見 [動態情境分析](../research/DYNAMIC_SCENARIOS_REVIEW.md)。

- 所有回合保留成功、取消、拒絕、失敗與逾時；每個預期抵達目標 90 秒內 SUCCEEDED，最終真值位置誤差建議 ≤0.20 公尺、yaw 誤差 ≤0.20 弧度。
- 靜態及人物 `minimum > 0` 為無幾何接觸的最低條件，建議 >0.10 公尺。行人 topic 必須有完整覆蓋、人數與順序正確，不能以 null 或缺 topic 判定通過；檢查 `.meta.json`。
- `/simulation/poses`、`/sim/pedestrians` 與雷射中的行人位置須一致；在行人位於雷射平面時能看到圓柱，且成本地圖有對應障礙。這部分必須在 Fortress 實測，XML 合法不代表雷射已驗收。
- 相遇時安全減速／等待／繞行，離開後恢復；跟隨過近及障礙保護檢查完整零 Twist，命令／實際停止延遲依 METRICS 建議 ≤0.30／0.80 秒。純 Nav2 讓行沒有新增自動事件偵測，需核對 bag／影像與命令，不能由 action 成功單獨推定。
- 額外圓柱由 Fortress VelocityControl 驅動，預設讓行；主要跟隨人物的 set_pose 是另一流程，亦可能有服務延遲。請驗證讓行前有行人移動、停等後恢復、缺真值停止、aggressive 旗標可還原；確認清空後兩份 costmap 清除並恢復導航，checkout 完成最終 yaw，以及 AMCL 超限時 ABORTED。接觸、雙方停等不恢復或墜出地板都記為失敗，不以零命令或 action 成功掩蓋。尚未提供完整 kidnap 偵測、自動 scenario_check、連續碰撞監測或本輪跨架構實測；本輪只有離線與靜態檢查。


## 高速跟隨情境

`follow_fast_1_0`／`follow_fast_1_2` 使用情境框架產生本輪唯一的 `person_path.yaml`，不另建控制器人物路徑來源。出生 `(-4.5,-1.8,0)`，人物從 `(-3,-1.8)` 出發，在 y=-1.8／-0.2 的南側開放區域循環，最長直線 7.5 公尺。路徑以 101 點／線段檢查 SDF 碰撞盒，最小淨空 0.80 公尺；標準地圖已佔用格中心最小距離約 0.795 公尺。這是幾何檢查，沒有驗證動態煞停或急轉。

依上表逐輪啟動，例如 `./scripts/dev.sh follow scenario:=follow_fast_1_0 headless:=true slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml`；1.2 m/s 輪次換成 `scenario:=follow_fast_1_2`。每輪保存 bag、跟隨狀態及 guard 狀態，沿用原有停止與故障門檻。

`follow_eval` 新增 `follow_performance` 資訊列：人物設定速度、設定跟隨距離、初始 5 秒暖機後且排除人物停留時的距離絕對誤差平均／RMSE，以及整輪最大真值間距。跟隨距離讀取本輪跟隨設定；空資料保留 count=0／最大間距 null。此列為 INFO，不增加或放寬既有通過門檻。真值只用於評估。

純 Python 可用 `python scripts/eval/follow_loop_sim.py --straight --person-speed 1.0 --duration 60` 比較舊參數與新預設；換成 1.2 可觀察速度飽和。這條 1 公里分析直線並非超市情境，不可與 `--world` 合用。模型僅含理想里程計、UWB 濾波與速度限制，不含 Gazebo 動力學、下游障礙守門器或 Nav2；不能當作模擬驗收。1.2 m/s 與機器人速度上限相同，起步或轉彎落後後沒有速度餘裕追回。

高速設定仍須在 Fortress 執行多速度 `yaw_rate_check.py`、1.2 m/s 故障煞停（命令 ≤0.8 秒、真值 ≤1.5 秒）、上述兩情境與既有貨架跟隨回歸。下游障礙守門器保留原有保守煞停掃掠與遲滯；可能因人物或貨架降速，不能用開放直線結果保證貨架環境也能維持同樣間距。
