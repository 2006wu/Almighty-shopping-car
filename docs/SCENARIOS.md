# 模擬情境驗收程序

適用 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress；本輪只完成 ROS-free 與靜態驗證，以下容器操作由維護者執行。每次啟動一個情境，不得同時執行不同 guide／follow／mapping、鍵盤、nav-check 或 goto 控制來源。不得因有模型或裝置就啟動硬體。

## 建置與列出

```bash
./scripts/dev.sh colcon-build
./scripts/dev.sh colcon-test
./scripts/dev.sh scenario-list
```

設定位於 `shopping_robot_gazebo/config/scenarios`。`robot_pose` 為世界 x、y、yaw，`person_path` 為字串 `person_path.yaml` 或 `{speed, waypoints, pauses?}`；pauses 與航點等長，缺省皆為 0 秒。`pedestrians` 缺省為空；每人必填唯一 `ped_` 名稱、waypoints、speed、radius、start_delay。速度單位公尺／秒、半徑公尺、延遲秒。軌跡循環包含末點回首點；速度為 0 可只有一點。延遲期間行人已存在並停在首點。未知欄位、非有限數、布林數值及非法名稱皆拒絕。

## 每個情境的操作

導航使用既有標準地圖；地圖不存在或座標不一致時，先在無人物的 `default` 情境建圖並驗證地圖，再跑動態測試，避免把行人寫入標準靜態地圖。表中 launch 皆加 `headless:=true`；導航另加 `slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml`。

| scenario | launch | 目標／觀察 |
| --- | --- | --- |
| default | guide | `goto snacks` → `goto entrance`，確認舊出生位置與無額外行人；另以 follow 核對舊人物路徑 |
| aisle_crossing | guide | `goto household`，行人在 x=−0.45…0.45、y=3 橫越中間走道；回 `goto entrance`，核對穿越相遇與讓行 |
| narrow_pass | guide | `goto household` → `goto entrance`，行人在中間 1.6 公尺走道沿 y=4.8…1.2 迎面走動；確認等待／安全繞行及恢復 |
| checkout_queue | guide | `goto checkout` → `goto entrance`，櫃臺南西側一靜止、一低速排隊者；確認抵達與間距 |
| follow_turns | follow | 不送人工目標，至少觀察一個完整人物循環，含 x=2.4 走道末端急轉及迴轉 |
| follow_stop_go | follow | 不送人工目標，起始 0.58 公尺 TOO_CLOSE、停留、離開、走道停走；至少錄製 120 秒並保留失敗 |
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
./scripts/dev.sh record aisle_crossing_01 --duration 120 /scan /simulation/poses /tf /tf_static /global_costmap/costmap /local_costmap/costmap
# Terminal 3: one command at a time, after navigation is ready.
./scripts/dev.sh goto household --timeout 90
./scripts/dev.sh goto entrance --timeout 90
# After recording finishes:
./scripts/dev.sh eval-bag aisle_crossing_01
```

其他情境替換名稱並依表送目標。follow 至少錄製 120 秒，完整循環的最低時間可用路徑總長／speed 加 pauses 總和計算；不足一圈應延長。動態行人的 `start_delay` 是世界秒，不是相對送目標；請保存目標送出時刻，若錯過相遇則重跑或依明確記錄的時機送目標，不得將未相遇當作通過。錄製完成後，將本次 scenario YAML、canonical person_path.yaml、zones.yaml、Git 版本與啟動命令保存到 bag 目錄旁；執行期展開檔會於 launch 結束刪除。

在另一個獨立無導航目標的輪次執行一般回歸；`nav-check` 自己會發目標，不能與 goto 同時執行。定位檢查建議使用相同標準地圖與 `alt_start`，並在任何移動前選對情境：

```bash
./scripts/dev.sh nav-check --case-timeout 120
./scripts/dev.sh loc-check -p scenario:=alt_start -p publish_initial_pose:=true
```

以上兩行也須分開輪次執行。故障回歸請停止原情境，以 `./scripts/dev.sh follow scenario:=default fault_injection:=true headless:=true` 啟動，另執行 `./scripts/dev.sh fault-check`，依既有故障驗收程序保存記錄；不可將初始就停的情境視為煞停通過。

## 判定與限制

- 所有回合保留成功、取消、拒絕、失敗與逾時；每個預期抵達目標 90 秒內 SUCCEEDED，最終真值位置誤差建議 ≤0.20 公尺、yaw 誤差 ≤0.20 弧度。
- 靜態及人物 `minimum > 0` 為無幾何接觸的最低條件，建議 >0.10 公尺。行人 topic 必須有完整覆蓋、人數與順序正確，不能以 null 或缺 topic 判定通過；檢查 `.meta.json`。
- `/simulation/poses`、`/sim/pedestrians` 與雷射中的行人位置須一致；在行人位於雷射平面時能看到圓柱，且成本地圖有對應障礙。這部分必須在 Fortress 實測，XML 合法不代表雷射已驗收。
- 相遇時安全減速／等待／繞行，離開後恢復；跟隨過近及障礙保護檢查完整零 Twist，命令／實際停止延遲依 METRICS 建議 ≤0.30／0.80 秒。純 Nav2 讓行沒有新增自動事件偵測，需核對 bag／影像與命令，不能由 action 成功單獨推定。
- 圓柱是受命令驅動的行人，沒有避讓機器人的決策；接觸後可能偏離理想軌跡，主要人物的 set_pose 亦可能有服務延遲。任何碰撞或不恢復都記為失敗，不修改控制器或隱藏失敗。尚未提供自動 scenario_check 執行器、連續碰撞監測或跨架構實測。
