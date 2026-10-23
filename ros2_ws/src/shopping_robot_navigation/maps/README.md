# 地圖輸出與版本管理

標準模擬地圖為 `supermarket.yaml`、`supermarket.pgm`、`supermarket.posegraph`、`supermarket.data`。`.gitignore` 明確允許這四個檔案加入版本管理；其他名稱的相同格式仍忽略。此工作階段未產生地圖，須由維護者在容器執行模擬、確認覆蓋與定位結果後，將四檔一起加入後續提交，不能以假資料代替。

## 重新產生

在主機的兩個終端機操作，先完成套件建置：

```bash
./scripts/dev.sh colcon-build
# Terminal 1
./scripts/dev.sh guide slam_mode:=mapping
# Terminal 2
./scripts/dev.sh map-tour supermarket
```

標準四檔應以預設 `odom_source:=ekf` 產生；也可在上述 `guide` 指令明確加上 `odom_source:=ekf`。巡航只送出 Nav2 目標，建圖品質取決於 EKF 與 slam_toolbox，須評估里程計漂移與閉環效果。若需理想真值里程計比較，先停止原模擬，再分別於兩個終端機執行：

```bash
# Terminal 1
./scripts/dev.sh guide slam_mode:=mapping odom_source:=ground_truth
# Terminal 2
./scripts/dev.sh map-tour supermarket_ground_truth
```

比較產物另存，不取代以 EKF 生成的標準地圖。重新載入比較地圖時使用其檔名前綴／YAML，並在 `guide` 明確傳入要評估的 `odom_source`；定位檢查預設仍使用獨立真值，不需更換來源。

`map-tour` 預設名稱即為 `supermarket`，最後回到出生原點，等待掃描更新並執行與 `save-map` 相同的雙格式儲存。輸出寫入掛載的本目錄。巡航摘要必須全部到達、沒有跳過或逾時、`maps_saved` 為 true，且指令回傳零；再檢視格網是否完整涵蓋三走道、南北橫向通道、入口與結帳區。失敗或部分儲存後須重新執行並確認四檔均已更新；不要混用不同次建圖的檔案。不要在 AMCL 模式要求 SLAM 序列化。

手動補跑可使用 `./scripts/dev.sh save-map supermarket`，但必須仍在原本的 SLAM 建圖工作階段，且先停止其他目標來源。格網 YAML 的 `image` 應能在本目錄解析到對應 PGM；序列化服務的 `filename` 為不含副檔名的前綴，其成功碼依 [Humble SerializePoseGraph 介面](https://github.com/SteveMacenski/slam_toolbox/blob/humble/srv/SerializePoseGraph.srv) 判斷。

## 離線重新載入與區域座標核對

先以 Ctrl-C 停止建圖的 `guide`，重新啟動模擬，出生姿態維持 `(0,0,0)`。每次只啟動以下一種定位模式，各自跑完六區檢查後停止整套 `guide`，再啟動另一種：

```bash
# Terminal 1: serialized pose graph prefix, without extension
./scripts/dev.sh guide slam_mode:=localization map:=/ws/src/shopping_robot_navigation/maps/supermarket
# Terminal 2
./scripts/dev.sh loc-check

# Stop the previous guide, then Terminal 1: occupancy grid YAML with AMCL
./scripts/dev.sh guide slam_mode:=map_server map:=/ws/src/shopping_robot_navigation/maps/supermarket.yaml
# Terminal 2
./scripts/dev.sh loc-check -p publish_initial_pose:=true
```

此處「離線」指重新載入既有檔案進行定位，仍需運行 Gazebo、雷射與 Nav2。工具使用 `shopping_zone_manager/config/zones.yaml` 的六區目標，輸出每區最後地圖位置／真值距區域座標的距離，及時間戳對齊的定位位置／角度誤差統計。每區至少五筆有效採樣，全部採樣定位誤差小於 0.15 公尺、5 度，最後真值距目標不超過 0.3 公尺且導航成功才算通過；六區全過才回傳零。可加 `-p report_file:=/tmp/slam_localization.json` 保存 JSON；AMCL 請使用不同報告檔名。

兩種模式的預設真值皆為 `/sim/ground_truth_odom`（`sim_world → base_footprint_gt`），不再自動探索或回退至 `/odom`；可用 `truth_topic` 覆寫。另接受舊版 `(odom|world|map, base_footprint)`，其他框架配對會明確報錯。真值須與地圖共用出生原點／朝向；即使 EKF 模式的 `sim_world` 與估測 TF 樹沒有連線，仍直接比較座標，不套用待測的 map 校正。EKF 模式 `/odom` 是估測，強制選用會警告並拒絕執行；僅在模擬以 `odom_source:=ground_truth` 啟動、檢查另傳入 `-p truth_topic:=/odom -p odom_source:=ground_truth` 宣告相同模式時允許舊版來源。檢查參數不會切換模擬模式。這份結果仍不代表真實硬體的定位精度。

重建地圖、調整出生姿態、世界幾何或區域設定後，皆須重跑兩種定位檢查，核對六區座標並記錄結果。維護者尚未產生四檔及執行上述模擬驗收；ROADMAP 項目維持未勾選。
