# SLAM 建圖與重載定位

使用 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 與 slam_toolbox。預設為模擬，使用 EKF 里程計；`map → odom` 由 slam_toolbox 發布，`odom → base_footprint` 由 EKF 發布。不要同時啟動 AMCL、另一個 SLAM 或另一套移動流程。

## 實際開啟建圖

先完成容器與 ROS 套件建置，再啟動服務。每行長駐命令各使用一個終端機：

```bash
./scripts/dev.sh up
# Terminal 1: simulator, SLAM, navigation and the guarded velocity pipeline.
./scripts/dev.sh guide slam_mode:=mapping headless:=true
# Terminal 2: map frame, top view, live laser, robot and global path.
./scripts/dev.sh rviz config:=/ws/src/shopping_robot_bringup/rviz/slam.rviz
# Terminal 3: one exclusive navigation client; a new name preserves existing maps.
./scripts/dev.sh map-tour store_trial
```

瀏覽器位址由 `./scripts/dev.sh novnc` 查詢。本機本次使用 [noVNC 6081](http://localhost:6081/vnc.html)，VNC 5970。等 Nav2 lifecycle active 後再開始巡航；RViz 初始只看到已掃描部分，隨移動逐步展開。`mapping` 命令只啟動模擬與 SLAM；需要自動巡航時使用上面的 `guide slam_mode:=mapping`。

專用 RViz 設定以 10 FPS 降低 Mac 軟體繪圖負載，使用 `map` 固定座標與俯視鏡頭，顯示 `/map`、`/scan`、機器人及 `/plan`。不提供第二個導航按鈕，避免巡航期間誤送競爭目標。若要另看 Gazebo，可在沒有其他 GUI 的獨立終端機執行 `./scripts/dev.sh gz-gui`；Mac 軟體繪圖同開兩個 GUI 會增加負載。

巡航依 `shopping_zone_manager/config/map_tour.yaml` 的 17 個航點走過入口、三條走道、北側回訪與結帳區。回訪提供閉環機會，但「航點全部成功」不等於已證明每個閉環正確；仍需檢查地圖是否重影、錯接相鄰走道，以及重載後的定位誤差。

## 儲存與檔案檢查

巡航結束會自動儲存格網及位姿圖；也可在沒有導航目標時執行：

```bash
./scripts/dev.sh save-map store_snapshot
```

先在目標目錄的暫存資料夾完成四檔檢查，再替換目的檔。輸出位於 `ros2_ws/src/shopping_robot_navigation/maps/`：

| 檔案 | 用途 |
| --- | --- |
| `store_snapshot.yaml`、`.pgm` | AMCL／map_server 的占據格網 |
| `store_snapshot.posegraph`、`.data` | slam_toolbox 的序列化位姿圖 |

`save-map` 與巡航共用儲存程序，確認序列化服務的結果碼，以及四個本次有更新的非空檔案；任一失敗會以非零結束碼回報。格網與位姿圖為兩步驟；未通過檢查不會替換目的檔，暫存資料夾會清除。四個替換動作並非單一跨檔案原子交易。請使用新的名稱保存實驗，避免覆寫標準 `supermarket` 地圖。只在 mapping 模式儲存；本次實測 localization 節點的序列化服務雖回覆成功，卻沒有寫出位姿圖，新檢查會以失敗回報，既有舊圖也不能冒充本次輸出。

## 重載定位

先停止建圖命令與巡航，再使用同一份位姿圖的**無副檔名前綴**啟動：

```bash
# Terminal 1, after the mapping launch has completely stopped.
./scripts/dev.sh guide slam_mode:=localization map:=/ws/src/shopping_robot_navigation/maps/store_trial headless:=true
# Terminal 2: keep the SLAM RViz view open, or launch it again.
./scripts/dev.sh rviz config:=/ws/src/shopping_robot_bringup/rviz/slam.rviz
# Terminal 3: six-zone independent localization measurement.
./scripts/dev.sh loc-check -p report_file:=/repo/output/slam-localization.json
```

啟動時會檢查 `.posegraph`／`.data` 是否存在且非空；傳入 `.yaml`、不完整檔案或非有限起始姿態會提早回報。檔案完整不表示內容相容或定位正確，仍須確認節點成功載入並通過實測。替代起點使用 `scenario:=alt_start`；定位檢查也須明確指定相同情境，詳見 SCENARIOS。

若要測 AMCL，先停止上述定位流程，再以 `slam_mode:=map_server map:=.../store_trial.yaml` 重新啟動。AMCL 與 slam_toolbox 定位是不同測試，不可同時發布 `map → odom`。

## 地圖品質與限制

```bash
# Execute inside compute, with the repository's Python dependencies available.
python3 /repo/scripts/map_geometry_check.py \
  --map /ws/src/shopping_robot_navigation/maps/store_trial.yaml \
  --json /repo/output/slam-map-geometry.json
```

幾何檢查比較格網占據格中心與 SDF 貨架／牆面，列出位移、旋轉、尺寸倍率與邊緣覆蓋；遮蔽、缺面與雙邊緣會限制推論，不能只看平均值。`loc-check` 另外使用模擬真值檢查六區定位；不可用 SLAM 自己的 TF 當作獨立真值。

目前參數針對小型商店的重複貨架，限制 scan matching 與閉環搜尋範圍，降低錯接相隔 2.4 公尺走道的風險。還沒驗證真實雷射、地面滑移與長時間漂移，也不保證任意大型商店適用。Mac arm64 實測與尚未完成項目統一記錄於 [本次驗收](VALIDATION_ARM64.md)。
