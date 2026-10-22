# 地圖輸出

此處尚未附上實測或模擬地圖。以 `scripts/dev.sh save-map supermarket` 儲存佔據格網與 SLAM 位姿圖，產物由 Git 忽略。

`slam_mode:=map_server map:=.../supermarket.yaml` 使用 AMCL；`slam_mode:=localization map:=.../supermarket` 使用 slam_toolbox 的序列化位姿圖前綴（不是格網 YAML）。預設 `mapping` 不需既有地圖。區域座標以機器人初始原點與朝向建立的 map 為準，地圖重建後應重新核對。
