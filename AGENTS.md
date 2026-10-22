# 協作規範

本專案為清華大學動力機械系28級畢業專題。2006wu 的範圍是 SLAM、Nav2 與 UWB 人物跟隨。2027 年 2 月之前沒有真實硬體，以模擬為開發基準。

- 一般整合分支為 `develop`；功能分支使用 `codex/*`，以 PR 合併至 `develop`。`main` 僅收錄里程碑。未經使用者授權，不提交或推送。
- 所有文件（包含本檔、README 與 docs）使用臺灣慣用繁體中文；程式碼、識別字、程式註解、launch／YAML 註解與提交文字使用英文。區域別名等產品資料可以包含中文。
- 固定 ROS 2 Humble、Ubuntu 22.04、Gazebo Fortress 與 `ros_gz`。不得混入 Gazebo Classic 或其他 Gazebo 版本的外掛名稱。
- 保持 amd64／arm64 原生建置；不得固定 Docker `--platform`。不針對特定車載電腦或板卡 SDK。
- 預設一律模擬，不新增硬體驅動或真實裝置連線。未來任何硬體驅動必須明確選用，例如 `use_hardware:=false` 預設值加上明確文件與啟用程序；不能因偵測到裝置就自動啟動。
- 純邏輯放在不匯入 ROS 的模組；`rclpy` 僅在 ROS 節點／命令入口匯入。重要演算法與安全行為須有 pytest 測試。
- 測試可以在 Docker 或指定的沙盒虛擬環境 `/workspace/scratch/venv-shop/bin/python` 執行；不必為純 Python 測試要求 Docker。不得在此沙盒執行 Docker。
- 修改後依範圍執行 pytest、xacro 展開、YAML 解析、launch 語法檢查與 XML 檢查。靜態檢查不能冒充容器或硬體驗收。
- 跟隨必須在過近、遺失、UWB 過期、前方障礙或雷射失效時輸出零 Twist。急停立即生效，一般移動限制速度與加速度，預設不倒車。
- 保持 `use_sim_time` 一致。`map → odom` 只可由 slam_toolbox 或 AMCL 其中一者發布；`odom → base_footprint` 由 Gazebo 差速外掛發布，其他車體 TF 由 robot_state_publisher 發布。
- 人物路徑唯一來源為 Gazebo 套件的 `config/person_path.yaml`。錨點設定與 xacro 安裝位置須同步，兩錨點假設與 NLOS 限制須說明。
- 在模式管理器完成前，不同移動流程必須互斥執行；不可同時由 Nav2、跟隨與鍵盤控制底盤。
- 每次工作階段都更新 `docs/PROGRESS.md`，記錄修改、限制與下一步；本次建立的審查驗證欄維持「待驗證」，由審查者填寫。
