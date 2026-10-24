#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
read -r -a docker_cmd <<< "${DOCKER:-docker}"
compose=("${docker_cmd[@]}" compose -f "$repo_dir/docker/docker-compose.yaml")
command="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
run() { "${compose[@]}" exec compute /usr/local/bin/compute-entrypoint "$@"; }
# Build and tests can run before the long-lived services start.
workspace_task() { "${compose[@]}" run --rm --no-deps compute "$@"; }
# All launch commands run in the foreground; Ctrl-C stops the active session.
case "$command" in
  help|-h|--help)
    cat <<'EOF'
Usage: ./scripts/dev.sh COMMAND [ARGS...]
  build [SERVICE...]       Build native-architecture images
  up | down | status       Manage compute and display containers
  shell [compute|display]  Open a ROS-sourced shell
  nav2-build              Build the initialized private Nav2 underlay
  nav2-status             Show fork checkout and build commit
  nav-compare [ARGS...]    Record one stack or compare two JSON reports
  colcon-build             Build ROS packages
  colcon-test              Test packages and report results
  test                     Run ROS-free Python tests in compute
  split-up [guide|follow] [ARGS...]  Start role containers (simulation only)
  split-down              Stop role containers and optional profiles
  split-status            Show split/profile container status
  split-logs SERVICE      Follow one role's logs
  scenario-list            List validated simulation scenarios (no Docker)
  comm-up | comm-down     Manage the opt-in UDP micro-ROS agent
  comm-sim [LAUNCH_ARGS...]  Standalone simulated MCU, monitor, mux and watchdog
  comm-e2e-up             Start agent + C client, then standalone mux/watchdog/monitor
  comm-e2e-check          Check isolated XRCE simulation (injects motion/faults)
  comm-check              Check an isolated running comm-sim (injects motion/faults)
  sim [LAUNCH_ARGS...]     Headless simulation
  mapping | slam [LAUNCH_ARGS...]    Simulation and online mapping
  guide | nav [LAUNCH_ARGS...]     Simulation, localization/mapping, Nav2, zones
  follow [LAUNCH_ARGS...]  Simulation, UWB estimator and enabled controller
  goto ZONE [--timeout S] | zones  Navigate to a zone or list zones
  cancel                   Request cancellation of the current zone goal
  nav-check [ARGS...]       Run simulated navigation action checks
  map-tour [NAME]          Run mapping tour and save maps (default: supermarket)
  loc-check [ROS_ARGS...]   Check all zones against simulation truth
  mode MODE | estop | reset  Request idle/guide/follow/teleop/estop/reset
  person-sync-check [ARGS...]  Compare person model, UWB truth and shared path
  follow-eval [ARGS...]     Measure latency, turns, near stops and reacquisition
  yaw-check [ARGS...]      Measure yaw/speed against a running plain sim
  fault-check [ARGS...]    Verify faults against a running opt-in follow sim
  record NAME [--duration S] [--meta KEY=VALUE] [TOPICS...]  Record evaluation bag (wall seconds)
  eval-bag NAME [ARGS...]  Convert bag and write report.json/report.md
  eval-verify NAME        Reconvert and compare stored evaluation
  eval-summary NAME... [--output DIR]  Aggregate evaluation reports
  save-map NAME            Save grid and serialized SLAM pose graph
  rviz | gz-gui            Start a GUI in the display container
  novnc                    Print browser URL
Environment: DOCKER="sudo docker", VNC_PASSWORD, ROS_DOMAIN_ID
EOF
    ;;
  scenario-list)
    [[ $# == 0 ]] || { echo 'No arguments expected' >&2; exit 2; }
    scenario_python="${PYTHON:-python3}"
    if [[ -z "${PYTHON:-}" && -x /workspace/scratch/venv-shop/bin/python ]]; then
      scenario_python=/workspace/scratch/venv-shop/bin/python
    fi
    PYTHONPATH="$repo_dir/ros2_ws/src/shopping_robot_gazebo${PYTHONPATH:+:$PYTHONPATH}" "$scenario_python" "$repo_dir/scripts/scenario_list.py"
    ;;
  build) "${compose[@]}" build "$@" ;;
  up) "${compose[@]}" up -d compute display ;;
  down) "${compose[@]}" down ;;
  status) "${compose[@]}" ps -a ;;
  split-up)
    split_mode="guide"
    if [[ "${1:-}" == guide || "${1:-}" == follow ]]; then split_mode="$1"; shift; fi
    for arg in "$@"; do
      [[ "$arg" != *$'\n'* ]] || { echo 'Newlines are not allowed in split arguments' >&2; exit 2; }
      case "$arg" in
        mcu_transport:=sim_ros|mcu_transport:=agent|headless:=*|odom_source:=*|nav2_stack:=*|slam_mode:=*|map:=*|fault_injection:=*|fault_schedule:=*|route_around:=*|person:=*) ;;
        *) echo "Unsupported split argument: $arg" >&2; exit 2 ;;
      esac
    done
    # Recreate the idle namespace owner so old foreground launches cannot survive.
    "${compose[@]}" --profile split --profile comm --profile record down
    export SPLIT_MODE="$split_mode"
    SPLIT_LAUNCH_ARGS="$(printf '%s\n' "$@")"
    export SPLIT_LAUNCH_ARGS
    "${compose[@]}" --profile split up -d compute sim localization navigation app display
    ;;
  split-down) "${compose[@]}" --profile split --profile comm --profile record down ;;
  split-status) "${compose[@]}" --profile split --profile comm --profile record ps -a ;;
  split-logs)
    [[ $# == 1 ]] || { echo 'Usage: dev.sh split-logs SERVICE' >&2; exit 2; }
    case "$1" in compute|sim|localization|navigation|app|display|microros_agent|microros_client|record) ;;
      *) echo 'Invalid split service' >&2; exit 2 ;;
    esac
    "${compose[@]}" --profile split --profile comm --profile record logs -f "$1"
    ;;
  shell)
    service="${1:-compute}"
    [[ "$service" == compute || "$service" == display ]] || { echo 'Invalid service' >&2; exit 2; }
    "${compose[@]}" exec "$service" /usr/local/bin/compute-entrypoint bash
    ;;
  nav2-build)
    [[ -f "$repo_dir/third_party/eurobot_nav2/src/custom_controller/package.xml" ]] || {
      echo 'Nav2 fork 未初始化 / not initialized: git submodule update --init third_party/eurobot_nav2' >&2
      exit 2
    }
    "${compose[@]}" run --rm --no-deps --entrypoint /bin/bash \
      -e "NAV2_FORK_COMMIT=$(git -C "$repo_dir/third_party/eurobot_nav2" rev-parse HEAD)" \
      -e "NAV2_PARALLEL_WORKERS=${NAV2_PARALLEL_WORKERS:-2}" -e "MAKEFLAGS=${MAKEFLAGS:--j3}" \
      compute /repo/scripts/nav2_build.sh "$@"
    ;;
  nav2-status)
    if [[ -f "$repo_dir/third_party/eurobot_nav2/src/custom_controller/package.xml" ]]; then
      echo "Fork initialized: $(git -C "$repo_dir/third_party/eurobot_nav2" rev-parse HEAD)"
    else
      echo 'Fork not initialized / 尚未初始化 (auto uses apt)'
    fi
    "${compose[@]}" run --rm --no-deps --entrypoint /bin/bash compute -c \
      'if [[ -f /nav2_ws/install/setup.bash && -f /nav2_ws/install/fork_commit ]]; then echo "Built fork: $(cat /nav2_ws/install/fork_commit)"; else echo "Fork not built / 尚未建置"; fi'
    ;;
  nav-compare) run ros2 run shopping_zone_manager nav_compare "$@" --ros-args -p use_sim_time:=true ;;
  colcon-build) workspace_task colcon build --symlink-install "$@" ;;
  colcon-test)
    workspace_task colcon test "$@"
    workspace_task colcon test-result --verbose
    ;;
  test) workspace_task bash /repo/scripts/run_unit_tests.sh python3 "$@" ;;
  comm-up) "${compose[@]}" --profile comm up -d microros_agent ;;
  comm-down) "${compose[@]}" --profile comm stop microros_client microros_agent ;;
  comm-e2e-up)
    "${compose[@]}" --profile comm up -d --build microros_agent microros_client
    run ros2 launch shopping_mcu_bridge comm.launch.py standalone:=true transport:=agent use_sim_time:=false
    ;;
  comm-e2e-check) run ros2 run shopping_mcu_bridge agent_check "$@" ;;
  comm-sim) run ros2 launch shopping_mcu_bridge comm.launch.py standalone:=true "$@" ;;
  comm-check) run ros2 run shopping_mcu_bridge comm_check "$@" ;;
  sim) run xvfb-run -a -n 50 ros2 launch shopping_robot_gazebo sim.launch.py "$@" ;;
  mapping|slam) run xvfb-run -a -n 50 ros2 launch shopping_robot_bringup mapping.launch.py "$@" ;;
  guide|nav) run xvfb-run -a -n 50 ros2 launch shopping_robot_bringup guide.launch.py "$@" ;;
  follow) run xvfb-run -a -n 50 ros2 launch shopping_robot_bringup follow.launch.py "$@" ;;
  person-sync-check) run ros2 run shopping_uwb_follow person_sync_check "$@" --ros-args -p use_sim_time:=true ;;
  follow-eval) run ros2 run shopping_uwb_follow follow_eval "$@" --ros-args -p use_sim_time:=true ;;
  yaw-check) run python3 /repo/scripts/yaw_rate_check.py "$@" ;;
  fault-check) run python3 /repo/scripts/fault_check.py "$@" ;;
  goto)
    [[ $# -ge 1 ]] || { echo 'Usage: dev.sh goto ZONE [--timeout S]' >&2; exit 2; }
    run ros2 run shopping_zone_manager goto_zone "$@" --ros-args -p use_sim_time:=true
    ;;
  cancel) run ros2 run shopping_zone_manager cancel_zone --ros-args -p use_sim_time:=true ;;
  nav-check) run ros2 run shopping_zone_manager nav_action_check "$@" --ros-args -p use_sim_time:=true ;;
  mode|estop|reset)
    mode_request="$command"
    if [[ "$command" == mode ]]; then
      [[ $# == 1 ]] || { echo 'Usage: dev.sh mode MODE' >&2; exit 2; }
      mode_request="$1"
    else
      [[ $# == 0 ]] || { echo 'No arguments expected' >&2; exit 2; }
    fi
    case "$mode_request" in idle|guide|follow|teleop|estop|reset) ;; *) echo 'Invalid mode' >&2; exit 2 ;; esac
    run ros2 topic pub --once /mode_manager/request std_msgs/msg/String "{data: '$mode_request'}"
    ;;
  record) run python3 /repo/scripts/record_bag.py "$@" ;;
  eval-bag)
    [[ $# -ge 1 && "$1" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Use a simple bag name' >&2; exit 2; }
    name="$1"
    shift
    run ros2 run shopping_zone_manager bag_to_eval "/repo/bags/$name" --output "/repo/bags/$name/observations.jsonl" \
      --zones /ws/src/shopping_zone_manager/config/zones.yaml --map-world-transform 0 0 0
    run ros2 run shopping_zone_manager eval_report "/repo/bags/$name/observations.jsonl" "$@" \
      --world /ws/src/shopping_robot_gazebo/worlds/supermarket.sdf --output "/repo/bags/$name/report"
    ;;
  eval-verify)
    [[ $# == 1 && "$1" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Use a simple bag name' >&2; exit 2; }
    run ros2 run shopping_zone_manager eval_verify "$1"
    ;;
  eval-summary)
    [[ $# -ge 1 ]] || { echo 'Provide bag names' >&2; exit 2; }
    summary_names=()
    summary_output=()
    while [[ $# -gt 0 ]]; do
      if [[ "$1" == --output ]]; then
        [[ $# == 2 && -n "$2" ]] || { echo 'Use --output DIR after bag names' >&2; exit 2; }
        summary_output=(--output "$2")
        break
      fi
      [[ "$1" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Use a simple bag name' >&2; exit 2; }
      summary_names+=("$1")
      shift
    done
    [[ ${#summary_names[@]} -gt 0 ]] || { echo 'Provide bag names' >&2; exit 2; }
    run ros2 run shopping_zone_manager eval_summary "${summary_names[@]}" "${summary_output[@]}"
    ;;
  zones) run ros2 run shopping_zone_manager list_zones ;;
  map-tour)
    [[ $# -le 1 ]] || { echo 'Usage: dev.sh map-tour [NAME]' >&2; exit 2; }
    name="${1:-supermarket}"
    [[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Use a simple map name' >&2; exit 2; }
    run ros2 run shopping_zone_manager map_tour --ros-args -p use_sim_time:=true -p "map_name:=$name"
    ;;
  loc-check)
    run ros2 run shopping_zone_manager localization_check --ros-args -p use_sim_time:=true "$@"
    ;;
  save-map)
    [[ $# == 1 && "$1" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Use a simple map name' >&2; exit 2; }
    path="/ws/src/shopping_robot_navigation/maps/$1"
    run ros2 run nav2_map_server map_saver_cli -f "$path" --ros-args -p use_sim_time:=true
    run ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$path'}"
    ;;
  rviz) "${compose[@]}" exec display /usr/local/bin/compute-entrypoint ros2 launch shopping_robot_bringup rviz.launch.py "$@" ;;
  gz-gui) "${compose[@]}" exec display /usr/local/bin/compute-entrypoint ign gazebo -g "$@" ;;
  novnc) echo 'http://localhost:6080/vnc.html (VNC: 127.0.0.1:5910)' ;;
  *) echo "Unknown command: $command" >&2; exit 2 ;;
esac
