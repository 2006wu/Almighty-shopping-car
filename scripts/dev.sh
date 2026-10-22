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
  colcon-build             Build ROS packages
  colcon-test              Test packages and report results
  test                     Run ROS-free Python tests in compute
  sim [LAUNCH_ARGS...]     Headless simulation
  mapping | slam [LAUNCH_ARGS...]    Simulation and online mapping
  guide | nav [LAUNCH_ARGS...]     Simulation, localization/mapping, Nav2, zones
  follow [LAUNCH_ARGS...]  Simulation, UWB estimator and enabled controller
  goto ZONE | zones        Navigate to a zone or list zones
  save-map NAME            Save grid and serialized SLAM pose graph
  rviz | gz-gui            Start a GUI in the display container
  novnc                    Print browser URL
Environment: DOCKER="sudo docker", VNC_PASSWORD, ROS_DOMAIN_ID
EOF
    ;;
  build) "${compose[@]}" build "$@" ;;
  up) "${compose[@]}" up -d compute display ;;
  down) "${compose[@]}" down ;;
  status) "${compose[@]}" ps -a ;;
  shell)
    service="${1:-compute}"
    [[ "$service" == compute || "$service" == display ]] || { echo 'Invalid service' >&2; exit 2; }
    "${compose[@]}" exec "$service" /usr/local/bin/compute-entrypoint bash
    ;;
  colcon-build) workspace_task colcon build --symlink-install "$@" ;;
  colcon-test)
    workspace_task colcon test "$@"
    workspace_task colcon test-result --verbose
    ;;
  test) workspace_task bash /repo/scripts/run_unit_tests.sh python3 "$@" ;;
  sim) run xvfb-run -a -n 50 ros2 launch shopping_robot_gazebo sim.launch.py "$@" ;;
  mapping|slam) run xvfb-run -a -n 50 ros2 launch shopping_robot_bringup mapping.launch.py "$@" ;;
  guide|nav) run xvfb-run -a -n 50 ros2 launch shopping_robot_bringup guide.launch.py "$@" ;;
  follow) run xvfb-run -a -n 50 ros2 launch shopping_robot_bringup follow.launch.py "$@" ;;
  goto)
    [[ $# == 1 ]] || { echo 'Usage: dev.sh goto ZONE' >&2; exit 2; }
    run ros2 run shopping_zone_manager goto_zone "$1" --ros-args -p use_sim_time:=true
    ;;
  zones) run ros2 run shopping_zone_manager list_zones ;;
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
