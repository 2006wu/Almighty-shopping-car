#!/usr/bin/env bash
set -euo pipefail
role="${1:?role required}"
case "$role" in sim|localization|navigation|app) ;; *) exit 2 ;; esac
mode="${SPLIT_MODE:-guide}"
case "$mode" in guide|follow) ;; *) exit 2 ;; esac
args=()
while IFS= read -r arg; do
  [[ -n "$arg" ]] || continue
  case "$arg" in
    mcu_transport:=sim_ros|mcu_transport:=agent|headless:=*|odom_source:=*|nav2_stack:=*|slam_mode:=*|map:=*|fault_injection:=*|fault_schedule:=*|route_around:=*|person:=*) ;;
    *) echo "Unsupported split argument: $arg" >&2; exit 2 ;;
  esac
  args+=("$arg")
done <<< "${SPLIT_LAUNCH_ARGS:-}"
launch=(ros2 launch shopping_robot_bringup "$role.launch.py" "mode:=$mode" "${args[@]}")
if [[ "$role" == sim ]]; then exec xvfb-run -a -n 50 "${launch[@]}"; fi
exec "${launch[@]}"
