#!/usr/bin/env bash
set -eo pipefail
# Build from the apt base, without a stale fork or application overlay.
source /opt/ros/humble/setup.bash
cd /nav2_ws
rm -f install/fork_commit
[[ -f src/custom_controller/package.xml && -f src/custom_bts/custom_nav_to_pose/package.xml ]] || {
  echo 'Nav2 fork 未初始化 / not initialized: run git submodule update --init third_party/eurobot_nav2 on host.' >&2
  exit 2
}
python3 /repo/scripts/nav2/check_nav2_contract.py --source-only
mapfile -t packages < /repo/docker/nav2_fork_packages.txt
export MAKEFLAGS="${MAKEFLAGS:--j3}"
override=()
# Older colcon-core releases have no --allow-overriding (they only warn about overlays).
if colcon build --help 2>/dev/null | grep -q -- '--allow-overriding'; then
  override=(--allow-overriding "${packages[@]}")
fi
colcon build --packages-up-to "${packages[@]}" --parallel-workers "${NAV2_PARALLEL_WORKERS:-2}" \
  "${override[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
source install/setup.bash
python3 /repo/scripts/nav2/check_nav2_linkage.py
# A successful marker is written only after the link check.
printf '%s\n' "${NAV2_FORK_COMMIT:?Host must provide NAV2_FORK_COMMIT}" > install/fork_commit
