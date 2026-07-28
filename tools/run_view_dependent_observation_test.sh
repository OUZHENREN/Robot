#!/usr/bin/env bash
set -Eeuo pipefail

# P1 software-only test: different virtual camera poses must produce different
# partial point clouds. This script never starts a real driver, camera, IO, or
# trajectory execution path.

WORKSPACE="${CS625_WORKSPACE:-$HOME/elite_ros_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
RUN_ROOT="${CS625_VIEW_TEST_DIR:-$HOME/nbv_view_dependent_runs}"
RUN_DIR="$RUN_ROOT/$(date -u +%Y%m%dT%H%M%SZ)"
SIM_PID=""
NBV_PID=""

mkdir -p "$RUN_DIR"

cleanup() {
  local exit_code=$?
  for pid in "$NBV_PID" "$SIM_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
      for _ in $(seq 1 15); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
      done
      if kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        sleep 2
      fi
      if kill -0 "$pid" 2>/dev/null; then
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
      fi
      wait "$pid" 2>/dev/null || true
    fi
  done
  printf '%s\n' "$exit_code" > "$RUN_DIR/exit_code.txt"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for() {
  local description="$1"
  local timeout_seconds="$2"
  shift 2
  local deadline=$((SECONDS + timeout_seconds))
  until "$@" >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      printf 'Timed out waiting for %s\n' "$description" >&2
      return 1
    fi
    sleep 2
  done
}

controllers_active() {
  local state
  state="$(ros2 control list_controllers -c /controller_manager 2>/dev/null || true)"
  grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' <<< "$state" \
    && grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' <<< "$state"
}

[[ -f "$ROS_SETUP" ]] || { printf 'ROS setup not found: %s\n' "$ROS_SETUP" >&2; exit 2; }
[[ -d "$WORKSPACE/src" ]] || { printf 'Workspace not found: %s\n' "$WORKSPACE" >&2; exit 2; }

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
cd "$WORKSPACE"
GIT_COMMIT="$(git rev-parse HEAD 2>/dev/null || printf 'unknown')"

cat > "$RUN_DIR/manifest.yaml" <<EOF
data_source: synthetic_view_dependent
validity_label: research_candidate
research_scope: software-generated partial point clouds with known cuboid truth
occlusion_level: light
depth_noise_std_m: 0.001
random_seed: 625
git_commit: $GIT_COMMIT
EOF

colcon build --symlink-install --packages-select \
  eli_cs_robot_description eli_cs_robot_simulation_gz cs625_nbv \
  2>&1 | tee "$RUN_DIR/build.log"

set +u
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

setsid ros2 launch eli_cs_robot_simulation_gz nbv_simulation.launch.py \
  enable_rgbd_sensor:=false launch_rviz:=false headless:=true \
  > "$RUN_DIR/simulation.log" 2>&1 &
SIM_PID=$!
wait_for "controller_manager" 90 ros2 service type /controller_manager/list_controllers
wait_for "active joint controllers" 90 controllers_active
ros2 control list_controllers -c /controller_manager > "$RUN_DIR/controllers.txt"

setsid ros2 launch cs625_nbv nbv_pipeline.launch.py \
  use_synthetic_camera:=true \
  view_dependent_synthetic:=true \
  occlusion_level:=light \
  depth_noise_std:=0.001 \
  data_source:=synthetic_view_dependent \
  validity_label:=research_candidate \
  random_seed:=625 \
  git_commit:="$GIT_COMMIT" \
  launch_profile:=view_dependent_synthetic_headless \
  > "$RUN_DIR/nbv_pipeline.log" 2>&1 &
NBV_PID=$!

wait_for "complete model cloud" 60 ros2 topic echo /cs625_nbv/model_cloud --once --field width
wait_for "NBV episode service" 60 ros2 service type /cs625_nbv/run_episode

timeout 180 ros2 service call /cs625_nbv/run_episode cs625_nbv/srv/RunNbvEpisode \
  "{target_object_id: 'target_object', strategy_name: 'fixed_order', max_views: 6}" \
  | tee "$RUN_DIR/run_episode_response.txt"
grep -Eqi 'success[=:][[:space:]]*true' "$RUN_DIR/run_episode_response.txt"
grep -q "synthetic_view_dependent" "$RUN_DIR/run_episode_response.txt"

EPISODE_DIR="$(sed -n "s/.*output_directory=['\"]\([^'\"]*\).*/\1/p" "$RUN_DIR/run_episode_response.txt" | tail -n 1)"
[[ -n "$EPISODE_DIR" && -f "$EPISODE_DIR/episode.csv" ]] || {
  printf 'Could not locate episode.csv from service response\n' >&2
  exit 1
}
cp "$EPISODE_DIR/episode.csv" "$RUN_DIR/episode.csv"
cp "$EPISODE_DIR/report_summary.csv" "$RUN_DIR/report_summary.csv"
cp "$EPISODE_DIR/summary.json" "$RUN_DIR/summary.json"
cp "$EPISODE_DIR/config.yaml" "$RUN_DIR/config.yaml"

# Column 22 is observation_points in the P1 episode CSV. A viewpoint-related
# observation test must see at least two distinct point counts.
awk -F, 'NR > 1 { print $22 }' "$RUN_DIR/episode.csv" | sort -u > "$RUN_DIR/observation_point_counts.txt"
[[ "$(wc -l < "$RUN_DIR/observation_point_counts.txt")" -ge 2 ]] || {
  printf 'Observation point counts did not vary across views\n' >&2
  exit 1
}

printf 'PASS: viewpoint-dependent observations verified in %s\n' "$RUN_DIR"
