#!/usr/bin/env bash
set -Eeuo pipefail

# Software-only CS625/PoseGain-NBV regression.
# This script never launches the real robot driver, camera, gripper IO, or RViz.

WORKSPACE="${CS625_WORKSPACE:-$HOME/elite_ros_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
RUN_ROOT="${CS625_REGRESSION_DIR:-$HOME/nbv_regression_runs}"
RANDOM_SEED="${CS625_RANDOM_SEED:-625}"
MAX_VIEWS="${CS625_MAX_VIEWS:-6}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$RUN_ROOT/$TIMESTAMP"
REPORT_CSV="$RUN_DIR/report_export.csv"
SIM_PID=""
NBV_PID=""

mkdir -p "$RUN_DIR"

cleanup() {
  local exit_code=$?
  if [[ -n "$NBV_PID" ]] && kill -0 "$NBV_PID" 2>/dev/null; then
    kill -INT "$NBV_PID" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 "$NBV_PID" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "$NBV_PID" 2>/dev/null || true
    wait "$NBV_PID" 2>/dev/null || true
  fi
  if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
    kill -INT "$SIM_PID" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 "$SIM_PID" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "$SIM_PID" 2>/dev/null || true
    wait "$SIM_PID" 2>/dev/null || true
  fi
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

if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'ROS setup not found: %s\n' "$ROS_SETUP" >&2
  exit 2
fi
if [[ ! -d "$WORKSPACE/src" ]]; then
  printf 'ROS workspace not found: %s\n' "$WORKSPACE" >&2
  exit 2
fi

# ROS Jazzy's generated setup script may read optional variables that are not
# defined in a fresh shell, so source it before enabling nounset protection.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
cd "$WORKSPACE"

GIT_COMMIT="$(git rev-parse HEAD 2>/dev/null || printf 'unknown')"
GIT_BRANCH="$(git branch --show-current 2>/dev/null || printf 'unknown')"
GIT_DIRTY="false"
if [[ -n "$(git status --porcelain 2>/dev/null)" ]]; then
  GIT_DIRTY="true"
fi

{
  printf 'run_id: %s\n' "$TIMESTAMP"
  printf 'data_source: synthetic_smoke_test\n'
  printf 'validity_label: interface_only\n'
  printf 'research_use_allowed: false\n'
  printf 'workspace: %s\n' "$WORKSPACE"
  printf 'git_commit: %s\n' "$GIT_COMMIT"
  printf 'git_branch: %s\n' "$GIT_BRANCH"
  printf 'git_dirty: %s\n' "$GIT_DIRTY"
  printf 'ros_distro: %s\n' "${ROS_DISTRO:-unknown}"
  printf 'random_seed: %s\n' "$RANDOM_SEED"
  printf 'max_views: %s\n' "$MAX_VIEWS"
  printf 'simulation_launch: enable_rgbd_sensor=false launch_rviz=false headless=true\n'
  printf 'nbv_launch: use_synthetic_camera=true\n'
} > "$RUN_DIR/manifest.yaml"

{
  dpkg-query -W \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control 2>&1 || true
} > "$RUN_DIR/package_versions.txt"

colcon build --symlink-install --packages-select \
  eli_cs_robot_description \
  eli_cs_robot_simulation_gz \
  cs625_nbv 2>&1 | tee "$RUN_DIR/build.log"

# colcon's generated setup script has the same optional-variable behavior as
# the ROS setup script above.
set +u
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

ros2 launch eli_cs_robot_simulation_gz nbv_simulation.launch.py \
  enable_rgbd_sensor:=false \
  launch_rviz:=false \
  headless:=true > "$RUN_DIR/simulation.log" 2>&1 &
SIM_PID=$!

wait_for "controller_manager" 90 \
  ros2 service type /controller_manager/list_controllers

controllers_active() {
  local controller_state
  controller_state="$(ros2 control list_controllers -c /controller_manager 2>/dev/null || true)"
  grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' <<< "$controller_state" \
    && grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' <<< "$controller_state"
}

wait_for "active joint controllers" 90 controllers_active
ros2 control list_controllers -c /controller_manager \
  | tee "$RUN_DIR/controllers.txt"
grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' \
  "$RUN_DIR/controllers.txt"
grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' \
  "$RUN_DIR/controllers.txt"

timeout 30 ros2 topic echo /joint_states --once \
  --qos-reliability best_effort > "$RUN_DIR/joint_states.yaml"
grep -Eq 'name:|position:' "$RUN_DIR/joint_states.yaml"

ros2 launch cs625_nbv nbv_pipeline.launch.py \
  use_synthetic_camera:=true \
  data_source:=synthetic_smoke_test \
  validity_label:=interface_only \
  random_seed:="$RANDOM_SEED" \
  git_commit:="$GIT_COMMIT" \
  launch_profile:=synthetic_headless > "$RUN_DIR/nbv_pipeline.log" 2>&1 &
NBV_PID=$!

wait_for "NBV episode service" 60 \
  ros2 service type /cs625_nbv/run_episode

timeout 30 ros2 topic echo /camera/points --once \
  --field width > "$RUN_DIR/camera_points_width.txt"

ros2 service call /cs625_nbv/run_episode cs625_nbv/srv/RunNbvEpisode \
  "{target_object_id: 'target_object', strategy_name: 'fixed_order', max_views: $MAX_VIEWS}" \
  | tee "$RUN_DIR/run_episode_response.txt"
grep -Eqi 'success[=:][[:space:]]*true' "$RUN_DIR/run_episode_response.txt"

ros2 service call /cs625_nbv/get_experiment_results \
  cs625_nbv/srv/GetExperimentResults \
  "{count: 1, filter_strategy: 'fixed_order'}" \
  > "$RUN_DIR/get_results_response.txt"

ros2 service call /cs625_nbv/export_experiment_data \
  cs625_nbv/srv/ExportExperimentData \
  "{output_path: '$REPORT_CSV', latest_count: 1, filter_strategy: 'fixed_order'}" \
  | tee "$RUN_DIR/export_response.txt"

test -s "$REPORT_CSV"
grep -q 'synthetic_smoke_test,interface_only' "$REPORT_CSV"
printf 'PASS: report-ready smoke-test data exported to %s\n' "$REPORT_CSV"
