#!/usr/bin/env bash
set -Eeuo pipefail

# P2/P3/P4/P5 software-only experiment: run matched-seed baseline and ablation
# episodes against the viewpoint-dependent virtual cuboid observation model.
# It never starts a real driver, camera, IO, or trajectory-execution path.

WORKSPACE="${CS625_WORKSPACE:-$HOME/elite_ros_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
RUN_ROOT="${CS625_BATCH_RUN_DIR:-$HOME/nbv_virtual_baseline_batches}"
REPLICATES="${CS625_REPLICATES:-10}"
SEED_BASE="${CS625_SEED_BASE:-625}"
BOOTSTRAP_SAMPLES="${CS625_BOOTSTRAP_SAMPLES:-10}"
OCCLUSION_LEVEL="${CS625_OCCLUSION_LEVEL:-light}"
SCENE_NAME="${CS625_SCENE_NAME:-virtual_cuboid_${OCCLUSION_LEVEL}}"
STRATEGIES="${CS625_STRATEGIES:-single_view fixed_order random_reachable uncertainty_only path_cost_only pose_gain}"
UNCERTAINTY_MODEL="${CS625_UNCERTAINTY_MODEL:-p4_sequential_information_fusion_virtual_only}"
OBSERVABILITY_MODEL="${CS625_OBSERVABILITY_MODEL:-projected_visibility_times_view_novelty}"
LAUNCH_PROFILE="${CS625_LAUNCH_PROFILE:-virtual_baseline_batch_headless}"
RUN_DIR="$RUN_ROOT/$(date -u +%Y%m%dT%H%M%SZ)"
SIM_PID=""
NBV_PID=""

mkdir -p "$RUN_DIR/episodes"

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
  local description="$1" timeout_seconds="$2"
  shift 2
  local deadline=$((SECONDS + timeout_seconds))
  until "$@" >/dev/null 2>&1; do
    (( SECONDS < deadline )) || { printf 'Timed out waiting for %s\n' "$description" >&2; return 1; }
    sleep 2
  done
}

controllers_active() {
  local state
  state="$(ros2 control list_controllers -c /controller_manager 2>/dev/null || true)"
  grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' <<< "$state" \
    && grep -Eq '^joint_trajectory_controller[[:space:]].*[[:space:]]active$' <<< "$state"
}

[[ -f "$ROS_SETUP" && -d "$WORKSPACE/src" ]] || { printf 'ROS/workspace setup missing\n' >&2; exit 2; }
set +u
source "$ROS_SETUP"
source install/setup.bash
set -u
cd "$WORKSPACE"
GIT_COMMIT="${CS625_GIT_COMMIT:-$(git rev-parse HEAD 2>/dev/null || printf 'unknown')}"

cat > "$RUN_DIR/manifest.yaml" <<EOF
data_source: synthetic_view_dependent
validity_label: research_candidate
research_scope: virtual-cuboid simulation; ground truth T_base_model is identity
planning_cost_model: euclidean_viewpoint_proxy
uncertainty_model: $UNCERTAINTY_MODEL
observability_model: $OBSERVABILITY_MODEL
occlusion_level: $OCCLUSION_LEVEL
depth_noise_std_m: 0.001
scene_name: $SCENE_NAME
seed_base: $SEED_BASE
covariance_bootstrap_samples: $BOOTSTRAP_SAMPLES
replicates_per_strategy: $REPLICATES
strategies: [$STRATEGIES]
git_commit: $GIT_COMMIT
EOF

colcon build --symlink-install --packages-select \
  eli_cs_robot_description eli_cs_robot_simulation_gz cs625_nbv \
  2>&1 | tee "$RUN_DIR/build.log"
set +u
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
  use_synthetic_camera:=true view_dependent_synthetic:=true \
  occlusion_level:="$OCCLUSION_LEVEL" depth_noise_std:=0.001 scene_name:="$SCENE_NAME" \
  data_source:=synthetic_view_dependent validity_label:=research_candidate \
  random_seed:="$SEED_BASE" covariance_bootstrap_samples:="$BOOTSTRAP_SAMPLES" git_commit:="$GIT_COMMIT" \
  launch_profile:="$LAUNCH_PROFILE" \
  > "$RUN_DIR/nbv_pipeline.log" 2>&1 &
NBV_PID=$!
wait_for "complete model cloud" 60 ros2 topic echo /cs625_nbv/model_cloud --once --field width
wait_for "NBV episode service" 60 ros2 service type /cs625_nbv/run_episode

printf 'strategy,replicate,seed,run_id,episode_directory\n' > "$RUN_DIR/run_index.csv"
for strategy in $STRATEGIES; do
  for ((replicate = 0; replicate < REPLICATES; ++replicate)); do
    seed=$((SEED_BASE + replicate))
    ros2 param set /cs625_nbv_server random_seed "$seed" >/dev/null
    ros2 param set /cs625_nbv_server episode_id "$replicate" >/dev/null
    ros2 param set /cs625_synthetic_camera random_seed "$seed" >/dev/null
    response="$RUN_DIR/${strategy}_${replicate}.response.txt"
    timeout 120 ros2 service call /cs625_nbv/run_episode cs625_nbv/srv/RunNbvEpisode \
      "{target_object_id: 'target_object', strategy_name: '$strategy', max_views: 6}" \
      | tee "$response"
    # ROS 2 Python clients currently print success=True, while some CLI
    # versions print success: true. Accept both representations.
    grep -Eqi 'success[[:space:]]*[=:][[:space:]]*(true|True)' "$response"
    episode_dir="$(sed -n "s/.*output_directory=['\"]\([^'\"]*\).*/\1/p" "$response" | tail -n 1)"
    [[ -n "$episode_dir" && -f "$episode_dir/report_summary.csv" ]] || { printf 'Missing output for %s/%s\n' "$strategy" "$replicate" >&2; exit 1; }
    run_id="$(basename "$episode_dir")"
    cp -a "$episode_dir" "$RUN_DIR/episodes/$run_id"
    printf '%s,%s,%s,%s,%s\n' "$strategy" "$replicate" "$seed" "$run_id" "$episode_dir" >> "$RUN_DIR/run_index.csv"
  done
done

for strategy in $STRATEGIES; do
  count="$(awk -F, -v strategy="$strategy" 'NR > 1 && $1 == strategy { ++n } END { print n + 0 }' "$RUN_DIR/run_index.csv")"
  [[ "$count" -eq "$REPLICATES" ]] || { printf 'Unexpected count for %s: %s\n' "$strategy" "$count" >&2; exit 1; }
done
find "$RUN_DIR/episodes" -name report_summary.csv | wc -l | grep -qx "$((REPLICATES * $(wc -w <<< "$STRATEGIES")))"
printf 'PASS: virtual baseline batch completed in %s\n' "$RUN_DIR"
