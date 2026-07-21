# Complete result of one NBV episode

# Episode metadata
string strategy_name           # Name of the strategy used
string scene_name              # Identifier of the occlusion scene
int32 episode_id               # Episode number (for repeats)

# Final pose estimation quality
float64 final_translation_error  # Final translation error (m)
float64 final_rotation_error     # Final rotation error (rad)
float64 final_add_score          # ADD metric (m)

# Efficiency metrics
int32 total_views              # Number of views taken (including initial)
float64 total_path_length      # Cumulative path length (m)
float64 total_planning_time    # Cumulative planning time (s)
float64 total_execution_time   # Cumulative execution time (s)

# Reachability metrics
int32 candidates_generated     # Total candidates across all iterations
int32 reachable_candidates     # How many passed IK
int32 unreachable_candidates   # How many failed IK
float64 unreachable_ratio      # unreachable / generated

# Convergence metrics
bool converged                 # True if stopped by convergence criteria
int32 stop_reason              # 0=converged, 1=max_views, 2=timeout, 3=error

# Per-viewpoint breakdown
float64[] ig_per_view          # Information gain achieved at each view
float64[] path_length_per_view # Path length for each view transition
float64[] error_per_view       # Error after each view
