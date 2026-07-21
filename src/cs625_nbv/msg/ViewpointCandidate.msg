# A single candidate viewpoint with its NBV evaluation
std_msgs/Header header

# The viewpoint pose in base_link frame
geometry_msgs/PoseStamped pose

# NBV evaluation
float64 information_gain      # IG_pose(v): expected pose uncertainty reduction
float64 utility_score          # U(v): cost-normalized utility
float64 path_length            # Estimated path length in meters
float64 planning_time          # Estimated planning time in seconds

# Reachability assessment
bool reachable                 # True if IK solution exists
bool collision_free            # True if collision-free path exists (from MoveIt)

# IK solution details (populated if reachable)
float64[6] joint_angles_deg    # Best IK solution joint angles
string configuration_label     # e.g. "Front/Up/NonFlip"
float64 ik_solution_cost       # Angular distance from reference
