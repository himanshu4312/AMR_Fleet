# The fleet manager's current belief about one robot in the fleet.
# Published purely for observability (e.g. `ros2 topic echo`) - nothing in
# the system subscribes to this yet; it carries no control information.

string robot_name
geometry_msgs/PoseStamped pose
float64 remaining_path_length
bool is_paused
int32 priority_rank
