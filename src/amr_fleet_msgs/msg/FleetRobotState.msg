# The fleet manager's current belief about one robot in the fleet.
# Published purely for observability (e.g. `ros2 topic echo`) - nothing in
# the system subscribes to this yet; it carries no control information.
#
# header.stamp is copied from the source amcl_pose message, i.e. it is the
# time this pose was actually true, NOT the time this FleetRobotState was
# published - a consumer should treat a large (now - header.stamp) as "this
# robot's data may be stale", the same signal the fleet manager itself uses
# internally (see fleet_manager_node.py's _track_is_fresh).

std_msgs/Header header
string robot_name
geometry_msgs/PoseStamped pose
float64 remaining_path_length
bool is_paused
int32 priority_rank
