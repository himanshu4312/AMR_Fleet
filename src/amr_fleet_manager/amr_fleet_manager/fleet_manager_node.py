import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Empty

from amr_fleet_msgs.msg import FleetRobotState

DEFAULT_ROBOT_NAMES = ['robot1', 'robot2', 'robot3', 'robot4']

# _update_conflicts already checks every pair of robots (not just a single
# fixed pair), so no logic changes were needed to go from 2 to 4 robots -
# only this list and the spawn poses in multi_robot_bringup.launch.py.

# nav2_params.yaml sets costmap robot_radius: 0.22m for both robots ->
# footprint (touching) diameter 0.44m. Safety distance = diameter + ~0.16m
# margin (AMCL localization noise + reaction time), rounded to 0.6m.
# Tightened down from an initial 1.0m default, which was triggering pauses
# while robots were still ~3.5m apart - much earlier than "stop when it's
# actually close".
DEFAULT_SAFETY_DISTANCE_M = 2

# Two robots' estimated arrival times at the same close point must be within
# this many seconds of each other, IN ADDITION to being spatially close, to
# count as a real conflict. See the comment in _pair_has_conflict for why.
# Tightened from an initial 7.0s so the fleet manager only reacts to
# genuinely imminent overlaps, not speculative far-future ones.
DEFAULT_TIME_WINDOW_S = 7.0

# Nominal cruise speed used only to estimate arrival times, matching
# controller_server.FollowPath.max_vel_x in nav2_params.yaml (currently
# identical for both robots, since it's one shared params file).
DEFAULT_NOMINAL_SPEED_MPS = 0.26

# Check every Nth waypoint pair instead of every consecutive pair, purely to
# bound the O(segments_a * segments_b) cost of the per-pair conflict scan on
# long/dense paths. 1 = check every segment (no downsampling).
DEFAULT_PATH_CHECK_STRIDE = 1

TIMER_PERIOD_S = 0.5  # ~2 Hz conflict-detection poll rate

# How old a robot's LAST-RECEIVED /<name>/amcl_pose is allowed to get before
# that robot is treated as "unknown" rather than trusted. Deliberately based
# on pose age, not /plan age: /plan stops updating the moment a robot
# finishes navigating to its goal and has nothing queued next - that is a
# normal, safe, terminal state, not a failure, so plan staleness alone must
# never be treated as "something is wrong". Pose, on the other hand, should
# keep arriving as long as the robot's localization is alive at all, so a
# long pose gap is a much more honest "I haven't heard from this robot"
# signal. 5.0s is generous relative to AMCL's own update cadence (avoids
# false "stale" flags on a robot that's just sitting still) while still
# being short enough to catch a genuinely dead/crashed robot within a few
# seconds. See _track_is_fresh and the fail-safe handling in
# _update_conflicts for how this is actually used - staleness can only ever
# PREVENT clearing an existing pause, never CAUSE one.
MAX_POSE_AGE_S = 5.0

HEARTBEAT_TOPIC = '/fleet_manager/heartbeat'

# SpeedLimit.msg defines speed_limit=0.0 (with percentage=True) as "no
# limit" - i.e. the CLEAR/resume value, not "stop". A paused robot is
# instead given a 1% speed limit, which is effectively stopped for our
# purposes without relying on the special zero value. See the comment at
# _publish_speed_limit for why getting this backwards would be easy.
PAUSE_SPEED_LIMIT_PERCENT = 1.0
RESUME_SPEED_LIMIT_PERCENT = 0.0


def _dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def _clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def _closest_pt_segment_segment(p1, p2, q1, q2):
    """
    Minimum distance between 2D segments p1-p2 and q1-q2, plus where along
    each segment (as a 0..1 fraction) that closest approach occurs.

    Standard clamped-parametric segment/segment closest-point algorithm
    (e.g. Ericson, "Real-Time Collision Detection", ClosestPtSegmentSegment)
    - needed because two segments can cross (or pass close) at a point that
    isn't near either segment's endpoints, so checking only endpoint-to-
    segment distances would miss real crossings.

    Returns (min_distance, s, t) where s/t in [0, 1] locate the closest
    point along p1->p2 and q1->q2 respectively.
    """
    d1x, d1y = p2[0] - p1[0], p2[1] - p1[1]
    d2x, d2y = q2[0] - q1[0], q2[1] - q1[1]
    rx, ry = p1[0] - q1[0], p1[1] - q1[1]

    a = d1x * d1x + d1y * d1y
    e = d2x * d2x + d2y * d2y
    f = d2x * rx + d2y * ry

    eps = 1e-9
    if a <= eps and e <= eps:
        s = t = 0.0
    elif a <= eps:
        s = 0.0
        t = _clamp01(f / e)
    else:
        c = d1x * rx + d1y * ry
        if e <= eps:
            t = 0.0
            s = _clamp01(-c / a)
        else:
            b = d1x * d2x + d1y * d2y
            denom = a * e - b * b
            s = _clamp01((b * f - c * e) / denom) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clamp01(-c / a)
            elif t > 1.0:
                t = 1.0
                s = _clamp01((b - c) / a)

    c1 = (p1[0] + d1x * s, p1[1] + d1y * s)
    c2 = (q1[0] + d2x * t, q1[1] + d2y * t)
    return math.hypot(c1[0] - c2[0], c1[1] - c2[1]), s, t


class RobotTrack:
    """Everything the fleet manager currently knows about one robot."""

    def __init__(self, name):
        self.name = name
        self.pose = None            # geometry_msgs/PoseStamped, from /<name>/amcl_pose
        self.pose_received_at = None  # rclpy.time.Time this node received that pose, for staleness checks
        self.points = []       # list[(x, y)], from latest /<name>/plan
        self.cumulative = []   # cumulative path distance up to points[i], same length as points

    @property
    def remaining_path_length(self):
        return self.cumulative[-1] if self.cumulative else 0.0

    def set_path(self, path_msg: Path):
        points = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        cumulative = []
        total = 0.0
        for i, pt in enumerate(points):
            if i > 0:
                total += _dist(points[i - 1], pt)
            cumulative.append(total)
        self.points = points
        self.cumulative = cumulative


class FleetManagerNode(Node):

    def __init__(self):
        super().__init__('fleet_manager_node')

        self.declare_parameter('robot_names', DEFAULT_ROBOT_NAMES)
        self.declare_parameter('safety_distance_m', DEFAULT_SAFETY_DISTANCE_M)
        self.declare_parameter('time_window_s', DEFAULT_TIME_WINDOW_S)
        self.declare_parameter('nominal_speed_mps', DEFAULT_NOMINAL_SPEED_MPS)
        self.declare_parameter('path_check_stride', DEFAULT_PATH_CHECK_STRIDE)
        self.declare_parameter('max_pose_age_s', MAX_POSE_AGE_S)

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.safety_distance_m = float(self.get_parameter('safety_distance_m').value)
        self.time_window_s = float(self.get_parameter('time_window_s').value)
        self.nominal_speed_mps = float(self.get_parameter('nominal_speed_mps').value)
        self.path_check_stride = max(1, int(self.get_parameter('path_check_stride').value))
        self.max_pose_age_s = float(self.get_parameter('max_pose_age_s').value)

        self.tracks = {name: RobotTrack(name) for name in self.robot_names}

        # Active conflict episodes: key = tuple(sorted((robot_a, robot_b))),
        # value = name of the robot that lost priority and is paused for
        # THIS episode. The winner/loser decision is made once, when the
        # episode starts, and deliberately not re-evaluated on every tick
        # while conflict_now stays True for that pair - see _update_conflicts.
        self.active_conflicts = {}

        self.speed_limit_pubs = {}
        for name in self.robot_names:
            self.speed_limit_pubs[name] = self.create_publisher(
                SpeedLimit, f'/{name}/speed_limit', 10)
            self.create_subscription(
                Path, f'/{name}/plan', self._plan_callback(name), 10)
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{name}/amcl_pose', self._pose_callback(name), 10)

        self.state_pub = self.create_publisher(FleetRobotState, '/fleet_manager/robot_states', 10)

        # A separate fleet_watchdog_node listens on this topic. This node
        # cannot detect its OWN death, so the watchdog has to be a different
        # process - see fleet_watchdog_node.py for what happens when this
        # heartbeat stops.
        self.heartbeat_pub = self.create_publisher(Empty, HEARTBEAT_TOPIC, 10)

        # Explicitly clear every robot's speed limit at startup so nobody is
        # left paused from a previous fleet_manager run.
        for name in self.robot_names:
            self._publish_speed_limit(name, resume=True)

        self.timer = self.create_timer(TIMER_PERIOD_S, self.on_timer)
        self.get_logger().info(
            f'fleet_manager watching {self.robot_names}: '
            f'safety_distance={self.safety_distance_m}m, '
            f'time_window={self.time_window_s}s, '
            f'nominal_speed={self.nominal_speed_mps}m/s, '
            f'max_pose_age={self.max_pose_age_s}s')

    # --- subscriptions -----------------------------------------------------

    def _plan_callback(self, name):
        def cb(msg: Path):
            self.tracks[name].set_path(msg)
        return cb

    def _pose_callback(self, name):
        def cb(msg: PoseWithCovarianceStamped):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            track = self.tracks[name]
            track.pose = pose
            track.pose_received_at = self.get_clock().now()
        return cb

    # --- main loop -----------------------------------------------------

    def on_timer(self):
        self.heartbeat_pub.publish(Empty())
        self._update_conflicts()
        self._publish_states()

    def _track_is_fresh(self, track: RobotTrack) -> bool:
        """True if we've heard this robot's pose recently enough to trust it."""
        if track.pose_received_at is None:
            return False
        age_s = (self.get_clock().now() - track.pose_received_at).nanoseconds / 1e9
        return age_s <= self.max_pose_age_s

    def _update_conflicts(self):
        names = self.robot_names
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                pair_key = tuple(sorted((a, b)))

                # Fail-safe: only trust a "no conflict" result when BOTH
                # robots' pose data is fresh. If either has gone stale (dead
                # AMCL, crashed nav stack, network partition, ...) we cannot
                # honestly claim to know it is safe, so conflict_now is
                # forced False for the purposes of ever STARTING a new
                # episode, but staleness must never be the reason an
                # EXISTING pause gets cleared - see the branch below.
                both_fresh = self._track_is_fresh(self.tracks[a]) and self._track_is_fresh(self.tracks[b])
                conflict_now = both_fresh and self._pair_has_conflict(a, b)

                if pair_key in self.active_conflicts:
                    if not both_fresh:
                        # Unknown is not the same as safe: keep whoever is
                        # paused, paused, until fresh data proves otherwise.
                        self.get_logger().warn(
                            f'{a} or {b} has stale pose data during an active conflict - '
                            f'keeping {self.active_conflicts[pair_key]} paused as a fail-safe '
                            f'until fresh data confirms it is actually clear.',
                            throttle_duration_sec=5.0)
                    elif not conflict_now:
                        paused_robot = self.active_conflicts.pop(pair_key)
                        self.get_logger().info(
                            f'Conflict cleared between {a} and {b}; resuming {paused_robot}')
                    # else: conflict still active - keep the existing
                    # winner/loser assignment, do not re-rank priority.
                elif conflict_now:
                    track_a = self.tracks[a]
                    track_b = self.tracks[b]
                    # Shorter remaining path keeps moving; the other pauses.
                    if track_a.remaining_path_length <= track_b.remaining_path_length:
                        winner, loser = a, b
                    else:
                        winner, loser = b, a
                    self.active_conflicts[pair_key] = loser
                    self.get_logger().info(
                        f'Conflict detected between {a} and {b}: pausing {loser} '
                        f'(remaining path {self.tracks[loser].remaining_path_length:.2f}m) '
                        f'while {winner} proceeds '
                        f'(remaining path {self.tracks[winner].remaining_path_length:.2f}m)')

        # Re-publish every robot's current desired speed limit every tick
        # (not only on state transitions), so a controller_server that
        # subscribed late, or a dropped message, still converges to the
        # correct pause/resume state instead of relying on a single publish.
        paused_now = set(self.active_conflicts.values())
        for name in names:
            self._publish_speed_limit(name, resume=(name not in paused_now))

    def _pair_has_conflict(self, name_a, name_b):
        track_a = self.tracks[name_a]
        track_b = self.tracks[name_b]
        if len(track_a.points) < 2 or len(track_b.points) < 2:
            return False

        stride = self.path_check_stride
        for i in range(0, len(track_a.points) - 1, stride):
            a1, a2 = track_a.points[i], track_a.points[i + 1]
            seg_len_a = _dist(a1, a2)
            for j in range(0, len(track_b.points) - 1, stride):
                b1, b2 = track_b.points[j], track_b.points[j + 1]

                dist, s, t = _closest_pt_segment_segment(a1, a2, b1, b2)
                if dist >= self.safety_distance_m:
                    continue

                seg_len_b = _dist(b1, b2)
                dist_a = track_a.cumulative[i] + s * seg_len_a
                dist_b = track_b.cumulative[j] + t * seg_len_b
                time_a = dist_a / self.nominal_speed_mps
                time_b = dist_b / self.nominal_speed_mps

                # Deliberately requiring BOTH spatial closeness AND arrival
                # times within time_window_s of each other. Two robots'
                # paths can cross the same physical point in the maze at
                # very different times - e.g. one robot passes through a
                # corridor junction long before the other robot ever
                # reaches it - and that is not a real collision risk.
                # Flagging every spatial path crossing regardless of timing
                # would pause robots for crossings that will never actually
                # coincide, which is both wrong and would make the fleet
                # manager unusable in a maze with many corridor
                # intersections. Do NOT "simplify" this to a spatial-only
                # check - it is intentional.
                if abs(time_a - time_b) < self.time_window_s:
                    return True
        return False

    def _publish_speed_limit(self, name, resume: bool):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.percentage = True
        # IMPORTANT: per nav2_msgs/SpeedLimit's own definition, speed_limit
        # = 0.0 (with percentage=True) means "no limit" - i.e. this is the
        # CLEAR/resume value, NOT "stop". To pause a robot we instead
        # publish 1.0 (1% of max speed, effectively stopped) - never 0.0.
        # Swapping these two constants would make paused robots drive at
        # full speed and "resumed" robots crawl at 1% forever.
        msg.speed_limit = RESUME_SPEED_LIMIT_PERCENT if resume else PAUSE_SPEED_LIMIT_PERCENT
        self.speed_limit_pubs[name].publish(msg)

    def _publish_states(self):
        paused_now = set(self.active_conflicts.values())
        # priority_rank: 1-indexed rank by remaining path length, ascending
        # (rank 1 = shortest remaining path = would win any conflict it is
        # currently part of). This is a general observability ranking,
        # computed fresh every tick regardless of whether a conflict is
        # currently active - it is NOT the same thing as active_conflicts,
        # which only decides real pause/resume actions.
        ranked = sorted(self.robot_names, key=lambda n: self.tracks[n].remaining_path_length)
        rank_of = {name: idx + 1 for idx, name in enumerate(ranked)}

        for name in self.robot_names:
            track = self.tracks[name]
            if track.pose is None:
                continue
            msg = FleetRobotState()
            # Reuse the source amcl_pose's own header: it's already
            # sim-time-correct and in the "map" frame, and represents when
            # this pose was actually true - exactly what a consumer wants
            # for judging staleness, as opposed to "when was this
            # FleetRobotState message published" which header.stamp is NOT.
            msg.header = track.pose.header
            msg.robot_name = name
            msg.pose = track.pose
            msg.remaining_path_length = track.remaining_path_length
            msg.is_paused = name in paused_now
            msg.priority_rank = rank_of[name]
            self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
