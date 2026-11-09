import sys
import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from my_code.OMPL_planning.goals import GOALS
from my_code.OMPL_planning.path_io import save_path, get_obstacles

ARM = [f"j{i}" for i in range(1, 7)]  # FR5: j1..j6
GROUP = "fairino5_v6_group"


def get_current_joints(joint_names=ARM, timeout_sec=5.0):
    """Listen to /joint_states once and return the current angles for
    joint_names, in that order. Returns None if no message arrives."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("joint_state_reader")

    result = {}

    def cb(msg):
        for n, p in zip(msg.name, msg.position):
            result[n] = p

    node.create_subscription(JointState, "/joint_states", cb, 10)

    end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
    while node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(n in result for n in joint_names):
            break

    node.destroy_node()

    if not all(n in result for n in joint_names):
        print("Did not receive current joint states in time")
        return None
    return [result[n] for n in joint_names]


def plan_waypoints(
    start, goal, planner_id="", group=GROUP, joint_names=ARM, planning_time=5.0
):
    """Ask MoveIt (OMPL inside) for a path between two joint-space poses.

    start, goal: lists of 6 joint angles (radians), in joint_names order
    Returns a list of joint-angle lists (radians), with no timing.
    Returns None if planning fails.
    """
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("waypoint_planner")
    client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("MoveIt (move_group) is not running")
        node.destroy_node()
        return None

    req = GetMotionPlan.Request()
    r = req.motion_plan_request
    r.pipeline_id = "ompl_raw"
    r.max_velocity_scaling_factor = 1.0
    r.max_acceleration_scaling_factor = 1.0
    r.planner_id = planner_id  # "" = default planner
    r.group_name = group
    r.allowed_planning_time = planning_time
    r.num_planning_attempts = 10
    r.start_state.joint_state.name = joint_names
    r.start_state.joint_state.position = list(start)

    # Workspace box (only affects mobile bases; set to silence the warning)
    ws = r.workspace_parameters
    ws.header.frame_id = "base_link"
    ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -4.0
    ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 4.0

    c = Constraints()
    for n, v in zip(joint_names, goal):
        jc = JointConstraint()
        jc.joint_name = n
        jc.position = float(v)
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    r.goal_constraints.append(c)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result().motion_plan_response
    node.destroy_node()

    if res.error_code.val != 1:
        print(f"Planning failed, code {res.error_code.val}")
        return None

    pts = res.trajectory.joint_trajectory.points
    return [list(p.positions) for p in pts]  # positions only, no timing


def print_available_goals():
    print("Available goals (from goals.py):")
    for name in GOALS:
        print("  -", name)


if __name__ == "__main__":
    # Usage:
    #   python3 plan_joint_space.py pick              -> current pose to "pick"
    #   python3 plan_joint_space.py pick place home   -> chain of goals
    names = sys.argv[1:]

    if not names:
        print("Please give at least one goal name.")
        print_available_goals()
        sys.exit(1)

    unknown = [n for n in names if n not in GOALS]
    if unknown:
        print("Unknown goal(s):", ", ".join(unknown))
        print_available_goals()
        sys.exit(1)

    print("Reading current joint state...")
    start = get_current_joints()
    if start is None:
        print("Could not read current joint state. Is /joint_states being published?")
        sys.exit(1)
    print("Start (current):", start)

    all_paths = {}
    for name in names:
        goal = GOALS[name]
        print(f"\nPlanning to '{name}': {goal}")
        wp = plan_waypoints(start, goal, planner_id="RRTConnectkConfigDefault")
        if wp is None:
            print(f"Planning to '{name}' failed, stopping here.")
            break
        print(f"Got {len(wp)} waypoints")
        for i, p in enumerate(wp):
            print(i, p)
        all_paths[name] = wp
        file = save_path(
            wp,
            start,
            goal,
            name,
            ARM,
            pipeline="ompl_raw",
            planner_id="RRTConnectkConfigDefault",
            group=GROUP,
            obstacles=get_obstacles(),
        )
        print("Saved to", file)
        start = wp[-1]  # next segment starts where this one ended
