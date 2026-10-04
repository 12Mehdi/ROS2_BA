import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint

ARM = [f"j{i}" for i in range(1, 7)]      # FR5: j1..j6
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

    sub = node.create_subscription(JointState, "/joint_states", cb, 10)

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


def plan_waypoints(start, goal, planner_id="", group=GROUP,
                    joint_names=ARM, planning_time=5.0):
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
    r.pipeline_id = "ompl"
    r.planner_id = planner_id            # "" = default planner
    r.group_name = group
    r.allowed_planning_time = planning_time
    r.num_planning_attempts = 1
    r.start_state.joint_state.name = joint_names
    r.start_state.joint_state.position = list(start)

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
    return [list(p.positions) for p in pts]   # positions only, no timing


if __name__ == "__main__":
    print("Reading current joint state...")
    START = get_current_joints()

    if START is None:
        print("Could not read current joint state. Is /joint_states being published?")
    else:
        print("Start (current):", START)

        # EDIT: your goal pose (radians, j1..j6)
        GOAL = [0.7854, -1.5708, 1.5708, 0.0, 1.5708, 0.0]

        print("Goal:", GOAL)
        print("Planning path...")

        wp = plan_waypoints(START, GOAL)
        if wp is None:
            print("Planning failed")
        else:
            print(f"Got {len(wp)} waypoints")
            for i, p in enumerate(wp):
                print(i, p)
