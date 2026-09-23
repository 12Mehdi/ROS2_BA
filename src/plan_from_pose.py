import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

ARM = [f"j{i}" for i in range(1, 7)]      # FR5: j1..j6
GROUP = "fairino5_v6_group"
FRAME = "base_link"
TIP_LINK = "tool0"


# ---------- NEW: read the robot's current joint angles ----------

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


# ---------- Goal builder (unchanged) ----------

def make_goal(x, y, z, fixed_joint_name=None, fixed_joint_value=None,
              tolerance=0.01, group=GROUP, frame=FRAME, tip_link=TIP_LINK):
    c = Constraints()

    pc = PositionConstraint()
    pc.header.frame_id = frame
    pc.link_name = tip_link
    pc.target_point_offset.x = 0.0
    pc.target_point_offset.y = 0.0
    pc.target_point_offset.z = 0.0

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [tolerance]

    region_pose = Pose()
    region_pose.position.x, region_pose.position.y, region_pose.position.z = \
        float(x), float(y), float(z)
    region_pose.orientation.w = 1.0

    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(region_pose)
    pc.weight = 1.0
    c.position_constraints.append(pc)

    if fixed_joint_name is not None:
        jc = JointConstraint()
        jc.joint_name = fixed_joint_name
        jc.position = float(fixed_joint_value)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)

    return c


# ---------- Planner (unchanged) ----------

def plan_to_goal(start, goal_constraints, planner_id="", group=GROUP,
                  joint_names=ARM, planning_time=5.0):
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
    r.planner_id = planner_id
    r.group_name = group
    r.allowed_planning_time = planning_time
    r.num_planning_attempts = 1
    r.start_state.joint_state.name = joint_names
    r.start_state.joint_state.position = list(start)
    r.goal_constraints.append(goal_constraints)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result().motion_plan_response
    node.destroy_node()

    if res.error_code.val != 1:
        print(f"Planning failed, code {res.error_code.val}")
        return None

    pts = res.trajectory.joint_trajectory.points
    return [list(p.positions) for p in pts]


if __name__ == "__main__":
    print("Reading current joint state...")
    START = get_current_joints()

    if START is None:
        print("Could not read current joint state. Is /joint_states being published?")
    else:
        print("Current joints:", START)

        # EDIT: target position + one fixed joint (j6 here, as an example)
        goal = make_goal(x=0.4, y=0.2, z=0.3,
                          fixed_joint_name=None, fixed_joint_value=0.0,
                          tolerance=0.01)

        print("Planning path...")
        wp = plan_to_goal(START, goal)
        if wp is None:
            print("Planning failed")
        else:
            print(f"Got {len(wp)} waypoints")
            for i, p in enumerate(wp):
                print(i, p)
