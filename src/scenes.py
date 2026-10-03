import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


# ---------- low-level helpers ----------

def _apply(objects):
    """Send a list of collision objects to MoveIt's planning scene."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("scene_editor")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("MoveIt (move_group) is not running")
        node.destroy_node()
        return False

    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True          # change the scene, don't replace it
    req.scene.world.collision_objects = objects

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ok = future.result().success
    node.destroy_node()
    return ok



def add_box(name, position, size, frame="Tisch",
            orientation=(0.0, 0.0, 0.0, 1.0)):
    """Add (or move) a box obstacle.

    position: (x, y, z) of the box centre, in metres, in the robot base frame
    size:     (length_x, width_y, height_z) in metres
    """
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name

    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = [float(s) for s in size]

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(p) for p in position]
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = orientation

    obj.primitives.append(shape)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return _apply([obj])


def remove_box(name, frame="Tisch"):
    """Remove one obstacle by name."""
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    obj.operation = CollisionObject.REMOVE
    return _apply([obj])


# ---------- named scenes ----------
# Define your obstacle sets here. Add as many as you like.

SCENES = {
    "empty": [],
    "stage_a": [   # before obstacle update
        {"name": "box1", "position": (-1, -0.3, 2.00001), "size": (0.9, 0.1, 3)},
        {"name": "box2", "position": (0, 0, 0.800001), "size": (0.5, 0.5, 0.6)},
    ],
    "stage_b": [   # after obstacle update (Stage B test case)
        {"name": "box1", "position": (0.5, 0.3, 0.3), "size": (0.2, 0.2, 0.2)},
        {"name": "box2", "position": (0.3, -0.2, 0.4), "size": (0.15, 0.15, 0.4)},
        {"name": "box3", "position": (0.5, 0.0, 0.55), "size": (0.2, 0.2, 0.2)},
    ],
}


def load_scene(name):
    """Add all boxes defined for this scene name."""
    for box in SCENES[name]:
        add_box(box["name"], box["position"], box["size"])
    print(f"Loaded scene '{name}' ({len(SCENES[name])} obstacle(s))")


def clear_scene(name):
    """Remove all boxes defined for this scene name."""
    for box in SCENES[name]:
        remove_box(box["name"])
    print(f"Cleared scene '{name}'")


if __name__ == "__main__":
    import sys
    load_scene(sys.argv[1] if len(sys.argv) > 1 else "single_box")
