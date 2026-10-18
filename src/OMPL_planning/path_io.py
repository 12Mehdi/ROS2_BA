"""Save and load raw geometric paths (waypoints) as self-contained JSON files.

One file per planned path. Each file stores everything needed to reproduce
and describe the experiment later: waypoints, start/goal, planner settings,
the obstacles in the planning scene at planning time, and the code version.
"""
import json
import os
import subprocess
from datetime import datetime

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
PATHS_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "paths_ompl"))

FORMAT_VERSION = 1
PRIMITIVE_TYPES = {1: "box", 2: "sphere", 3: "cylinder", 4: "cone"}


def _pose_to_dict(p):
    return {
        "position": [p.position.x, p.position.y, p.position.z],
        "orientation_xyzw": [p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w],
    }


def get_obstacles(timeout_sec=5.0):
    """Ask MoveIt for the collision objects currently in the planning scene.
    Returns a list of dicts (empty list if there are none).
    Returns None if the scene could not be read."""
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("scene_reader")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error("/get_planning_scene not available")
        node.destroy_node()
        return None

    req = GetPlanningScene.Request()
    req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                 | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    scene = future.result().scene
    node.destroy_node()

    obstacles = []
    for obj in scene.world.collision_objects:
        obstacles.append({
            "id": obj.id,
            "frame_id": obj.header.frame_id,
            "pose": _pose_to_dict(obj.pose),
            "primitives": [
                {
                    "type": PRIMITIVE_TYPES.get(prim.type, str(prim.type)),
                    "dimensions": list(prim.dimensions),
                    "pose": _pose_to_dict(pose),
                }
                for prim, pose in zip(obj.primitives, obj.primitive_poses)
            ],
            "num_meshes": len(obj.meshes),
        })
    return obstacles


def _git_commit():
    """Commit hash of the code that produced the path (None if not in a git repo)."""
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                             text=True, check=True,
                             cwd=os.path.dirname(os.path.abspath(__file__)))
        dirty = subprocess.run(["git", "status", "--porcelain"], capture_output=True,
                               text=True, cwd=os.path.dirname(os.path.abspath(__file__)))
        return out.stdout.strip() + ("-dirty" if dirty.stdout.strip() else "")
    except Exception:
        return None


def save_path(waypoints, start, goal, goal_name, joint_names, *,
              pipeline, planner_id, group, start_name="current",
              obstacles=None, folder=PATHS_DIR, note=""):
    """Write one planned path to paths/<timestamp>_<start>_to_<goal>.json
    and return the file path."""
    os.makedirs(folder, exist_ok=True)
    now = datetime.now()
    data = {
        "format_version": FORMAT_VERSION,
        "created": now.isoformat(timespec="seconds"),
        "git_commit": _git_commit(),
        "note": note,
        "units": "rad",
        "joint_names": list(joint_names),
        "group": group,
        "pipeline": pipeline,
        "planner_id": planner_id,
        "start": {"name": start_name, "joints": list(start)},
        "goal": {"name": goal_name, "joints": list(goal)},
        "obstacles": obstacles if obstacles is not None else [],
        "num_waypoints": len(waypoints),
        "waypoints": [list(w) for w in waypoints],
    }
    filename = f"{now:%Y%m%d_%H%M%S}_{start_name}_to_{goal_name}.json"
    path = os.path.join(folder, filename)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    return path


def load_path(path):
    """Read a saved path file. Returns the dict exactly as saved."""
    with open(path) as f:
        data = json.load(f)
    if data.get("format_version") != FORMAT_VERSION:
        raise ValueError(f"{path}: unexpected format_version {data.get('format_version')}")
    return data
