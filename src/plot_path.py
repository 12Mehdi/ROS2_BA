"""Plot the joint angles of a saved path, one subplot per joint.

Usage:
    python3 plot_path.py paths/<file>.json            # x-axis = path length
    python3 plot_path.py paths/<file>.json --index    # x-axis = waypoint number
    python3 plot_path.py paths/<file>.json --deg      # angles in degrees

Saves a PNG next to the JSON file and opens a window (if a display exists).

There is no time axis yet: the raw path has no timing. The x-axis is either
the waypoint number or the joint-space path length s (sum of straight-line
distances between waypoints, in rad), which shows how long each segment is.
"""
import argparse
import math
import os

import matplotlib.pyplot as plt

from path_io import load_path


def path_length_axis(waypoints):
    """Cumulative joint-space distance at each waypoint (s = 0 at the start)."""
    s = [0.0]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        s.append(s[-1] + math.dist(a, b))
    return s


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="saved path (.json)")
    parser.add_argument("--index", action="store_true", help="x-axis = waypoint number")
    parser.add_argument("--deg", action="store_true", help="show angles in degrees")
    args = parser.parse_args()

    data = load_path(args.file)
    wps = data["waypoints"]
    names = data["joint_names"]
    goal = data["goal"]["joints"]
    conv = math.degrees if args.deg else (lambda v: v)
    unit = "deg" if args.deg else "rad"

    if args.index:
        x = list(range(len(wps)))
        xlabel = "waypoint number"
    else:
        x = path_length_axis(wps)
        xlabel = "joint-space path length s [rad]"

    fig, axes = plt.subplots(len(names), 1, sharex=True, figsize=(8, 1.8 * len(names)))
    for j, (ax, name) in enumerate(zip(axes, names)):
        y = [conv(w[j]) for w in wps]
        ax.plot(x, y, "-o", color="tab:blue", markersize=5, label="waypoints")
        ax.axhline(conv(goal[j]), color="tab:gray", linestyle="--", linewidth=1,
                   label="goal")
        ax.set_ylabel(f"{name} [{unit}]")
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)
    axes[-1].set_xlabel(xlabel)

    title = f"{data['start']['name']} -> {data['goal']['name']}  |  " \
            f"{data['num_waypoints']} waypoints  |  {data['pipeline']}"
    if data["obstacles"]:
        title += f"  |  {len(data['obstacles'])} obstacle(s)"
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()

    out = os.path.splitext(args.file)[0] + ".png"
    fig.savefig(out, dpi=150)
    print("Saved plot to", out)
    try:
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()