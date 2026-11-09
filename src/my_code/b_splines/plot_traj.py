"""Plot an exported trajectory (made by bspline_traj.py) per joint.

Usage:
    python3 plot_traj.py <file>.json
    python3 plot_traj.py <file>.json --deg

--deg:    show angles in degrees.
Plots position (with the waypoints), velocity, acceleration and jerk:
one row per joint, one column per quantity. Saves a PNG in b_splines_traj/,
named after the JSON file, so plots of different trajectories do not overwrite
each other. No ROS needed: everything is read from the exported JSON.
"""

import argparse
import math
import os

import matplotlib.pyplot as plt

from my_code.b_splines.bspline_traj import load_trajectory

QUANTITIES = [
    ("q", "position", ""),
    ("qd", "velocity", "/s"),
    ("qdd", "acceleration", "/s²"),
    ("qddd", "jerk", "/s³"),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="exported trajectory (.json)")
    parser.add_argument("--deg", action="store_true", help="show angles in degrees")
    args = parser.parse_args()

    s = load_trajectory(args.file)
    meta = s["meta"]
    wps = meta["waypoints"]
    tk = meta["knot_times"]
    dts = meta["dts"]
    degree = meta["degree"]
    names = meta["joint_names"] or [f"j{i + 1}" for i in range(s["q"].shape[1])]

    scale = math.degrees(1.0) if args.deg else 1.0
    unit = "deg" if args.deg else "rad"

    fig, axes = plt.subplots(
        len(names), len(QUANTITIES), sharex=True, figsize=(14, 1.9 * len(names))
    )
    for j, name in enumerate(names):
        for c, (key, label, suffix) in enumerate(QUANTITIES):
            ax = axes[j, c]
            ax.plot(s["t"], s[key][:, j] * scale, color="tab:blue", linewidth=1.5)
            if key == "q":
                ax.plot(
                    tk,
                    [w[j] * scale for w in wps],
                    "o",
                    color="tab:orange",
                    markersize=5,
                    label="waypoints",
                )
            for t in tk[1:-1]:
                ax.axvline(t, color="tab:gray", linestyle=":", linewidth=0.8)
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.set_title(label, fontsize=10)
            if c == 0:
                ax.set_ylabel(f"{name}\n[{unit}]")
            elif j == 0:
                ax.set_ylabel(f"[{unit}{suffix}]", fontsize=8)
    axes[0, 0].legend(loc="best", fontsize=8)
    for c in range(len(QUANTITIES)):
        axes[-1, c].set_xlabel("time [s]")

    dts_txt = ", ".join(f"{d:g}" for d in dts)
    start = (meta.get("start") or {}).get("name", "start")
    goal = (meta.get("goal") or {}).get("name", "goal")
    fig.suptitle(
        f"{start} -> {goal}  |  "
        f"B-spline degree {degree}  |  "
        f"dt = [{dts_txt}] s  |  T = {tk[-1]:g} s",
        fontsize=10,
    )
    fig.tight_layout()

    out_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "b_splines_traj")
    )
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(
        out_dir, os.path.splitext(os.path.basename(args.file))[0] + "_plot.png"
    )
    fig.savefig(out, dpi=150)
    print("Saved plot to", out)
    try:
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
