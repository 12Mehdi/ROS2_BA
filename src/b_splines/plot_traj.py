"""Build a B-spline through a saved path and plot it per joint.

Usage:
    python3 plot_trajectory.py paths/<file>.json --dts 1.5 1.5
    python3 plot_trajectory.py paths/<file>.json --dts 1.5 1.5 --degree 5
    python3 plot_trajectory.py paths/<file>.json --dts 1.5 1.5 --deg

--dts:    one duration in seconds per segment (number of waypoints - 1).
--degree: spline degree (default 4).
--deg:    show angles in degrees.
Plots position (with the waypoints), velocity, acceleration and jerk:
one row per joint, one column per quantity. Saves a PNG next to the JSON,
named with the degree so plots of different degrees do not overwrite each other.
"""
import argparse
import math
import os

import matplotlib.pyplot as plt

from bspline_traj import DEFAULT_DEGREE, build_trajectory, knot_times, sample
from OMPL_planning.path_io import load_path

QUANTITIES = [("q", "position", ""), ("qd", "velocity", "/s"),
              ("qdd", "acceleration", "/s²"), ("qddd", "jerk", "/s³")]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="saved path (.json)")
    parser.add_argument("--dts", type=float, nargs="+", required=True,
                        help="segment durations in seconds")
    parser.add_argument("--degree", type=int, default=DEFAULT_DEGREE,
                        help=f"spline degree (default {DEFAULT_DEGREE})")
    parser.add_argument("--deg", action="store_true", help="show angles in degrees")
    args = parser.parse_args()

    data = load_path(args.file)
    wps = data["waypoints"]
    names = data["joint_names"]
    traj = build_trajectory(wps, args.dts, args.degree)
    tk = knot_times(args.dts)
    s = sample(traj, tk[-1])

    scale = math.degrees(1.0) if args.deg else 1.0
    unit = "deg" if args.deg else "rad"

    fig, axes = plt.subplots(len(names), len(QUANTITIES), sharex=True,
                             figsize=(14, 1.9 * len(names)))
    for j, name in enumerate(names):
        for c, (key, label, suffix) in enumerate(QUANTITIES):
            ax = axes[j, c]
            ax.plot(s["t"], s[key][:, j] * scale, color="tab:blue", linewidth=1.5)
            if key == "q":
                ax.plot(tk, [w[j] * scale for w in wps], "o", color="tab:orange",
                        markersize=5, label="waypoints")
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

    dts_txt = ", ".join(f"{d:g}" for d in args.dts)
    fig.suptitle(f"{data['start']['name']} -> {data['goal']['name']}  |  "
                 f"B-spline degree {args.degree}  |  "
                 f"dt = [{dts_txt}] s  |  T = {tk[-1]:g} s", fontsize=10)
    fig.tight_layout()

    out_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "b_splines_traj"))
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, os.path.splitext(os.path.basename(args.file))[0] + f"_spline_deg{args.degree}.png")
    fig.savefig(out, dpi=150)
    print("Saved plot to", out)
    try:
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()