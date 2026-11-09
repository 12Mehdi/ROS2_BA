"""Interpolating B-spline trajectory through fixed joint-space waypoints.

Run it to build a trajectory from a saved path and export ALL values to JSON:
    python3 bspline_traj.py <file>.json --dts 1.5 1.5
    python3 bspline_traj.py <file>.json --dts 1.5 1.5 --degree 5 --dt 0.005
    python3 bspline_traj.py <file>.json --dts 1.5 1.5 --out my_traj.json
  --dts:    one duration in seconds per segment (number of waypoints - 1).
  --degree: spline degree (default 4).
  --dt:     sampling step of the exported grid in seconds (default 0.002).
  --out:    output file (default: src/trajectories/<path name>_deg<degree>.json).
Then plot it with plot_traj.py, or load it in a test script (no ROS needed):
    from bspline_traj import load_trajectory
    s = load_trajectory("trajectories/<file>.json")   # same dict as sample()

Decision variables (option 1): the segment durations dts = [dt_1, ..., dt_n],
one per segment between consecutive waypoints.

For every joint the spline
  - passes exactly through every waypoint, at time t_k = dt_1 + ... + dt_k,
  - has the chosen degree p (default 4; p >= 4 gives a continuous jerk),
  - starts and ends at rest.

Knots and extra conditions depend on whether the degree is odd or even:

  odd p   knots AT the interior waypoints        -> p - 1 extra conditions
  even p  knots in the MIDDLE of every segment   -> p     extra conditions

Both cases give an even number of conditions, which is split evenly between
the two ends. At each end the first c/2 derivatives are set to zero:

  p = 3         velocity = 0 at both ends
  p = 4, 5      velocity = acceleration = 0 at both ends
  p = 6, 7      velocity = acceleration = jerk = 0 at both ends

Reference: Biagiotti & Melchiorri (2008), Sec. 4.5 "B-spline functions for
trajectories with high degree of continuity". Check the exact statement on
knot placement and extra conditions there before citing it.
"""

import argparse
import json
import os
from datetime import datetime

import numpy as np
from scipy.interpolate import make_interp_spline

DEFAULT_DEGREE = 5
FORMAT_VERSION = 1
DEFAULT_OUT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "b_splines_traj")
)


def knot_times(dts):
    """Segment durations -> times at which the waypoints are reached."""
    dts = np.asarray(dts, dtype=float)
    if dts.ndim != 1 or np.any(dts <= 0.0):
        raise ValueError("all segment durations must be > 0")
    return np.concatenate(([0.0], np.cumsum(dts)))


def knot_vector(t, degree):
    """Clamped knot vector for waypoint times t (see module docstring)."""
    if degree % 2 == 1:
        interior = t[1:-1]  # at the interior waypoints
    else:
        interior = (t[:-1] + t[1:]) / 2.0  # middle of every segment
    return np.concatenate(([t[0]] * (degree + 1), interior, [t[-1]] * (degree + 1)))


def n_end_conditions(degree):
    """Number of derivatives set to zero at EACH end (1 = velocity, 2 = + acceleration, ...)."""
    total = degree - 1 if degree % 2 == 1 else degree
    return total // 2


def build_trajectory(waypoints, dts, degree=DEFAULT_DEGREE):
    """Return a spline q(t) for all joints at once.

    waypoints: (n+1) x 6 joint angles [rad]
    dts:       n segment durations [s]
    degree:    spline degree p (>= 3)
    The result is callable: traj(t) -> angles, traj(t, 1) -> velocities, etc.
    """
    if degree < 3:
        raise ValueError("degree must be at least 3")
    Q = np.asarray(waypoints, dtype=float)
    t = knot_times(dts)
    if len(t) != len(Q):
        raise ValueError(
            f"{len(Q)} waypoints need {len(Q) - 1} durations, got {len(dts)}"
        )
    zero = np.zeros(Q.shape[1])  # one zero per joint
    rest = [(d, zero) for d in range(1, n_end_conditions(degree) + 1)]
    return make_interp_spline(
        t, Q, k=degree, t=knot_vector(t, degree), bc_type=(rest, rest)
    )


def sample(traj, total_time, dt=0.002):
    """Evaluate position, velocity, acceleration and jerk on a time grid.
    Values come from the spline's exact derivatives (no numerical differencing)."""
    t = np.linspace(0.0, total_time, int(round(total_time / dt)) + 1)
    return {
        "t": t,
        "q": traj(t),  # rad
        "qd": traj(t, 1),  # rad/s
        "qdd": traj(t, 2),  # rad/s^2
        "qddd": traj(t, 3),  # rad/s^3
    }


def export_trajectory(
    out_file,
    waypoints,
    dts,
    degree=DEFAULT_DEGREE,
    dt=0.002,
    joint_names=None,
    start=None,
    goal=None,
    source=None,
):
    """Build the spline, sample it, and write everything to out_file (JSON)."""
    tk = knot_times(dts)
    s = sample(build_trajectory(waypoints, dts, degree), tk[-1], dt)
    data = {
        "format_version": FORMAT_VERSION,
        "created": datetime.now().isoformat(timespec="seconds"),
        "source_path_file": source,
        "start": start,
        "goal": goal,
        "joint_names": joint_names,
        "degree": degree,
        "dts": [float(d) for d in dts],
        "knot_times": tk.tolist(),
        "total_time": float(tk[-1]),
        "sample_dt": dt,
        "waypoints": np.asarray(waypoints, dtype=float).tolist(),
        "units": {
            "t": "s",
            "q": "rad",
            "qd": "rad/s",
            "qdd": "rad/s^2",
            "qddd": "rad/s^3",
        },
        "t": s["t"].tolist(),
        "q": s["q"].tolist(),
        "qd": s["qd"].tolist(),
        "qdd": s["qdd"].tolist(),
        "qddd": s["qddd"].tolist(),
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_file)), exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(data, f)
    return data


def load_trajectory(file):
    """Read an exported file. Returns the same dict as sample() (numpy arrays
    t, q, qd, qdd, qddd) plus the metadata under the key "meta"."""
    with open(file) as f:
        data = json.load(f)
    s = {k: np.asarray(data[k], dtype=float) for k in ("t", "q", "qd", "qdd", "qddd")}
    s["meta"] = {k: v for k, v in data.items() if k not in s}
    return s


def main():
    from my_code.OMPL_planning.path_io import (
        load_path,
    )  # needs ROS; only used when run as a script

    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="saved path (.json)")
    parser.add_argument(
        "--dts",
        type=float,
        nargs="+",
        required=True,
        help="segment durations in seconds",
    )
    parser.add_argument(
        "--degree",
        type=int,
        default=DEFAULT_DEGREE,
        help=f"spline degree (default {DEFAULT_DEGREE})",
    )
    parser.add_argument("--dt", type=float, default=0.002, help="sampling step [s]")
    parser.add_argument("--out", default=None, help="output .json file")
    args = parser.parse_args()

    data = load_path(args.file)
    out = args.out or os.path.join(
        DEFAULT_OUT_DIR,
        os.path.splitext(os.path.basename(args.file))[0] + f"_deg{args.degree}.json",
    )
    d = export_trajectory(
        out,
        data["waypoints"],
        args.dts,
        args.degree,
        args.dt,
        data["joint_names"],
        data.get("start"),
        data.get("goal"),
        os.path.basename(args.file),
    )
    print(f"Saved {len(d['t'])} samples, T = {d['total_time']:g} s -> {out}")


if __name__ == "__main__":
    main()
