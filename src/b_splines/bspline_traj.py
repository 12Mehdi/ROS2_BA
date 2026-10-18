"""Interpolating B-spline trajectory through fixed joint-space waypoints.

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
import numpy as np
from scipy.interpolate import make_interp_spline

DEFAULT_DEGREE = 4


def knot_times(dts):
    """Segment durations -> times at which the waypoints are reached."""
    dts = np.asarray(dts, dtype=float)
    if dts.ndim != 1 or np.any(dts <= 0.0):
        raise ValueError("all segment durations must be > 0")
    return np.concatenate(([0.0], np.cumsum(dts)))


def knot_vector(t, degree):
    """Clamped knot vector for waypoint times t (see module docstring)."""
    if degree % 2 == 1:
        interior = t[1:-1]                   # at the interior waypoints
    else:
        interior = (t[:-1] + t[1:]) / 2.0    # middle of every segment
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
        raise ValueError(f"{len(Q)} waypoints need {len(Q) - 1} durations, got {len(dts)}")
    zero = np.zeros(Q.shape[1])                                  # one zero per joint
    rest = [(d, zero) for d in range(1, n_end_conditions(degree) + 1)]
    return make_interp_spline(t, Q, k=degree, t=knot_vector(t, degree), bc_type=(rest, rest))


def sample(traj, total_time, dt=0.002):
    """Evaluate position, velocity, acceleration and jerk on a time grid.
    Values come from the spline's exact derivatives (no numerical differencing)."""
    t = np.linspace(0.0, total_time, int(round(total_time / dt)) + 1)
    return {
        "t": t,
        "q": traj(t),        # rad
        "qd": traj(t, 1),    # rad/s
        "qdd": traj(t, 2),   # rad/s^2
        "qddd": traj(t, 3),  # rad/s^3
    }