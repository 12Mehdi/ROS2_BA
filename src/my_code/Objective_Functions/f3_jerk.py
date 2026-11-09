"""f3 - Smoothness: integrated squared jerk [rad^2/s^5].

Input `s` is the dict returned by bspline_traj.sample():
  s["t"]    (N,)   time stamps [s]
  s["qddd"] (N,6)  joint jerk [rad/s^3]
"""

import numpy as np

_integrate = getattr(np, "trapezoid", None) or np.trapz  # numpy < 2 compatibility


def jerk_cost(s):
    """f3 = integral over time of sum_j (jerk_j)^2 dt."""
    return float(_integrate((s["qddd"] ** 2).sum(axis=1), s["t"]))


def jerk_peak(s):
    """Largest |jerk| of any joint [rad/s^3] - use as a constraint check."""
    return float(np.abs(s["qddd"]).max())
