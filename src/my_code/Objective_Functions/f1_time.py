"""f1 - Total motion time [s].

Decision variables: dts = segment durations between the fixed OMPL waypoints.
f1 is simply their sum (the duration of the whole trajectory).
"""

import numpy as np


def time_cost(dts):
    """dts: array-like of segment durations [s]. Returns total time [s]."""
    return float(np.sum(dts))
