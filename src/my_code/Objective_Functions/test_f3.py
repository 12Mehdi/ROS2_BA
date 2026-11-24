import sys
import numpy as np
from my_code.b_splines.bspline_traj import load_trajectory
from my_code.Objective_Functions.f3_jerk import jerk_cost, jerk_peak

s = load_trajectory(sys.argv[1])  # exported trajectory (.json)

print("f3 (integral of jerk^2) =", jerk_cost(s), "rad^2/s^5")
print("peak jerk               =", jerk_peak(s), "rad/s^3")

# Check 1: same integral computed by hand (sum of rectangles).
dt = s["t"][1] - s["t"][0]
print(
    "check 1: rectangle sum        =",
    float((s["qddd"] ** 2).sum() * dt),
    "(close to f3)",
)
