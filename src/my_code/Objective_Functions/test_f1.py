import sys
from my_code.b_splines.bspline_traj import load_trajectory
from my_code.Objective_Functions.f1_time import time_cost

s = load_trajectory(sys.argv[1])  # exported trajectory (.json)
dts = s["meta"]["dts"]  # segment durations = the decision variables

print("segment durations:", dts)
print("f1 =", time_cost(dts), "s")
print("check: last time stamp in the file =", s["t"][-1], "s")
