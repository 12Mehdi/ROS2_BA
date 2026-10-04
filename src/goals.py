"""Named goal poses for the FR5 (joints j1..j6).
 
Add a new pose by adding a line to GOALS:
    "name": [j1, j2, j3, j4, j5, j6],
 
Values are in RADIANS. If degrees are preferred, wrap them in deg(...):
    "name": deg([45, -90, 90, 0, 90, 0]),
"""
import math
 
 
def deg(angles):
    """Convert a list of angles from degrees to radians."""
    return [math.radians(a) for a in angles]
 
 
GOALS = {
    "home":       deg([45, -90, 90, 0, 90, 0]),
    "pick":       deg([135, 0, 45, -45, 90, 0]),
    "place":      deg([-27, 35, -30,  0,  0, 0]),
}
 
