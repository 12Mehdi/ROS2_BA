# Obstacle scene definitions.
# Each entry: {"name": ..., "position": (x, y, z), "size": (length_x, width_y, height_z)}
# All coordinates are in metres, relative to the frame used in scene_loader.py (currently "Tisch").

GAP = 0.001  # small buffer so obstacles don't touch the table exactly (avoids a collision-checker crash)
TABLE_SURFACE_Z = 0.5  # Tisch frame: table top is at z = 0.5

SCENES = {
    "empty": [],
    "stage_a": [  # before obstacle update
        {"name": "box1", "position": (-1, -0.3, 0.75 + GAP), "size": (0.9, 0.1, 1.5)},
        {"name": "box2", "position": (0, 0, 0.3 + GAP), "size": (0.5, 0.5, 0.6)},
        {"name": "box3", "position": (1, 0, 1.0 + GAP), "size": (0.8, 1.5, 0.6)},
    ],
    "stage_b": [  # after obstacle update (Stage B test case)
        {"name": "box1", "position": (0.5, 0.3, 0.3), "size": (0.2, 0.2, 0.2)},
        {"name": "box2", "position": (0.3, -0.2, 0.4), "size": (0.15, 0.15, 0.4)},
        {"name": "box3", "position": (0.5, 0.0, 0.55), "size": (0.2, 0.2, 0.2)},
    ],
}
