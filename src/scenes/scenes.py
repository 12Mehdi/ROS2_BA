# Obstacle scene definitions.
# Each entry: {"name": ..., "position": (x, y, z), "size": (length_x, width_y, height_z)}
# All coordinates are in metres, relative to the frame used in scene_loader.py (currently "Tisch").


SCENES = {
    "empty": [],
    "stage_a": [   # before obstacle update
        {"name": "box1", "position": (-1, -0.3, 2.00001), "size": (0.9, 0.1, 3)},
        {"name": "box2", "position": (0, 0, 0.800001), "size": (0.5, 0.5, 0.6)},
        {"name": "box3", "position": (1, 0, 0.800001), "size": (0.5, 0.5, 0.6)},
    ],
    "stage_b": [   # after obstacle update (Stage B test case)
        {"name": "box1", "position": (0.5, 0.3, 0.3), "size": (0.2, 0.2, 0.2)},
        {"name": "box2", "position": (0.3, -0.2, 0.4), "size": (0.15, 0.15, 0.4)},
        {"name": "box3", "position": (0.5, 0.0, 0.55), "size": (0.2, 0.2, 0.2)},
    ],
}