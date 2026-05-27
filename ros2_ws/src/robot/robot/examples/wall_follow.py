# place in ros2_ws/src/robot/robot/examples/wall_follow.py
from __future__ import annotations
import math
import numpy as np
from robot.lidar_scan import LidarConfig, LidarScan

DESIRED_DIST_MM = 300.0
KP = 0.5                # tune this
FORWARD_SPEED_MM_S = 120.0
SIDE_CONE_DEG = 30.0    # ± half-angle around -90°

cfg = LidarConfig(yaw_deg=180.0, range_max_mm=4000.0, units='mm')
scanner = LidarScan(cfg)

def wall_follow_step(robot):
    # Read robot obstacle cache (robot-frame in mm)
    obs = robot._obstacles_mm.copy()  # repo examples use this internal cache
    if len(obs) == 0:
        robot.set_velocity(FORWARD_SPEED_MM_S, 0.0)
        return

    # obs is (N,2) with columns [x_mm, y_mm], +y = left, so right side has y<0
    angles = np.arctan2(obs[:,1], obs[:,0])  # radians
    # Select points near -90°: angle close to -pi/2
    cone_rad = math.radians(SIDE_CONE_DEG)
    mask = np.abs(angles + math.pi/2) <= cone_rad
    side_pts = obs[mask]
    if side_pts.shape[0] == 0:
        robot.set_velocity(FORWARD_SPEED_MM_S, 0.0)
        return

    lat_dist = np.median(np.abs(side_pts[:,1]))  # lateral distance to wall (mm)
    error = DESIRED_DIST_MM - lat_dist
    ang_deg_s = -KP * error  # negative rotates clockwise (toward right wall)
    ang_deg_s = max(min(ang_deg_s, 60.0), -60.0)  # limit turning rate

    robot.set_velocity(FORWARD_SPEED_MM_S, ang_deg_s)