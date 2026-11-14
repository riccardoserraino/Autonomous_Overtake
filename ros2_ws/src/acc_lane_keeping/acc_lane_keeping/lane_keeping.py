"""
Lane keeping with pure pursuit (pure math).   Lecture 5.

Kinematic bicycle: a circle through the rear axle and a goal point on the path
at look-ahead distance L_d has curvature k = 2 sin(alpha) / L_d, where alpha is the
angle of the goal point seen from the car. Front wheel angle:
    delta = atan(L k) = atan(2 L sin(alpha) / L_d),    L_d = max(L_min, k_ld v)
CARLA-style steer command in [-1, 1], +1 = full RIGHT (ROS angles are + LEFT):
    steer = -delta / delta_max
"""
import math


def to_vehicle_frame(px, py, x, y, yaw):
    """Map point (px, py) seen from a car at (x, y, yaw): x forward, y left."""
    dx, dy = px - x, py - y
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def pure_pursuit(path_xy, x, y, yaw, v, p):
    """path_xy: list of (x, y) in the map frame. Returns the steer command [-1, 1]."""
    l_d = max(p['lookahead_min'], p['lookahead_gain'] * v)
    for px, py in path_xy:
        gx, gy = to_vehicle_frame(px, py, x, y, yaw)
        if gx > 0.0 and math.hypot(gx, gy) >= l_d:          # first point ahead at distance L_d
            alpha = math.atan2(gy, gx)
            delta = math.atan(2.0 * p['wheelbase'] * math.sin(alpha) / l_d)
            return max(-1.0, min(1.0, -delta / p['delta_max']))
    return 0.0
