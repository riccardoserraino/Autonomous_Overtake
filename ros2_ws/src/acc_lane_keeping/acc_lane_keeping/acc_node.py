"""
acc_node — adaptive cruise control + lane keeping. The node that drives the car.

  in : odometry (param odom_topic)    our car's position and speed (ground truth from the simulator)
       /carla/ego_vehicle/waypoints   centre line of the current lane (from the simulator)
       /carla/ego_vehicle/objects     other vehicles (ground truth) -> the car ahead of us
  out: /carla/ego_vehicle/vehicle_control_cmd   throttle, brake, steer

Every 0.05 s: find the car ahead on our lane -> ACC acceleration -> pedals;
              pure pursuit on the lane centre line -> steering.
"""
import math

import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from acc_lane_keeping.acc import acc_acceleration, pedals
from acc_lane_keeping.lane_keeping import pure_pursuit, to_vehicle_frame

DEFAULTS = {'odom_topic': '/carla/ego_vehicle/odometry', 'v_set': 25.0, 'lane_width': 3.5, 'ego_length': 4.8,
            'k_v': 0.5, 'k_s': 0.25, 'k_dv': 0.6, 's0': 5.0, 'time_gap': 1.5, 'a_min': -8.0, 'a_max': 2.5,
            'a_throttle_full': 3.0, 'a_brake_full': 8.0, 'wheelbase': 2.84, 'delta_max': 0.60,
            'lookahead_min': 6.0, 'lookahead_gain': 0.6}


class AccNode(Node):
    def __init__(self):
        super().__init__('acc_node')
        self.p = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        self.odom = self.lane = self.objects = None

        sub = self.create_subscription
        sub(Odometry, self.p['odom_topic'], lambda m: setattr(self, 'odom', m), 10)
        sub(Path, '/carla/ego_vehicle/waypoints', lambda m: setattr(self, 'lane', m), 10)
        sub(ObjectArray, '/carla/ego_vehicle/objects', lambda m: setattr(self, 'objects', m), 10)
        self.pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.create_timer(0.05, self.step)   # 20 Hz

    def lead_vehicle(self, x, y, yaw, path_xy):
        """Closest object ahead that is ON THE PATH we follow (within half a lane of it).
        Returns (gap, speed along our heading) or (None, 0) if the lane ahead is free."""
        best = (None, 0.0)
        for o in (self.objects.objects if self.objects else []):
            px, py = o.pose.position.x, o.pose.position.y
            ox, _ = to_vehicle_frame(px, py, x, y, yaw)
            if ox < 1.0 or min(math.hypot(px - a, py - b) for a, b in path_xy) > self.p['lane_width'] / 2:
                continue
            gap = ox - self.p['ego_length']                           # ~ bumper to bumper
            v_lead = o.twist.linear.x * math.cos(yaw) + o.twist.linear.y * math.sin(yaw)
            if best[0] is None or gap < best[0]:
                best = (gap, v_lead)
        return best

    def step(self):
        cmd = CarlaEgoVehicleControl()
        path = self.lane
        if self.odom is None or path is None:
            cmd.brake = 1.0
            self.pub.publish(cmd)
            return
        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y, v = pose.position.x, pose.position.y, self.odom.twist.twist.linear.x

        xy = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]
        gap, v_lead = self.lead_vehicle(x, y, yaw, xy)
        cmd.throttle, cmd.brake = pedals(acc_acceleration(v, self.p['v_set'], gap, v_lead, self.p), self.p)
        cmd.steer = float(pure_pursuit(xy, x, y, yaw, v, self.p))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(AccNode())
