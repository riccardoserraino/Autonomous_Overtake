"""
carla_adapter — gives CARLA the same extra topics that WebDriveSim already has,
so our modules run unchanged on both simulators.

  out: /carla/ego_vehicle/waypoints       nav_msgs/Path: centre line of the current lane, 200 m ahead
       /webdrivesim/ego_vehicle/lane_state       Float32MultiArray (same layout as WebDriveSim):
         [0] offset from lane centre (m, + left)  [1] heading error (rad)  [2] curvature (1/m)
         [3] lane width  [4] lane number from the right (1 = rightmost)  [5] left marking
         [6] right marking (0 none, 1 broken, 2 solid)  [7] lanes left, same direction
         [8] lanes right  [9] s along the road  [10] speed limit (km/h)  [11] oncoming lanes
  in : /carla/ego_vehicle/turn_signal_cmd std_msgs/Int8 -> CARLA vehicle lights (blinkers)

It reads the CARLA map through the Python API (port CARLA_PORT). CARLA is left-handed
(y to the right): ROS values are y_ros = -y_carla, yaw_ros = -yaw_carla.
"""
import math
import os

import carla
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8

LIGHTS = {-1: carla.VehicleLightState.LeftBlinker, 1: carla.VehicleLightState.RightBlinker,
          2: carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CarlaAdapter(Node):
    def __init__(self):
        super().__init__('carla_adapter')
        client = carla.Client(os.environ.get('CARLA_HOST', 'localhost'), int(os.environ.get('CARLA_PORT', '2000')))
        client.set_timeout(20.0)
        self.client, self.ego, self.map = client, None, None
        self.pub_path = self.create_publisher(Path, '/carla/ego_vehicle/waypoints', 10)
        self.pub_lane = self.create_publisher(Float32MultiArray, '/webdrivesim/ego_vehicle/lane_state', 10)
        self.create_subscription(Int8, '/carla/ego_vehicle/turn_signal_cmd', self.on_blinker, 10)
        self.create_timer(0.1, self.step)

    def on_blinker(self, msg):
        if self.ego is not None:
            self.ego.set_light_state(carla.VehicleLightState(LIGHTS.get(msg.data, carla.VehicleLightState.NONE)))

    def count(self, wp, side):
        """Driving lanes beside wp, in its direction (side 'left'/'right'), and the first other-direction lane."""
        n, w = 0, wp
        while True:
            w = w.get_left_lane() if side == 'left' else w.get_right_lane()
            if w is None or w.lane_type != carla.LaneType.Driving or w.lane_id * wp.lane_id < 0:
                return n, w
            n += 1

    @staticmethod
    def marking(m, crossing_allowed):
        if m.type == carla.LaneMarkingType.NONE:
            return 0
        return 1 if crossing_allowed else 2

    def step(self):
        if self.ego is None or not self.ego.is_alive:
            world = self.client.get_world()
            egos = [a for a in world.get_actors().filter('vehicle.*') if a.attributes.get('role_name') == 'ego_vehicle']
            if not egos:
                return
            self.ego, self.map = egos[0], world.get_map()
        tf = self.ego.get_transform()
        wp = self.map.get_waypoint(tf.location)
        lane_yaw = math.radians(wp.transform.rotation.yaw)

        # lateral offset (+ left) and heading error, converted to ROS signs
        dx, dy = tf.location.x - wp.transform.location.x, tf.location.y - wp.transform.location.y
        offset = -(dx * -math.sin(lane_yaw) + dy * math.cos(lane_yaw))          # CARLA right vector -> ROS left
        heading_err = -wrap(math.radians(tf.rotation.yaw) - lane_yaw)
        ahead = wp.next(10.0)
        curvature = -wrap(math.radians(ahead[0].transform.rotation.yaw) - lane_yaw) / 10.0 if ahead else 0.0
        # overtaking on a two-way road: we are inside the ONCOMING lane, driving against it.
        # Express everything in our driving direction and walk the lane backwards.
        against = abs(heading_err) > math.pi / 2
        if against:
            offset, heading_err, curvature = -offset, wrap(heading_err + math.pi), -curvature

        lanes_left, first_other = self.count(wp, 'left')
        lanes_right, _ = self.count(wp, 'right')
        oncoming = 0
        while first_other is not None and first_other.lane_type == carla.LaneType.Driving:
            oncoming += 1
            first_other = first_other.get_right_lane()      # seen from the other direction, "further left" is its right
        left_ok = wp.lane_change in (carla.LaneChange.Left, carla.LaneChange.Both)
        right_ok = wp.lane_change in (carla.LaneChange.Right, carla.LaneChange.Both)
        data = [offset, heading_err, curvature, wp.lane_width, lanes_right + 1,
                self.marking(wp.left_lane_marking, left_ok), self.marking(wp.right_lane_marking, right_ok),
                lanes_left, lanes_right, wp.s, self.ego.get_speed_limit(), oncoming]
        self.pub_lane.publish(Float32MultiArray(data=[float(v) for v in data]))

        path = Path()
        path.header.frame_id, path.header.stamp = 'map', self.get_clock().now().to_msg()
        w = wp
        for _ in range(100):                                  # 100 x 2 m
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = w.transform.location.x, -w.transform.location.y
            path.poses.append(ps)
            nxt = w.previous(2.0) if against else w.next(2.0)
            if not nxt:
                break
            w = nxt[0]
        self.pub_path.publish(path)


def main():
    rclpy.init()
    rclpy.spin(CarlaAdapter())
