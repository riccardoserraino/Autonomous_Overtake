// =====================================================================
// ros.js — ROS 2 link through rosbridge (WebSocket, JSON protocol v2)
//
// The browser cannot speak DDS, so it talks to `rosbridge_server`
// (ws://localhost:9090) which republishes everything as normal ROS 2
// topics. Topic names and message types copy the CARLA ROS bridge
// (role name "ego_vehicle"), so a node written for this simulator runs
// unchanged with CARLA + carla-ros-bridge, and vice versa.
// Coordinates: map frame ENU (x east, y north, z up); vehicle frame
// x forward, y left, z up (REP-103). CARLA convention kept for
// CarlaEgoVehicleControl.steer: +1 = full RIGHT.
// =====================================================================
import { SENSOR_LIB } from './sim.js';
import { BLUEPRINTS } from './carModel.js';

export const TOPICS = {
  pub: [
    ['/clock', 'rosgraph_msgs/msg/Clock', 'simulation time (use_sim_time:=true)'],
    ['/tf', 'tf2_msgs/msg/TFMessage', 'map → ego_vehicle'],
    ['/tf_static', 'tf2_msgs/msg/TFMessage', 'ego_vehicle → ego_vehicle/<sensor>'],
    ['/carla/status', 'carla_msgs/msg/CarlaStatus', 'frame, fixed_delta_seconds, synchronous mode'],
    ['/carla/world_info', 'carla_msgs/msg/CarlaWorldInfo', 'map name + full OpenDRIVE of the road (latched, every 5 s)'],
    ['/carla/ego_vehicle/odometry', 'nav_msgs/msg/Odometry', 'ground-truth pose + body twist, 50 Hz'],
    ['/carla/ego_vehicle/vehicle_status', 'carla_msgs/msg/CarlaEgoVehicleStatus', 'speed, acceleration, applied control, 20 Hz'],
    ['/carla/ego_vehicle/vehicle_info', 'carla_msgs/msg/CarlaEgoVehicleInfo', 'mass, wheels (max_steer_angle in rad), 1 Hz'],
    ['/carla/ego_vehicle/speedometer', 'std_msgs/msg/Float32', 'speed m/s'],
    ['/carla/ego_vehicle/imu', 'sensor_msgs/msg/Imu', 'noisy IMU (if enabled)'],
    ['/carla/ego_vehicle/gnss', 'sensor_msgs/msg/NavSatFix', 'noisy GNSS lat/lon/alt (if enabled)'],
    ['/webdrivesim/ego_vehicle/gnss_local', 'geometry_msgs/msg/PointStamped', 'same GNSS fix in map metres'],
    ['/carla/ego_vehicle/<radar_id>', 'sensor_msgs/msg/PointCloud2', 'radar detections: x y z Range Velocity AzimuthAngle ElevationAngle'],
    ['/carla/ego_vehicle/lidar', 'sensor_msgs/msg/PointCloud2', 'x y z intensity (if enabled)'],
    ['/carla/ego_vehicle/rgb_front/image/compressed', 'sensor_msgs/msg/CompressedImage', 'JPEG camera image (if enabled)'],
    ['/webdrivesim/ego_vehicle/lane_state', 'std_msgs/msg/Float32MultiArray', 'camera lane measurement (see guide for the layout)'],
    ['/carla/ego_vehicle/objects', 'derived_object_msgs/msg/ObjectArray', 'ground-truth objects within 250 m (CARLA pseudo-sensor)'],
    ['/carla/objects', 'derived_object_msgs/msg/ObjectArray', 'same, all vehicles'],
    ['/carla/ego_vehicle/waypoints', 'nav_msgs/msg/Path', 'centre line of the current lane, 250 m ahead, 2 Hz'],
    ['/carla/ego_vehicle/collision', 'carla_msgs/msg/CarlaCollisionEvent', 'on collision'],
    ['/carla/ego_vehicle/lane_invasion', 'carla_msgs/msg/CarlaLaneInvasionEvent', 'on crossing a marking (1 broken, 2 solid)'],
    ['/carla/ego_vehicle/turn_signal', 'std_msgs/msg/Int8', 'blinker state: -1 left, 0 off, 1 right, 2 hazard'],
    ['/webdrivesim/sim/status', 'std_msgs/msg/String', 'JSON: mode, autopilot state, metrics'],
  ],
  sub: [
    ['/carla/ego_vehicle/vehicle_control_cmd', 'carla_msgs/msg/CarlaEgoVehicleControl', 'throttle 0..1, steer -1..1 (+ right), brake 0..1, hand_brake, reverse'],
    ['/carla/ego_vehicle/ackermann_cmd', 'ackermann_msgs/msg/AckermannDrive', 'steering_angle rad (+ left), speed m/s, acceleration m/s²'],
    ['/carla/ego_vehicle/enable_autopilot', 'std_msgs/msg/Bool', 'true = built-in demo autopilot drives'],
    ['/carla/ego_vehicle/turn_signal_cmd', 'std_msgs/msg/Int8', '-1 left, 0 off, 1 right, 2 hazard'],
    ['/carla/control', 'carla_msgs/msg/CarlaControl', 'command 0 PLAY, 1 PAUSE, 2 STEP_ONCE'],
    ['/webdrivesim/sim/reset', 'std_msgs/msg/Empty', 'restart the scenario'],
    ['/webdrivesim/sim/load_world', 'std_msgs/msg/String', 'load <name>.json from the worlds folder'],
  ],
};

// ---------- rosbridge client ----------
export class RosBridge {
  constructor(url, onStatus) { this.url = url; this.onStatus = onStatus; this.ws = null; this.status = 'offline'; this.subs = {}; this.adv = []; this.wanted = true; this.sent = 0; this.recv = 0; }
  connect() {
    this.wanted = true;
    try { this.ws = new WebSocket(this.url); } catch (e) { this.set('error: ' + e.message); return; }
    this.set('connecting');
    this.ws.onopen = () => {
      this.set('connected');
      for (const [topic, type, latch] of this.adv) this.send({ op: 'advertise', topic, type, latch: !!latch, queue_size: 10 });
      for (const [topic, s] of Object.entries(this.subs)) this.send({ op: 'subscribe', topic, type: s.type, queue_length: 1 });
      this.onOpen && this.onOpen();
    };
    this.ws.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch (e) { return; }
      if (m.op === 'publish' && this.subs[m.topic]) { this.recv++; this.subs[m.topic].cb(m.msg); }
    };
    this.ws.onclose = () => { this.set('offline'); if (this.wanted) setTimeout(() => this.connect(), 2000); };
    this.ws.onerror = () => {};
  }
  close() { this.wanted = false; if (this.ws) this.ws.close(); }
  set(s) { this.status = s; this.onStatus && this.onStatus(s); }
  get ok() { return this.ws && this.ws.readyState === 1; }
  send(o) { if (this.ok) { this.ws.send(JSON.stringify(o)); this.sent++; } }
  advertise(topic, type, latch = false) { this.adv.push([topic, type, latch]); if (this.ok) this.send({ op: 'advertise', topic, type, latch, queue_size: 10 }); }
  publish(topic, msg) { if (this.ok && this.ws.bufferedAmount < 4e6) this.send({ op: 'publish', topic, msg }); }
  subscribe(topic, type, cb) { this.subs[topic] = { type, cb }; if (this.ok) this.send({ op: 'subscribe', topic, type, queue_length: 1 }); }
}

// ---------- helpers ----------
const stamp = (t) => ({ sec: Math.floor(t), nanosec: Math.round((t - Math.floor(t)) * 1e9) % 1000000000 });
const header = (frame, t) => ({ stamp: stamp(t), frame_id: frame });
// quaternion from yaw (about z) and pitch (about y, + = nose DOWN in ROS)
export function quat(yaw, pitch = 0) {
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2), cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  return { x: -sy * sp, y: cy * sp, z: sy * cp, w: cy * cp };
}
const v3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
function b64(u8) { let s = ''; const C = 0x8000; for (let i = 0; i < u8.length; i += C) s += String.fromCharCode.apply(null, u8.subarray(i, i + C)); return btoa(s); }
// sensor_msgs/PointCloud2 with FLOAT32 fields, little endian
function cloud(frame, t, names, rows) {
  const n = rows.length, k = names.length, buf = new Float32Array(n * k);
  rows.forEach((r, i) => { for (let j = 0; j < k; j++) buf[i * k + j] = r[j]; });
  return { header: header(frame, t), height: 1, width: n, fields: names.map((name, j) => ({ name, offset: 4 * j, datatype: 7, count: 1 })),
    is_bigendian: false, point_step: 4 * k, row_step: 4 * k * n, data: b64(new Uint8Array(buf.buffer)), is_dense: true };
}
// local metres -> WGS84 around a reference (equirectangular, fine for a few km)
const LAT0 = 45.4642, LON0 = 9.19, RE = 6378137;
const toGeo = (x, y) => ({ lat: LAT0 + (y / RE) * 180 / Math.PI, lon: LON0 + (x / (RE * Math.cos(LAT0 * Math.PI / 180))) * 180 / Math.PI });

// ---------- simulator <-> ROS ----------
export class RosLink {
  constructor(url, hooks) {
    this.hooks = hooks;                       // { getSim, getRoad, getCfg, getXodr, getWorldName, onControl, cameraJpeg }
    this.bridge = new RosBridge(url, (s) => hooks.onStatus && hooks.onStatus(s));
    this.acc = {}; this.lastInfo = -10; this.lastWorld = -10; this.lastStatic = -10;
    for (const [topic, type] of TOPICS.pub) if (!topic.includes('<')) this.bridge.advertise(topic, type, topic === '/carla/world_info' || topic === '/tf_static');
    for (const id of Object.keys(SENSOR_LIB)) if (SENSOR_LIB[id].kind === 'radar') this.bridge.advertise('/carla/ego_vehicle/' + id, 'sensor_msgs/msg/PointCloud2');
    const ext = () => hooks.getSim().ext, now = () => hooks.getSim().t;
    const B = this.bridge;
    B.subscribe('/carla/ego_vehicle/vehicle_control_cmd', 'carla_msgs/msg/CarlaEgoVehicleControl', (m) => { ext().vcc = { ...m, t: now() }; ext().lastMsgT = performance.now(); hooks.onControl && hooks.onControl('vcc'); });
    B.subscribe('/carla/ego_vehicle/ackermann_cmd', 'ackermann_msgs/msg/AckermannDrive', (m) => { ext().ack = { ...m, t: now() }; ext().lastMsgT = performance.now(); hooks.onControl && hooks.onControl('ack'); });
    B.subscribe('/carla/ego_vehicle/enable_autopilot', 'std_msgs/msg/Bool', (m) => { ext().autopilot = !!m.data; });
    B.subscribe('/carla/ego_vehicle/turn_signal_cmd', 'std_msgs/msg/Int8', (m) => { ext().signal = m.data; });
    B.subscribe('/carla/control', 'carla_msgs/msg/CarlaControl', (m) => hooks.onCarlaControl && hooks.onCarlaControl(m.command));
    B.subscribe('/webdrivesim/sim/reset', 'std_msgs/msg/Empty', () => hooks.onReset && hooks.onReset());
    B.subscribe('/webdrivesim/sim/load_world', 'std_msgs/msg/String', (m) => hooks.onLoadWorld && hooks.onLoadWorld(m.data));
    B.onOpen = () => { this.lastWorld = -10; this.lastStatic = -10; this.lastInfo = -10; };
  }
  connect() { this.bridge.connect(); }
  close() { this.bridge.close(); }
  get ok() { return this.bridge.ok; }
  due(key, hz, t) { const a = this.acc[key]; if (a === undefined || t - a >= 1 / hz - 1e-6 || t < a) { this.acc[key] = t; return true; } return false; }

  // called after every simulation step
  tick(sync) {
    if (!this.ok) return;
    const sim = this.hooks.getSim(), road = this.hooks.getRoad(), cfg = this.hooks.getCfg(), e = sim.ego, t = sim.t, b = e.bp, B = this.bridge;
    const pub = (topic, msg) => B.publish(topic, msg);
    const q = quat(e.psi, -e.pitch);
    pub('/clock', { clock: stamp(t) });
    if (this.due('odo', 50, t)) {
      pub('/carla/ego_vehicle/odometry', { header: header('map', t), child_frame_id: 'ego_vehicle',
        pose: { pose: { position: v3(e.x, e.y, e.z), orientation: q } }, twist: { twist: { linear: v3(e.v, 0, 0), angular: v3(0, 0, e.yawRate) } } });
      pub('/tf', { transforms: [{ header: header('map', t), child_frame_id: 'ego_vehicle', transform: { translation: v3(e.x, e.y, e.z), rotation: q } }] });
    }
    if (t - this.lastStatic > 2) {
      this.lastStatic = t;
      pub('/tf_static', { transforms: cfg.sensors.filter(s => s.enabled).map(s => ({ header: header('ego_vehicle', t), child_frame_id: 'ego_vehicle/' + s.id,
        transform: { translation: v3(s.x, s.y, s.z), rotation: quat(s.yaw * Math.PI / 180) } })) });
    }
    if (this.due('status', 20, t)) {
      const c = sim.ctrl;
      pub('/carla/ego_vehicle/vehicle_status', { header: header('map', t), velocity: e.v, acceleration: { linear: v3(e.a, e.v * e.yawRate, 0), angular: v3() }, orientation: q,
        control: { header: header('map', t), throttle: c.throttle, steer: c.steer, brake: c.brake, hand_brake: !!c.hand_brake, reverse: !!c.reverse, gear: c.reverse ? -1 : 1, manual_gear_shift: false } });
      pub('/carla/ego_vehicle/speedometer', { data: e.v });
      pub('/carla/ego_vehicle/turn_signal', { data: e.signal });
    }
    if (this.due('cstatus', 10, t)) pub('/carla/status', { frame: sim.frame, fixed_delta_seconds: sync ? 0.05 : 0.02, synchronous_mode: !!sync, synchronous_mode_running: !this.hooks.isPaused() });
    if (t - this.lastInfo > 1) {
      this.lastInfo = t;
      const xf = b.L / 2 - b.fo, xr = xf - b.wb, tr = b.track / 2;
      pub('/carla/ego_vehicle/vehicle_info', { id: 1, type: 'vehicle.webdrivesim.' + cfg.vehicle.bp, rolename: 'ego_vehicle',
        wheels: [[xf, tr, b.steerMax], [xf, -tr, b.steerMax], [xr, tr, 0], [xr, -tr, 0]].map(([x, y, st]) => ({ tire_friction: 3.5, damping_rate: 0.25, max_steer_angle: st, radius: b.r, max_brake_torque: 1500, max_handbrake_torque: 3000, position: v3(x, y, b.r) })),
        max_rpm: 6000, moi: 1, damping_rate_full_throttle: 0.15, damping_rate_zero_throttle_clutch_engaged: 2, damping_rate_zero_throttle_clutch_disengaged: 0.35,
        use_gear_autobox: true, gear_switch_time: 0.5, clutch_strength: 10, mass: b.m, drag_coefficient: 0.3, center_of_mass: v3(b.L / 2 - b.fo - b.lf, 0, b.hcg) });
    }
    if (t - this.lastWorld > 5) { this.lastWorld = t; pub('/carla/world_info', { map_name: this.hooks.getWorldName(), opendrive: this.hooks.getXodr() }); }

    // sensors: publish each fresh measurement once
    for (const s of cfg.sensors) {
      if (!s.enabled) continue;
      const m = sim.sensorOut[s.id]; if (!m || m._pub) continue; m._pub = true;
      const kind = SENSOR_LIB[s.id].kind, frame = 'ego_vehicle/' + s.id, mt = m.t;
      if (kind === 'radar') {
        pub('/carla/ego_vehicle/' + s.id, cloud(frame, mt, ['x', 'y', 'z', 'Range', 'Velocity', 'AzimuthAngle', 'ElevationAngle'],
          m.det.map(d => [d.r * Math.cos(d.az) * Math.cos(d.el), d.r * Math.sin(d.az) * Math.cos(d.el), d.r * Math.sin(d.el), d.r, d.rr, -d.az, d.el])));
      } else if (kind === 'lidar') {
        const sp = m.sensorPose, c = Math.cos(sp.psi), sn = Math.sin(sp.psi), rows = [];
        for (let i = 0; i < m.points.length; i += 3) { const dx = m.points[i] - sp.x, dy = m.points[i + 1] - sp.y; rows.push([dx * c + dy * sn, -dx * sn + dy * c, m.points[i + 2] - sp.z, 0.6]); }
        pub('/carla/ego_vehicle/lidar', cloud(frame, mt, ['x', 'y', 'z', 'intensity'], rows));
      } else if (kind === 'camera') {
        const rc = road.cfg, w = rc.laneWidth, lane = m.lane;
        const mk = sim.markingsAt(e.s), lc = road.laneD(lane);
        const leftM = mk.filter(x => x.d > lc).sort((a, b2) => a.d - b2.d)[0], rightM = mk.filter(x => x.d < lc).sort((a, b2) => b2.d - a.d)[0];
        const fromRight = lane > 0 ? rc.nRight - lane + 1 : lane;
        const data = [m.laneOffset, m.headingErr, m.curvature, w, fromRight, leftM ? leftM.type : 0, rightM ? rightM.type : 0, lane > 0 ? lane - 1 : 0, lane > 0 ? rc.nRight - lane : 0, e.s, road.limitAt(e.s), rc.nLeft];
        pub('/webdrivesim/ego_vehicle/lane_state', { layout: { dim: [{ label: 'offset,heading_err,curvature,lane_width,lane_from_right,left_marking,right_marking,lanes_left,lanes_right,s,speed_limit_kmh,oncoming_lanes', size: data.length, stride: data.length }], data_offset: 0 }, data });
      } else if (kind === 'gnss') {
        const g = toGeo(m.x, m.y), sg = s.sxy * s.sxy;
        pub('/carla/ego_vehicle/gnss', { header: header(frame, mt), status: { status: 0, service: 1 }, latitude: g.lat, longitude: g.lon, altitude: m.z, position_covariance: [sg, 0, 0, 0, sg, 0, 0, 0, 4 * sg], position_covariance_type: 2 });
        pub('/webdrivesim/ego_vehicle/gnss_local', { header: header('map', mt), point: v3(m.x, m.y, m.z) });
      } else if (kind === 'imu') {
        pub('/carla/ego_vehicle/imu', { header: header(frame, mt), orientation: q, angular_velocity: v3(0, 0, m.wz), linear_acceleration: v3(m.ax, m.ay, 9.81) });
      }
    }
    if (this.due('cam', 10, t) && cfg.sensors.some(s => s.id === 'rgb_front' && s.enabled) && cfg.ros.camera) {
      const jpg = this.hooks.cameraJpeg && this.hooks.cameraJpeg();
      if (jpg) pub('/carla/ego_vehicle/rgb_front/image/compressed', { header: header('ego_vehicle/rgb_front', t), format: 'jpeg', data: jpg });
    }
    if (this.due('obj', 20, t)) {
      const objs = sim.traffic.filter(o => Math.hypot(o.x - e.x, o.y - e.y) < 250).map(o => ({
        header: header('map', t), id: o.id + 1, detection_level: 1, object_classified: true,
        pose: { position: v3(o.x, o.y, o.z + o.bp.H / 2), orientation: quat(o.psi) }, twist: { linear: v3(o.v, 0, 0), angular: v3() }, accel: { linear: v3(), angular: v3() },
        polygon: { points: [] }, shape: { type: 1, dimensions: [o.bp.L, o.bp.W, o.bp.H] },
        classification: o.bpKey === 'truck' ? 7 : 6, classification_certainty: 255, classification_age: 1 }));
      const arr = { header: header('map', t), objects: objs };
      pub('/carla/ego_vehicle/objects', arr); pub('/carla/objects', arr);
    }
    if (this.due('wp', 2, t)) {
      const lane = e.lane, d = road.laneD(lane), poses = [];
      for (let ds = -10; ds <= 250; ds += 2) {
        if (!road.closed && (e.s + ds < 0 || e.s + ds > road.length)) continue;
        const sgn = lane > 0 ? 1 : -1, p = road.pose(e.s + sgn * ds, d);
        poses.push({ header: header('map', t), pose: { position: v3(p[0], p[1], p[3]), orientation: quat(p[2] + (lane < 0 ? Math.PI : 0)) } });
      }
      pub('/carla/ego_vehicle/waypoints', { header: header('map', t), poses });
    }
    for (const ev of sim.rosEvents) {
      if (ev.type === 'collision') pub('/carla/ego_vehicle/collision', { header: header('ego_vehicle', ev.t), other_actor_id: ev.other + 1, normal_impulse: v3(...ev.impulse) });
      if (ev.type === 'lane_invasion') pub('/carla/ego_vehicle/lane_invasion', { header: header('ego_vehicle', ev.t), crossed_lane_markings: ev.crossed });
    }
    sim.rosEvents.length = 0;
    if (this.due('sim', 2, t)) {
      const mtr = sim.metrics;
      pub('/webdrivesim/sim/status', { data: JSON.stringify({ world: this.hooks.getWorldName(), t: +t.toFixed(2), control: sim.source, autopilot_state: sim.ap.mode, autopilot_status: sim.ap.status,
        speed_kmh: +(e.v * 3.6).toFixed(1), lane: road.laneName(e.lane), s_m: +e.s.toFixed(1), end_of_road: sim.endOfRoad,
        metrics: { collisions: mtr.collisions, overtakes: mtr.overtakes, aborts: mtr.aborts, solid_line_crossings: mtr.solidCross, min_ttc_s: isFinite(mtr.minTTC) ? +mtr.minTTC.toFixed(2) : null, distance_m: Math.round(mtr.dist) } }) });
    }
  }
}
