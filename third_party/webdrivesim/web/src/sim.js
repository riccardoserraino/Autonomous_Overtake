// =====================================================================
// sim.js — simulation core (no graphics). In your real project this is
// replaced by your ROS 2 nodes; here it also contains a DEMO autopilot.
//
//  EGO        kinematic bicycle + first-order actuators + road grade
//  TRAFFIC    Intelligent Driver Model (IDM) + lane changes with blinkers
//  SENSORS    object-level radar / camera / lidar: range, field of view,
//             occlusion by vehicles AND by crests (line of sight), noise
//  CONTROL    manual (keyboard/gamepad) | autopilot (demo) | ros (your node)
//  AUTOPILOT  ACC (constant time gap) + overtaking state machine with turn
//             signals + pure-pursuit steering
// =====================================================================
import { BLUEPRINTS } from './carModel.js';
import { gauss, rng } from './road.js';

const D2R = Math.PI / 180;
const wrapA = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export function q5(t) { t = clamp(t, 0, 1); return t * t * t * (10 - 15 * t + 6 * t * t); }  // quintic smoothstep

// ---------------------------------------------------------------------
// Sensors. Mount pose in the vehicle frame (x fwd, y left, z up; yaw deg).
// The id is also the ROS topic / frame name: /carla/ego_vehicle/<id>
// ---------------------------------------------------------------------
export const SENSOR_LIB = {
  radar_front: { label: 'Front long-range radar', kind: 'radar', color: '#e5484d', rate: 20,
    def: (b) => ({ x: b.L / 2 - 0.05, y: 0, z: 0.5, yaw: 0, range: 250, fov: 18, sr: 0.25, sa: 0.3, sv: 0.15 }),
    help: 'Detections with range, azimuth and radial velocity (PointCloud2, CARLA radar format). Main input for ACC.' },
  rgb_front: { label: 'Front camera', kind: 'camera', color: '#3e9be9', rate: 15,
    def: (b) => ({ x: b.type === 'truck' ? b.L / 2 - 0.3 : b.L / 2 - (b.hood || 1) - (b.wind || 0.8) + 0.12, y: 0, z: b.H - 0.1, yaw: 0, range: 120, fov: 60, sr: 1.0, sa: 0.15, sv: 0.6, slat: 0.04 }),
    help: 'Image (compressed) + lane measurement (offset, heading error, curvature) + camera object detections.' },
  lidar: { label: 'Roof lidar 360°', kind: 'lidar', color: '#a4d65e', rate: 10,
    def: (b) => ({ x: -0.1, y: 0, z: b.H + 0.18, yaw: 0, range: 100, fov: 360, sr: 0.03, sa: 0.1, sv: 0.4 }),
    help: 'Point cloud on the vehicles it hits (PointCloud2 x, y, z, intensity).' },
  radar_rear_left: { label: 'Rear-left corner radar', kind: 'radar', color: '#f2a33a', rate: 20,
    def: (b) => ({ x: -b.L / 2 + 0.1, y: b.W / 2 - 0.12, z: 0.5, yaw: 145, range: 80, fov: 110, sr: 0.3, sa: 0.8, sv: 0.2 }),
    help: 'Blind spot + cars approaching from behind in the left lane. Needed before moving out to overtake.' },
  radar_rear_right: { label: 'Rear-right corner radar', kind: 'radar', color: '#f2a33a', rate: 20,
    def: (b) => ({ x: -b.L / 2 + 0.1, y: -(b.W / 2 - 0.12), z: 0.5, yaw: -145, range: 80, fov: 110, sr: 0.3, sa: 0.8, sv: 0.2 }),
    help: 'Checks the lane you return to after the overtake.' },
  radar_front_left: { label: 'Front-left corner radar', kind: 'radar', color: '#f2a33a', rate: 20,
    def: (b) => ({ x: b.L / 2 - 0.15, y: b.W / 2 - 0.15, z: 0.5, yaw: 45, range: 80, fov: 110, sr: 0.3, sa: 0.8, sv: 0.2 }),
    help: 'Covers the side while you are alongside the overtaken vehicle.' },
  gnss: { label: 'GNSS receiver', kind: 'gnss', color: '#c084fc', rate: 10,
    def: (b) => ({ x: 0, y: 0, z: b.H, yaw: 0, sxy: 1.5 }), help: 'Position (NavSatFix + local x,y) with ~1.5 m noise at 10 Hz.' },
  imu: { label: 'IMU', kind: 'imu', color: '#94a3b8', rate: 50,
    def: (b) => ({ x: 0, y: 0, z: 0.45, yaw: 0, sacc: 0.05, sgyr: 0.004 }), help: 'Linear acceleration and angular velocity with noise.' },
};
export const SENSOR_PRESETS = {
  acc: { label: 'ACC only', ids: ['radar_front', 'rgb_front', 'imu', 'gnss'] },
  overtake: { label: 'Overtake-ready', ids: ['radar_front', 'rgb_front', 'radar_rear_left', 'radar_rear_right', 'imu', 'gnss'] },
  full: { label: 'Full suite', ids: Object.keys(SENSOR_LIB) },
};
export function makeSensors(bpKey, ids) {
  const b = BLUEPRINTS[bpKey];
  return Object.keys(SENSOR_LIB).map(id => ({ id, enabled: ids.includes(id), ...SENSOR_LIB[id].def(b) }));
}
export const NOISE = {
  L0: { label: 'L0 · perfect', k: 0, lat: 0, bias: 0, pd: 1, clutter: 0 },
  L1: { label: 'L1 · Gaussian', k: 1, lat: 0, bias: 0, pd: 1, clutter: 0 },
  L2: { label: 'L2 · + bias, latency', k: 1.5, lat: 0.1, bias: 1, pd: 1, clutter: 0 },
  L3: { label: 'L3 · + misses, clutter', k: 1.5, lat: 0.1, bias: 1, pd: 0.88, clutter: 0.6 },
};
export const TRAFFIC_TYPES = { hatch: 'Hatchback', sedan: 'Sedan', suv: 'SUV', van: 'Van', truck: 'Truck' };
const PAINTS_TOY = ['#e63946', '#f4a261', '#2a9d8f', '#457b9d', '#e9c46a', '#8d6cc4', '#f1faee', '#6c757d', '#ef476f', '#06d6a0'];

export class Sim {
  constructor(cfg, road) {
    this.cfg = cfg; this.road = road;
    this.R = rng(cfg.traffic.seed || 1);
    this.t = 0; this.frame = 0;
    const bp = BLUEPRINTS[cfg.vehicle.bp], rc = road.cfg;
    const lane = Math.min(rc.nRight, Math.max(1, cfg.ego.lane || rc.nRight));
    const s0 = road.wrap(cfg.ego.s ?? 50), d0 = road.laneD(lane), p = road.pose(s0, d0);
    this.ego = { bp, x: p[0], y: p[1], z: p[3], psi: p[2], pitch: 0, v: (cfg.ego.speed ?? 80) / 3.6, a: 0, delta: 0, s: s0, d: d0, lane,
      reverse: false, wheelRot: 0, braking: false, yawRate: 0, prevA: 0, jerk: 0, signal: 0 };
    this.ctrl = { throttle: 0, brake: 0, steer: 0, hand_brake: false, reverse: false, source: 'manual' };
    this.ap = { mode: 'FOLLOW', lane, lc: null, otId: null, fromLane: lane, status: 'Cruising', lastLc: -10, keepRightT: 0, prep: null };
    this.ext = { vcc: null, ack: null, autopilot: false, signal: null, lastMsgT: -1 };
    this.tracks = new Map(); this.sensorOut = {}; this.pending = []; this.bias = {};
    this.metrics = { minTTC: Infinity, minTHW: Infinity, collisions: 0, overtakes: 0, aborts: 0, maxLat: 0, maxJerk: 0, laneErrSq: 0, laneErrN: 0, dist: 0, offRoad: 0, solidCross: 0, speeding: 0 };
    this.events = []; this.rosEvents = []; this.hist = []; this.log = []; this.recording = false;
    this.input = { throttle: 0, brake: 0, steer: 0, hand: false };
    this.collideFlash = 0; this.endOfRoad = false;
    this.spawnTraffic();
  }

  // ------------------------------------------------------------------
  // TRAFFIC: explicit vehicles (lane, distance from ego, speed) + random background
  // ------------------------------------------------------------------
  spawnTraffic() {
    const c = this.cfg.traffic, road = this.road, rc = road.cfg, R = this.R, e = this.ego;
    this.traffic = [];
    let id = 1;
    const ok = (lane) => lane > 0 ? lane <= rc.nRight : -lane <= rc.nLeft;
    const add = (bpKey, lane, s, vKmh, beh, paint) => {
      if (!ok(lane)) return;
      if (!road.closed && (s < 2 || s > road.length - 2)) return;
      const bp = BLUEPRINTS[bpKey], dir = lane > 0 ? 1 : -1;
      this.traffic.push({ id: id++, bpKey, bp, dir, lane, s: road.wrap(s), d: road.laneD(lane), v: vKmh / 3.6, v0: vKmh / 3.6, behaviour: beh || c.behaviour || 'idm',
        lc: null, prep: null, paint: paint || PAINTS_TOY[Math.floor(R() * PAINTS_TOY.length)], x: 0, y: 0, z: 0, psi: 0, stopT: 0, signal: 0 });
    };
    for (const v of c.vehicles || []) add(v.type, v.lane, e.s + v.ds, v.speed, v.behaviour, v.paint);
    // background traffic, kept away from the explicit vehicles and the ego
    const mix = Object.entries(c.mix || {}).filter(([, v]) => v > 0), tot = mix.reduce((a, [, v]) => a + v, 0) || 1;
    const pick = () => { let u = R() * tot; for (const [k, v] of mix) if ((u -= v) <= 0) return k; return 'sedan'; };
    const free = (lane, s) => this.traffic.every(t => t.lane !== lane || Math.abs(road.diff(t.s, s)) > 25) && (lane !== e.lane || Math.abs(road.diff(e.s, s)) > 40);
    const L = road.length;
    const bg = (n, dir) => {
      for (let i = 0; i < n; i++) {
        for (let tries = 0; tries < 20; tries++) {
          const lane = dir > 0 ? 1 + Math.floor(R() * rc.nRight) : -(1 + Math.floor(R() * rc.nLeft));
          const s = road.closed ? e.s + 60 + R() * (L - 120) : 10 + R() * (L - 20);
          if (!free(lane, s)) continue;
          const k = pick(), v = (c.vMin + R() * Math.max(0, c.vMax - c.vMin));
          add(k, lane, s, k === 'truck' ? Math.min(v, 85) : v);
          break;
        }
      }
    };
    bg(c.nSame || 0, 1); if (rc.nLeft > 0) bg(c.nOncoming || 0, -1);
    for (const t of this.traffic) this.poseTraffic(t, 0);
  }

  // distance ahead along direction dir from s to s2 (negative = behind)
  ahead(s, s2, dir) { return dir * this.road.diff(s, s2); }
  idm(v, v0, gap, dv, aMax) {
    const s0 = 2.5, T = 1.4, b = 2.0;
    const sStar = s0 + Math.max(0, v * T + v * dv / (2 * Math.sqrt(aMax * b)));
    return aMax * (1 - Math.pow(v / Math.max(v0, 0.1), 4) - Math.pow(sStar / Math.max(gap, 0.1), 2));
  }
  leaderOf(self, lane, dir) {
    const dl = this.road.laneD(lane), w = this.road.cfg.laneWidth;
    let best = null, bestDs = Infinity;
    const consider = (o, s, d, len, vAlong) => {
      if (Math.abs(d - dl) > w * 0.7) return;
      const ds = this.ahead(self.s, s, dir);
      if (ds > 0.01 && ds < bestDs && ds < 400) { bestDs = ds; best = { o, gap: ds - (len + self.bp.L) / 2, vAlong }; }
    };
    for (const o of this.traffic) if (o !== self) consider(o, o.s, o.d, o.bp.L, o.v * o.dir * dir);
    consider(this.ego, this.ego.s, this.ego.d, this.ego.bp.L, this.ego.v * dir);
    return best;
  }
  laneGap(lane, s, dir, exclude) {
    let front = Infinity, back = Infinity, vBack = 0;
    const dl = this.road.laneD(lane), w = this.road.cfg.laneWidth;
    const all = [...this.traffic.map(o => ({ s: o.s, d: o.d, v: o.v * o.dir * dir, o })), { s: this.ego.s, d: this.ego.d, v: this.ego.v * dir, o: this.ego }];
    for (const q of all) {
      if (q.o === exclude || Math.abs(q.d - dl) > w * 0.7) continue;
      const a = this.ahead(s, q.s, dir);
      if (a >= 0 && a < front) front = a;
      if (a < 0 && -a < back) { back = -a; vBack = q.v; }
    }
    return { front, back, vBack };
  }
  stepTraffic(dt) {
    const c = this.cfg.traffic, road = this.road, n = road.cfg.nRight;
    for (const t of this.traffic) {
      if (t.behaviour === 'constant') t.v = t.v0;
      else {
        const ld = this.leaderOf(t, t.lc ? t.lc.to : t.lane, t.dir);
        let a = ld ? this.idm(t.v, t.v0, ld.gap, t.v - ld.vAlong, t.bp.aMax * 0.85) : this.idm(t.v, t.v0, 1e4, 0, t.bp.aMax * 0.85);
        a = clamp(a, -t.bp.bMax, t.bp.aMax);
        if (t.stopT > 0) { t.stopT -= dt; a = -t.bp.bMax; }
        t.v = Math.max(0, t.v + a * dt);
        // lane changes (same direction, multi-lane): signal 1.5 s first, then move
        if (t.prep && this.t - t.prep.t0 > 1.5) { t.lc = { from: t.lane, to: t.prep.to, t0: this.t, T: t.bp.tlc }; t.prep = null; }
        if (c.laneChanges && !t.lc && !t.prep && t.dir === 1 && n > 1 && this.R() < dt * 0.5) {
          if (ld && ld.gap < 45 && ld.vAlong < t.v0 - 3 && t.lane > 1) {
            const g = this.laneGap(t.lane - 1, t.s, 1, t);
            if (g.front > 35 && g.back > 15 + Math.max(0, g.vBack - t.v) * 3) { t.prep = { to: t.lane - 1, t0: this.t }; t.signal = -1; }
          } else if (t.lane < n) {
            const g = this.laneGap(t.lane + 1, t.s, 1, t);
            if (g.front > 70 && g.back > 25) { t.prep = { to: t.lane + 1, t0: this.t }; t.signal = 1; }
          }
        }
      }
      t.s = t.s + t.dir * t.v * dt;
      if (road.closed) t.s = road.wrap(t.s);
      else if (t.s > road.length - 3 || t.s < 3) {     // open road: recycle at the other end, far from the ego
        const ns = t.dir > 0 ? 4 : road.length - 4;
        if (Math.abs(road.diff(this.ego.s, ns)) > 150) { if (t.lc) t.lane = t.lc.to; t.s = ns; t.lc = null; t.prep = null; t.d = road.laneD(t.lane); t.signal = 0; }
        else t.s = clamp(t.s, 3, road.length - 3);
      }
      this.poseTraffic(t, dt);
    }
  }
  poseTraffic(t, dt) {
    const road = this.road;
    let yawOff = 0;
    if (t.lc) {
      const tau = (this.t - t.lc.t0) / t.lc.T, d0 = road.laneD(t.lc.from), d1 = road.laneD(t.lc.to);
      const dn = d0 + (d1 - d0) * q5(tau);
      yawOff = Math.atan2((dn - t.d) / Math.max(dt, 1e-3), Math.max(t.v, 0.5)) * t.dir;
      t.d = dn;
      if (tau >= 1) { t.lane = t.lc.to; t.d = d1; t.lc = null; t.signal = 0; }
    }
    const p = road.pose(t.s, t.d);
    t.x = p[0]; t.y = p[1]; t.z = p[3]; t.psi = p[2] + (t.dir < 0 ? Math.PI : 0) + (dt ? yawOff : 0);
  }

  // ------------------------------------------------------------------
  // SENSORS
  // ------------------------------------------------------------------
  sensorPose(s) {
    const e = this.ego, c = Math.cos(e.psi), sn = Math.sin(e.psi);
    return { x: e.x + c * s.x - sn * s.y, y: e.y + sn * s.x + c * s.y, z: e.z + s.z, psi: e.psi + s.yaw * D2R, s: e.s + s.x };
  }
  // geometry: in range, in field of view, not hidden by a closer vehicle or by a crest
  visibleTargets(sp, range, fov, sensZ) {
    const out = [], road = this.road;
    for (const t of this.traffic) {
      const c = Math.cos(t.psi), s = Math.sin(t.psi), dx = sp.x - t.x, dy = sp.y - t.y;
      const lx = clamp(dx * c + dy * s, -t.bp.L / 2, t.bp.L / 2), ly = clamp(-dx * s + dy * c, -t.bp.W / 2, t.bp.W / 2);
      const px = t.x + lx * c - ly * s, py = t.y + lx * s + ly * c;
      const r = Math.hypot(px - sp.x, py - sp.y);
      if (r > range || r < 0.3) continue;
      const az = wrapA(Math.atan2(py - sp.y, px - sp.x) - sp.psi);
      if (fov < 360 && Math.abs(az) > fov * D2R / 2) continue;
      if (!road.los(sp.s, sensZ, t.s, t.bp.H * 0.6)) continue;               // hidden behind a crest
      let amin = Infinity, amax = -Infinity;
      for (const [cx, cy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
        const qx = t.x + cx * t.bp.L / 2 * c - cy * t.bp.W / 2 * s, qy = t.y + cx * t.bp.L / 2 * s + cy * t.bp.W / 2 * c;
        const qa = az + wrapA(Math.atan2(qy - sp.y, qx - sp.x) - sp.psi - az); amin = Math.min(amin, qa); amax = Math.max(amax, qa);
      }
      out.push({ t, r, az, px, py, amin, amax });
    }
    out.sort((a, b) => a.r - b.r);
    return out.filter((o, i) => !out.slice(0, i).some(q => o.az > q.amin + 0.005 && o.az < q.amax - 0.005 && q.t.bp.H > 1.0));
  }
  runSensors(dt) {
    const N = NOISE[this.cfg.noise], e = this.ego, R = this.R;
    for (const s of this.cfg.sensors) {
      if (!s.enabled) continue;
      const lib = SENSOR_LIB[s.id];
      s._acc = (s._acc || 0) + dt;
      if (s._acc < 1 / lib.rate - 1e-6) continue;
      s._acc = 0;
      if (!this.bias[s.id]) this.bias[s.id] = { r: gauss(R) * 0.3, a: gauss(R) * 0.3 * D2R };
      const k = N.k, bias = N.bias ? this.bias[s.id] : { r: 0, a: 0 }, sp = this.sensorPose(s);
      const msg = { t: this.t, kind: lib.kind };
      if (['radar', 'camera', 'lidar'].includes(lib.kind)) {
        const det = [];
        for (const o of this.visibleTargets(sp, s.range, s.fov, s.z)) {
          const pd = lib.kind === 'camera' ? N.pd * (1 - 0.3 * o.r / s.range) : N.pd;
          if (R() > pd) continue;
          const sr = lib.kind === 'camera' ? s.sr * (0.2 + o.r / 40) : s.sr;
          const r = o.r + k * (sr * gauss(R) + bias.r), az = o.az + k * (s.sa * D2R * gauss(R) + bias.a);
          const ux = Math.cos(sp.psi + o.az), uy = Math.sin(sp.psi + o.az);
          const tx = Math.cos(o.t.psi) * o.t.v, ty = Math.sin(o.t.psi) * o.t.v, ex = Math.cos(e.psi) * e.v, ey = Math.sin(e.psi) * e.v;
          const rr = (tx - ex) * ux + (ty - ey) * uy + k * s.sv * gauss(R);
          const dz = o.t.z + o.t.bp.H * 0.45 - sp.z, el = Math.atan2(dz, r);
          det.push({ id: o.t.id, r, az, el, rr, wx: sp.x + r * Math.cos(sp.psi + az), wy: sp.y + r * Math.sin(sp.psi + az), wz: sp.z + dz });
        }
        if (N.clutter && R() < N.clutter * dt * lib.rate * 0.2) {
          const r = R() * s.range * 0.6, az = (R() - 0.5) * Math.min(s.fov, 120) * D2R;
          det.push({ id: -1, r, az, el: 0, rr: (R() - 0.5) * 10, wx: sp.x + r * Math.cos(sp.psi + az), wy: sp.y + r * Math.sin(sp.psi + az), wz: sp.z });
        }
        msg.det = det;
        if (lib.kind === 'camera') {
          const lane = this.road.laneOf(e.d), lc = this.road.laneD(lane);
          msg.laneOffset = (e.d - lc) + k * s.slat * gauss(R);
          msg.headingErr = wrapA(e.psi - this.road.pose(e.s, 0)[2]) + k * 0.003 * gauss(R);
          msg.curvature = this.road.kappa(e.s) + k * 0.0003 * gauss(R);
          msg.lane = lane;
        }
        if (lib.kind === 'lidar') {   // points on the visible faces (x, y, z world) for display and PointCloud2
          const pts = [];
          for (const d of det) {
            if (d.id < 0) continue;
            const t = this.traffic.find(q => q.id === d.id); if (!t) continue;
            const c = Math.cos(t.psi), sn = Math.sin(t.psi);
            for (let i = 0; i < 46; i++) {
              const u = R() - 0.5, face = R() < 0.5;
              const lx = face ? Math.sign((sp.x - t.x) * c + (sp.y - t.y) * sn) * t.bp.L / 2 : u * t.bp.L;
              const ly = face ? u * t.bp.W : Math.sign(-(sp.x - t.x) * sn + (sp.y - t.y) * c) * t.bp.W / 2;
              pts.push(t.x + lx * c - ly * sn + k * s.sr * gauss(R), t.y + lx * sn + ly * c + k * s.sr * gauss(R), t.z + 0.25 + R() * (t.bp.H - 0.35));
            }
          }
          msg.points = pts;
        }
      } else if (lib.kind === 'gnss') { msg.x = e.x + k * s.sxy * gauss(R); msg.y = e.y + k * s.sxy * gauss(R); msg.z = e.z + s.z + k * s.sxy * gauss(R); }
      else if (lib.kind === 'imu') { msg.ax = e.a + k * s.sacc * gauss(R); msg.ay = e.v * e.yawRate + k * s.sacc * gauss(R); msg.wz = e.yawRate + k * s.sgyr * gauss(R); }
      msg.sensorPose = sp;
      if (N.lat) this.pending.push({ due: this.t + N.lat, id: s.id, msg }); else { this.sensorOut[s.id] = msg; msg.fresh = true; }
    }
    this.pending = this.pending.filter(p => { if (p.due <= this.t) { this.sensorOut[p.id] = p.msg; p.msg.fresh = true; return false; } return true; });
  }
  // detections -> tracks in the road frame (ideal association by id, alpha filter). A Kalman filter is your job :)
  updateTracks() {
    const e = this.ego, road = this.road;
    for (const m of Object.values(this.sensorOut)) {
      if (!m.det || m._used) continue; m._used = true;
      for (const d of m.det) {
        if (d.id < 0) continue;
        const p = road.project(d.wx, d.wy, e.s), cosA = Math.cos(Math.atan2(d.wy - e.y, d.wx - e.x) - p.heading);
        const vMeas = Math.abs(cosA) > 0.35 ? e.v + d.rr / cosA : null;
        let tr = this.tracks.get(d.id);
        if (!tr) { tr = { id: d.id, s: p.s, d: p.d, v: vMeas ?? e.v, t: m.t }; this.tracks.set(d.id, tr); }
        else { tr.s = road.wrap(tr.s + 0.5 * road.diff(tr.s, p.s)); tr.d += 0.5 * (p.d - tr.d); if (vMeas != null) tr.v += 0.3 * (vMeas - tr.v); tr.t = m.t; }
      }
    }
    for (const [id, tr] of this.tracks) if (this.t - tr.t > 0.6) this.tracks.delete(id);
  }
  rel(tr) { return this.road.diff(this.ego.s, tr.s); }

  // ------------------------------------------------------------------
  // AUTOPILOT (demo): ACC + overtaking with turn signals + pure pursuit
  // ------------------------------------------------------------------
  autopilot() {
    const e = this.ego, ap = this.ap, road = this.road, rc = road.cfg, w = rc.laneWidth, D = this.cfg.drive;
    const has = (id) => this.cfg.sensors.some(s => s.id === id && s.enabled);
    const frontSensor = has('radar_front') || has('rgb_front') || has('lidar');
    const vLim = road.limitAt(e.s) / 3.6, vSet = Math.min(D.setSpeed / 3.6, vLim), h = D.timeGap, d0 = 6, now = this.t;
    const dRef = (t) => ap.lc ? ap.lc.d0 + (ap.lc.d1 - ap.lc.d0) * q5((t - ap.lc.t0) / ap.lc.T) : road.laneD(ap.lane);
    const tracks = [...this.tracks.values()];
    const inLane = (lane, tr) => Math.abs(tr.d - road.laneD(lane)) < w * 0.6;
    const leaderIn = (lane) => { let best = null; for (const tr of tracks) { const ds = this.rel(tr); if (ds > 0 && tr.v > -1 && inLane(lane, tr) && (!best || ds < best.ds)) best = { tr, ds, gap: ds - e.bp.L / 2 }; } return best; };
    let lead = frontSensor ? leaderIn(ap.lc ? ap.lc.toLane : ap.lane) : null;
    if (ap.lc && !lead && frontSensor) lead = leaderIn(ap.lc.fromLane);
    const passing = ap.mode === 'PASS' || ap.mode === 'CHANGE_OUT';
    let aCmd = 0.6 * ((passing ? vLim : vSet) - e.v), gapInfo = null;
    if (lead) {   // constant time-gap ACC: d_des = d0 + h v
      const dDes = d0 + h * e.v;
      aCmd = Math.min(aCmd, 0.25 * (lead.gap - dDes) + 0.7 * (lead.tr.v - e.v));
      const ttc = lead.tr.v < e.v ? lead.gap / (e.v - lead.tr.v) : Infinity;
      if (ttc < 2.0) aCmd = -e.bp.bMax;
      gapInfo = { gap: lead.gap, dDes, vLead: lead.tr.v, ttc };
    }
    if (this.endOfRoad) { aCmd = Math.min(aCmd, -3); ap.status = 'End of the road: stopping'; }
    const sideOf = (to) => road.laneD(to) > road.laneD(ap.lane) ? -1 : 1;     // -1 = left blinker
    const startLc = (toLane, T) => { ap.lc = { d0: e.d, d1: road.laneD(toLane), t0: now, T, toLane, fromLane: ap.lane }; e.signal = sideOf(toLane); ap.lane = toLane; ap.lastLc = now; ap.prep = null; };
    if (ap.lc && now - ap.lc.t0 >= ap.lc.T) { ap.lc = null; e.signal = 0; }
    // "prepare" = blinker on for 1.2 s while the conditions stay true, then steer
    const prepare = (toLane, okNow, T, onStart) => {
      if (!okNow) { if (ap.prep) { ap.prep = null; e.signal = 0; } return false; }
      if (!ap.prep || ap.prep.to !== toLane) { ap.prep = { to: toLane, t0: now }; e.signal = sideOf(toLane); }
      if (now - ap.prep.t0 > 1.2) { startLc(toLane, T); onStart(); return true; }
      ap.status = `Signalling ${e.signal < 0 ? 'left' : 'right'}…`;
      return false;
    };
    const oncomingIn = (lane) => tracks.filter(tr => inLane(lane, tr) && this.rel(tr) > 0 && tr.v < -1);
    if (!frontSensor) ap.status = 'No front sensor: cruise control only';

    if (ap.mode === 'FOLLOW') {
      if (!ap.prep) ap.status = lead ? `Following at ${lead.gap.toFixed(0)} m` : 'Cruising';
      const want = D.allowOvertake && lead && lead.tr.v < vSet - 2.5 && lead.gap < d0 + h * e.v + 25 && now - ap.lastLc > 4 && !this.endOfRoad;
      if (!want && ap.prep && ap.prep.kind === 'out') { ap.prep = null; e.signal = 0; }
      if (want) {
        let target = null;
        if (ap.lane > 1) target = ap.lane - 1; else if (ap.lane === 1 && rc.nLeft > 0 && D.allowOncoming) target = -1;
        if (target === null) ap.status = 'Slower vehicle ahead, no lane to overtake';
        else if (target > 0) {
          if (!has('radar_rear_left') && !has('lidar')) ap.status = 'Overtake blocked: no rear-left radar (blind spot)';
          else {
            const g = this.perceivedGap(target, tracks, inLane);
            const ok = g.front > 30 && g.back > 10 + Math.max(0, g.vBack - e.v) * 3.5;
            if (!ok) ap.status = 'Waiting for a gap in the left lane';
            if (prepare(target, ok, e.bp.tlc, () => { ap.mode = 'CHANGE_OUT'; ap.otId = lead.tr.id; ap.fromLane = ap.lc.fromLane; })) {} else if (ap.prep) ap.prep.kind = 'out';
          }
        } else {
          const vPass = vLim, dv = vPass - lead.tr.v, dist = lead.gap + 12 + e.bp.L + 25, T = e.bp.tlc + dist / Math.max(dv, 0.5);
          let solid = false; for (let q = 0; q < vPass * T; q += 10) if (road.isSolid(e.s + q)) { solid = true; break; }
          const sensRange = Math.max(0, ...this.cfg.sensors.filter(s => s.enabled && ['radar', 'camera', 'lidar'].includes(SENSOR_LIB[s.id].kind) && Math.abs(s.yaw) < 30).map(s => s.range));
          const need = T * (vPass + vLim) + 40, sight = Math.min(sensRange, road.sightAt(e.s));
          const blocked = oncomingIn(-1).some(tr => this.rel(tr) < T * (vPass - tr.v) + 40);
          let why = null;
          if (dv < 2) why = 'Lead vehicle too fast to overtake';
          else if (solid) why = 'No-overtaking zone (solid centre line)';
          else if (blocked) why = 'Oncoming traffic: waiting';
          else if (D.policy === 'conservative' && sight < need) why = `Sight distance too short: need ${need.toFixed(0)} m, see ${sight.toFixed(0)} m`;
          if (why) ap.status = why;
          if (prepare(-1, !why, e.bp.tlc, () => { ap.mode = 'CHANGE_OUT'; ap.otId = lead.tr.id; ap.fromLane = ap.lc.fromLane; })) {} else if (ap.prep) ap.prep.kind = 'out';
        }
      }
    } else if (ap.mode === 'CHANGE_OUT' || ap.mode === 'PASS') {
      if (ap.mode === 'CHANGE_OUT' && !ap.lc) ap.mode = 'PASS';
      const ot = this.tracks.get(ap.otId), otRel = ot ? this.rel(ot) : null;
      ap.status = ot ? `Overtaking: ${otRel > 0 ? otRel.toFixed(0) + ' m behind its rear' : 'alongside / ahead'}` : 'Overtaking';
      if (ap.lane < 0) {   // in the oncoming lane: abort if an oncoming car arrives too early
        const ttc = Math.min(...oncomingIn(ap.lane).map(tr => this.rel(tr) / (e.v - tr.v)), Infinity);
        const remaining = ot ? (otRel + 12 + e.bp.L + 8) / Math.max(e.v - ot.v, 0.5) : 2;
        if (ttc < remaining + 3) {
          if (ot && otRel < -(e.bp.L / 2 + 4)) { ap.mode = 'CHANGE_BACK'; startLc(ap.fromLane, 2.2); }
          else {
            ap.mode = 'ABORT'; this.metrics.aborts++; this.events.push([now, 'Overtake aborted (oncoming traffic)']);
            if (!ot || otRel - e.bp.L / 2 > 4) startLc(ap.fromLane, Math.max(2.0, e.bp.tlc * 0.7)); else { ap.lc = null; e.signal = 1; }
          }
        }
      }
      const passed = ot ? otRel < -(e.bp.L / 2 + 6 + 0.3 * e.v) : now - ap.lastLc > 8;
      if (ap.mode === 'PASS' && passed) {
        const canCheck = ap.fromLane < 0 || ap.lane < 0 || has('radar_rear_right') || has('lidar');
        const g = this.perceivedGap(ap.fromLane, tracks, inLane);
        if (!canCheck) ap.status = 'Cannot check the right lane: no rear-right radar';
        else prepare(ap.fromLane, g.front > 15 && g.back > 6, e.bp.tlc, () => { ap.mode = 'CHANGE_BACK'; });
      }
    } else if (ap.mode === 'ABORT') {
      const ot = this.tracks.get(ap.otId), otRel = ot ? this.rel(ot) : 99;
      if (ap.lane < 0) {
        ap.status = 'Aborting: braking to drop back behind';
        const behind = otRel - e.bp.L / 2 > 4;
        aCmd = Math.min(aCmd, behind ? 0.5 * ((ot ? ot.v : e.v) - 1 - e.v) : -4.5);
        if (behind) startLc(ap.fromLane, Math.max(2.0, e.bp.tlc * 0.7));
      } else { ap.status = 'Aborting: back in lane'; if (!ap.lc) ap.mode = 'FOLLOW'; }
    } else if (ap.mode === 'CHANGE_BACK') {
      ap.status = 'Returning to lane';
      if (!ap.lc) { this.metrics.overtakes++; this.events.push([now, 'Overtake completed']); ap.mode = 'FOLLOW'; }
    }
    if (ap.mode === 'FOLLOW' && !ap.lc && !ap.prep && ap.lane > 0 && ap.lane < rc.nRight && (has('radar_rear_right') || has('lidar'))) {   // keep right
      const g = this.perceivedGap(ap.lane + 1, tracks, inLane);
      ap.keepRightT = g.front > 80 && g.back > 25 ? ap.keepRightT + 0.02 : 0;
      if (ap.keepRightT > 3 && now - ap.lastLc > 5) { ap.prep = { to: ap.lane + 1, t0: now, kind: 'right' }; e.signal = 1; ap.mode = 'KEEP_RIGHT'; }
    }
    if (ap.mode === 'KEEP_RIGHT') { ap.status = 'Moving back to the right lane'; if (!ap.lc && ap.prep && now - ap.prep.t0 > 1.2) startLc(ap.prep.to, e.bp.tlc); if (!ap.lc && !ap.prep) ap.mode = 'FOLLOW'; }

    // pure pursuit: delta = atan(2 L sin(alpha) / Ld) towards a point Ld ahead on the reference
    const Ld = clamp(5 + 0.7 * e.v, 7, 35), tp = road.pose(e.s + Ld, dRef(now + Ld / Math.max(e.v, 1)));
    const lr = e.bp.wb - e.bp.lf, rx = e.x - lr * Math.cos(e.psi), ry = e.y - lr * Math.sin(e.psi);
    const alpha = wrapA(Math.atan2(tp[1] - ry, tp[0] - rx) - e.psi);
    const delta = Math.atan(2 * e.bp.wb * Math.sin(alpha) / Math.hypot(tp[0] - rx, tp[1] - ry));
    return { aCmd: clamp(aCmd, -e.bp.bMax, e.bp.aMax), deltaCmd: delta, gapInfo };
  }
  perceivedGap(lane, tracks, inLane) {
    let front = Infinity, back = Infinity, vBack = 0;
    for (const tr of tracks) {
      if (!inLane(lane, tr)) continue;
      const ds = this.rel(tr);
      if (ds >= 0 && ds - this.ego.bp.L / 2 < front) front = ds - this.ego.bp.L / 2;
      if (ds < 0 && -ds - this.ego.bp.L / 2 < back) { back = -ds - this.ego.bp.L / 2; vBack = tr.v; }
    }
    return { front, back, vBack };
  }

  // ------------------------------------------------------------------
  // EGO: kinematic bicycle at the centre of gravity
  //   beta = atan(lr/L tan(delta)); x' = v cos(psi+beta); y' = v sin(psi+beta); psi' = v cos(beta) tan(delta) / L
  //   v' = a - drag - g sin(grade)   (uphill slows you down)
  // Actuators: steering first-order lag (0.12 s) + rate limit 0.7 rad/s; acceleration lag 0.35 s.
  // ------------------------------------------------------------------
  stepEgo(dt, source) {
    const e = this.ego, b = e.bp, road = this.road;
    let aCmd = 0, deltaCmd = 0, apOut = null, hand = false;
    const ext = this.ext;
    if (source === 'ros' && ext.autopilot) source = 'auto';
    if (source === 'manual') {
      const I = this.input; hand = I.hand;
      if (e.reverse) aCmd = -I.throttle * 2 + (e.v < 0 ? I.brake * b.bMax * 0.9 : 0);
      else aCmd = I.throttle * b.aMax * Math.max(0, 1 - e.v / b.vmax) - I.brake * b.bMax * 0.9;
      deltaCmd = I.steer * b.steerMax / (1 + Math.abs(e.v) / 14);
      this.ctrl = { throttle: I.throttle, brake: I.brake, steer: -I.steer, hand_brake: hand, reverse: e.reverse, source };
    } else if (source === 'auto') {
      apOut = this.autopilot(); aCmd = apOut.aCmd; deltaCmd = apOut.deltaCmd;
      this.ctrl = { throttle: Math.max(0, aCmd) / b.aMax, brake: Math.max(0, -aCmd) / b.bMax, steer: -deltaCmd / b.steerMax, hand_brake: false, reverse: false, source };
    } else {   // ROS: CARLA vehicle_control_cmd (steer +1 = full RIGHT) or Ackermann (steering_angle + = LEFT, rad)
      const fresh = (m) => m && this.t - m.t < 0.5;
      if (fresh(ext.ack) && (!fresh(ext.vcc) || ext.ack.t >= ext.vcc.t)) {
        const A = ext.ack, lim = A.acceleration > 0 ? A.acceleration : null;
        aCmd = clamp(1.2 * (A.speed - e.v), lim ? -lim : -b.bMax, lim || b.aMax);
        deltaCmd = A.steering_angle;
        this.ctrl = { throttle: Math.max(0, aCmd) / b.aMax, brake: Math.max(0, -aCmd) / b.bMax, steer: -deltaCmd / b.steerMax, hand_brake: false, reverse: false, source: 'ros-ackermann' };
      } else if (fresh(ext.vcc)) {
        const C = ext.vcc; hand = !!C.hand_brake; e.reverse = !!C.reverse;
        const th = clamp(C.throttle, 0, 1), br = clamp(C.brake, 0, 1);
        if (e.reverse) aCmd = -th * 2 + (e.v < 0 ? br * b.bMax : 0);
        else aCmd = th * b.aMax * Math.max(0, 1 - e.v / b.vmax) - br * b.bMax;
        deltaCmd = -clamp(C.steer, -1, 1) * b.steerMax;
        this.ctrl = { throttle: th, brake: br, steer: clamp(C.steer, -1, 1), hand_brake: hand, reverse: e.reverse, source: 'ros' };
      } else { aCmd = -1.0; deltaCmd = e.delta; this.ctrl = { throttle: 0, brake: 0.1, steer: -e.delta / b.steerMax, hand_brake: false, reverse: false, source: 'ros (no command)' }; }
      if (ext.signal != null) e.signal = ext.signal;
    }
    if (this.endOfRoad && source !== 'manual') aCmd = Math.min(aCmd, -3);
    if (hand) aCmd = Math.min(aCmd, -6);
    deltaCmd = clamp(deltaCmd, -b.steerMax, b.steerMax);
    e.delta += clamp((deltaCmd - e.delta) / 0.12, -0.7, 0.7) * dt;
    e.a += (aCmd - e.a) * Math.min(1, dt / 0.35);
    const grade = road.gradeAt(e.s);
    const drag = 0.012 * 9.81 + 0.0004 * e.v * e.v;
    let vNew = e.v + (e.a - Math.sign(e.v) * drag - 9.81 * Math.sin(Math.atan(grade))) * dt;
    if (!e.reverse && vNew < 0) vNew = 0; if (e.reverse && vNew > 0) vNew = 0;
    if (hand && Math.abs(vNew) < 0.3) vNew = 0;
    e.v = vNew;
    const mu = this.cfg.road.wet ? 0.5 : 0.9, L = b.wb, lr = b.wb - b.lf;
    let delta = e.delta;
    if (e.v * e.v * Math.abs(Math.tan(delta)) / L > mu * 9.81) delta = Math.sign(delta) * Math.atan(mu * 9.81 * L / (e.v * e.v));   // tyres saturate
    const beta = Math.atan(lr / L * Math.tan(delta));
    e.yawRate = e.v * Math.cos(beta) * Math.tan(delta) / L;
    e.x += e.v * Math.cos(e.psi + beta) * dt; e.y += e.v * Math.sin(e.psi + beta) * dt;
    e.psi = wrapA(e.psi + e.yawRate * dt);
    e.wheelRot += e.v * dt / b.r;
    e.jerk = (e.a - e.prevA) / dt; e.prevA = e.a; e.braking = e.a < -0.5;
    const prevD = e.d;
    const pr = road.project(e.x, e.y, e.s);
    e.s = pr.s; e.d = pr.d; e.z = road.zAt(e.s); e.pitch = Math.atan(grade);
    e.lane = road.laneOf(e.d);
    this.endOfRoad = !road.closed && e.s > road.length - 80;
    this.laneInvasion(prevD, e.d);
    return apOut;
  }
  // which lane markings exist at s: [{d, type}] type 1 = broken, 2 = solid (CARLA LaneInvasion convention)
  markingsAt(s) {
    const c = this.road.cfg, w = c.laneWidth, out = [{ d: this.road.edgeRight, type: 2 }];
    for (let k = 1; k < c.nRight; k++) out.push({ d: -k * w, type: 1 });
    if (c.nLeft === 0) out.push({ d: 0.1, type: 2 });
    else { out.push({ d: 0, type: this.road.isSolid(s) ? 2 : 1 }); for (let k = 1; k < c.nLeft; k++) out.push({ d: k * w, type: 1 }); out.push({ d: this.road.edgeLeft, type: 2 }); }
    return out;
  }
  laneInvasion(d0, d1) {
    const crossed = this.markingsAt(this.ego.s).filter(m => (d0 - m.d) * (d1 - m.d) < 0).map(m => m.type);
    if (!crossed.length) return;
    this.rosEvents.push({ type: 'lane_invasion', t: this.t, crossed });
    if (crossed.includes(2)) { this.metrics.solidCross++; this.events.push([this.t, 'Crossed a SOLID line']); }
  }

  checks(dt) {
    const e = this.ego, m = this.metrics, road = this.road;
    for (const t of this.traffic) {
      const ds = road.diff(e.s, t.s), dd = t.d - e.d;
      if (Math.abs(ds) < (t.bp.L + e.bp.L) / 2 - 0.15 && Math.abs(dd) < (t.bp.W + e.bp.W) / 2 - 0.1) {
        if (this.collideFlash <= 0) {
          m.collisions++; this.events.push([this.t, `Collision with vehicle #${t.id} (${t.bp.name})`]);
          const dv = e.v - t.v * t.dir;
          this.rosEvents.push({ type: 'collision', t: this.t, other: t.id, impulse: [e.bp.m * dv * Math.cos(e.psi), e.bp.m * dv * Math.sin(e.psi), 0] });
        }
        this.collideFlash = 1.5; e.v *= 0.3; t.stopT = 2;
      }
    }
    if (this.collideFlash > 0) this.collideFlash -= dt;
    if (e.d < road.edgeRight - 0.8 || e.d > (road.cfg.nLeft ? road.edgeLeft + 0.8 : 1.5)) m.offRoad += dt;
    const ld = this.leaderOfEgo();
    let ttc = Infinity, thw = Infinity;
    if (ld) { thw = ld.gap / Math.max(e.v, 0.1); if (e.v > ld.vAlong) ttc = ld.gap / (e.v - ld.vAlong); }
    if (e.v > 3) { m.minTTC = Math.min(m.minTTC, ttc); m.minTHW = Math.min(m.minTHW, thw); m.maxJerk = Math.max(m.maxJerk, Math.abs(e.jerk)); }
    m.maxLat = Math.max(m.maxLat, Math.abs(e.v * e.yawRate));
    if (!this.ap.lc && e.lane > 0) { const err = e.d - road.laneD(e.lane); m.laneErrSq += err * err; m.laneErrN++; }
    if (e.v * 3.6 > road.limitAt(e.s) * 1.05) m.speeding += dt;
    m.dist += Math.abs(e.v) * dt;
    this.gt = { ld, ttc, thw };
  }
  leaderOfEgo() {
    const e = this.ego, w = this.road.cfg.laneWidth;
    let best = null;
    for (const t of this.traffic) {
      if (Math.abs(t.d - e.d) > w * 0.6) continue;
      const ds = this.road.diff(e.s, t.s);
      if (ds > 0 && ds < 300 && (!best || ds < best.ds)) best = { ds, gap: ds - (t.bp.L + e.bp.L) / 2, vAlong: t.v * t.dir, t };
    }
    return best;
  }

  step(dt, source) {
    this.t += dt; this.frame++;
    this.stepTraffic(dt);
    const apOut = this.stepEgo(dt, source);
    this.runSensors(dt);
    this.updateTracks();
    this.checks(dt);
    this.apOut = apOut; this.source = source;
    const e = this.ego;
    this._h = (this._h || 0) + dt;
    if (this._h >= 0.1) {
      this._h = 0;
      const g = this.gt.ld;
      this.hist.push({ t: this.t, v: e.v * 3.6, vl: g ? g.vAlong * 3.6 : null, gap: g && g.gap < 150 ? g.gap : null, dDes: 6 + this.cfg.drive.timeGap * e.v, d: e.d, lim: this.road.limitAt(e.s) });
      if (this.hist.length > 400) this.hist.shift();
    }
    if (this.recording) { this._r = (this._r || 0) + dt; if (this._r >= 0.05 - 1e-9) { this._r = 0; this.log.push(this.csvRow()); } }
  }

  // one CSV row (20 Hz), column names carry the units
  csvRow() {
    const e = this.ego, g = this.gt, so = this.sensorOut, rf = so.radar_front, cam = so.rgb_front, gn = so.gnss, im = so.imu;
    const n = (v, d = 3) => (v == null || !isFinite(v)) ? '' : +v.toFixed(d);
    const near = rf && rf.det && rf.det.length ? rf.det.reduce((a, b) => (a.r < b.r ? a : b)) : null;
    return {
      time_s: n(this.t, 2), x_m: n(e.x, 2), y_m: n(e.y, 2), z_m: n(e.z, 2), yaw_deg: n(e.psi / D2R, 2), speed_kmh: n(e.v * 3.6, 2), accel_mps2: n(e.a), jerk_mps3: n(e.jerk, 2),
      steer_angle_deg: n(e.delta / D2R, 2), throttle_0to1: n(this.ctrl.throttle), brake_0to1: n(this.ctrl.brake), turn_signal: e.signal === -1 ? 'left' : e.signal === 1 ? 'right' : e.signal === 2 ? 'hazard' : 'off',
      control_source: this.source || '', autopilot_state: this.source === 'auto' ? this.ap.mode : '',
      road_s_m: n(e.s, 2), lateral_offset_m: n(e.d), lane: this.road.laneName(e.lane), speed_limit_kmh: this.road.limitAt(e.s), grade_pct: n(this.road.gradeAt(e.s) * 100, 2),
      lead_gap_m: g.ld ? n(g.ld.gap, 2) : '', lead_speed_kmh: g.ld ? n(g.ld.vAlong * 3.6, 2) : '', ttc_s: n(g.ttc, 2), time_headway_s: n(g.thw, 2),
      radar_front_detections: rf && rf.det ? rf.det.length : '', radar_front_nearest_range_m: near ? n(near.r, 2) : '', radar_front_nearest_range_rate_mps: near ? n(near.rr, 2) : '',
      camera_lane_offset_m: cam && cam.laneOffset != null ? n(cam.laneOffset) : '', gnss_x_m: gn ? n(gn.x, 2) : '', gnss_y_m: gn ? n(gn.y, 2) : '',
      imu_ax_mps2: im ? n(im.ax) : '', imu_ay_mps2: im ? n(im.ay) : '', imu_yaw_rate_dps: im ? n(im.wz / D2R) : '', collision: this.collideFlash > 1.45 ? 1 : 0,
    };
  }
}
