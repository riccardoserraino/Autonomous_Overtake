// =====================================================================
// main.js — WebDriveSim application: wizard UI (vehicle -> sensors -> road
// -> traffic -> drive & test), 3D visuals, cameras, HUD, plots, ROS 2
// link, export / load, and automatic world loading when served by the
// ROS 2 launch file (http://localhost:8080).
// =====================================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { PRESETS, SEG_TYPES, presetRoad, newSegment, buildRoad, laneList, laneName } from './road.js';
import { BLUEPRINTS, initMaterials, buildVehicle, MAT } from './carModel.js';
import { buildToy, TOY_MAT } from './toyModel.js';
import { World, STYLES } from './world.js';
import { Sim, SENSOR_LIB, SENSOR_PRESETS, NOISE, TRAFFIC_TYPES, makeSensors, q5 } from './sim.js';
import { toYAML, toXODR, toCSV, makeZip, readme, scenarioObject } from './export.js';
import { RosLink, TOPICS } from './ros.js';

const $ = (id) => document.getElementById(id);
const DT = 0.02;
const EGO_TYPES = ['hatch', 'sedan', 'suv', 'van'];
const STEPS = ['Vehicle', 'Sensors', 'Road', 'Traffic', 'Drive & test'];
const ROLE = { ego: '#2ee6c5', same: '#4f8cff', onc: '#ff8a3d', truck: '#c084fc' };
const EGO_PAINTS = ['#2563eb', '#e63946', '#f4a261', '#2a9d8f', '#e9c46a', '#8d6cc4', '#f1faee', '#343a40', '#06d6a0', '#ef476f'];
const LOCAL = /^(localhost|127\.0\.0\.1|\[::1\])$/.test(location.hostname) || location.protocol === 'file:';
const SERVED = location.protocol.startsWith('http') && /^(localhost|127\.0\.0\.1|\[::1\])$/.test(location.hostname);

// scenario quick-picks for the traffic step. lane: 'ego' | 'left' (one lane to the left) | 'onc' (first oncoming lane)
const TRAFFIC_SCENARIOS = {
  empty: { label: 'Empty road', vehicles: [], nSame: 0, nOncoming: 0 },
  slowTruck: { label: 'Slow truck ahead', vehicles: [{ type: 'truck', lane: 'ego', ds: 120, speed: 60 }], nSame: 6, nOncoming: 6 },
  oncoming: { label: 'Truck + oncoming cars', vehicles: [{ type: 'truck', lane: 'ego', ds: 110, speed: 60 }, { type: 'sedan', lane: 'onc', ds: 520, speed: 85 }, { type: 'van', lane: 'onc', ds: 1150, speed: 80 }], nSame: 4, nOncoming: 6 },
  motorway: { label: 'Motorway overtake', vehicles: [{ type: 'truck', lane: 'ego', ds: 110, speed: 80 }, { type: 'sedan', lane: 'left', ds: -90, speed: 135 }, { type: 'suv', lane: 'left', ds: 260, speed: 110 }], nSame: 14, nOncoming: 0 },
};
function resolveScenario(key, roadCfg, egoLane) {
  const sc = TRAFFIC_SCENARIOS[key];
  return sc.vehicles.map(v => ({ type: v.type, ds: v.ds, speed: v.speed, behaviour: 'idm',
    lane: v.lane === 'ego' ? egoLane : v.lane === 'left' ? Math.max(1, egoLane - 1) : (roadCfg.nLeft ? -1 : null) })).filter(v => v.lane !== null);
}
function DEFAULT() {
  const road = { ...presetRoad('rural'), wet: false, rails: true, scenery: true };
  return {
    version: 2, name: 'rural_overtake',
    vehicle: { bp: 'sedan', color: '#2563eb' },
    sensors: makeSensors('sedan', SENSOR_PRESETS.overtake.ids), noise: 'L1', road,
    env: { style: 'toy', time: 'day', fog: false },
    ego: { lane: road.nRight, s: 60, speed: 80 },
    traffic: { vehicles: resolveScenario('oncoming', road, road.nRight), nSame: 4, nOncoming: 6, mix: { hatch: 25, sedan: 30, suv: 20, van: 10, truck: 15 }, vMin: 70, vMax: 100, behaviour: 'idm', laneChanges: true, seed: 42 },
    drive: { mode: 'auto', setSpeed: 90, timeGap: 1.6, allowOvertake: true, allowOncoming: true, policy: 'normal' },
    ros: { url: `ws://${SERVED ? location.hostname : 'localhost'}:9090`, camera: true, sync: false, auto: SERVED },
    view: { fov: true, det: true, lidar: true, tracks: true, path: true, pip: true, plots: true, help: true },
  };
}
function sanitize(c) {     // merge a loaded/stored config with the defaults (older files keep working)
  const d = DEFAULT();
  const out = { ...d, ...c };
  for (const k of ['vehicle', 'road', 'env', 'ego', 'traffic', 'drive', 'ros', 'view']) out[k] = { ...d[k], ...(c[k] || {}) };
  if (!Array.isArray(out.road.segments) || !out.road.segments.length) out.road = d.road;
  if (!Array.isArray(out.sensors)) out.sensors = d.sensors;
  if (SERVED) out.ros.url = `ws://${location.hostname}:9090`;
  return out;
}
function loadCfg() { try { const s = localStorage.getItem('webdrivesim_cfg_v2'); if (s) return sanitize(JSON.parse(s)); } catch (e) {} return null; }
function saveCfg() { try { localStorage.setItem('webdrivesim_cfg_v2', JSON.stringify(cfg)); } catch (e) {} }

let cfg = loadCfg() || DEFAULT();
let step = 0, paused = false, selSensor = 'radar_front', selSeg = 0, worldName = cfg.name;
let world, road, sim, camera, pipCam, controls, egoG, sensorG, thumbs = {}, ros = null, rosStatus = 'offline';
const trafficG = new Map();
let camMode = 'showroom';

// ---------------------------------------------------------------------
// INIT
// ---------------------------------------------------------------------
function init() {
  initMaterials();
  thumbs = makeThumbs();
  world = new World($('c3d'));
  world.style = cfg.env.style;
  world.setQuality($('quality').value);
  camera = new THREE.PerspectiveCamera(60, 1, 0.25, 8000); camera.layers.enable(1);
  pipCam = new THREE.PerspectiveCamera(40, 16 / 9, 0.1, 3000);
  controls = new OrbitControls(camera, $('c3d'));
  controls.enableDamping = true; controls.maxPolarAngle = Math.PI * 0.49; controls.minDistance = 3; controls.maxDistance = 9000;
  applyAll();
  bindStatic();
  renderSteps(); renderStep();
  onResize(); addEventListener('resize', onResize);
  $('loading').style.display = 'none';
  window.__webdrivesim = { get world() { return world; }, get sim() { return sim; }, get road() { return road; }, get cfg() { return cfg; }, get ros() { return ros; }, THREE, goStep: (i) => goStep(i), loadScenario };
  const qp = new URLSearchParams(location.search);
  if (qp.get('world')) loadWorldByName(qp.get('world'), { control: qp.get('control'), autostart: qp.get('autostart') !== '0' });
  if (SERVED) pollSession();
  if (qp.get('ros')) cfg.ros.url = qp.get('ros');
  if (!SERVED && (cfg.ros.auto || qp.get('ros'))) connectRos();
  requestAnimationFrame(frame);
}
// rebuild everything from cfg
function applyAll() {
  world.style = cfg.env.style;
  world.setTime(cfg.env.style, cfg.env.time, cfg.env.fog);
  rebuildRoad(); rebuildEgo(); resetSim(); setLights();
  worldName = cfg.name || worldName;
}

function makeThumbs() {
  const out = {};
  try {
    const r = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true, alpha: true });
    r.setSize(312, 180); r.outputColorSpace = THREE.SRGBColorSpace;
    const sc = new THREE.Scene();
    sc.add(new THREE.HemisphereLight(0xffffff, 0x888888, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.6); key.position.set(4, 8, 6); sc.add(key);
    const cam = new THREE.PerspectiveCamera(28, 312 / 180, 0.1, 100);
    for (const k of EGO_TYPES) {
      const b = BLUEPRINTS[k], g = buildToy(k, cfg.vehicle.color || '#2563eb', 'toy');
      sc.add(g); const s = b.L / 4.7;
      cam.position.set(5.6 * s, 2.4 * s, 6.2 * s); cam.lookAt(0, b.H * 0.42, 0);
      r.setClearColor(0x1b1f29, 1); r.render(sc, cam);
      out[k] = r.domElement.toDataURL('image/png'); sc.remove(g);
    }
    r.dispose(); r.forceContextLoss();
  } catch (e) { console.warn('thumbs', e); }
  return out;
}

// ---------------------------------------------------------------------
// WORLD / SIM building
// ---------------------------------------------------------------------
function rebuildRoad() {
  road = buildRoad(cfg.road);
  cfg.ego.lane = Math.min(cfg.road.nRight, Math.max(1, cfg.ego.lane || cfg.road.nRight));
  cfg.ego.s = Math.min(cfg.ego.s, Math.max(0, road.length - 300));
  world.buildRoad(road, { rails: cfg.road.rails, scenery: cfg.road.scenery });
  if (world.q) world.setQuality($('quality').value);
  xodrCache = null;
}
const realistic = () => cfg.env.style === 'realistic';
function vehicleModel(key, color, opts) {
  return realistic() ? buildVehicle(key, color, opts) : buildToy(key, color, cfg.env.style, opts);
}
function rebuildEgo() {
  if (egoG) world.scene.remove(egoG);
  const ghost = cfg.env.style === 'ghost';
  egoG = vehicleModel(cfg.vehicle.bp, ghost ? ROLE.ego : cfg.vehicle.color, { separateWheels: true, ownTail: true });
  const b = BLUEPRINTS[cfg.vehicle.bp];
  egoG.userData.spots = [1, -1].map(sd => {
    const sp = new THREE.SpotLight(0xfff3e0, 0, 120, 0.42, 0.55, 1.2); sp.position.set(b.L / 2, b.belt - 0.1, sd * 0.6);
    const tg = new THREE.Object3D(); tg.position.set(b.L / 2 + 25, -1.5, sd * 0.6); egoG.add(sp, tg); sp.target = tg; return sp;
  });
  buildInterior();
  world.scene.add(egoG);
  buildSensorViz(); setLights();
}
function buildInterior() {
  const b = BLUEPRINTS[cfg.vehicle.bp], xw0 = b.L / 2 - b.hood, xw1 = xw0 - b.wind, hx = xw1 - 0.35;
  const it = new THREE.Group(); it.visible = false;
  const dm = new THREE.MeshStandardMaterial({ color: 0x1c1e23, roughness: 0.85 });
  const add = (geo, x, y, z, m = dm) => { const o = new THREE.Mesh(geo, m); o.position.set(x, y, z); it.add(o); return o; };
  const d0 = hx + 0.5, d1 = xw0 + 0.05, dh = b.belt + 0.05 - (b.clr + 0.1);
  add(new THREE.BoxGeometry(d1 - d0, dh, b.W * 0.94), (d0 + d1) / 2, b.belt + 0.05 - dh / 2, 0);
  add(new THREE.BoxGeometry(2.4, 0.05, b.W * 0.94), hx - 0.5, b.clr + 0.12, 0);
  add(new THREE.BoxGeometry(0.2, 0.07, 0.42), hx + 0.56, b.belt + 0.085, -0.36);
  add(new THREE.BoxGeometry(b.hood + 0.1, 0.04, b.W * 0.92), xw0 + b.hood / 2 + 0.05, b.belt + 0.07, 0, new THREE.MeshStandardMaterial({ color: cfg.vehicle.color }));
  add(new THREE.BoxGeometry(1.7, 0.03, b.W * 0.9), xw1 - 0.8, b.H - 0.05, 0);
  const doorH = b.belt + 0.03 - b.clr;
  for (const sd of [1, -1]) {
    add(new THREE.BoxGeometry(xw0 - hx + 0.8, doorH, 0.06), (xw0 + hx - 0.8) / 2, b.clr + doorH / 2, sd * (b.W / 2 - 0.1));
    const A = new THREE.Vector3(xw0 - 0.02, b.belt + 0.08, sd * (b.W / 2 - 0.12)), B = new THREE.Vector3(xw1 + 0.02, b.H - 0.05, sd * (b.W / 2 - 0.2));
    const p = add(new THREE.CylinderGeometry(0.045, 0.06, A.distanceTo(B), 8), (A.x + B.x) / 2, (A.y + B.y) / 2, (A.z + B.z) / 2);
    p.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize());
  }
  const wg = new THREE.Group(); wg.position.set(hx + 0.43, b.belt + 0.08, -0.36); wg.rotation.z = -0.38;
  const spin = new THREE.Group(); wg.add(spin);
  const rim = new THREE.Mesh(new THREE.TorusGeometry(0.18, 0.022, 10, 36), dm); rim.rotation.y = Math.PI / 2; spin.add(rim);
  for (const a of [0, 2.1, -2.1]) { const sp = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.17, 0.03), dm); sp.position.set(0, Math.cos(a) * 0.085, Math.sin(a) * 0.085); sp.rotation.x = -a; spin.add(sp); }
  it.add(wg); egoG.add(it);
  egoG.userData.interior = it; egoG.userData.wheelSpin = spin;
}
function resetSim() {
  sim = new Sim(cfg, road);
  for (const g of trafficG.values()) world.scene.remove(g);
  trafficG.clear();
  const ghost = cfg.env.style === 'ghost';
  for (const t of sim.traffic) {
    const col = ghost ? (t.bpKey === 'truck' ? ROLE.truck : t.dir > 0 ? ROLE.same : ROLE.onc) : t.paint;
    const g = vehicleModel(t.bpKey, col, {}); world.scene.add(g); trafficG.set(t.id, g);
  }
  gnssTrail.length = 0;
  ensureOverlays(); updateVisuals();
}
function setLights() {
  const night = cfg.env.time === 'night';
  MAT.head.emissiveIntensity = night ? 3.0 : 0.12; MAT.tail.emissiveIntensity = night ? 1.0 : 0.25;
  const tm = TOY_MAT(); tm.head.emissiveIntensity = night ? 2.5 : 0.25; tm.tail.emissiveIntensity = night ? 1.0 : 0.25;
  if (egoG) for (const s of egoG.userData.spots) s.intensity = night ? 60 : 0;
}

// sensor mount markers + field-of-view fans (layer 1 = overlays, not seen by the camera sensor)
function buildSensorViz() {
  if (sensorG) egoG.remove(sensorG);
  sensorG = new THREE.Group(); egoG.add(sensorG);
  const dark = new THREE.MeshStandardMaterial({ color: 0x15171c, metalness: 0.4, roughness: 0.4 });
  for (const s of cfg.sensors) {
    if (!s.enabled) continue;
    const lib = SENSOR_LIB[s.id], g = new THREE.Group();
    g.position.set(s.x, s.z, -s.y); g.rotation.y = s.yaw * Math.PI / 180;
    const mk = lib.kind === 'lidar' ? new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.085, 0.11, 24), dark)
      : new THREE.Mesh(lib.kind === 'camera' ? new THREE.BoxGeometry(0.06, 0.05, 0.1) : lib.kind === 'radar' ? new THREE.BoxGeometry(0.03, 0.08, 0.11) : new THREE.BoxGeometry(0.06, 0.03, 0.06), dark);
    g.add(mk);
    const col = new THREE.Color(lib.color), sel = step === 1 && selSensor === s.id, op = step === 1 ? (sel ? 0.26 : 0.07) : 0.07;
    if (['radar', 'camera'].includes(lib.kind)) {
      const fan = new THREE.Mesh(new THREE.CircleGeometry(s.range, 48, -s.fov * Math.PI / 360, s.fov * Math.PI / 180), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: op, depthWrite: false, side: THREE.DoubleSide }));
      fan.rotation.x = -Math.PI / 2; fan.layers.set(1); fan.userData.fan = true; g.add(fan);
      const edge = new THREE.LineSegments(new THREE.EdgesGeometry(fan.geometry), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: sel ? 0.9 : 0.35 }));
      edge.rotation.x = -Math.PI / 2; edge.layers.set(1); edge.userData.fan = true; g.add(edge);
    } else if (lib.kind === 'lidar') {
      const ring = new THREE.Mesh(new THREE.RingGeometry(s.range - 0.4, s.range, 96), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: sel ? 0.5 : 0.18, depthWrite: false, side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2; ring.position.y = -s.z + 0.05; ring.layers.set(1); ring.userData.fan = true; g.add(ring);
    }
    if (sel) { const halo = new THREE.Mesh(new THREE.SphereGeometry(0.16, 16, 12), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.55, depthTest: false })); halo.layers.set(1); halo.renderOrder = 10; g.add(halo); }
    sensorG.add(g);
  }
}
const ov = {}; const gnssTrail = [];
function ensureOverlays() {
  if (ov.det) return;
  const sc = world.scene;
  ov.det = new THREE.InstancedMesh(new THREE.SphereGeometry(0.22, 10, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }), 400);
  ov.det.layers.set(1); ov.det.frustumCulled = false; ov.det.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(400 * 3), 3); sc.add(ov.det);
  const pg = new THREE.BufferGeometry(); pg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(9000 * 3), 3));
  ov.pts = new THREE.Points(pg, new THREE.PointsMaterial({ color: 0x9be15d, size: 0.09 })); ov.pts.layers.set(1); ov.pts.frustumCulled = false; sc.add(ov.pts);
  const tg = new THREE.BufferGeometry(); tg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(60 * 24 * 3), 3));
  ov.trk = new THREE.LineSegments(tg, new THREE.LineBasicMaterial({ color: 0x38e1ff })); ov.trk.layers.set(1); ov.trk.frustumCulled = false; sc.add(ov.trk);
  const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(60 * 3), 3));
  ov.path = new THREE.Line(lg, new THREE.LineBasicMaterial({ color: 0x00c48c })); ov.path.layers.set(1); ov.path.frustumCulled = false; sc.add(ov.path);
  ov.gnss = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.07, 0.07, 0.02, 10), new THREE.MeshBasicMaterial({ color: 0xc084fc, transparent: true, opacity: 0.55 }), 40);
  ov.gnss.layers.set(1); ov.gnss.frustumCulled = false; sc.add(ov.gnss);
}

// ---------------------------------------------------------------------
// PER-FRAME VISUALS
// ---------------------------------------------------------------------
const tmpC = new THREE.Color(), Y = new THREE.Vector3(0, 1, 0), Zax = new THREE.Vector3(0, 0, 1), qa = new THREE.Quaternion(), qb = new THREE.Quaternion();
function placeVehicle(g, x, y, z, yaw, pitch) { g.position.set(x, z, -y); g.quaternion.copy(qa.setFromAxisAngle(Y, yaw).multiply(qb.setFromAxisAngle(Zax, pitch))); }
function blink(g, sig, on) { const ind = g.userData.ind; if (!ind) return; ind.L.visible = on && (sig === -1 || sig === 2); ind.R.visible = on && (sig === 1 || sig === 2); }
function updateVisuals() {
  const e = sim.ego, blinkOn = Math.floor(performance.now() / 333) % 2 === 0;
  placeVehicle(egoG, e.x, e.y, e.z, e.psi, e.pitch);
  for (const w of egoG.userData.wheels) { if (w.front) w.pivot.rotation.y = e.delta; w.spin.rotation.z = -e.wheelRot * w.side; }
  const cockpit = camMode === 'cockpit';
  for (const c of egoG.children) if (c.isMesh || c.isLineSegments) { if (c !== egoG.userData.ind.L && c !== egoG.userData.ind.R) c.visible = !cockpit; }
  for (const w of egoG.userData.wheels) w.pivot.visible = !cockpit;
  egoG.userData.interior.visible = cockpit;
  egoG.userData.wheelSpin.rotation.x = -e.delta * 15;
  blink(egoG, e.signal, blinkOn && !cockpit);
  const tail = egoG.userData.tail; if (tail) tail.emissiveIntensity = e.braking ? 3 : (cfg.env.time === 'night' ? 1 : 0.25);
  for (const t of sim.traffic) { const g = trafficG.get(t.id); if (g) { placeVehicle(g, t.x, t.y, t.z, t.psi, Math.atan(road.gradeAt(t.s)) * t.dir); blink(g, t.signal, blinkOn); } }
  const V = cfg.view;
  sensorG.traverse(o => { if (o.userData.fan) o.visible = V.fov || step === 1; });
  // detections
  let n = 0;
  if (V.det || step === 1) for (const s of cfg.sensors) {
    if (!s.enabled) continue;
    const m = sim.sensorOut[s.id], lib = SENSOR_LIB[s.id];
    if (!m || !m.det || lib.kind === 'lidar') continue;
    tmpC.set(lib.color);
    for (const d of m.det) { if (n >= 400) break; ov.det.setMatrixAt(n, new THREE.Matrix4().makeTranslation(d.wx, d.wz + (lib.kind === 'camera' ? 0.5 : 0), -d.wy)); ov.det.setColorAt(n, d.id < 0 ? tmpC.clone().set(0xffffff) : tmpC); n++; }
  }
  ov.det.count = n; ov.det.instanceMatrix.needsUpdate = true; if (ov.det.instanceColor) ov.det.instanceColor.needsUpdate = true;
  const lid = sim.sensorOut.lidar, pa = ov.pts.geometry.attributes.position; let np = 0;
  if ((V.lidar || step === 1) && lid && lid.points && cfg.sensors.find(s => s.id === 'lidar' && s.enabled)) for (let i = 0; i < lid.points.length && np < 9000; i += 3) pa.setXYZ(np++, lid.points[i], lid.points[i + 2], -lid.points[i + 1]);
  ov.pts.geometry.setDrawRange(0, np); pa.needsUpdate = true;
  // tracks (what the demo autopilot believes)
  const ta = ov.trk.geometry.attributes.position; let nt = 0;
  if (V.tracks && step >= 1) for (const tr of sim.tracks.values()) {
    if (nt >= 60) break;
    const rel = sim.rel(tr), onc = tr.v < -1, L = 4.4, W = 1.9, H = 1.6;
    const p = road.pose(tr.s + Math.sign(rel || 1) * L / 2, tr.d), h = p[2] + (onc ? Math.PI : 0), c = Math.cos(h), s = Math.sin(h), z0 = p[3] + 0.05;
    const cor = [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(([a, b]) => [p[0] + a * L / 2 * c - b * W / 2 * s, p[1] + a * L / 2 * s + b * W / 2 * c]);
    let k = 0; const seg = (A, B, ha, hb) => { ta.setXYZ(nt * 24 + k++, A[0], ha, -A[1]); ta.setXYZ(nt * 24 + k++, B[0], hb, -B[1]); };
    for (let i = 0; i < 4; i++) { const A = cor[i], B = cor[(i + 1) % 4]; seg(A, B, z0, z0); seg(A, B, z0 + H, z0 + H); seg(A, A, z0, z0 + H); }
    nt++;
  }
  ov.trk.geometry.setDrawRange(0, nt * 24); ta.needsUpdate = true;
  // planned path (autopilot) or predicted path (manual / ROS: current steering held for 3 s)
  const la = ov.path.geometry.attributes.position; let nl = 0;
  if (V.path && step >= 3) {
    if (sim.source === 'auto') {
      const ap = sim.ap, span = Math.max(25, e.v * 4);
      for (let i = 0; i < 60; i++) { const ds = span * i / 59, t = sim.t + ds / Math.max(e.v, 1); const d = ap.lc ? ap.lc.d0 + (ap.lc.d1 - ap.lc.d0) * q5((t - ap.lc.t0) / ap.lc.T) : road.laneD(ap.lane); const p = road.pose(e.s + ds, d); la.setXYZ(nl++, p[0], p[3] + 0.1, -p[1]); }
    } else {
      let x = e.x, y = e.y, psi = e.psi; const v = Math.max(e.v, 2);
      for (let i = 0; i < 60; i++) { la.setXYZ(nl++, x, road.zAt(road.project(x, y, e.s).s) + 0.1, -y); psi += e.yawRate / Math.max(e.v, 0.1) * v * 0.05; x += v * Math.cos(psi) * 0.05; y += v * Math.sin(psi) * 0.05; }
    }
  }
  ov.path.geometry.setDrawRange(0, nl); la.needsUpdate = true;
  const gn = sim.sensorOut.gnss;
  if (gn && gn.t !== ov.lastG) { ov.lastG = gn.t; gnssTrail.push([gn.x, gn.y, e.z]); if (gnssTrail.length > 40) gnssTrail.shift(); }
  gnssTrail.forEach((q, i) => ov.gnss.setMatrixAt(i, new THREE.Matrix4().makeTranslation(q[0], q[2] + 0.05, -q[1])));
  ov.gnss.count = V.det && step >= 3 && cfg.sensors.find(s => s.id === 'gnss' && s.enabled) ? gnssTrail.length : 0; ov.gnss.instanceMatrix.needsUpdate = true;
}

// ---------------------------------------------------------------------
// CAMERAS
// ---------------------------------------------------------------------
const CAMS = [['chase', 'Chase'], ['cockpit', 'Cockpit'], ['bird', "Bird's-eye"], ['orbit', 'Orbit'], ['sensor', 'Camera sensor']];
const v3 = (x, y, h) => new THREE.Vector3(x, h, -y);
let lastEgo = null;
function updateCamera(dt) {
  const e = sim.ego, b = e.bp, fwd = new THREE.Vector3(Math.cos(e.psi), 0, -Math.sin(e.psi)), ego = v3(e.x, e.y, e.z);
  controls.enabled = ['showroom', 'orbit', 'overview'].includes(camMode);
  controls.autoRotate = camMode === 'showroom'; controls.autoRotateSpeed = 0.8;
  if (controls.enabled) {
    if (camMode !== 'overview' && lastEgo) camera.position.add(ego.clone().sub(lastEgo));
    if (camMode !== 'overview') controls.target.copy(ego.clone().add(new THREE.Vector3(0, b.H * 0.5, 0)));
    controls.update(); camera.fov = 50;
  } else if (camMode === 'chase') {
    const want = ego.clone().addScaledVector(fwd, -(b.L + 5.5)).add(new THREE.Vector3(0, b.H + 1.4, 0));
    camera.position.lerp(want, 1 - Math.exp(-dt * 6));
    camera.lookAt(ego.clone().addScaledVector(fwd, 8).add(new THREE.Vector3(0, 1.0, 0))); camera.fov = 62;
  } else if (camMode === 'cockpit') {
    egoG.updateMatrixWorld();
    const head = new THREE.Vector3(b.L / 2 - b.hood - b.wind - 0.35, b.belt + 0.33, -0.36).applyMatrix4(egoG.matrixWorld);
    camera.position.copy(head);
    const look = new THREE.Vector3(b.L / 2 + 20, b.belt - 0.5, -0.36 + e.delta * 6).applyMatrix4(egoG.matrixWorld);
    camera.lookAt(look); camera.fov = 72;
  } else if (camMode === 'bird') {
    camera.position.lerp(ego.clone().addScaledVector(fwd, -12).add(new THREE.Vector3(0, 55, 0)), 1 - Math.exp(-dt * 4));
    camera.lookAt(ego.clone().addScaledVector(fwd, 14)); camera.fov = 50;
  } else if (camMode === 'sensor') setSensorCam(camera);
  camera.updateProjectionMatrix();
  lastEgo = ego;
}
function setSensorCam(cam) {
  const s = cfg.sensors.find(q => q.id === 'rgb_front') || { x: 1, y: 0, z: 1.3, yaw: 0, fov: 60 };
  egoG.updateMatrixWorld();
  const p = new THREE.Vector3(s.x + 0.08, s.z, -s.y).applyMatrix4(egoG.matrixWorld);
  const ahead = new THREE.Vector3(s.x + 10 * Math.cos(s.yaw * Math.PI / 180), s.z - 0.4, -s.y - 10 * Math.sin(s.yaw * Math.PI / 180)).applyMatrix4(egoG.matrixWorld);
  cam.position.copy(p); cam.lookAt(ahead);
  cam.fov = 2 * Math.atan(Math.tan(s.fov * Math.PI / 360) / cam.aspect) * 180 / Math.PI; cam.updateProjectionMatrix();
}
function setCam(m) {
  camMode = m;
  if (m === 'overview') {
    let a = 1e9, b = -1e9, c = 1e9, d = -1e9;
    for (let i = 0; i < road.N; i += 20) { a = Math.min(a, road.X[i]); b = Math.max(b, road.X[i]); c = Math.min(c, road.Y[i]); d = Math.max(d, road.Y[i]); }
    const cx = (a + b) / 2, cy = (c + d) / 2, ext = Math.max(b - a, (d - c) * camera.aspect) + 200;
    const dist = ext / (2 * Math.tan(25 * Math.PI / 180) * camera.aspect) * 1.05;
    controls.target.set(cx, 0, -cy); camera.position.set(cx, dist * 0.85, -cy + dist * 0.5);
  } else if (m === 'showroom' || m === 'orbit') {
    const e = sim.ego, b = e.bp, fwd = new THREE.Vector3(Math.cos(e.psi), 0, -Math.sin(e.psi)), right = new THREE.Vector3(Math.sin(e.psi), 0, Math.cos(e.psi));
    camera.position.copy(v3(e.x, e.y, e.z)).addScaledVector(fwd, b.L * 1.25).addScaledVector(right, b.L * 1.1).add(new THREE.Vector3(0, b.H * 1.3, 0));
    controls.target.copy(v3(e.x, e.y, e.z + b.H * 0.5));
  }
  lastEgo = null; renderCamBar();
}

// ---------------------------------------------------------------------
// INPUT: keys like CARLA manual_control (+ blinkers) and gamepad
// ---------------------------------------------------------------------
const keys = {}; let steerKb = 0;
addEventListener('keydown', (ev) => {
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName)) return;
  keys[ev.code] = true;
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space', 'Tab'].includes(ev.code)) ev.preventDefault();
  if (step < 3) return;
  const e = sim.ego;
  if (ev.code === 'KeyP') { cfg.drive.mode = cfg.drive.mode === 'manual' ? 'auto' : 'manual'; if (step === 4) renderStep(); toast(cfg.drive.mode === 'auto' ? 'Autopilot ON' : 'Manual driving'); }
  if (ev.code === 'Tab') { const i = CAMS.findIndex(c => c[0] === camMode); setCam(CAMS[(i + 1) % CAMS.length][0]); }
  if (ev.code === 'KeyQ' && Math.abs(e.v) < 0.5) { e.reverse = !e.reverse; toast(e.reverse ? 'Reverse gear' : 'Drive'); }
  if (ev.code === 'KeyZ') e.signal = e.signal === -1 ? 0 : -1;
  if (ev.code === 'KeyX') e.signal = e.signal === 1 ? 0 : 1;
  if (ev.code === 'KeyC') e.signal = e.signal === 2 ? 0 : 2;
  if (ev.code === 'KeyR') { resetSim(); toast('Run reset'); }
  if (ev.code === 'KeyH') cfg.view.help = !cfg.view.help;
  if (ev.code === 'KeyF') cfg.view.fov = !cfg.view.fov;
});
addEventListener('keyup', (ev) => { keys[ev.code] = false; });
function pollInput(dt) {
  const I = sim.input, left = keys.KeyA || keys.ArrowLeft, right = keys.KeyD || keys.ArrowRight;
  const target = (left ? 1 : 0) - (right ? 1 : 0), rate = target === 0 ? 3.0 : 1.6;
  steerKb += Math.max(-rate * dt, Math.min(rate * dt, target - steerKb));
  I.steer = steerKb; I.throttle = keys.KeyW || keys.ArrowUp ? 1 : 0; I.brake = keys.KeyS || keys.ArrowDown ? 1 : 0; I.hand = !!keys.Space;
  try {
    const gp = navigator.getGamepads && [...navigator.getGamepads()].find(g => g);
    if (gp) { if (Math.abs(gp.axes[0]) > 0.06) I.steer = -gp.axes[0]; if (gp.buttons[7] && gp.buttons[7].value > 0.05) I.throttle = gp.buttons[7].value; if (gp.buttons[6] && gp.buttons[6].value > 0.05) I.brake = gp.buttons[6].value; }
  } catch (e) {}
}

// ---------------------------------------------------------------------
// ROS 2 link
// ---------------------------------------------------------------------
let xodrCache = null;
function connectRos() {
  if (ros) ros.close();
  ros = new RosLink(cfg.ros.url, {
    getSim: () => sim, getRoad: () => road, getCfg: () => cfg, getWorldName: () => worldName, isPaused: () => paused,
    getXodr: () => (xodrCache = xodrCache || toXODR(road, worldName)), cameraJpeg,
    onStatus: (s) => { rosStatus = s; renderRosPill(); if (step === 4) { const el = $('rosState'); if (el) el.textContent = s; } },
    onReset: () => { resetSim(); toast('Reset by /webdrivesim/sim/reset'); },
    onLoadWorld: (name) => loadWorldByName(name, {}),
    onCarlaControl: (cmd) => { if (cmd === 0) paused = false; if (cmd === 1) paused = true; if (cmd === 2) { paused = true; stepOnce = true; } if (step === 4) renderStep(); },
  });
  ros.connect();
}
let stepOnce = false;
function renderRosPill() {
  const p = $('rosPill'); if (!p) return;
  const ok = rosStatus === 'connected';
  p.innerHTML = `<span class="dot" style="background:${ok ? 'var(--good)' : rosStatus === 'connecting' ? 'var(--warn)' : '#566'}"></span>ROS ${ok ? 'connected' : rosStatus === 'connecting' ? 'connecting…' : 'offline'}`;
  p.title = ros ? cfg.ros.url : 'Not connected';
}
// render the camera sensor into a small target and return a base64 JPEG (for /rgb_front/image/compressed)
let camRT, camCanvas;
function cameraJpeg() {
  const W = 480, H = 270, r = world.renderer;
  if (!camRT) { camRT = new THREE.WebGLRenderTarget(W, H); camCanvas = document.createElement('canvas'); camCanvas.width = W; camCanvas.height = H; }
  const cam = new THREE.PerspectiveCamera(40, W / H, 0.1, 3000); setSensorCam(cam);
  const intVis = egoG.userData.interior.visible; egoG.userData.interior.visible = false;
  r.setRenderTarget(camRT); r.render(world.scene, cam); r.setRenderTarget(null);
  egoG.userData.interior.visible = intVis;
  const px = new Uint8Array(W * H * 4); r.readRenderTargetPixels(camRT, 0, 0, W, H, px);
  const ctx = camCanvas.getContext('2d'), img = ctx.createImageData(W, H);
  for (let y = 0; y < H; y++) img.data.set(px.subarray((H - 1 - y) * W * 4, (H - y) * W * 4), y * W * 4);
  ctx.putImageData(img, 0, 0);
  return camCanvas.toDataURL('image/jpeg', 0.7).split(',')[1];
}

// ---------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------
let last = performance.now(), accT = 0, fps = 60, hudT = 0, frozenT = 0, lastSyncFrame = 0;
function source() { return step === 4 ? cfg.drive.mode : 'auto'; }
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  fps = fps * 0.95 + 0.05 / Math.max(dt, 1e-3);
  const src = source(), running = step >= 3 && !paused;
  const doStep = (h) => { if (src === 'manual') pollInput(h); sim.step(h, src); };
  if (step >= 3 && src === 'ros' && cfg.ros.sync && ros && ros.ok) {
    // CARLA-like synchronous mode: one 0.05 s frame per received command (1 s timeout)
    if ((running || stepOnce) && (sim.ext.lastMsgT > lastSyncFrame || now - lastSyncFrame > 1000)) {
      doStep(0.025); doStep(0.025); ros.tick(true); lastSyncFrame = performance.now(); stepOnce = false;
    }
  } else if (running || stepOnce) {
    accT += stepOnce ? 0.05 : dt; let n = 0;
    while (accT >= DT && n < 6) { doStep(DT); if (ros) ros.tick(false); accT -= DT; n++; }
    if (n === 6) accT = 0; stepOnce = false;
  } else if (step < 3) {
    frozenT += dt;
    if (frozenT > 0.1) { frozenT = 0; sim.t += 0.1; sim.runSensors(1); sim.updateTracks(); }
  }
  updateVisuals(); updateCamera(dt);
  world.scene.fog = camMode === 'overview' ? null : world.fogObj;   // no haze on the map overview
  world.follow(v3(sim.ego.x, sim.ego.y, sim.ego.z));
  const r = world.renderer, W = $('c3d').clientWidth, H = $('c3d').clientHeight;
  r.setViewport(0, 0, W, H); r.setScissorTest(false); r.render(world.scene, camera);
  const camOn = cfg.sensors.some(s => s.id === 'rgb_front' && s.enabled), showPip = cfg.view.pip && camOn && step >= 3 && camMode !== 'sensor';
  $('pipLbl').style.display = showPip ? 'block' : 'none';
  if (showPip) {
    const pw = Math.round(Math.min(360, W * 0.3)), ph = Math.round(pw * 9 / 16), px = W - pw - 12, py = 12;
    pipCam.aspect = pw / ph; setSensorCam(pipCam);
    const iv = egoG.userData.interior.visible; egoG.userData.interior.visible = false;
    r.setScissorTest(true); r.setScissor(px, py, pw, ph); r.setViewport(px, py, pw, ph); r.render(world.scene, pipCam);
    r.setScissorTest(false); r.setViewport(0, 0, W, H); egoG.userData.interior.visible = iv;
    $('pipLbl').style.bottom = (py + ph + 4) + 'px'; $('help').style.bottom = (py + ph + 30) + 'px';
  } else $('help').style.bottom = '12px';
  hudT += dt;
  if (hudT > 0.12) { hudT = 0; drawHud(); drawPlots(); if (step === 4) liveStep5(); }
  $('flash').style.opacity = sim.collideFlash > 0 ? Math.min(1, sim.collideFlash) : 0;
}

// ---------------------------------------------------------------------
// HUD (CARLA manual_control style)
// ---------------------------------------------------------------------
function bar(v, n = 12) { v = Math.max(0, Math.min(1, v)); const k = Math.round(v * n); return '[' + '█'.repeat(k) + ' '.repeat(n - k) + ']'; }
function steerBar(v, n = 12) { v = Math.max(-1, Math.min(1, v)); const k = Math.round((v + 1) / 2 * (n - 1)); return '[' + Array.from({ length: n }, (_, i) => i === k ? '|' : (i === Math.floor(n / 2) ? '·' : ' ')).join('') + ']'; }
function drawHud() {
  const e = sim.ego, b = e.bp, gn = sim.sensorOut.gnss, src = source();
  const deg = ((e.psi * 180 / Math.PI) % 360 + 360) % 360, bearing = (450 - deg) % 360, card = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][Math.round(bearing / 45) % 8];
  const tt = Math.floor(sim.t), clock = `${String(Math.floor(tt / 3600)).padStart(2, '0')}:${String(Math.floor(tt / 60) % 60).padStart(2, '0')}:${String(tt % 60).padStart(2, '0')}`;
  const near = sim.traffic.map(t => ({ t, d: Math.hypot(t.x - e.x, t.y - e.y) })).filter(o => o.d < 250).sort((a, c) => a.d - c.d).slice(0, 4);
  const sig = e.signal === -1 ? '◀ left' : e.signal === 1 ? 'right ▶' : e.signal === 2 ? '◀ hazard ▶' : 'off';
  const mode = step < 3 ? 'setup (paused)' : src === 'manual' ? 'MANUAL' : src === 'ros' ? `ROS 2 · ${sim.ctrl.source}` : 'AUTOPILOT · ' + sim.ap.mode;
  const lines = [
    `WebDriveSim${' '.repeat(18)}${String(Math.round(fps)).padStart(3)} FPS`,
    `Vehicle:  ${b.name}`, `Map:      ${worldName} (${(road.length / 1000).toFixed(1)} km)`, `Sim time: ${clock}`, '',
    `Speed:    ${(e.v * 3.6).toFixed(0).padStart(4)} km/h  (limit ${road.limitAt(e.s)})${e.reverse ? ' R' : ''}`,
    `Heading:  ${bearing.toFixed(0).padStart(4)}° ${card}`,
    `Location: (${e.x.toFixed(1).padStart(7)}, ${e.y.toFixed(1).padStart(7)}, ${e.z.toFixed(1)})`,
    `GNSS:     ${gn ? `(${gn.x.toFixed(1).padStart(7)}, ${gn.y.toFixed(1).padStart(7)})` : 'n/a'}`,
    `Road:     s=${e.s.toFixed(0)} m  grade ${(road.gradeAt(e.s) * 100).toFixed(1)}%`,
    `Lane:     ${road.laneName(e.lane)}  d=${e.d.toFixed(2)} m`, '',
    `Throttle: ${bar(sim.ctrl.throttle)}`, `Steer:    ${steerBar(sim.ctrl.steer)}`, `Brake:    ${bar(sim.ctrl.brake)}`,
    `Blinker:  ${sig}`, `Control:  ${mode}`, `ROS:      ${rosStatus}`, `Collisions: ${sim.metrics.collisions}`,
  ];
  if (near.length) { lines.push('Nearby vehicles:'); for (const o of near) lines.push(`  ${o.d.toFixed(0).padStart(4)} m  ${o.t.bp.name}${o.t.dir < 0 ? ' (oncoming)' : ''}`); }
  $('hud').textContent = lines.join('\n');
  const hints = ['Pick a vehicle and paint. Drag to orbit.', 'Place sensors; the fans show range and field of view.', 'Pick a preset or build your road segment by segment.'];
  $('status').innerHTML = step < 3 ? `<b>Setup ${step + 1}/5</b> · ${hints[step]}` : step === 3 ? `<b>Preview</b> · demo autopilot · ${sim.ap.status}` : paused ? '<b>Paused</b>' :
    src === 'manual' ? '<b>MANUAL</b> · W/S throttle–brake · A/D steer · Z/X blinkers · P autopilot' :
    src === 'ros' ? `<b>ROS 2</b> · ${rosStatus !== 'connected' ? 'not connected to rosbridge' : sim.ctrl.source === 'ros (no command)' ? 'waiting for /carla/ego_vehicle/vehicle_control_cmd' : 'driven by your node'}${sim.endOfRoad ? ' · end of road' : ''}` :
    `<b>AUTOPILOT · ${sim.ap.mode}</b> · ${sim.ap.status}`;
  $('help').style.display = step >= 3 && cfg.view.help ? 'block' : 'none';
  $('help').innerHTML = 'W / ↑   throttle      S / ↓   brake\nA D ← → steer         Space   hand brake\nZ / X   blinker L/R   C       hazard\nQ       reverse       P       autopilot on/off\nTab     camera        R       reset run\nF       sensor FOV    H       hide this help';
}
function drawPlots() {
  const c = $('plotC'), x = c.getContext('2d');
  $('plots').style.display = step >= 3 && cfg.view.plots ? 'block' : 'none';
  if (step < 3 || !cfg.view.plots) return;
  const W = 440, H = 168, h = sim.hist; x.clearRect(0, 0, W, H);
  const strips = [{ n: 'speed km/h (limit dotted)', lo: 0, hi: 150, s: [['v', '#4fd1a5', []], ['vl', '#ff8a5b', [4, 3]], ['lim', '#8a93a6', [1, 3]]] },
    { n: 'gap m (desired dashed)', lo: 0, hi: 120, s: [['gap', '#4fd1a5', []], ['dDes', '#8a93a6', [4, 3]]] },
    { n: 'lateral d m', lo: -cfg.road.nRight * cfg.road.laneWidth, hi: Math.max(cfg.road.laneWidth * 0.5, cfg.road.nLeft * cfg.road.laneWidth), s: [['d', '#7fb0ff', []]] }];
  const sh = H / 3, x0 = 34, x1 = W - 6; x.font = '11px ui-monospace, monospace';
  strips.forEach((st, i) => {
    const top = i * sh + 6, bot = (i + 1) * sh - 6;
    x.strokeStyle = '#2a3040'; x.lineWidth = 1; x.beginPath(); x.moveTo(x0, top); x.lineTo(x1, top); x.moveTo(x0, bot); x.lineTo(x1, bot); x.stroke();
    x.fillStyle = '#9aa3b2'; x.fillText(st.n, 4, top + 10);
    x.fillStyle = '#6b7383'; x.fillText(String(Math.round(st.hi)), x0 - 28, top + 22); x.fillText(String(Math.round(st.lo)), x0 - 28, bot);
    for (const [k, col, dash] of st.s) {
      x.strokeStyle = col; x.lineWidth = 1.8; x.setLineDash(dash); x.beginPath(); let pen = false;
      h.forEach((q, j) => { const v = q[k]; if (v == null) { pen = false; return; } const px = x0 + (x1 - x0) * j / 399, py = bot - (Math.min(st.hi, Math.max(st.lo, v)) - st.lo) / (st.hi - st.lo) * (bot - top); pen ? x.lineTo(px, py) : x.moveTo(px, py); pen = true; });
      x.stroke(); x.setLineDash([]);
    }
  });
}

// ---------------------------------------------------------------------
// WIZARD UI helpers
// ---------------------------------------------------------------------
const fmt = (v, d = 0) => (+v).toFixed(d);
const get = (path) => path.split('.').reduce((o, k) => o[k], cfg);
const set = (path, v) => { const ks = path.split('.'); const l = ks.pop(); ks.reduce((o, k) => o[k], cfg)[l] = v; };
const slider = (path, label, min, max, st, unit = '', dec = 0, extra = '') => { const v = get(path); return `<div class="row"><label>${label}</label><input type="range" min="${min}" max="${max}" step="${st}" value="${v}" data-bind="${path}" data-num="1" ${extra}><span class="val" data-out="${path}">${fmt(v, dec)}${unit}</span></div>`; };
const numIn = (path, label, min, max, st, unit = '') => `<div class="row"><label>${label}</label><input type="number" min="${min}" max="${max}" step="${st}" value="${get(path)}" data-bind="${path}" data-num="1" data-change="1"><span class="val" style="min-width:34px">${unit}</span></div>`;
const toggle = (path, label, extra = '') => `<div class="row"><label>${label}</label><label class="switch"><input type="checkbox" data-bind="${path}" ${get(path) ? 'checked' : ''} ${extra}><span></span></label></div>`;
const seg = (path, opts, extra = '') => { const v = get(path); return `<div class="seg" ${extra}>${opts.map(([k, l]) => `<button data-seg="${path}" data-v="${k}" class="${String(v) === String(k) ? 'on' : ''}">${l}</button>`).join('')}</div>`; };
const stepper = (path, label, min, max) => `<div class="row"><label>${label}</label><span class="stepper"><button data-stp="${path}" data-d="-1" data-min="${min}" data-max="${max}">−</button><span>${get(path)}</span><button data-stp="${path}" data-d="1" data-min="${min}" data-max="${max}">+</button></span></div>`;
const select = (path, opts, extra = '') => `<select data-bind="${path}" ${extra}>${opts.map(([k, l]) => `<option value="${k}" ${String(get(path)) === String(k) ? 'selected' : ''}>${l}</option>`).join('')}</select>`;

function renderSteps() {
  $('steps').innerHTML = STEPS.map((s, i) => `<button class="step ${i === step ? 'on' : ''} ${i < step ? 'done' : ''}" data-go="${i}"><span class="n">${i < step ? '✓' : i + 1}</span><span class="lbl">${s}</span></button>`).join('');
  $('back').style.visibility = step === 0 ? 'hidden' : 'visible';
  $('next').textContent = step === 4 ? 'Export world' : 'Next →';
  $('styleSel').value = cfg.env.style;
}
function goStep(i) {
  step = Math.max(0, Math.min(4, i)); paused = false;
  if (step === 4 || step === 3) resetSim();
  setCam(step <= 1 ? 'showroom' : step === 2 ? 'overview' : 'chase');
  buildSensorViz(); renderSteps(); renderStep();
}
function ovEst(b, vl = 70 / 3.6, vp = 90 / 3.6) {
  const D = (6 + 1.6 * vl) + 12 + b.L + 25, dv = vp - vl, t1 = dv / b.aMax, x1 = 0.5 * b.aMax * t1 * t1;
  const t = D <= x1 ? Math.sqrt(2 * D / b.aMax) : t1 + (D - x1) / dv; return { t: t + b.tlc, d: vl * (t + b.tlc) + D };
}

// plan view + elevation profile of the current road (SVG)
function roadSvg() {
  const W = 330, H = 150, pad = 8;
  let a = 1e9, b = -1e9, c = 1e9, d = -1e9;
  for (let i = 0; i <= road.N; i += 10) { a = Math.min(a, road.X[i]); b = Math.max(b, road.X[i]); c = Math.min(c, road.Y[i]); d = Math.max(d, road.Y[i]); }
  const sc = Math.min((W - 2 * pad) / (b - a || 1), (H - 2 * pad) / (d - c || 1)), ox = (W - (b - a) * sc) / 2, oy = (H - (d - c) * sc) / 2;
  const X = (x) => (ox + (x - a) * sc).toFixed(1), Yv = (y) => (H - oy - (y - c) * sc).toFixed(1);
  const path = (s0, s1) => { let p = ''; const n = Math.max(2, Math.ceil((s1 - s0) / 20)); for (let k = 0; k <= n; k++) { const q = road.pose(s0 + (s1 - s0) * k / n); p += (k ? 'L' : 'M') + X(q[0]) + ' ' + Yv(q[1]); } return p; };
  let out = `<path d="${path(0, road.length)}" fill="none" stroke="#3c4658" stroke-width="5" stroke-linejoin="round" stroke-linecap="round"/>`;
  const si = road.segInfo[selSeg]; if (si) out += `<path d="${path(si.s0, si.s1)}" fill="none" stroke="#4f8cff" stroke-width="5" stroke-linecap="round"/>`;
  for (const f of road.features) { const q = road.pose((f.s0 + f.s1) / 2); out += f.type === 'bridge' ? `<path d="${path(f.s0, f.s1)}" fill="none" stroke="#e3a33b" stroke-width="2"/>` : `<circle cx="${X(q[0])}" cy="${Yv(q[1])}" r="3.5" fill="#c084fc"/>`; }
  const st = road.pose(cfg.ego.s), en = road.pose(road.length);
  out += `<circle cx="${X(st[0])}" cy="${Yv(st[1])}" r="4.5" fill="#3fb950"/>`;
  if (!road.closed) out += `<rect x="${(+X(en[0]) - 3).toFixed(1)}" y="${(+Yv(en[1]) - 3).toFixed(1)}" width="6" height="6" fill="#f85149"/>`;
  // elevation
  const EH = 70, zmin = Math.min(...road.Z), zmax = Math.max(...road.Z, zmin + 5), ex = (s) => (pad + (W - 2 * pad) * s / road.length).toFixed(1), ey = (z) => (EH - 8 - (EH - 20) * (z - zmin) / (zmax - zmin)).toFixed(1);
  let ep = ''; for (let k = 0; k <= 200; k++) { const s = road.length * k / 200; ep += (k ? 'L' : 'M') + ex(s) + ' ' + ey(road.zAt(s)); }
  let zones = ''; if (cfg.road.nLeft) for (let j = 0; j < road.solidArr.length; j++) if (road.solidArr[j]) zones += `<rect x="${ex(j * road.solidStep)}" y="${EH - 6}" width="${Math.max(0.6, (W - 2 * pad) * road.solidStep / road.length).toFixed(2)}" height="4" fill="#f85149" opacity=".7"/>`;
  const selBand = si ? `<rect x="${ex(si.s0)}" y="2" width="${(+ex(si.s1) - +ex(si.s0)).toFixed(1)}" height="${EH - 10}" fill="#4f8cff" opacity=".15"/>` : '';
  return `<svg viewBox="0 0 ${W} ${H}" class="map" style="height:${H}px">${out}</svg>
  <svg viewBox="0 0 ${W} ${EH}" class="map" style="height:${EH}px;margin-top:4px">${selBand}<path d="${ep}" fill="none" stroke="#7fb0ff" stroke-width="2"/>${zones}<text x="${pad}" y="11" fill="#6b7383" font-size="10">elevation ${zmin.toFixed(0)}…${zmax.toFixed(0)} m${cfg.road.nLeft ? ' · red = no-overtaking' : ''}</text></svg>`;
}
function laneShort(c, lane) {
  if (lane < 0) return c.nLeft > 1 ? `Onc. ${-lane}` : 'Oncoming';
  if (c.nRight === 1) return 'Driving';
  if (lane === c.nRight) return 'Right'; if (lane === 1) return 'Left'; return 'Middle';
}
function segSummary(g) {
  if (g.type === 'straight') return `${g.length} m`;
  if (g.type === 'hairpin') return `R ${g.radius} m · ${g.dir}`;
  if (g.type === 'tighten') return `R ${g.radius}→${g.radius2} m · ${g.angle}° ${g.dir}`;
  return `R ${g.radius} m · ${g.angle}° ${g.type === 'scurve' ? g.dir + ' then opposite' : g.dir}`;
}
function segEditor(i) {
  const g = cfg.road.segments[i], p = `road.segments.${i}`;
  const dirSeg = seg(`${p}.dir`, [['left', 'Left'], ['right', 'Right']]);
  let f = '';
  if (g.type === 'straight') f += numIn(`${p}.length`, 'Length', 10, 10000, 10, 'm');
  else {
    f += numIn(`${p}.radius`, g.type === 'tighten' ? 'Entry radius' : 'Radius', 20, 10000, 10, 'm');
    if (g.type === 'tighten') f += numIn(`${p}.radius2`, 'Exit radius', 20, 10000, 10, 'm');
    if (g.type !== 'hairpin') f += numIn(`${p}.angle`, 'Turn angle', 1, 270, 1, '°');
    if (g.type !== 'tighten') f += numIn(`${p}.transition`, 'Clothoid in/out', 0, 500, 5, 'm');
    f += `<div class="row"><label>Direction${g.type === 'scurve' ? ' (first bend)' : ''}</label></div>${dirSeg}`;
  }
  f += numIn(`${p}.dz`, 'Elevation change', -60, 60, 1, 'm');
  f += `<div class="row"><label>Feature</label>${select(`${p}.feature`, [['none', 'None'], ['bridge', 'Bridge (viaduct over a valley)'], ['overpass', 'Overpass (road crossing above)']])}</div>`;
  f += numIn(`${p}.limit`, 'Speed limit (0 = road default)', 0, 150, 10, 'km/h');
  f += toggle(`${p}.noOvertake`, 'Force no-overtaking (solid line)');
  return f;
}
// lane diagram: lanes as rows, distance relative to the ego as x. Click to add a vehicle.
function laneDiagram() {
  const c = cfg.road, lanes = []; for (let k = 1; k <= c.nLeft; k++) lanes.unshift(-k); for (let k = 1; k <= c.nRight; k++) lanes.push(k);
  const W = 336, rowH = 22, top = 16, H = top + lanes.length * rowH + 22, xmin = -200, xmax = 800, X = (ds) => 6 + (W - 12) * (ds - xmin) / (xmax - xmin);
  let s = `<rect x="0" y="${top}" width="${W}" height="${lanes.length * rowH}" fill="#2a2f3a" rx="4"/>`;
  lanes.forEach((ln, i) => {
    const y = top + i * rowH;
    if (i > 0) { const centre = lanes[i - 1] < 0 && ln > 0; s += `<line x1="0" x2="${W}" y1="${y}" y2="${y}" stroke="${centre ? '#e9e9e9' : '#8a93a6'}" stroke-width="${centre ? 1.6 : 1}" ${centre ? '' : 'stroke-dasharray="6 6"'}/>`; }
    s += `<text x="4" y="${y + 14}" fill="#6b7383" font-size="9">${ln < 0 ? '◀ ' : ''}${laneName(c, ln)}${ln > 0 ? ' ▶' : ''}</text>`;
    s += `<rect class="lanehit" data-lane="${ln}" x="0" y="${y}" width="${W}" height="${rowH}" fill="transparent" style="cursor:copy"/>`;
  });
  for (let d = -200; d <= 800; d += 100) s += `<text x="${X(d) - 8}" y="${H - 6}" fill="#6b7383" font-size="9">${d}</text><line x1="${X(d)}" x2="${X(d)}" y1="${top + lanes.length * rowH}" y2="${top + lanes.length * rowH + 3}" stroke="#6b7383"/>`;
  const box = (ds, ln, len, col, label, idx) => { const i = lanes.indexOf(ln); if (i < 0) return ''; const y = top + i * rowH + 5, w = Math.max(5, len * (W - 12) / (xmax - xmin) * 3); return `<rect x="${X(ds) - w / 2}" y="${y}" width="${w}" height="12" rx="2" fill="${col}" ${idx != null ? `data-veh="${idx}" style="cursor:pointer"` : ''}/><text x="${X(ds) + w / 2 + 2}" y="${y + 10}" fill="#cfd5df" font-size="9" pointer-events="none">${label}</text>`; };
  s += box(0, cfg.ego.lane, BLUEPRINTS[cfg.vehicle.bp].L, '#2ee6c5', 'EGO ' + cfg.ego.speed);
  cfg.traffic.vehicles.forEach((v, i) => { if (v.ds >= xmin && v.ds <= xmax) s += box(v.ds, v.lane, BLUEPRINTS[v.type].L, v.type === 'truck' ? '#c084fc' : v.lane < 0 ? '#ff8a3d' : '#4f8cff', String(v.speed), i); });
  return `<svg id="laneSvg" viewBox="0 0 ${W} ${H}" style="width:100%;height:${H}px;display:block">${s}</svg><div style="font-size:11.5px;color:var(--dim);margin-top:2px">Distance from your car in metres. Click a lane to add a vehicle there.</div>`;
}

function renderStep() {
  const body = $('stepBody'); let h = '';
  if (step === 0) {
    const b = BLUEPRINTS[cfg.vehicle.bp], lr = b.wb - b.lf, K = b.m / b.wb * (lr / b.Cf - b.lf / b.Cr) * 9.81 * 180 / Math.PI, ssf = b.track / (2 * b.hcg), o = ovEst(b);
    h = `<h2>Choose your vehicle</h2><p class="sub">Like a CARLA blueprint: body, dynamics and default sensor mounts all come from it.</p>
    <div class="cards">${EGO_TYPES.map(k => { const q = BLUEPRINTS[k]; return `<button class="card ${k === cfg.vehicle.bp ? 'on' : ''}" data-bp="${k}"><img alt="" src="${thumbs[k] || ''}"><div class="t">${q.name}</div><div class="m">${q.L.toFixed(2)} × ${q.W.toFixed(2)} × ${q.H.toFixed(2)} m · ${q.m} kg</div></button>`; }).join('')}</div>
    <div class="sec">Paint</div><div class="swatches">${EGO_PAINTS.map(c => `<button class="sw ${c === cfg.vehicle.color ? 'on' : ''}" style="background:${c}" data-color="${c}" title="${c}"></button>`).join('')}</div>
    <div class="sec">Specifications</div><table class="specs">
      <tr><td>Length × width × height</td><td>${b.L.toFixed(2)} × ${b.W.toFixed(2)} × ${b.H.toFixed(2)} m</td></tr>
      <tr><td>Wheelbase · track</td><td>${b.wb.toFixed(2)} m · ${b.track.toFixed(2)} m</td></tr>
      <tr><td>Mass · CoG height</td><td>${b.m} kg · ${b.hcg.toFixed(2)} m</td></tr>
      <tr><td>l<sub>f</sub> / l<sub>r</sub></td><td>${b.lf.toFixed(2)} / ${lr.toFixed(2)} m</td></tr>
      <tr><td>Max acceleration / braking</td><td>${b.aMax.toFixed(1)} / ${b.bMax.toFixed(1)} m/s²</td></tr>
      <tr><td>Max steering angle</td><td>${(b.steerMax * 180 / Math.PI).toFixed(0)}°</td></tr>
      <tr><td>Understeer gradient K</td><td>${K.toFixed(2)} deg/g</td></tr>
      <tr><td>Static stability factor</td><td>${ssf.toFixed(2)}</td></tr>
      <tr><td>Overtake 70 km/h truck (limit 90)</td><td>≈ ${o.t.toFixed(0)} s · ${o.d.toFixed(0)} m</td></tr></table>
    <div class="note">K = m/L·(l<sub>r</sub>/C<sub>f</sub> − l<sub>f</sub>/C<sub>r</sub>) &gt; 0 means understeer. SSF = track/(2·h<sub>CoG</sub>): SUV and van roll over at a lower lateral acceleration. Heavier vehicles need more road to overtake, and slopes slow them down (v' = a − g·sin θ).</div>`;
  } else if (step === 1) {
    h = `<h2>Sensors</h2><p class="sub">Mounts are in <b>ego_vehicle</b> (x forward, y left, z up, origin at the vehicle centre on the ground, ROS REP-103). Each sensor publishes on <code>/carla/ego_vehicle/&lt;id&gt;</code>.</p>
    <div class="sec">Preset</div><div class="seg">${Object.entries(SENSOR_PRESETS).map(([k, p]) => `<button data-preset="${k}">${p.label}</button>`).join('')}</div>
    <div class="sec">Noise level</div>${seg('noise', Object.keys(NOISE).map(k => [k, k]))}
    <div class="note" style="margin-top:8px">${NOISE[cfg.noise].label}. ${{ L0: 'Ground truth, for debugging the logic.', L1: 'White Gaussian noise with range and field-of-view limits.', L2: 'Adds a constant bias per sensor and 100 ms latency.', L3: 'Adds missed detections and clutter (false targets): you need tracking with gating.' }[cfg.noise]} Vehicles and crests hide what is behind them.</div>
    <div class="sec">Sensors</div>
    ${cfg.sensors.map((s, i) => { const L = SENSOR_LIB[s.id], sel = s.id === selSensor; return `<div class="sens ${sel ? 'sel' : ''}">
      <div class="hd" data-sel="${s.id}"><span class="dot" style="background:${L.color}"></span><span class="nm">${L.label}<small>${s.id} · ${L.kind} · ${L.rate} Hz${['radar', 'camera', 'lidar'].includes(L.kind) ? ` · ${s.range} m · ${s.fov}°` : ''}</small></span><label class="switch" onclick="event.stopPropagation()"><input type="checkbox" data-bind="sensors.${i}.enabled" ${s.enabled ? 'checked' : ''}><span></span></label></div>
      ${sel ? `<div class="bd"><div class="note" style="margin:6px 0">${L.help}</div>
        ${slider(`sensors.${i}.x`, 'x (forward)', -3, 3, 0.05, ' m', 2)}${slider(`sensors.${i}.y`, 'y (left)', -1.2, 1.2, 0.05, ' m', 2)}${slider(`sensors.${i}.z`, 'z (up)', 0.2, 3, 0.05, ' m', 2)}
        ${['radar', 'camera'].includes(L.kind) ? slider(`sensors.${i}.yaw`, 'yaw', -180, 180, 1, '°') + slider(`sensors.${i}.range`, 'range', 10, 300, 5, ' m') + slider(`sensors.${i}.fov`, 'horizontal FOV', 5, 150, 1, '°') : ''}
        ${L.kind === 'lidar' ? slider(`sensors.${i}.range`, 'range', 20, 200, 5, ' m') : ''}
        ${['radar', 'camera', 'lidar'].includes(L.kind) ? slider(`sensors.${i}.sr`, 'σ range', 0, 2, 0.01, ' m', 2) + slider(`sensors.${i}.sa`, 'σ azimuth', 0, 3, 0.05, '°', 2) + slider(`sensors.${i}.sv`, 'σ range-rate', 0, 2, 0.05, ' m/s', 2) : ''}
        ${L.kind === 'gnss' ? slider(`sensors.${i}.sxy`, 'σ position', 0, 5, 0.1, ' m', 1) : ''}
        ${L.kind === 'imu' ? slider(`sensors.${i}.sacc`, 'σ acceleration', 0, 0.5, 0.01, ' m/s²', 2) + slider(`sensors.${i}.sgyr`, 'σ yaw rate', 0, 0.05, 0.001, ' rad/s', 3) : ''}
        <div class="row"><span style="flex:1"></span><button class="btn sm" data-act="resetSensor" data-id="${s.id}">Reset to default mount</button></div></div>` : ''}</div>`; }).join('')}`;
  } else if (step === 2) {
    const c = cfg.road;
    h = `<h2>World &amp; road</h2><p class="sub">Choose how the world looks, then a preset road or build your own segment by segment. The road is exported as OpenDRIVE.</p>
    <div class="sec">World style</div><div class="cards" style="grid-template-columns:1fr 1fr 1fr">${Object.entries(STYLES).map(([k, s]) => `<button class="card ${k === cfg.env.style ? 'on' : ''}" data-style="${k}"><div class="t" style="margin:0">${s.label}</div><div class="m">${s.desc}</div></button>`).join('')}</div>
    <div class="row" style="margin-top:10px">${seg('env.time', [['day', 'Day'], ['sunset', 'Sunset'], ['night', 'Night']], 'style="flex:1"')}</div>${toggle('env.fog', 'Fog')}
    <div class="sec">Road preset</div><div class="cards" style="grid-template-columns:1fr">${Object.entries(PRESETS).map(([k, p]) => `<button class="card ${k === c.preset ? 'on' : ''}" data-preset-road="${k}"><div class="t" style="margin:0">${p.name}</div><div class="m">${p.desc}</div></button>`).join('')}
      <div class="card ${c.preset === 'custom' ? 'on' : ''}" style="cursor:default"><div class="t" style="margin:0">Custom road</div><div class="m">Edit any segment below (it becomes custom), or <button class="btn sm" data-act="newRoad">start from scratch</button></div></div></div>
    <div class="sec">Map</div>${roadSvg()}
    <div class="note">Length ${(road.length / 1000).toFixed(2)} km · ${road.closed ? 'closed loop' : 'open road (start ● end ■)'} · ${road.features.filter(f => f.type === 'bridge').length} bridges · ${road.features.filter(f => f.type === 'overpass').length} overpasses${c.nLeft ? ` · ${Math.round(100 * road.solidArr.reduce((a, b) => a + b, 0) / road.solidArr.length)}% no-overtaking` : ''}</div>
    <div class="sec">Lanes &amp; rules</div>
    ${stepper('road.nRight', 'Lanes in your direction', 1, 4)}${stepper('road.nLeft', 'Oncoming lanes (0 = divided motorway)', 0, 3)}
    ${slider('road.laneWidth', 'Lane width', 2.75, 3.75, 0.05, ' m', 2)}
    <div class="row"><label>Default speed limit</label></div>${seg('road.speedLimit', [[50, '50'], [70, '70'], [90, '90'], [110, '110'], [130, '130 km/h']])}
    <div class="row"><label>Centre line ${c.nLeft ? '' : '<small style="color:var(--dim)">(two-way roads only)</small>'}</label></div>${seg('road.centerLine', [['auto', 'Solid where sight is short'], ['dashed', 'Always dashed'], ['solid', 'Always solid']], c.nLeft ? '' : 'style="opacity:.4;pointer-events:none"')}
    ${toggle('road.wet', 'Wet road (μ 0.5 instead of 0.9)')}${toggle('road.rails', 'Guardrails / median barrier')}${cfg.env.style === 'realistic' ? toggle('road.scenery', 'Trees') : ''}
    <div class="sec">Segments (${c.segments.length}) — click one to edit</div>
    ${c.segments.map((g, i) => `<div class="sens ${i === selSeg ? 'sel' : ''}"><div class="hd" data-segsel="${i}"><span class="dot" style="background:${i === selSeg ? '#4f8cff' : '#3c4658'}"></span><span class="nm">${i + 1}. ${SEG_TYPES[g.type].label}${g.feature !== 'none' ? ` · <span style="color:#e3a33b">${g.feature}</span>` : ''}<small>${segSummary(g)}${g.dz ? ` · ${g.dz > 0 ? '+' : ''}${g.dz} m` : ''}${g.limit ? ` · ${g.limit} km/h` : ''}</small></span>
      <button class="btn sm" data-segmv="${i}" data-d="-1" title="Move up">↑</button><button class="btn sm" data-segmv="${i}" data-d="1" title="Move down">↓</button><button class="btn sm" data-segdel="${i}" title="Delete">✕</button></div>
      ${i === selSeg ? `<div class="bd">${segEditor(i)}</div>` : ''}</div>`).join('')}
    <div class="row" style="flex-wrap:wrap;gap:6px">${Object.entries(SEG_TYPES).map(([k, t]) => `<button class="btn sm" data-segadd="${k}">+ ${t.label}</button>`).join('')}</div>
    <div class="note">Curves use clothoid transitions (curvature grows linearly) like real roads. Elevation changes are smooth (cubic): an up-segment followed by a down-segment makes a <b>crest</b> that hides oncoming cars, so the centre line becomes solid there. Bridges cross a valley, overpasses cross your road.</div>`;
  } else if (step === 3) {
    const lanes = laneList(cfg.road), lk = (l) => lanes.map(([k, n]) => [k, n]);
    h = `<h2>Traffic &amp; start</h2><p class="sub">Place vehicles relative to your car: lane, distance (+ ahead, − behind) and speed. The preview runs live with the demo autopilot.</p>
    <div class="sec">Your car</div>
    <div class="row"><label>Start lane</label>${select('ego.lane', lk().filter(([k]) => k > 0))}</div>
    ${slider('ego.s', 'Start position on the road', 0, Math.max(0, Math.floor(road.length - 300)), 10, ' m')}
    ${slider('ego.speed', 'Start speed', 0, 140, 5, ' km/h')}
    <div class="sec">Quick scenarios</div><div class="row" style="flex-wrap:wrap;gap:6px;margin:0">${Object.entries(TRAFFIC_SCENARIOS).map(([k, s]) => `<button class="btn sm" data-scen="${k}">${s.label}</button>`).join('')}</div>
    <div class="sec">Lane diagram</div>${laneDiagram()}
    <div class="sec">Placed vehicles (${cfg.traffic.vehicles.length})</div>
    <div class="vtable"><div class="vrow vhead"><span>Type</span><span>Lane</span><span>Δs m</span><span>km/h</span><span>Model</span><span></span></div>${cfg.traffic.vehicles.map((v, i) => `<div class="vrow">
      <select data-bind="traffic.vehicles.${i}.type">${Object.entries(TRAFFIC_TYPES).map(([k, n]) => `<option value="${k}" ${k === v.type ? 'selected' : ''}>${n}</option>`).join('')}</select>
      <select data-bind="traffic.vehicles.${i}.lane" data-num="1" title="${laneName(cfg.road, v.lane)}">${lanes.map(([k, n]) => `<option value="${k}" ${k === v.lane ? 'selected' : ''}>${laneShort(cfg.road, k)}</option>`).join('')}${lanes.some(([k]) => k === v.lane) ? '' : `<option selected value="${v.lane}">(lane not on this road)</option>`}</select>
      <input type="number" step="10" value="${v.ds}" title="distance from your car in m (+ ahead, − behind)" data-bind="traffic.vehicles.${i}.ds" data-num="1" data-change="1">
      <input type="number" step="5" min="0" max="160" value="${v.speed}" title="speed km/h" data-bind="traffic.vehicles.${i}.speed" data-num="1" data-change="1">
      <select data-bind="traffic.vehicles.${i}.behaviour" title="IDM = keeps distance from the car ahead"><option value="idm" ${v.behaviour !== 'constant' ? 'selected' : ''}>IDM</option><option value="constant" ${v.behaviour === 'constant' ? 'selected' : ''}>Const.</option></select>
      <button class="btn sm" data-vdel="${i}" title="Remove">✕</button></div>`).join('') || '<div class="note" style="margin:0">No placed vehicles. Click the diagram or use a quick scenario.</div>'}</div>
    <div class="row"><span style="flex:1"></span><button class="btn sm" data-act="addVeh">+ Add vehicle ahead</button></div>
    <div class="sec">Background traffic (random, repeatable with the seed)</div>
    ${slider('traffic.nSame', 'Vehicles in your direction', 0, 60, 1)}${slider('traffic.nOncoming', 'Oncoming vehicles', 0, 40, 1, '', 0, cfg.road.nLeft ? '' : 'disabled')}
    ${slider('traffic.vMin', 'Speed from', 30, 140, 5, ' km/h')}${slider('traffic.vMax', 'to', 30, 150, 5, ' km/h')}
    <div class="sec">Mix</div>${Object.entries(TRAFFIC_TYPES).map(([k, n]) => slider('traffic.mix.' + k, n, 0, 100, 5)).join('')}
    ${toggle('traffic.laneChanges', 'Traffic changes lane with blinkers (multi-lane roads)')}
    <div class="row"><label>Random seed</label><input type="number" data-bind="traffic.seed" data-num="1" data-change="1" value="${cfg.traffic.seed}"><button class="btn sm" data-act="reroll">New seed</button></div>
    <div class="note">IDM: a = a<sub>max</sub>[1 − (v/v<sub>0</sub>)<sup>4</sup> − (s*/s)<sup>2</sup>], s* = s<sub>0</sub> + vT + vΔv/(2√(a<sub>max</sub>b)). On an open road, vehicles reaching the end reappear at the start.</div>`;
  } else {
    const d = cfg.drive, m = d.mode;
    h = `<h2>Drive &amp; test</h2><p class="sub">Who drives the car: you, the built-in demo autopilot, or <b>your ROS 2 node</b>.</p>
    ${seg('drive.mode', [['manual', 'Manual'], ['auto', 'Autopilot (demo)'], ['ros', 'ROS 2 node']])}
    <div class="row" style="margin-top:10px"><button class="btn" data-act="pause">${paused ? '▶ Resume' : '❚❚ Pause'}</button><button class="btn" data-act="reset">↺ Reset run</button><span style="flex:1"></span><button class="btn ${sim.recording ? 'on' : ''}" data-act="rec">${sim.recording ? '■ Stop recording' : '● Record CSV'}</button></div>
    ${m === 'manual' ? `<div class="note">Click the 3D view first. <kbd>W</kbd>/<kbd>S</kbd> throttle and brake, <kbd>A</kbd>/<kbd>D</kbd> steer, <kbd>Z</kbd>/<kbd>X</kbd> left/right blinker, <kbd>C</kbd> hazard, <kbd>Space</kbd> hand brake, <kbd>Q</kbd> reverse, <kbd>P</kbd> autopilot, <kbd>Tab</kbd> camera. A gamepad also works.</div>` : ''}
    ${m === 'auto' ? `<div class="sec">Demo autopilot</div>${slider('drive.setSpeed', 'Set speed', 30, 140, 5, ' km/h')}${slider('drive.timeGap', 'ACC time gap h', 0.8, 3, 0.1, ' s', 1)}
      ${toggle('drive.allowOvertake', 'Overtake slower vehicles')}${toggle('drive.allowOncoming', 'Use the oncoming lane to overtake')}
      <div class="row"><label>Sight-distance policy</label></div>${seg('drive.policy', [['normal', 'Normal (detected cars)'], ['conservative', 'Conservative (full sight)']])}
      <div class="note">ACC: d<sub>des</sub> = d<sub>0</sub> + h·v. Overtaking: FOLLOW → blinker → CHANGE_OUT → PASS → blinker → CHANGE_BACK (or ABORT). Allowed only with a dashed line, a free target lane and enough distance from oncoming traffic: D &gt; T<sub>overtake</sub>·(v<sub>ego</sub> + v<sub>oncoming</sub>) + margin. Steering: pure pursuit δ = atan(2L sin α / L<sub>d</sub>).</div>` : ''}
    <div class="sec">ROS 2 link (rosbridge)</div>
    <div class="row"><input type="text" data-bind="ros.url" value="${cfg.ros.url}" style="flex:1;width:auto"><button class="btn sm" data-act="rosConnect">${ros && ros.bridge.wanted ? 'Reconnect' : 'Connect'}</button>${ros && ros.bridge.wanted ? '<button class="btn sm" data-act="rosClose">Disconnect</button>' : ''}</div>
    <div class="note" style="margin-top:4px">Status: <b id="rosState">${rosStatus}</b>. ${SERVED ? 'Served by the ROS 2 launch file: worlds load automatically.' : LOCAL ? 'Start the Docker container (or <code>ros2 launch webdrivesim_bringup sim.launch.py</code>) so rosbridge listens on port 9090.' : 'Inside the Claude preview the browser blocks connections to your computer. Open the page from the Docker container: <b>http://localhost:8080</b>.'}</div>
    ${toggle('ros.sync', 'Synchronous mode (CARLA-like: 1 frame of 0.05 s per received command)')}${toggle('ros.camera', 'Publish camera images (10 Hz JPEG)')}${toggle('ros.auto', 'Connect automatically on start')}
    <details class="topics"><summary>Topics (CARLA ROS bridge style, role name ego_vehicle)</summary>
      <div class="sec" style="margin-top:8px">Published by the simulator</div>${TOPICS.pub.map(([t, ty, d]) => `<div class="tp"><code>${t}</code><span>${ty}</span><small>${d}</small></div>`).join('')}
      <div class="sec">Subscribed (your node publishes)</div>${TOPICS.sub.map(([t, ty, d]) => `<div class="tp"><code>${t}</code><span>${ty}</span><small>${d}</small></div>`).join('')}</details>
    <div class="sec">Display</div>
    <div class="cards" style="grid-template-columns:1fr 1fr;gap:0 14px">${[['fov', 'Sensor FOV'], ['det', 'Detections'], ['lidar', 'Lidar points'], ['tracks', 'Demo tracks'], ['path', 'Planned path'], ['pip', 'Camera feed'], ['plots', 'Live plots'], ['help', 'Key help']].map(([k, l]) => toggle('view.' + k, l)).join('')}</div>
    <div class="sec">Metrics (ground truth)</div><div class="metrics" id="metricsBox"></div>
    <div class="sec">Recording</div><div class="row" style="margin:0"><span id="recInfo" style="flex:1;color:var(--muted);font-size:13px"></span><button class="btn sm" data-act="csv">Download CSV</button><button class="btn sm" data-act="copycsv">Copy</button></div>
    <div class="sec">Events</div><div class="events" id="eventsBox"></div>`;
  }
  body.innerHTML = h;
  if (step === 4) liveStep5();
}
function liveStep5() {
  const m = sim.metrics, mb = $('metricsBox'); if (!mb) return;
  const f = (v, d = 1, u = '') => isFinite(v) ? v.toFixed(d) + u : '–';
  mb.innerHTML = [['Distance', f(m.dist / 1000, 2, ' km')], ['Overtakes / aborts', m.overtakes + ' / ' + m.aborts], ['Collisions', m.collisions], ['Solid-line crossings', m.solidCross],
    ['Min TTC', f(m.minTTC, 1, ' s')], ['Min time headway', f(m.minTHW, 2, ' s')], ['Max lateral acc.', f(m.maxLat, 2, ' m/s²')], ['Lane RMS error', f(Math.sqrt(m.laneErrSq / Math.max(1, m.laneErrN)), 2, ' m')],
    ['Off-road time', f(m.offRoad, 1, ' s')], ['Speeding time', f(m.speeding, 1, ' s')]]
    .map(([l, v]) => `<div class="metric"><div class="l">${l}</div><div class="v">${v}</div></div>`).join('');
  $('eventsBox').textContent = sim.events.slice(-30).reverse().map(([t, s]) => `${t.toFixed(1).padStart(7)} s  ${s}`).join('\n') || 'No events yet.';
  $('recInfo').textContent = sim.recording ? `Recording… ${sim.log.length} rows (20 Hz)` : `${sim.log.length} rows recorded`;
}

// what to rebuild after a change
let roadTimer = null, trafficTimer = null;
function markCustom() { if (cfg.road.preset !== 'custom') { cfg.road.preset = 'custom'; } }
function onChange(path, rerender = false) {
  saveCfg();
  if (path.startsWith('vehicle.bp')) { const ids = cfg.sensors.filter(s => s.enabled).map(s => s.id); cfg.sensors = makeSensors(cfg.vehicle.bp, ids); rebuildEgo(); resetSim(); setCam(camMode); renderStep(); }
  else if (path.startsWith('vehicle.color')) { rebuildEgo(); thumbs = makeThumbs(); renderStep(); }
  else if (path.startsWith('sensors')) { buildSensorViz(); if (path.endsWith('enabled')) renderStep(); }
  else if (path.startsWith('road')) {
    if (path.startsWith('road.segments')) markCustom();
    clearTimeout(roadTimer);
    roadTimer = setTimeout(() => { rebuildRoad(); world.setTime(cfg.env.style, cfg.env.time, cfg.env.fog); resetSim(); if (camMode === 'overview') setCam('overview'); if (rerender || !path.includes('segments.')) renderStep(); else { const m = document.querySelector('#stepBody .map'); if (m) { const tmp = document.createElement('div'); tmp.innerHTML = roadSvg(); const svgs = document.querySelectorAll('#stepBody svg.map'); svgs[0].replaceWith(tmp.children[0]); svgs[1].replaceWith(tmp.children[0]); } } }, 250);
  } else if (path.startsWith('env.style')) { world.style = cfg.env.style; world.setTime(cfg.env.style, cfg.env.time, cfg.env.fog); rebuildRoad(); rebuildEgo(); resetSim(); setLights(); renderStep(); }
  else if (path.startsWith('env')) { world.setTime(cfg.env.style, cfg.env.time, cfg.env.fog); setLights(); }
  else if (path.startsWith('traffic') || path.startsWith('ego')) { clearTimeout(trafficTimer); trafficTimer = setTimeout(() => { resetSim(); if (rerender) renderStep(); else { const l = $('laneSvg'); if (l) { const tmp = document.createElement('div'); tmp.innerHTML = laneDiagram(); l.parentNode.replaceChild(tmp.children[0], l); } } }, 200); }
  else if (path === 'noise') renderStep();
  else if (path === 'drive.mode') { renderStep(); toast({ auto: 'Autopilot ON', manual: 'Manual: click the 3D view, then W A S D', ros: 'Driven by your ROS 2 node' }[cfg.drive.mode]); $('c3d').focus(); if (cfg.drive.mode === 'ros' && !ros) connectRos(); }
}

function bindStatic() {
  $('steps').addEventListener('click', (ev) => { const b = ev.target.closest('[data-go]'); if (b) goStep(+b.dataset.go); });
  $('back').onclick = () => goStep(step - 1);
  $('next').onclick = () => step === 4 ? openExport() : goStep(step + 1);
  $('quality').onchange = () => world.setQuality($('quality').value);
  $('styleSel').onchange = () => { cfg.env.style = $('styleSel').value; onChange('env.style'); };
  $('btnExport').onclick = openExport;
  $('btnLoad').onclick = () => $('fileIn').click();
  $('fileIn').onchange = async (ev) => {
    const f = ev.target.files[0]; if (!f) return;
    try { loadScenario(JSON.parse(await f.text()), f.name.replace(/\.json$/, '')); toast('Scenario loaded: ' + f.name); } catch (e) { toast('Could not load: ' + e.message); }
    ev.target.value = '';
  };
  const body = $('stepBody');
  const onInput = (ev) => {
    const el = ev.target, p = el.dataset.bind; if (!p) return;
    if (el.dataset.change && ev.type === 'input') return;          // number fields apply on change/enter
    let v = el.type === 'checkbox' ? el.checked : el.dataset.num ? +el.value : el.value;
    if (el.dataset.num && !isFinite(v)) return;
    set(p, v);
    const out = body.querySelector(`[data-out="${p}"]`);
    if (out) { const unit = out.textContent.replace(/^[-\d.]+/, ''); const dec = (el.step.split('.')[1] || '').length; out.textContent = fmt(v, dec) + unit; }
    if (p === 'traffic.vMin' && cfg.traffic.vMax < v) cfg.traffic.vMax = v;
    if (p === 'traffic.vMax' && cfg.traffic.vMin > v) cfg.traffic.vMin = v;
    if (p === 'ros.url') return saveCfg();
    onChange(p, el.tagName === 'SELECT' && p.startsWith('traffic.vehicles') ? false : el.dataset.change && p.startsWith('road.segments'));
  };
  body.addEventListener('input', onInput); body.addEventListener('change', onInput);
  body.addEventListener('click', (ev) => {
    const lh = ev.target.closest('.lanehit');
    if (lh) {
      const svg = $('laneSvg'), r = svg.getBoundingClientRect(), xv = (ev.clientX - r.left) / r.width * 336;
      const ds = Math.round((-200 + (xv - 6) / (336 - 12) * 1000) / 10) * 10, lane = +lh.dataset.lane;
      cfg.traffic.vehicles.push({ type: 'sedan', lane, ds, speed: lane > 0 ? 80 : 85, behaviour: 'idm' });
      saveCfg(); resetSim(); renderStep(); return;
    }
    const t = ev.target.closest('button,[data-sel],[data-segsel],[data-veh]'); if (!t) return;
    const ds = t.dataset;
    if (ds.bp) { cfg.vehicle.bp = ds.bp; onChange('vehicle.bp'); }
    else if (ds.color) { cfg.vehicle.color = ds.color; onChange('vehicle.color'); }
    else if (ds.style) { cfg.env.style = ds.style; $('styleSel').value = ds.style; onChange('env.style'); }
    else if (ds.seg) { const cur = get(ds.seg); set(ds.seg, typeof cur === 'number' ? +ds.v : ds.v); renderStep(); onChange(ds.seg, true); }
    else if (ds.stp) { const v = Math.max(+ds.min, Math.min(+ds.max, get(ds.stp) + +ds.d)); set(ds.stp, v); renderStep(); onChange(ds.stp, true); }
    else if (ds.presetRoad) { const keep = { wet: cfg.road.wet, rails: cfg.road.rails, scenery: cfg.road.scenery }; cfg.road = { ...presetRoad(ds.presetRoad), ...keep }; cfg.ego.lane = cfg.road.nRight; selSeg = 0; cfg.name = ds.presetRoad; worldName = cfg.name; onChange('road.preset', true); }
    else if (ds.act === 'newRoad') { cfg.road = { ...cfg.road, preset: 'custom', closed: false, segments: [{ ...newSegment('straight'), length: 800 }, newSegment('curve'), { ...newSegment('straight'), length: 800 }] }; selSeg = 0; cfg.name = 'custom_road'; worldName = cfg.name; onChange('road.segments', true); }
    else if (ds.segsel != null) { selSeg = +ds.segsel; renderStep(); }
    else if (ds.segadd) { cfg.road.segments.splice(selSeg + 1, 0, newSegment(ds.segadd)); selSeg = Math.min(selSeg + 1, cfg.road.segments.length - 1); cfg.road.closed = false; onChange('road.segments', true); }
    else if (ds.segdel != null) { if (cfg.road.segments.length > 1) { cfg.road.segments.splice(+ds.segdel, 1); selSeg = Math.max(0, Math.min(selSeg, cfg.road.segments.length - 1)); cfg.road.closed = false; onChange('road.segments', true); } }
    else if (ds.segmv != null) { const i = +ds.segmv, j = i + +ds.d, S = cfg.road.segments; if (j >= 0 && j < S.length) { [S[i], S[j]] = [S[j], S[i]]; selSeg = j; cfg.road.closed = false; onChange('road.segments', true); } }
    else if (ds.sel) { selSensor = ds.sel; buildSensorViz(); renderStep(); }
    else if (ds.preset) { const ids = SENSOR_PRESETS[ds.preset].ids; cfg.sensors.forEach(s => s.enabled = ids.includes(s.id)); buildSensorViz(); renderStep(); saveCfg(); toast('Preset: ' + SENSOR_PRESETS[ds.preset].label); }
    else if (ds.act === 'resetSensor') { const i = cfg.sensors.findIndex(s => s.id === ds.id); cfg.sensors[i] = { ...cfg.sensors[i], ...SENSOR_LIB[ds.id].def(BLUEPRINTS[cfg.vehicle.bp]) }; buildSensorViz(); renderStep(); saveCfg(); }
    else if (ds.scen) { const sc = TRAFFIC_SCENARIOS[ds.scen]; cfg.traffic.vehicles = resolveScenario(ds.scen, cfg.road, cfg.ego.lane); cfg.traffic.nSame = sc.nSame; cfg.traffic.nOncoming = cfg.road.nLeft ? sc.nOncoming : 0; saveCfg(); resetSim(); renderStep(); toast(sc.label); }
    else if (ds.vdel != null) { cfg.traffic.vehicles.splice(+ds.vdel, 1); saveCfg(); resetSim(); renderStep(); }
    else if (ds.act === 'addVeh') { cfg.traffic.vehicles.push({ type: 'sedan', lane: cfg.ego.lane, ds: 150, speed: 70, behaviour: 'idm' }); saveCfg(); resetSim(); renderStep(); }
    else if (ds.act === 'reroll') { cfg.traffic.seed = Math.floor(Math.random() * 99999); renderStep(); onChange('traffic.seed', true); }
    else if (ds.act === 'pause') { paused = !paused; renderStep(); renderCamBar(); }
    else if (ds.act === 'reset') { resetSim(); renderStep(); toast('Run reset'); }
    else if (ds.act === 'rec') { if (!sim.recording) { sim.log = []; sim.recording = true; } else sim.recording = false; renderStep(); }
    else if (ds.act === 'csv') saveFile(`webdrivesim_run_${worldName}_${Date.now()}.csv`, toCSV(sim.log));
    else if (ds.act === 'copycsv') copy(toCSV(sim.log));
    else if (ds.act === 'rosConnect') { connectRos(); renderStep(); }
    else if (ds.act === 'rosClose') { if (ros) ros.close(); ros = null; rosStatus = 'offline'; renderRosPill(); renderStep(); }
  });
  $('mClose').onclick = () => $('modalBg').style.display = 'none';
  $('modalBg').onclick = (ev) => { if (ev.target === $('modalBg')) $('modalBg').style.display = 'none'; };
  $('mCopy').onclick = () => copy($('mText').value);
  $('dlZip').onclick = () => { const f = exportFiles(); saveFile(pkgName() + '.zip', new Blob([makeZip(f.map(x => ({ name: pkgName() + '/' + x.name, text: x.text })))])); };
  $('dlJson').onclick = () => saveFile(pkgName() + '.json', exportFiles()[0].text);
  $('c3d').addEventListener('pointerdown', () => $('c3d').focus());
  renderRosPill();
}
function renderCamBar() {
  const bar = $('camBar');
  bar.innerHTML = step >= 3 ? CAMS.map(([k, l]) => `<button class="btn sm ${camMode === k ? 'on' : ''}" data-cam="${k}">${l}</button>`).join('') + `<button class="btn sm" data-cam="__pause">${paused ? '▶' : '❚❚'}</button>` : '';
  bar.onclick = (ev) => { const b = ev.target.closest('[data-cam]'); if (!b) return; if (b.dataset.cam === '__pause') { paused = !paused; renderCamBar(); if (step === 4) renderStep(); } else setCam(b.dataset.cam); };
}

// ---------------------------------------------------------------------
// LOAD / EXPORT / SESSION
// ---------------------------------------------------------------------
function loadScenario(j, name) {
  if (!j.editor_state) throw new Error('not an WebDriveSim scenario (missing editor_state)');
  cfg = sanitize(j.editor_state); cfg.name = name || j.name || cfg.name; worldName = cfg.name; saveCfg();
  thumbs = makeThumbs(); $('styleSel').value = cfg.env.style; applyAll(); goStep(step);
}
async function loadWorldByName(name, opts) {
  try {
    const file = name.endsWith('.json') ? name : name + '.json';
    const r = await fetch('worlds/' + encodeURIComponent(file), { cache: 'no-store' });
    if (!r.ok) throw new Error(`worlds/${file}: HTTP ${r.status}`);
    loadScenario(await r.json(), file.replace(/\.json$/, ''));
    if (opts.control) cfg.drive.mode = opts.control;
    if (opts.autostart !== false) goStep(4);
    toast('World loaded: ' + file);
  } catch (e) { toast('Could not load world: ' + e.message); console.warn(e); }
}
// when served by the ROS 2 launch file, follow /session.json (world, control mode, sync)
let sessionRev = null;
async function pollSession() {
  try {
    const r = await fetch('session.json', { cache: 'no-store' });
    if (r.ok) {
      const s = await r.json();
      if (s.rev !== sessionRev) {
        sessionRev = s.rev;
        if (s.world) await loadWorldByName(s.world, { control: s.control || 'ros', autostart: s.autostart !== false });
        else if (s.control) cfg.drive.mode = s.control;
        // the launch file decides the rosbridge address and sync mode (applied after the world)
        const url = s.ros_url ? s.ros_url.replace('localhost', location.hostname) : cfg.ros.url;
        if (typeof s.sync === 'boolean') cfg.ros.sync = s.sync;
        if (!ros || !ros.bridge.wanted || cfg.ros.url !== url) { cfg.ros.url = url; connectRos(); }
        if (step === 4) renderStep();
      }
    }
  } catch (e) {}
  setTimeout(pollSession, 2000);
}
const pkgName = () => ($('pkgName').value || 'webdrivesim_world').replace(/[^\w-]+/g, '_');
function exportFiles() {
  const name = pkgName(), fresh = new Sim(cfg, road), obj = scenarioObject({ ...cfg, name }, road, fresh, name, { BLUEPRINTS, SENSOR_LIB });
  const { editor_state, ...forYaml } = obj;
  return [
    { name: name + '.json', text: JSON.stringify(obj, null, 2) },
    { name: 'world.yaml', text: '# WebDriveSim world — generated ' + obj.created + '\n' + toYAML(forYaml).replace(/^\n/, '') + '\n' },
    { name: 'road.xodr', text: toXODR(road, name) },
    { name: 'README.md', text: readme(name) },
  ];
}
let mTab = 0;
function openExport() {
  $('pkgName').value = worldName || 'my_world';
  $('modalBg').style.display = 'flex';
  const show = () => {
    const files = exportFiles();
    $('mTabs').innerHTML = files.map((f, i) => `<button class="btn sm ${i === mTab ? 'on' : ''}" data-t="${i}">${f.name}</button>`).join('');
    $('mText').value = files[mTab].text; $('mInfo').textContent = `${files[mTab].name} · ${(files[mTab].text.length / 1024).toFixed(1)} kB`;
  };
  $('mTabs').onclick = (ev) => { const b = ev.target.closest('[data-t]'); if (b) { mTab = +b.dataset.t; show(); } };
  $('pkgName').oninput = () => { worldName = pkgName(); show(); };
  show();
}
let dlApi;
async function saveFile(name, data) {
  try {
    if (dlApi === undefined) dlApi = window.claude && window.claude.use ? await Promise.race([window.claude.use('downloads'), new Promise(r => setTimeout(() => r(null), 3000))]) : null;
    if (dlApi) { await dlApi.save({ filename: name, data }); toast('Saved ' + name); return; }
  } catch (e) { if (e && e.code === 'declined') return; if (e && e.code) { toast('Download not available here: use Copy'); return; } }
  const blob = data instanceof Blob ? data : new Blob([data]);
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = name; document.body.appendChild(a); a.click(); a.remove();
}
async function copy(text) { try { await navigator.clipboard.writeText(text); toast('Copied to clipboard'); } catch (e) { toast('Copy blocked: select the text and press Ctrl+C'); } }
let toastT;
function toast(s) { const t = $('toast'); t.textContent = s; t.style.opacity = 1; clearTimeout(toastT); toastT = setTimeout(() => t.style.opacity = 0, 2400); }
function onResize() {
  const m = $('c3d'), W = m.clientWidth, H = m.clientHeight;
  world.renderer.setSize(W, H, false); camera.aspect = W / Math.max(H, 1); camera.updateProjectionMatrix();
  const pc = $('plotC'), dpr = Math.min(2, devicePixelRatio || 1);
  pc.style.width = '440px'; pc.style.height = '168px'; pc.width = 440 * dpr; pc.height = 168 * dpr; pc.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
}

try { init(); } catch (e) { console.error(e); $('loading').textContent = 'Error: ' + e.message; }
