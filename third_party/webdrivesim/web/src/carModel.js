// =====================================================================
// carModel.js — realistic procedural vehicles (no downloaded models)
//
// IDEA: a car body is built from its SIDE PROFILE (a 2D outline with
// hood, windscreen, roof, rear window, wheel arches) that is extruded to
// the vehicle width with rounded (bevelled) edges. Then:
//   - "tumblehome": the greenhouse leans inwards above the belt line
//   - plan-view rounding: corners pulled in near the bumpers
//   - glass, lights, plates, mirrors, door seams, real wheels (tyre + rim)
// Physically based materials (clear-coat paint, reflective glass) plus an
// environment map from the sky give the "real car" look.
// All parts with the same material are merged into one mesh, so a car
// costs ~9 draw calls: 40 cars stay cheap (much lighter than CARLA).
//
// Dimensions and dynamics are typical values of each vehicle class
// (not a specific brand). Frame: x forward, y up, z right (three.js).
// =====================================================================
import * as THREE from 'three';
import { mergeGeometries, mergeVertices } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export const BLUEPRINTS = {
  hatch: { name: 'Compact hatchback', type: 'hatch', L: 4.05, W: 1.76, H: 1.47, wb: 2.56, fo: 0.80, r: 0.31, clr: 0.15, belt: 0.88, hood: 0.85, wind: 0.80, roof: 1.35,
    m: 1180, lf: 1.02, Cf: 72000, Cr: 78000, aMax: 2.6, bMax: 8.5, hcg: 0.53, track: 1.52, steerMax: 0.62, tlc: 3.0, vmax: 50, color: '#7a1712' },
  sedan: { name: 'Mid-size sedan', type: 'sedan', L: 4.78, W: 1.84, H: 1.44, wb: 2.84, fo: 0.92, r: 0.33, clr: 0.14, belt: 0.90, hood: 1.20, wind: 0.82, roof: 1.30, rearWin: 0.62,
    m: 1520, lf: 1.18, Cf: 90000, Cr: 100000, aMax: 2.9, bMax: 9.0, hcg: 0.55, track: 1.58, steerMax: 0.60, tlc: 3.0, vmax: 61, color: '#1f3a5f' },
  suv: { name: 'Mid-size SUV', type: 'suv', L: 4.70, W: 1.90, H: 1.70, wb: 2.78, fo: 0.90, r: 0.37, clr: 0.21, belt: 1.06, hood: 1.05, wind: 0.72, roof: 1.85,
    m: 1950, lf: 1.30, Cf: 88000, Cr: 100000, aMax: 2.3, bMax: 8.0, hcg: 0.70, track: 1.62, steerMax: 0.58, tlc: 3.5, vmax: 55, color: '#a7adb3' },
  van: { name: 'Passenger van', type: 'van', L: 5.10, W: 1.95, H: 1.95, wb: 3.10, fo: 0.92, r: 0.34, clr: 0.17, belt: 1.08, hood: 0.70, wind: 0.95, roof: 2.80,
    m: 2450, lf: 1.50, Cf: 95000, Cr: 120000, aMax: 1.7, bMax: 7.0, hcg: 0.85, track: 1.68, steerMax: 0.55, tlc: 4.0, vmax: 45, color: '#e9ecef' },
  truck: { name: 'Box truck', type: 'truck', L: 9.5, W: 2.5, H: 3.7, wb: 5.6, fo: 1.30, r: 0.50, clr: 0.36,
    m: 11000, lf: 3.5, Cf: 200000, Cr: 300000, aMax: 1.0, bMax: 6.0, hcg: 1.4, track: 2.0, steerMax: 0.5, tlc: 5.0, vmax: 25, color: '#d9dde1' },
};
export const PAINTS = ['#e9ecef', '#1c1c1e', '#a7adb3', '#5a6068', '#1f3a5f', '#7a1712', '#2f4f3a', '#b9a07a', '#2b5c8a', '#8a8f2a'];

// ---------- shared materials (created once) ----------
export const MAT = {};
export function initMaterials() {
  MAT.glass = new THREE.MeshStandardMaterial({ color: 0x0d1418, metalness: 0.9, roughness: 0.06, side: THREE.DoubleSide });
  MAT.trim = new THREE.MeshStandardMaterial({ color: 0x141414, metalness: 0.1, roughness: 0.65 });
  MAT.chrome = new THREE.MeshStandardMaterial({ color: 0xc9cdd1, metalness: 1.0, roughness: 0.22 });
  MAT.tyre = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, metalness: 0.0, roughness: 0.92 });
  MAT.head = new THREE.MeshStandardMaterial({ color: 0xdfe7ee, metalness: 0.6, roughness: 0.08, emissive: 0xffffff, emissiveIntensity: 0.12 });
  MAT.tail = new THREE.MeshStandardMaterial({ color: 0x4a0000, metalness: 0.3, roughness: 0.2, emissive: 0xff1a1a, emissiveIntensity: 0.25 });
  MAT.plate = new THREE.MeshStandardMaterial({ map: plateTexture('WDS 001'), roughness: 0.4 });
  MAT.shadow = new THREE.MeshBasicMaterial({ map: shadowTexture(), transparent: true, depthWrite: false, opacity: 0.85 });
  MAT.box = new THREE.MeshStandardMaterial({ color: 0xe4e6e8, metalness: 0.05, roughness: 0.55 });
}
const paintCache = {};
export function paint(hex) {
  if (!paintCache[hex]) paintCache[hex] = new THREE.MeshPhysicalMaterial({ color: hex, metalness: 0.55, roughness: 0.33, clearcoat: 1.0, clearcoatRoughness: 0.07 });
  return paintCache[hex];
}
function plateTexture(text) {
  const c = document.createElement('canvas'); c.width = 256; c.height = 56;
  const x = c.getContext('2d');
  x.fillStyle = '#f4f4f0'; x.fillRect(0, 0, 256, 56);
  x.fillStyle = '#1d4fa3'; x.fillRect(0, 0, 26, 56);
  x.fillStyle = '#111'; x.font = 'bold 34px monospace'; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(text, 141, 30);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}
function shadowTexture() {
  const c = document.createElement('canvas'); c.width = 128; c.height = 128;
  const x = c.getContext('2d');
  const g = x.createRadialGradient(64, 64, 10, 64, 64, 64);
  g.addColorStop(0, 'rgba(0,0,0,0.75)'); g.addColorStop(0.6, 'rgba(0,0,0,0.45)'); g.addColorStop(1, 'rgba(0,0,0,0)');
  x.fillStyle = g; x.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

// ---------- 2D helpers ----------
// Chaikin corner cutting: each pass replaces every corner by two points -> smooth outline
function chaikin(P, it = 2) {
  for (let k = 0; k < it; k++) {
    const Q = [];
    for (let i = 0; i < P.length; i++) {
      const a = P[i], b = P[(i + 1) % P.length];
      Q.push([0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]], [0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]]);
    }
    P = Q;
  }
  return P;
}
// x of the outline at height y (front = max x, rear = min x)
function surfaceX(P, y, front) {
  let best = front ? -1e9 : 1e9;
  for (let i = 0; i < P.length; i++) {
    const a = P[i], b = P[(i + 1) % P.length];
    if ((a[1] - y) * (b[1] - y) > 0 || a[1] === b[1]) continue;
    const x = a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]);
    best = front ? Math.max(best, x) : Math.min(best, x);
  }
  return best;
}

// Side profile of a passenger car (counter-clockwise outline, x forward, y up)
function carProfile(b) {
  const L2 = b.L / 2, c = b.clr, Hb = b.belt, Ht = b.H;
  const xw0 = L2 - b.hood, xw1 = xw0 - b.wind, xr = xw1 - 0.12 - b.roof;
  const xf = L2 - b.fo, xrw = xf - b.wb;
  const P = [[L2 - 0.30, c + 0.02], [L2 - 0.03, c + 0.14], [L2, Hb - 0.30], [L2 - 0.02, Hb - 0.12], [L2 - 0.18, Hb - 0.01],
    [xw0, Hb + 0.07], [xw1, Ht - 0.03], [xw1 - 0.12, Ht], [xr + 0.15, Ht], [xr, Ht - 0.03]];
  let rear;
  if (b.type === 'sedan') rear = [[xr - b.rearWin, Hb + 0.14], [-L2 + 0.10, Hb + 0.10], [-L2 + 0.01, Hb - 0.02]];
  else if (b.type === 'hatch') rear = [[-L2 + 0.10, Hb + 0.17], [-L2 + 0.01, Hb - 0.04]];
  else rear = [[-L2 + 0.06, Ht - 0.18], [-L2 + 0.01, Hb - 0.05]];
  P.push(...rear, [-L2, c + 0.30], [-L2 + 0.05, c + 0.10], [-L2 + 0.30, c + 0.02]);
  pushArches(P, b, [xrw, xf], c);
  return { P: chaikin(P, 2), xw0, xw1, xr, rearPt: rear[0], xf, xrw };
}
// wheel arches cut into the bottom edge (rear wheel first, we walk towards the front)
function pushArches(P, b, xs, c) {
  const ra = b.r + 0.06;
  for (const xw of xs) {
    P.push([xw - ra, c]);
    for (let i = 0; i <= 10; i++) { const a = Math.PI - Math.PI * i / 10; P.push([xw + ra * Math.cos(a), b.r + 0.03 + ra * Math.sin(a)]); }
    P.push([xw + ra, c]);
  }
}

// "shaping" applied to every vertex: tumblehome above the belt + rounded corners in plan view
function shaper(b) {
  const L2 = b.L / 2, Hb = b.belt, Ht = b.H;
  return (x, y, z) => {
    let f = 1;
    if (y > Hb) f *= 1 - 0.13 * Math.min(1, (y - Hb) / (Ht - Hb));
    const e = Math.abs(x) - (L2 - 0.6);
    if (e > 0) f *= 1 - 0.10 * (e / 0.6) ** 2;
    return z * f;
  };
}
function shapeGeo(g, shape) {
  const p = g.attributes.position;
  for (let i = 0; i < p.count; i++) p.setZ(i, shape(p.getX(i), p.getY(i), p.getZ(i)));
  g.computeVertexNormals();
  return g;
}
function clean(g) { // keep only position/normal/uv, indexed (needed to merge)
  for (const k of Object.keys(g.attributes)) if (!['position', 'normal', 'uv'].includes(k)) g.deleteAttribute(k);
  if (!g.attributes.uv) g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(g.attributes.position.count * 2), 2));
  if (!g.index) { const idx = []; for (let i = 0; i < g.attributes.position.count; i++) idx.push(i); g.setIndex(idx); }
  g.clearGroups();
  return g;
}
// extrude a profile to width W with rounded edges, centred on z = 0
function extrude(P, W, bt = 0.07, bs = 0.05) {
  const s = new THREE.Shape(P.map(([x, y]) => new THREE.Vector2(x, y)));
  let g = new THREE.ExtrudeGeometry(s, { depth: W - 2 * bt, bevelEnabled: true, bevelThickness: bt, bevelSize: bs, bevelOffset: -bs, bevelSegments: 3, steps: 1, curveSegments: 6 });
  g.translate(0, 0, -(W - 2 * bt) / 2);
  g.deleteAttribute('uv'); g.deleteAttribute('normal');
  g = mergeVertices(g, 1e-4);
  g.computeVertexNormals();
  return g;
}
// flat polygon on the side of the car (both sides), shaped like the body
function sidePoly(poly, zAbs, shape) {
  const out = [];
  for (const sgn of [1, -1]) {
    const g = new THREE.ShapeGeometry(new THREE.Shape(poly.map(([x, y]) => new THREE.Vector2(x, y))));
    g.translate(0, 0, sgn * zAbs);
    out.push(shapeGeo(g, shape));
  }
  return out;
}
// quad from 4 points (indexed), used for windscreen / rear glass
function quad(a, b, c, d) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute([...a, ...b, ...c, ...d], 3));
  g.setIndex([0, 1, 2, 0, 2, 3]);
  g.computeVertexNormals();
  return g;
}
function box(sx, sy, sz, x, y, z, ry = 0) {
  const g = new THREE.BoxGeometry(sx, sy, sz);
  if (ry) g.rotateY(ry);
  g.translate(x, y, z);
  return g;
}

// ---------- wheels ----------
function wheelGeos(r, tw) {
  // tyre: revolve a rounded cross-section around the axle (lathe)
  const prof = [[r * 0.64, -tw / 2], [r - 0.05, -tw / 2], [r - 0.01, -tw / 2 + 0.03], [r, -tw / 2 + 0.07], [r, tw / 2 - 0.07], [r - 0.01, tw / 2 - 0.03], [r - 0.05, tw / 2], [r * 0.64, tw / 2]]
    .map(([a, b]) => new THREE.Vector2(a, b));
  const tyre = new THREE.LatheGeometry(prof, 28); tyre.rotateX(Math.PI / 2);
  const rim = new THREE.CylinderGeometry(r * 0.66, r * 0.66, tw * 0.8, 24); rim.rotateX(Math.PI / 2);
  const parts = [rim];
  for (let k = 0; k < 5; k++) { // 5 spokes on the outer face
    const sp = new THREE.BoxGeometry(0.055, r * 1.15, 0.035); sp.rotateZ(k * 2 * Math.PI / 5); sp.translate(0, 0, tw * 0.41); parts.push(sp);
  }
  const hub = new THREE.CylinderGeometry(r * 0.16, r * 0.16, 0.05, 16); hub.rotateX(Math.PI / 2); hub.translate(0, 0, tw * 0.42); parts.push(hub);
  const dark = new THREE.CylinderGeometry(r * 0.6, r * 0.6, 0.02, 24); dark.rotateX(Math.PI / 2); dark.translate(0, 0, tw * 0.405);
  return { tyre: clean(tyre), chrome: mergeGeometries(parts.map(clean)), trim: clean(dark) };
}

// ---------- assemble one vehicle into geometry lists per material ----------
function partsFor(b, withWheels) {
  const parts = { paint: [], glass: [], trim: [], chrome: [], tyre: [], head: [], tail: [], plate: [], shadow: [], box: [] };
  const L2 = b.L / 2, W = b.W, c = b.clr, Hb = b.belt;
  const tw = b.type === 'truck' ? 0.32 : 0.22;
  let wheelX;

  if (b.type === 'truck') {
    // cab (front 2.3 m) + cargo box
    const cabL = 2.3, xf = L2 - b.fo;
    const P = [[L2 - 0.12, c], [L2, c + 0.35], [L2, 2.05], [L2 - 0.10, 2.85], [L2 - 0.35, 3.05], [L2 - cabL, 3.05], [L2 - cabL, c]];
    pushArches(P, b, [xf], c);            // front wheel arch cut into the cab bottom
    const Pa = chaikin(P, 1);
    const shape = (x, y, z) => z;
    parts.paint.push(extrude(Pa, W, 0.08, 0.06));
    parts.glass.push(quad([L2 + 0.005, 2.0, -W / 2 + 0.15], [L2 + 0.005, 2.0, W / 2 - 0.15], [L2 - 0.09, 2.78, W / 2 - 0.18], [L2 - 0.09, 2.78, -W / 2 + 0.18]));
    parts.glass.push(...sidePoly([[L2 - 0.25, 1.85], [L2 - 0.3, 2.75], [L2 - 1.2, 2.8], [L2 - 1.2, 1.85]], W / 2 + 0.004, shape));
    parts.box.push(box(b.L - cabL - 0.15, 2.75, W, -cabL / 2 - 0.075, 1.0 + 2.75 / 2, 0));
    parts.trim.push(box(b.L - 0.3, 0.35, W * 0.8, 0, 0.75, 0));               // chassis
    parts.trim.push(box(0.03, 0.4, W * 0.7, L2 + 0.01, 1.15, 0));              // grille
    for (const s of [-1, 1]) {
      parts.head.push(box(0.03, 0.18, 0.32, L2 + 0.012, 0.85, s * (W / 2 - 0.3)));
      parts.tail.push(box(0.03, 0.14, 0.30, -L2 - 0.012, 1.15, s * (W / 2 - 0.25)));
    }
    parts.plate.push(box(0.02, 0.11, 0.52, L2 + 0.015, 0.55, 0), box(0.02, 0.11, 0.52, -L2 + 0.135, 0.75, 0));
    wheelX = [xf, -L2 + 1.6, -L2 + 2.95];
  } else {
    const pr = carProfile(b), P = pr.P, shape = shaper(b);
    parts.paint.push(shapeGeo(extrude(P, W), shape));
    const Ht = b.H, xw0 = pr.xw0, xw1 = pr.xw1, xr = pr.xr;
    // windscreen (offset outwards along its normal)
    const off = (p0, p1, k) => { const dx = p1[0] - p0[0], dy = p1[1] - p0[1], n = Math.hypot(dx, dy); return [dy / n * k, -dx / n * k]; };
    const wsA = [xw0 - 0.04, Hb + 0.10], wsB = [xw1 + 0.03, Ht - 0.06];
    let o = off(wsA, wsB, 0.012);
    const hz = (x, y) => shape(x, y, W / 2 - 0.09);
    parts.glass.push(quad([wsA[0] + o[0], wsA[1] + o[1], -hz(...wsA)], [wsA[0] + o[0], wsA[1] + o[1], hz(...wsA)], [wsB[0] + o[0], wsB[1] + o[1], hz(...wsB)], [wsB[0] + o[0], wsB[1] + o[1], -hz(...wsB)]));
    // rear glass
    const rp = pr.rearPt;
    const rgA = [xr - 0.03, Ht - 0.06], rgB = b.type === 'sedan' ? [rp[0] + 0.06, rp[1] + 0.03] : b.type === 'hatch' ? [xr + (rp[0] - xr) * 0.78, Ht + (rp[1] - Ht) * 0.78] : [rp[0] + 0.02, Hb + 0.45];
    o = off(rgA, rgB, 0.012);
    parts.glass.push(quad([rgA[0] + o[0], rgA[1] + o[1], -hz(...rgA)], [rgA[0] + o[0], rgA[1] + o[1], hz(...rgA)], [rgB[0] + o[0], rgB[1] + o[1], hz(...rgB)], [rgB[0] + o[0], rgB[1] + o[1], -hz(...rgB)]));
    // side windows (one polygon, B-pillar painted black on top)
    const yl = Hb + 0.10, yt = Ht - 0.075;
    const lerpX = (y) => { const t = (y - (Ht - 0.03)) / (rp[1] - (Ht - 0.03)); return xr + (rp[0] - xr) * Math.min(1, Math.max(0, t)); };
    let xC0 = b.type === 'suv' || b.type === 'van' ? -L2 + 0.22 : lerpX(yl) + 0.22;
    let xC1 = b.type === 'suv' || b.type === 'van' ? -L2 + 0.26 : xr + 0.10;
    const win = chaikin([[xw0 - 0.12, yl], [xw1 - 0.03, yt], [xC1, yt], [xC0, yl]], 1);
    parts.glass.push(...sidePoly(win, W / 2 + 0.006, shape));
    const xB = (xw0 + xC0) / 2 - (b.type === 'van' ? 0.4 : 0.05);
    parts.trim.push(...sidePoly([[xB - 0.05, yl], [xB - 0.04, yt], [xB + 0.05, yt], [xB + 0.04, yl]], W / 2 + 0.010, shape));
    if (b.type === 'suv' || b.type === 'van') { const xD = (xB + xC0) / 2 - 0.15; parts.trim.push(...sidePoly([[xD - 0.04, yl], [xD - 0.04, yt], [xD + 0.04, yt], [xD + 0.04, yl]], W / 2 + 0.010, shape)); }
    // door seams + handles
    for (const xs of [xw0 - 0.08, xB, b.type === 'sedan' || b.type === 'hatch' ? pr.xrw + b.r + 0.12 : xB - 0.95]) {
      parts.trim.push(...sidePoly([[xs - 0.004, c + 0.2], [xs - 0.004, Hb + 0.06], [xs + 0.004, Hb + 0.06], [xs + 0.004, c + 0.2]], W / 2 + 0.003, shape));
    }
    for (const xh of [xB + 0.25, xB - 0.65]) for (const s of [1, -1]) parts.chrome.push(box(0.17, 0.03, 0.025, xh, Hb - 0.07, s * (shape(xh, Hb - 0.07, W / 2) + 0.005)));
    // lights, grille, plates placed on the real surface of the nose / tail
    const fy = Hb - 0.14, fx = surfaceX(P, fy, true), ry = b.type === 'sedan' ? Hb - 0.02 : Hb + 0.02, rx = surfaceX(P, ry, false);
    for (const s of [1, -1]) {
      const zh = shape(fx, fy, W / 2) - 0.28;
      parts.head.push(box(0.05, 0.10, 0.40, fx - 0.012, fy, s * zh, s * 0.12));
      const zt = shape(rx, ry, W / 2) - 0.24;
      parts.tail.push(box(0.05, 0.12, 0.36, rx + 0.012, ry, s * zt, -s * 0.12));
      // side mirrors
      parts.paint.push(box(0.14, 0.11, 0.18, xw0 - 0.12, Hb + 0.13, s * (shape(xw0, Hb + 0.13, W / 2) + 0.09)));
      parts.glass.push(box(0.01, 0.08, 0.14, xw0 - 0.195, Hb + 0.13, s * (shape(xw0, Hb + 0.13, W / 2) + 0.09)));
    }
    const gy = Hb - 0.30, gx = surfaceX(P, gy, true);
    parts.trim.push(box(0.04, 0.17, W * 0.44, gx - 0.012, gy, 0));
    const iy = c + 0.22, ix = surfaceX(P, iy, true);
    parts.trim.push(box(0.04, 0.10, W * 0.62, ix - 0.012, iy, 0));
    const py = c + 0.36; parts.plate.push(box(0.02, 0.11, 0.52, surfaceX(P, py, true) + 0.002, py, 0));
    const qy = b.type === 'sedan' ? Hb - 0.20 : Hb - 0.16; parts.plate.push(box(0.02, 0.11, 0.52, surfaceX(P, qy, false) - 0.002, qy, 0));
    parts.trim.push(box(b.L * 0.84, 0.12, W * 0.86, 0, c + 0.08, 0));         // underbody
    wheelX = [pr.xf, pr.xrw];
  }
  // contact shadow
  const sh = new THREE.PlaneGeometry(b.L + 0.7, W + 0.7); sh.rotateX(-Math.PI / 2); sh.translate(0, 0.015, 0);
  parts.shadow.push(sh);

  const wheelPos = [];
  for (const x of wheelX) for (const s of [1, -1]) wheelPos.push([x, b.r, s * (W / 2 - tw / 2 - 0.03), s]);
  if (withWheels) {
    const wg = wheelGeos(b.r, tw);
    for (const [x, y, z, s] of wheelPos) {
      const m = new THREE.Matrix4().makeTranslation(x, y, z);
      if (s < 0) m.multiply(new THREE.Matrix4().makeRotationY(Math.PI)); // spokes face outwards on both sides
      parts.tyre.push(wg.tyre.clone().applyMatrix4(m));
      parts.chrome.push(wg.chrome.clone().applyMatrix4(m));
      parts.trim.push(wg.trim.clone().applyMatrix4(m));
    }
  }
  const merged = {};
  for (const [k, list] of Object.entries(parts)) if (list.length) merged[k] = mergeGeometries(list.map(clean));
  return { merged, wheelPos, tw };
}

const geoCache = {};
// Build a vehicle. opts.separateWheels -> wheels are own objects (ego: steer + spin)
export function buildVehicle(bpKey, colorHex, opts = {}) {
  const b = BLUEPRINTS[bpKey];
  const key = bpKey + (opts.separateWheels ? '_w' : '');
  if (!geoCache[key]) geoCache[key] = partsFor(b, !opts.separateWheels);
  const { merged, wheelPos, tw } = geoCache[key];
  const g = new THREE.Group();
  const matFor = (k) => k === 'paint' ? paint(colorHex) : k === 'tail' && opts.ownTail ? (g.userData.tail = MAT.tail.clone()) : MAT[k];
  for (const [k, geo] of Object.entries(merged)) {
    const m = new THREE.Mesh(geo, matFor(k));
    m.castShadow = k !== 'shadow' && k !== 'glass'; m.receiveShadow = k === 'paint' || k === 'box';
    if (k === 'shadow') m.renderOrder = -1;
    g.add(m);
  }
  if (opts.separateWheels) {
    const wg = wheelGeos(b.r, tw);
    g.userData.wheels = wheelPos.map(([x, y, z, s]) => {
      const pivot = new THREE.Group(); pivot.position.set(x, y, z);       // steering pivot (rotation.y)
      const flip = new THREE.Group(); if (s < 0) flip.rotation.y = Math.PI; // spokes face outwards
      const spin = new THREE.Group();                                       // rolling (rotation.z = -angle*side)
      for (const [geo, mat] of [[wg.tyre, MAT.tyre], [wg.chrome, MAT.chrome], [wg.trim, MAT.trim]]) { const m = new THREE.Mesh(geo, mat); m.castShadow = true; spin.add(m); }
      flip.add(spin); pivot.add(flip); g.add(pivot);
      return { pivot, spin, front: x > 0, side: s };
    });
  }
  addIndicators(g, b);
  g.userData.bp = b;
  return g;
}

// amber turn indicators at the 4 corners: dull base + lit copy toggled by the blinker
let indMat, indOnMat;
export function addIndicators(g, b) {
  indMat = indMat || new THREE.MeshStandardMaterial({ color: 0xa8630c, roughness: 0.4 });
  indOnMat = indOnMat || new THREE.MeshBasicMaterial({ color: 0xffb020 });
  const yF = b.type === 'truck' ? 0.95 : b.belt - 0.14, yR = b.type === 'truck' ? 1.4 : b.belt;
  const mk = (sd, lit) => {
    const k = lit ? 1.08 : 1;
    const geos = [new THREE.BoxGeometry(0.07 * k, 0.07 * k, 0.12 * k).translate(b.L / 2 - 0.06, yF, sd * (b.W / 2 - 0.12)),
      new THREE.BoxGeometry(0.07 * k, 0.07 * k, 0.12 * k).translate(-b.L / 2 + 0.06, yR, sd * (b.W / 2 - 0.12))];
    const m = new THREE.Mesh(mergeGeometries(geos.map(clean)), lit ? indOnMat : indMat);
    if (lit) m.visible = false;
    g.add(m); return m;
  };
  mk(1, false); mk(-1, false);
  g.userData.ind = { R: mk(1, true), L: mk(-1, true) };
}
