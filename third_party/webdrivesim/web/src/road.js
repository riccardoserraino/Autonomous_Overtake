// =====================================================================
// road.js — road geometry (pure math, no graphics)
//
// THEORY
// Like OpenDRIVE, the road is a REFERENCE LINE (centre line between the
// two directions) described by its curvature kappa(s) along the arc
// length s. Every user "segment" is expanded into three primitives:
//   line   : kappa = 0
//   arc    : kappa = 1/R (constant)
//   spiral : kappa changes LINEARLY with s (clothoid), so the steering
//            angle never jumps: used to enter and leave every curve.
// Integrating theta' = kappa, x' = cos(theta), y' = sin(theta) gives the
// centre line. A lane is a lateral OFFSET d (Frenet frame, d>0 = left).
//
// ELEVATION z(s): each segment rises by dz with a cubic "smoothstep"
// z = z0 + dz (3u^2 - 2u^3), u = (s - s0)/L. The grade is zero at both
// ends, so consecutive up/down segments create CRESTS and SAGS. It is
// exactly an OpenDRIVE <elevation> cubic: a=z0, b=0, c=3dz/L^2, d=-2dz/L^3.
//
// SIGHT DISTANCE: on a crest or in a sharp curve you cannot see far
// enough to overtake safely -> those stretches get a SOLID centre line
// (no-overtaking zone) when centerLine = 'auto'.
// =====================================================================

const PI = Math.PI, D2R = PI / 180;

// ---------------------------------------------------------------------
// Segment types the user can add in the editor
// ---------------------------------------------------------------------
export const SEG_TYPES = {
  straight: { label: 'Straight', icon: '━', help: 'Constant heading.' },
  curve: { label: 'Curve', icon: '◜', help: 'Circular arc with clothoid entry/exit (transition length).' },
  scurve: { label: 'S-curve', icon: '∿', help: 'Two opposite curves in a row (chicane / lane-shift style).' },
  tighten: { label: 'Tightening curve', icon: '🌀', help: 'Radius decreases from entry to exit: the driver must keep braking.' },
  hairpin: { label: 'Hairpin', icon: '∩', help: '180° turn with a small radius (mountain roads).' },
};
export const newSegment = (type) => ({
  type, length: 600, radius: type === 'hairpin' ? 45 : 500, radius2: 150, angle: type === 'scurve' ? 25 : 45,
  dir: 'right', transition: type === 'hairpin' ? 25 : 80, dz: 0, feature: 'none', limit: 0, noOvertake: false,
});

// ---------------------------------------------------------------------
// Presets (long and varied, motorway / superstrada / rural road)
// ---------------------------------------------------------------------
const S = (length, o = {}) => ({ ...newSegment('straight'), length, ...o });
const C = (radius, angle, dir, transition, o = {}) => ({ ...newSegment('curve'), radius, angle, dir, transition, ...o });
const SC = (radius, angle, dir, transition, o = {}) => ({ ...newSegment('scurve'), radius, angle, dir, transition, ...o });
const T = (radius, radius2, angle, dir, o = {}) => ({ ...newSegment('tighten'), radius, radius2, angle, dir, ...o });
const H = (radius, dir, o = {}) => ({ ...newSegment('hairpin'), radius, dir, ...o });

export const PRESETS = {
  motorway: {
    name: 'Motorway A (3+3, divided)', desc: '≈14 km, 3 lanes, wide clothoid curves, overpasses, a viaduct and gentle crests. 130 km/h.',
    road: { closed: false, nRight: 3, nLeft: 0, laneWidth: 3.75, speedLimit: 130, centerLine: 'solid' },
    segments: [S(1200), C(1500, 22, 'right', 250, { dz: 6 }), S(900, { feature: 'overpass' }), C(1200, 28, 'left', 250, { dz: -4 }),
      S(1400, { dz: 10 }), S(700, { dz: -6, feature: 'bridge' }), SC(1000, 15, 'right', 200), S(1100, { feature: 'overpass', dz: 8 }),
      C(900, 40, 'right', 220, { dz: -8, limit: 110 }), S(1300, { dz: 12 }), S(900, { dz: -14, feature: 'overpass' }), C(1300, 30, 'left', 250),
      S(1500, { feature: 'bridge', dz: 4 }), S(1000)],
  },
  superstrada: {
    name: 'Superstrada SS (2+2, divided)', desc: '≈10 km, 2 lanes, tighter curves (R 450–700), crests, bridges and overpasses. 110 km/h.',
    road: { closed: false, nRight: 2, nLeft: 0, laneWidth: 3.5, speedLimit: 110, centerLine: 'solid' },
    segments: [S(800), C(600, 35, 'left', 120, { dz: 8 }), S(500, { dz: -10, feature: 'bridge' }), SC(550, 25, 'right', 100),
      S(700, { feature: 'overpass' }), C(450, 60, 'right', 110, { dz: 12, limit: 90 }), S(600, { dz: -12 }), S(900, { feature: 'overpass' }),
      T(700, 350, 60, 'left', { limit: 90 }), S(1000, { dz: 10 }), S(600, { dz: -10, feature: 'bridge' }), C(650, 40, 'right', 120), S(1200)],
  },
  rural: {
    name: 'Rural road SP (1+1, two-way)', desc: '≈8 km, oncoming traffic, crests that hide oncoming cars, S-bends, a tightening curve, a bridge and a hairpin. 90 km/h.',
    road: { closed: false, nRight: 1, nLeft: 1, laneWidth: 3.5, speedLimit: 90, centerLine: 'auto' },
    segments: [S(900), S(450, { dz: 9 }), S(450, { dz: -9 }), SC(300, 30, 'left', 70), S(1000, { feature: 'overpass' }), C(350, 50, 'right', 80, { dz: 5 }),
      S(500, { feature: 'bridge', dz: -3 }), S(700, { dz: 7 }), S(400, { dz: -7 }), T(400, 160, 75, 'left', { limit: 70 }), S(800),
      H(45, 'right', { limit: 40 }), S(600, { dz: 10 }), C(500, 30, 'left', 90), S(900)],
  },
  oval: {
    name: 'Empty highway oval (test track)', desc: 'Closed 4.6 km loop, 2 lanes, no traffic by default. Ideal for lane keeping and ACC tests.',
    road: { closed: true, nRight: 2, nLeft: 0, laneWidth: 3.5, speedLimit: 110, centerLine: 'solid' },
    segments: [S(900), C(400, 180, 'left', 150), S(900), C(400, 180, 'left', 150)],
  },
};
export function presetRoad(key) {
  const p = PRESETS[key];
  return { preset: key, name: key, ...p.road, segments: JSON.parse(JSON.stringify(p.segments)) };
}

// expand one user segment into line/arc/spiral primitives [type, length, k0, k1]
function expand(g) {
  const sg = g.dir === 'left' ? 1 : -1, out = [];
  const curve = (R, angDeg, dirSign, Lt) => {
    const k = dirSign / Math.max(R, 5), th = Math.abs(angDeg) * D2R;
    Lt = Math.max(0, Math.min(Lt, th * R));
    const La = th * R - Lt;
    if (Lt > 0.01) out.push(['spiral', Lt, 0, k]);
    if (La > 0.01) out.push(['arc', La, k, k]);
    if (Lt > 0.01) out.push(['spiral', Lt, k, 0]);
  };
  if (g.type === 'straight') out.push(['line', Math.max(1, g.length), 0, 0]);
  else if (g.type === 'curve') curve(g.radius, g.angle, sg, g.transition);
  else if (g.type === 'hairpin') curve(g.radius, 180, sg, g.transition);
  else if (g.type === 'scurve') { curve(g.radius, g.angle, sg, g.transition); curve(g.radius, g.angle, -sg, g.transition); }
  else if (g.type === 'tighten') {
    const k1 = 1 / Math.max(g.radius, 5), k2 = 1 / Math.max(Math.min(g.radius2, g.radius), 5), Lt = 40;
    // heading = Lt k1/2 + Lm (k1+k2)/2 + Lt k2/2  ->  solve the middle (tightening) clothoid length Lm
    const Lm = Math.max(10, (Math.abs(g.angle) * D2R - Lt * (k1 + k2) / 2) / ((k1 + k2) / 2));
    out.push(['spiral', Lt, 0, sg * k1], ['spiral', Lm, sg * k1, sg * k2], ['spiral', Lt, sg * k2, 0]);
  }
  return out;
}

export function buildRoad(cfg) {
  const prims = [], segInfo = [];
  let s0 = 0;
  cfg.segments.forEach((g, i) => {
    const ps = expand(g); const start = s0;
    for (const [type, len, k0, k1] of ps) { prims.push({ type, len, k0, k1, s0, seg: i }); s0 += len; }
    segInfo.push({ i, s0: start, s1: s0, g });
  });
  const length = s0, closed = !!cfg.closed;
  const N = Math.max(2, Math.round(length / 0.5)), DS = length / N;
  const kappa = (s) => { for (const p of prims) if (s < p.s0 + p.len) return p.k0 + (p.k1 - p.k0) * (s - p.s0) / p.len; return prims[prims.length - 1].k1; };

  // elevation: cubic smoothstep per segment
  let z0 = 0; for (const si of segInfo) { si.z0 = z0; z0 += si.g.dz || 0; si.z1 = z0; }
  const segAt = (s) => { for (const si of segInfo) if (s < si.s1) return si; return segInfo[segInfo.length - 1]; };
  const zOf = (s) => { const si = segAt(s), u = Math.min(1, Math.max(0, (s - si.s0) / (si.s1 - si.s0))); return si.z0 + (si.z1 - si.z0) * (3 * u * u - 2 * u * u * u); };

  const X = new Float64Array(N + 1), Y = new Float64Array(N + 1), TH = new Float64Array(N + 1), K = new Float64Array(N + 1), Z = new Float64Array(N + 1);
  let x = 0, y = 0, th = 0;
  for (let i = 0; i <= N; i++) {
    X[i] = x; Y[i] = y; TH[i] = th; K[i] = kappa(i * DS); Z[i] = zOf(Math.min(i * DS, length - 1e-6));
    const k = kappa((i + 0.5) * DS);
    x += Math.cos(th + k * DS / 2) * DS; y += Math.sin(th + k * DS / 2) * DS; th += k * DS;
  }
  // exact start pose of every primitive (for OpenDRIVE)
  { let gx = 0, gy = 0, gh = 0; for (const p of prims) { p.x = gx; p.y = gy; p.hdg = gh; const n = 2000, h = p.len / n; for (let j = 0; j < n; j++) { const k = p.k0 + (p.k1 - p.k0) * (j + 0.5) / n; gx += Math.cos(gh + k * h / 2) * h; gy += Math.sin(gh + k * h / 2) * h; gh += k * h; } } }

  const w = cfg.laneWidth;
  const road = { cfg, prims, segInfo, length, closed, N, DS, X, Y, TH, K, Z, kappa };
  road.wrap = closed ? (s) => ((s % length) + length) % length : (s) => Math.min(length, Math.max(0, s));
  road.diff = closed ? (a, b) => { let d = ((b - a) % length + length) % length; return d > length / 2 ? d - length : d; } : (a, b) => b - a;
  road.zAt = (s) => { s = road.wrap(s); const u = s / DS, i = Math.min(N - 1, Math.floor(u)), f = u - i; return Z[i] + (Z[i + 1] - Z[i]) * f; };
  road.gradeAt = (s) => (road.zAt(s + 1) - road.zAt(s - 1)) / 2;
  road.pose = (s, d = 0) => {
    s = road.wrap(s);
    const u = s / DS, i = Math.min(N - 1, Math.floor(u)), f = u - i;
    const px = X[i] + (X[i + 1] - X[i]) * f, py = Y[i] + (Y[i + 1] - Y[i]) * f, t = TH[i] + (TH[i + 1] - TH[i]) * f;
    return [px - d * Math.sin(t), py + d * Math.cos(t), t, Z[i] + (Z[i + 1] - Z[i]) * f];
  };
  road.project = (px, py, hint = null) => {
    let best = 1e18, bi = 0;
    if (hint == null) { for (let i = 0; i <= N; i += 4) { const e = (X[i] - px) ** 2 + (Y[i] - py) ** 2; if (e < best) { best = e; bi = i; } } }
    else bi = Math.round(road.wrap(hint) / DS);
    const c = bi; best = 1e18;
    for (let k = -120; k <= 120; k++) {
      let i = c + k;
      if (closed) i = ((i % N) + N) % N; else if (i < 0 || i > N) continue;
      const e = (X[i] - px) ** 2 + (Y[i] - py) ** 2;
      if (e < best) { best = e; bi = i; }
    }
    const t = TH[bi], dx = px - X[bi], dy = py - Y[bi];
    return { s: road.wrap(bi * DS + dx * Math.cos(t) + dy * Math.sin(t)), d: -dx * Math.sin(t) + dy * Math.cos(t), heading: t };
  };

  // lanes. Our direction: lanes 1..nRight on the right of the centre line, 1 = next to the centre
  // (the fast / overtaking lane), nRight = rightmost. Oncoming lanes: -1..-nLeft.
  road.laneD = (lane) => lane > 0 ? -(lane - 0.5) * w : (-lane - 0.5) * w;
  road.edgeRight = -cfg.nRight * w;
  road.edgeLeft = cfg.nLeft > 0 ? cfg.nLeft * w : 0;
  road.laneOf = (d) => {
    if (d <= 0 || cfg.nLeft === 0) return Math.min(cfg.nRight, Math.max(1, Math.floor(-d / w) + 1));
    return -Math.min(cfg.nLeft, Math.floor(d / w) + 1);
  };
  road.laneName = (lane) => laneName(cfg, lane);

  // speed limit per segment
  road.limitAt = (s) => { const si = segAt(road.wrap(s)); return si.g.limit > 0 ? si.g.limit : cfg.speedLimit; };
  road.features = segInfo.filter(si => si.g.feature && si.g.feature !== 'none').map(si => ({ type: si.g.feature, s0: si.s0, s1: si.s1, seg: si.i }));
  road.isBridge = (s) => road.features.some(f => f.type === 'bridge' && s >= f.s0 && s <= f.s1);

  // vertical sight distance at s: how far a driver (eye 1.1 m) sees an object 1.0 m tall
  road.sightAt = (s) => {
    const ze = road.zAt(s) + 1.1;
    let vis = 0;
    for (let dd = 20; dd <= 600; dd += 20) {
      if (!closed && s + dd > length) { vis = dd; break; }
      const zt = road.zAt(s + dd) + 1.0;
      let ok = true;
      for (let q = 10; q < dd; q += 10) { const zl = ze + (zt - ze) * q / dd; if (road.zAt(s + q) + 0.15 > zl) { ok = false; break; } }
      if (!ok) break; vis = dd;
    }
    return vis;
  };
  // line-of-sight for sensors (vertical profile only)
  road.los = (sA, hA, sB, hB) => {
    const dsum = road.diff(sA, sB), n = Math.max(2, Math.ceil(Math.abs(dsum) / 8));
    const za = road.zAt(sA) + hA, zb = road.zAt(sB) + hB;
    for (let i = 1; i < n; i++) { const u = i / n; if (road.zAt(sA + dsum * u) + 0.2 > za + (zb - za) * u) return false; }
    return true;
  };

  // no-overtaking zones
  const step = 5, M = Math.ceil(length / step);
  const solid = new Uint8Array(M);
  for (let j = 0; j < M; j++) {
    const s = j * step, si = segAt(s);
    if (cfg.centerLine === 'solid' || si.g.noOvertake) { solid[j] = 1; continue; }
    if (cfg.centerLine === 'dashed') continue;
    let mx = 0;
    for (let q = -100; q <= 300; q += 10) { const ss = s + q; if (!closed && (ss < 0 || ss > length)) continue; mx = Math.max(mx, Math.abs(kappa(road.wrap(ss)))); }
    solid[j] = mx > 1 / 600 || road.sightAt(s) < 300 ? 1 : 0;
  }
  road.isSolid = (s) => solid[Math.min(M - 1, Math.floor(road.wrap(s) / step))] === 1;
  road.solidStep = step; road.solidArr = solid;
  return road;
}

export function laneName(c, lane) {
  if (lane < 0) return c.nLeft > 1 ? `Oncoming lane ${-lane}` : 'Oncoming lane';
  if (c.nRight === 1) return 'Driving lane';
  if (lane === c.nRight) return 'Right lane';
  if (lane === 1) return c.nRight === 2 ? 'Left lane (overtaking)' : 'Left lane (fast)';
  return 'Middle lane';
}
export function laneList(c) {
  const out = [];
  for (let k = c.nRight; k >= 1; k--) out.push([k, laneName(c, k)]);
  for (let k = 1; k <= c.nLeft; k++) out.push([-k, laneName(c, -k)]);
  return out;
}

// Seeded random generator (mulberry32) and Gaussian sample (Box-Muller)
export function rng(seed) {
  let a = seed >>> 0;
  return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
}
export function gauss(r = Math.random) { let u = 0, v = 0; while (!u) u = r(); while (!v) v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
