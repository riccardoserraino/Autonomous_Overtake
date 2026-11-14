// =====================================================================
// world.js — 3D environment: renderer, lights, sky, road surface with
// elevation, lane markings, shoulders, terrain strips, bridges (viaducts
// over a valley), overpasses, guardrails / median barrier, signs.
// Three visual styles:
//   toy       stylised colours, road-focused, no grass/trees (default)
//   ghost     dark background, transparent surfaces, bright outlines
//   realistic textured asphalt, grass, trees, physical sky
// Coordinates: road (x, y, z-up) -> three.js (x, z, -y).
// =====================================================================
import * as THREE from 'three';
import { Sky } from 'three/examples/jsm/objects/Sky.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { rng } from './road.js';

export const QUALITY = {
  low: { label: 'Low', pr: 1, shadow: 0, trees: 250 },
  medium: { label: 'Medium', pr: 1, shadow: 1024, trees: 500 },
  high: { label: 'Epic', pr: 2, shadow: 2048, trees: 900 },
};
export const STYLES = {
  toy: { label: 'Stylised colours', desc: 'Scale-model vehicles, clean road-focused world.' },
  ghost: { label: 'Ghost / debug', desc: 'Transparent vehicles with outlines on a dark background, like a debug view.' },
  realistic: { label: 'Realistic', desc: 'Detailed cars, textured asphalt, grass, trees and sky.' },
};

function noiseTexture(base, spread, size = 256, speck = 0, repeat = 1) {
  const c = document.createElement('canvas'); c.width = c.height = size;
  const x = c.getContext('2d'), img = x.createImageData(size, size);
  for (let i = 0; i < size * size; i++) {
    const n = (Math.random() - 0.5) * spread + (Math.random() < speck ? 30 : 0);
    img.data[i * 4] = base[0] + n; img.data[i * 4 + 1] = base[1] + n; img.data[i * 4 + 2] = base[2] + n; img.data[i * 4 + 3] = 255;
  }
  x.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(repeat, repeat); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8;
  return t;
}
function signTexture(kind, txt) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 128; const x = cv.getContext('2d');
  x.fillStyle = '#fff'; x.beginPath(); x.arc(64, 64, 62, 0, 7); x.fill();
  x.lineWidth = 13; x.strokeStyle = '#d0201c'; x.beginPath(); x.arc(64, 64, 54, 0, 7); x.stroke();
  if (kind === 'limit') { x.fillStyle = '#111'; x.font = 'bold 50px Arial'; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(txt, 64, 68); }
  else if (kind === 'noov') { // no overtaking: red car left, black car right
    const car = (cx, col) => { x.fillStyle = col; x.fillRect(cx - 13, 52, 26, 22); x.fillRect(cx - 9, 42, 18, 12); x.fillRect(cx - 12, 74, 6, 6); x.fillRect(cx + 6, 74, 6, 6); };
    car(46, '#d0201c'); car(82, '#111');
  } else if (kind === 'endnoov') { // end of no overtaking: grey with black diagonal
    x.fillStyle = '#fff'; x.beginPath(); x.arc(64, 64, 62, 0, 7); x.fill(); x.lineWidth = 3; x.strokeStyle = '#333'; x.beginPath(); x.arc(64, 64, 58, 0, 7); x.stroke();
    x.fillStyle = '#aaa'; x.fillRect(32, 52, 26, 22); x.fillRect(70, 52, 26, 22);
    x.strokeStyle = '#111'; x.lineWidth = 8; x.beginPath(); x.moveTo(30, 100); x.lineTo(98, 28); x.stroke();
  }
  const t = new THREE.CanvasTexture(cv); t.colorSpace = THREE.SRGBColorSpace; return t;
}

export class World {
  constructor(canvas) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance', preserveDrawingBuffer: false });
    const r = this.renderer;
    r.toneMapping = THREE.ACESFilmicToneMapping; r.outputColorSpace = THREE.SRGBColorSpace; r.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene = new THREE.Scene();
    this.pmrem = new THREE.PMREMGenerator(r);
    this.sky = new Sky(); this.sky.scale.setScalar(20000); this.scene.add(this.sky);
    this.envSky = new Sky(); this.envSky.scale.setScalar(20000); this.envScene = new THREE.Scene(); this.envScene.add(this.envSky);
    for (const s of [this.sky, this.envSky]) { const u = s.material.uniforms; u.turbidity.value = 5; u.rayleigh.value = 1.4; u.mieCoefficient.value = 0.004; u.mieDirectionalG.value = 0.8; }
    this.hemi = new THREE.HemisphereLight(0xdfeaff, 0x4a5a3a, 0.5); this.scene.add(this.hemi);
    this.sun = new THREE.DirectionalLight(0xffffff, 2.2);
    const sc = this.sun.shadow.camera; sc.left = -45; sc.right = 45; sc.top = 45; sc.bottom = -45; sc.near = 1; sc.far = 400;
    this.sun.shadow.bias = -0.0004; this.sun.shadow.normalBias = 0.03;
    this.scene.add(this.sun, this.sun.target);
    this.sunDir = new THREE.Vector3();
    this.asphalt = noiseTexture([58, 58, 60], 26, 256, 0.02, 1);
    this.grass = noiseTexture([78, 104, 52], 40, 256, 0, 1);
    this.static = new THREE.Group(); this.scene.add(this.static);
    this.style = 'toy'; this.time = 'day'; this.fog = false;
  }

  setQuality(q) {
    this.q = QUALITY[q];
    this.renderer.setPixelRatio(Math.min(this.q.pr, window.devicePixelRatio || 1));
    const sh = this.q.shadow > 0 && this.style !== 'ghost';
    this.renderer.shadowMap.enabled = sh; this.sun.castShadow = sh;
    if (this.q.shadow) { this.sun.shadow.mapSize.set(this.q.shadow, this.q.shadow); if (this.sun.shadow.map) { this.sun.shadow.map.dispose(); this.sun.shadow.map = null; } }
    if (this.trees) for (const t of this.trees) t.count = Math.min(t.userData.max, this.q.trees);
    this.refreshMaterials();
  }
  refreshMaterials() { this.scene.traverse(o => { if (o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => m.needsUpdate = true); }); }

  // time of day + style: background, lights, reflections, fog
  setTime(style, t, fog) {
    this.style = style; this.time = t; this.fog = fog;
    const night = t === 'night', real = style === 'realistic', ghost = style === 'ghost';
    const elev = t === 'day' ? 42 : t === 'sunset' ? 4 : -6;
    this.sunDir.setFromSphericalCoords(1, THREE.MathUtils.degToRad(90 - elev), THREE.MathUtils.degToRad(215));
    for (const s of [this.sky, this.envSky]) s.material.uniforms.sunPosition.value.copy(this.sunDir);
    this.sun.intensity = ghost ? 0.6 : night ? 0.05 : t === 'sunset' ? 1.4 : real ? 2.4 : 1.9;
    this.sun.color.set(t === 'sunset' ? 0xffb37a : 0xffffff);
    this.hemi.intensity = ghost ? 1.2 : night ? 0.1 : t === 'sunset' ? 0.45 : real ? 0.55 : 1.0;
    this.hemi.groundColor.set(real ? 0x4a5a3a : 0x9a9a92);
    this.renderer.toneMappingExposure = night ? 0.9 : real ? 0.6 : 0.95;
    if (this.envRT) { this.envRT.dispose(); this.envRT = null; }
    if (real && !night) { this.envRT = this.pmrem.fromScene(this.envScene); this.scene.environment = this.envRT.texture; }
    else this.scene.environment = null;
    const bg = ghost ? 0x0b0f17 : night ? 0x070a12 : real ? 0xc8d4e0 : t === 'sunset' ? 0xf2c9a8 : 0xcfe4f4;
    const fogCol = fog ? (night ? 0x0a0d14 : ghost ? 0x0b0f17 : 0xc9cfd6) : bg;
    this.sky.visible = real && !night && !fog;
    this.scene.background = this.sky.visible ? null : new THREE.Color(fog ? fogCol : bg);
    this.scene.fog = this.fogObj = fog ? new THREE.Fog(fogCol, 12, 170) : new THREE.Fog(bg, ghost ? 900 : 1200, ghost ? 3000 : 6000);
    const sh = this.q && this.q.shadow > 0 && !ghost;
    this.renderer.shadowMap.enabled = !!sh; this.sun.castShadow = !!sh;
    this.refreshMaterials();
  }
  follow(p) { this.sun.position.copy(p).addScaledVector(this.sunDir, 150); this.sun.target.position.copy(p); }

  // ------------------------------------------------------------------
  buildRoad(road, opts) {
    for (const o of [...this.static.children]) { this.static.remove(o); o.traverse(c => { if (c.geometry) c.geometry.dispose(); }); }
    this.trees = null;
    const style = this.style, ghost = style === 'ghost', real = style === 'realistic';
    const c = road.cfg, w = c.laneWidth, L = road.length;
    const P = (s, d, dy = 0) => { const p = road.pose(s, d); return [p[0], p[3] + dy, -p[1]]; };
    const Z = (s) => road.zAt(s);
    const mat = (o) => ghost ? new THREE.MeshBasicMaterial({ color: o.ghost ?? 0x3a4a66, transparent: true, opacity: o.op ?? 0.25, depthWrite: false, side: THREE.DoubleSide })
      : new THREE.MeshStandardMaterial({ color: o.color, map: o.map || null, roughness: o.rough ?? 0.9, metalness: o.metal ?? 0, side: o.side ?? THREE.FrontSide, polygonOffset: !!o.po, polygonOffsetFactor: o.po || 0 });
    const add = (geo, m, shadow = true) => { if (!geo) return null; const mesh = new THREE.Mesh(geo, m); mesh.receiveShadow = shadow && !ghost; this.static.add(mesh); return mesh; };

    // generic strip between lateral offsets d0 < d1 with height functions h0(s), h1(s) (absolute)
    const strip = (sa, sb, d0, d1, h0, h1, step = 2) => {
      const pos = [], uv = [], idx = [];
      const n = Math.max(1, Math.ceil((sb - sa) / step));
      for (let j = 0; j <= n; j++) {
        const s = sa + (sb - sa) * j / n;
        const a = road.pose(s, d1), b = road.pose(s, d0);
        pos.push(a[0], h1(s), -a[1], b[0], h0(s), -b[1]);
        uv.push(s / 10, d1 / 10, s / 10, d0 / 10);
        if (j < n) { const k = j * 2; idx.push(k, k + 1, k + 2, k + 1, k + 3, k + 2); }
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
      g.setIndex(idx); g.computeVertexNormals(); return g;
    };
    const flat = (dy) => (s) => Z(s) + dy;

    // ranges with and without bridges
    const bridges = road.features.filter(f => f.type === 'bridge');
    const ranges = []; { let a = 0; for (const b of bridges) { if (b.s0 > a) ranges.push([a, b.s0]); a = b.s1; } if (a < L) ranges.push([a, L]); }
    const valley = (s) => { for (const b of bridges) if (s >= b.s0 && s <= b.s1) { const u = (s - b.s0) / (b.s1 - b.s0); return Math.min(16, (b.s1 - b.s0) / 25) * Math.pow(Math.sin(Math.PI * u), 0.6); } return 0; };

    const right = road.edgeRight - 0.8, left = c.nLeft ? road.edgeLeft + 0.8 : 1.4;
    // ---- asphalt
    const asph = mat({ color: real ? 0xffffff : 0x5d6472, map: real ? this.asphalt : null, ghost: 0x26324a, op: 0.55, rough: 0.92 });
    add(strip(0, L, right, left, flat(0), flat(0), 2), asph);
    // ---- shoulders (gravel) + terrain strips (not on bridges)
    const shoulderMat = mat({ color: real ? 0x8a8070 : 0xd7d0c0 });
    const terrMat = real ? mat({ color: 0xffffff, map: this.grass }) : mat({ color: 0xe9e5db });
    const tw = real ? [2.4, 12, 30, 60, 110] : [2.4, 8, 18, 28];
    const terrH = (s, k) => Z(s) - 0.45 - (k > 0 ? 0.25 * k : 0) - valley(s) * Math.min(1, 0.4 + 0.3 * k);
    if (!ghost) {
      const sh = [], te = [];
      for (const [a, b] of ranges) {
        sh.push(strip(a, b, right - tw[0], right, flat(-0.45), flat(-0.005), 4), strip(a, b, left, left + tw[0], flat(-0.005), flat(-0.45), 4));
      }
      for (let k = 0; k < tw.length - 1; k++) {
        te.push(strip(0, L, right - tw[k + 1], right - tw[k], (s) => terrH(s, k + 1), (s) => k === 0 && !road.isBridge(s) ? Z(s) - 0.45 : terrH(s, k), 6));
        te.push(strip(0, L, left + tw[k], left + tw[k + 1], (s) => k === 0 && !road.isBridge(s) ? Z(s) - 0.45 : terrH(s, k), (s) => terrH(s, k + 1), 6));
      }
      for (const b of bridges) te.push(strip(b.s0, b.s1, right - tw[0], left + tw[0], (s) => terrH(s, 0), (s) => terrH(s, 0), 4)); // valley floor under the bridge
      // outer slope down to a flat base plane, so the road never floats in the air
      const baseY = Math.min(...road.Z) - 20, last = tw.length - 1, far = real ? 320 : 220;
      te.push(strip(0, L, right - far, right - tw[last], () => baseY, (s) => terrH(s, last), 10));
      te.push(strip(0, L, left + tw[last], left + far, (s) => terrH(s, last), () => baseY, 10));
      add(mergeGeometries(sh.filter(Boolean)), shoulderMat);
      add(mergeGeometries(te), terrMat);
      const base = new THREE.Mesh(new THREE.PlaneGeometry(30000, 30000).rotateX(-Math.PI / 2), real ? mat({ color: 0x6f8a52 }) : mat({ color: 0xdcd7cb }));
      base.position.y = baseY - 0.05; base.receiveShadow = false; this.static.add(base);
      // river at the bottom of each valley
      const water = mat({ color: 0x6fb2e2, rough: 0.2, metal: 0.1 });
      for (const b of bridges) { const sm = (b.s0 + b.s1) / 2; add(strip(sm - 6, sm + 6, right - tw[tw.length - 1], left + tw[tw.length - 1], (s) => terrH(s, 3) + 0.08, (s) => terrH(s, 3) + 0.08, 3), water); }
    }

    // ---- lane markings
    const marks = [];
    const line = (sa, sb, d, h = 0.015) => marks.push(strip(sa, sb, d - 0.075, d + 0.075, flat(h), flat(h), 2));
    const dashed = (d, filter = () => true) => { for (let s = 0; s < L - 5; s += 12) if (filter(s)) marks.push(strip(s, s + 4.5, d - 0.075, d + 0.075, flat(0.015), flat(0.015), 1.5)); };
    line(0, L, road.edgeRight);
    for (let k = 1; k < c.nRight; k++) dashed(-k * w);
    if (c.nLeft === 0) line(0, L, 0.1);
    else {
      line(0, L, road.edgeLeft);
      for (let k = 1; k < c.nLeft; k++) dashed(k * w);
      dashed(0, s => !road.isSolid(s) && !road.isSolid(s + 4.5));
      let start = null;
      for (let s = 0; s <= L; s += road.solidStep) { const on = s < L && road.isSolid(s); if (on && start === null) start = s; if (!on && start !== null) { line(start, s, 0); start = null; } }
    }
    add(mergeGeometries(marks), ghost ? new THREE.MeshBasicMaterial({ color: 0x7fd1ff }) : new THREE.MeshStandardMaterial({ color: 0xf6f6f2, roughness: 0.55, polygonOffset: true, polygonOffsetFactor: -2 }));

    // ---- bridges: deck sides, underside, parapets, pillars
    const concrete = mat({ color: 0xc4c7cc, ghost: 0x5c7cfa, op: 0.18, rough: 0.85 });
    const bridgeG = [], pillars = [];
    for (const b of bridges) {
      for (const d of [right, left]) bridgeG.push(strip(b.s0, b.s1, d, d + 0.001, flat(-1.4), flat(0), 3));
      bridgeG.push(strip(b.s0, b.s1, right, left, flat(-1.4), flat(-1.4), 3));
      for (const [d0, d1] of [[right - 0.4, right], [left, left + 0.4]]) {
        bridgeG.push(strip(b.s0, b.s1, d0, d1, flat(0.95), flat(0.95), 3), strip(b.s0, b.s1, d0, d0 + 0.001, flat(0), flat(0.95), 3), strip(b.s0, b.s1, d1 - 0.001, d1, flat(0.95), flat(0), 3));
      }
      const span = 38, n = Math.max(1, Math.round((b.s1 - b.s0) / span));
      for (let i = 1; i < n; i++) {
        const s = b.s0 + (b.s1 - b.s0) * i / n, top = Z(s) - 1.4, bot = terrH(s, 0) - 1;
        for (const d of [right + 2.5, left - 2.5]) { const p = road.pose(s, d); pillars.push(new THREE.CylinderGeometry(0.75, 0.9, top - bot, 12).translate(p[0], (top + bot) / 2, -p[1])); }
      }
    }
    if (bridgeG.length) add(mergeGeometries(bridgeG), concrete);
    if (pillars.length) { const m = add(mergeGeometries(pillars.map(g => g.toNonIndexed())), concrete); if (m) m.castShadow = !ghost; }

    // ---- overpasses: a crossing road bridge above ours
    const overG = [], overTop = [];
    for (const f of road.features.filter(f => f.type === 'overpass')) {
      const s = (f.s0 + f.s1) / 2, p = road.pose(s, (right + left) / 2), th = p[2], zc = Z(s) + 6.6;
      const span = (left - right) + 2 * 22, dW = 11;
      const g = (geo) => { geo.rotateY(th + Math.PI / 2); geo.translate(p[0], 0, -p[1]); return geo.toNonIndexed(); };
      overG.push(g(new THREE.BoxGeometry(span, 1.1, dW).translate(0, zc, 0)));
      overTop.push(g(new THREE.BoxGeometry(span - 0.2, 0.05, dW - 1.6).translate(0, zc + 0.58, 0)));
      for (const sd of [-1, 1]) overG.push(g(new THREE.BoxGeometry(span, 0.9, 0.3).translate(0, zc + 1.0, sd * (dW / 2 - 0.15))));
      // piers just outside our road edges + abutments at the deck ends
      for (const dd of [right - 3.2, left + 3.2]) {
        const q = road.pose(s, dd), h = zc - 0.55 - (Z(s) - 0.6);
        for (const off of [-3.5, 0, 3.5]) {
          const geo = new THREE.BoxGeometry(1.0, h, 1.0).translate(off, Z(s) - 0.6 + h / 2, 0);
          geo.rotateY(th); geo.translate(q[0], 0, -q[1]); overG.push(geo.toNonIndexed());
        }
      }
      for (const sd of [-1, 1]) {
        const geo = new THREE.BoxGeometry(4, zc - (Z(s) - 1.5), dW).translate(sd * (span / 2 - 2), (zc + Z(s) - 1.5) / 2, 0);
        overG.push(g(geo));
      }
    }
    if (overG.length) { const m = add(mergeGeometries(overG), concrete); if (m) m.castShadow = !ghost; add(mergeGeometries(overTop), mat({ color: 0x5d6472, ghost: 0x26324a })); }

    // ---- guardrails (W-beam) / median barrier
    if (opts.rails) {
      const railMat = mat({ color: 0xb8bcc0, ghost: 0x8899bb, op: 0.35, metal: real ? 0.8 : 0.2, rough: 0.4, side: THREE.DoubleSide });
      const rails = [], posts = [];
      for (const [a, b] of ranges) {
        const sides = [right - 0.9]; if (c.nLeft > 0) sides.push(left + 0.9);
        for (const d of sides) {
          rails.push(strip(a, b, d, d + 0.001, flat(0.5), flat(0.82), 3));
          for (let s = a + 2; s < b - 2; s += 4) { const p = road.pose(s, d + (d < 0 ? -0.1 : 0.1)); posts.push(new THREE.BoxGeometry(0.1, 1.1, 0.12).rotateY(p[2]).translate(p[0], p[3] + 0.2, -p[1]).toNonIndexed()); }
        }
      }
      if (c.nLeft === 0) { // concrete median barrier ("New Jersey") all along, also on bridges
        rails.push(strip(0, L, 0.55, 0.95, flat(0.9), flat(0.9), 3), strip(0, L, 0.35, 0.55, flat(0), flat(0.9), 3), strip(0, L, 0.95, 1.15, flat(0.9), flat(0), 3));
      }
      add(mergeGeometries(rails), railMat);
      if (posts.length) add(mergeGeometries(posts), mat({ color: 0x8f949a, ghost: 0x8899bb, op: 0.3, metal: 0.5 }));
    }

    // ---- signs: speed limits where they change, no-overtaking zones on two-way roads
    if (!ghost) {
      const poleM = new THREE.MeshStandardMaterial({ color: 0x9aa0a6, metalness: 0.6, roughness: 0.4 });
      const texCache = {};
      const sign = (s, kind, txt) => {
        if (!road.closed && (s < 0 || s > L)) return;
        const k = kind + txt; texCache[k] = texCache[k] || new THREE.MeshStandardMaterial({ map: signTexture(kind, txt), roughness: 0.5 });
        const p = road.pose(s, right - 1.6);
        const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 2.6, 8), poleM); pole.position.set(p[0], p[3] + 0.9, -p[1]);
        const disc = new THREE.Mesh(new THREE.CircleGeometry(0.45, 32), texCache[k]); disc.position.set(p[0], p[3] + 2.2, -p[1]); disc.rotation.y = p[2] - Math.PI / 2;
        const back = new THREE.Mesh(new THREE.CircleGeometry(0.45, 32), poleM); back.position.copy(disc.position); back.rotation.y = p[2] + Math.PI / 2;
        pole.castShadow = disc.castShadow = true; this.static.add(pole, disc, back);
      };
      let prev = null;
      for (const si of road.segInfo) { const lim = si.g.limit > 0 ? si.g.limit : c.speedLimit; if (lim !== prev) sign(si.s0 + 15, 'limit', String(lim)); prev = lim; }
      for (const si of road.segInfo) if (si.g.type === 'straight' && si.s1 - si.s0 > 1500) sign(si.s0 + 700, 'limit', String(si.g.limit > 0 ? si.g.limit : c.speedLimit));
      if (c.nLeft > 0) {
        let on = false;
        for (let s = 0; s < L; s += road.solidStep) {
          const sol = road.isSolid(s);
          if (sol && !on) sign(s - 20, 'noov', ''); if (!sol && on) sign(s + 5, 'endnoov', '');
          on = sol;
        }
      }
    }

    // ---- trees (realistic style only)
    if (real && opts.scenery) {
      const R = rng(7), maxT = 900;
      const trunkG = new THREE.CylinderGeometry(0.18, 0.28, 2.2, 6).translate(0, 1.1, 0);
      const fol = mergeGeometries([new THREE.ConeGeometry(2.2, 4.2, 7).translate(0, 3.8, 0), new THREE.ConeGeometry(1.6, 3.4, 7).translate(0, 5.6, 0)]);
      const trunks = new THREE.InstancedMesh(trunkG, new THREE.MeshStandardMaterial({ color: 0x5b4330, roughness: 1 }), maxT);
      const leaves = new THREE.InstancedMesh(fol, new THREE.MeshStandardMaterial({ color: 0x35532a, roughness: 0.95, flatShading: true }), maxT);
      const dum = new THREE.Object3D(); let n = 0, tries = 0;
      while (n < maxT && tries++ < maxT * 5) {
        const s = R() * L, side = R() < 0.5 ? -1 : 1, off = 12 + R() * 90;
        const d = side < 0 ? right - off : left + off;
        const p = road.pose(s, d), pr = road.project(p[0], p[1]);
        if (pr.d > road.edgeRight - 10 && pr.d < road.edgeLeft + 10) continue;
        if (road.isBridge(s)) continue;
        const k = off < 30 ? 2 : off < 60 ? 3 : 4, y = terrH(s, Math.min(k, tw.length - 1)), sc = 0.7 + R() * 0.8;
        dum.position.set(p[0], y, -p[1]); dum.rotation.set(0, R() * 6.28, 0); dum.scale.set(sc, sc * (0.85 + R() * 0.4), sc); dum.updateMatrix();
        trunks.setMatrixAt(n, dum.matrix); leaves.setMatrixAt(n, dum.matrix); n++;
      }
      for (const t of [trunks, leaves]) { t.castShadow = true; t.userData.max = n; t.count = Math.min(n, this.q ? this.q.trees : 500); this.static.add(t); }
      this.trees = [trunks, leaves];
    }
  }
}
