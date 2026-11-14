// =====================================================================
// toyModel.js — stylised "scale model" vehicles (light, readable)
//
// Each class has its own silhouette so it is recognisable at a glance:
//   hatchback: short, cabin reaching the rear
//   sedan:     long, three boxes (bonnet, cabin, boot)
//   SUV:       tall, boxy, big wheels
//   van:       one-box, very tall
//   truck:     cab + cargo box
// Dimensions come from the same BLUEPRINTS used by the physics, so what
// you see is exactly the size the sensors and collision checks use.
// Styles: 'toy' (flat colours) or 'ghost' (transparent + edge lines).
// Turn indicators: amber lights at the 4 corners, blinking at 1.5 Hz.
// =====================================================================
import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { BLUEPRINTS } from './carModel.js';

// cabin outline as fractions of the length: [bottom rear, bottom front, top rear, top front]
const CAB = { hatch: [-0.46, 0.13, -0.40, -0.03], sedan: [-0.27, 0.18, -0.19, 0.04], suv: [-0.47, 0.16, -0.45, 0.01], van: [-0.49, 0.31, -0.48, 0.17] };
const BODY_H = { hatch: 0.42, sedan: 0.42, suv: 0.47, van: 0.45 };   // body height / total height

const M = {};
function mats() {
  if (M.dark) return M;
  M.dark = new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.7 });
  M.glass = new THREE.MeshStandardMaterial({ color: 0x2e4257, roughness: 0.25, metalness: 0.3, side: THREE.DoubleSide });
  M.tyre = new THREE.MeshStandardMaterial({ color: 0x1d1f23, roughness: 0.9 });
  M.hub = new THREE.MeshStandardMaterial({ color: 0xe9edf2, roughness: 0.4 });
  M.head = new THREE.MeshStandardMaterial({ color: 0xfff6d8, emissive: 0xfff2c0, emissiveIntensity: 0.25 });
  M.tail = new THREE.MeshStandardMaterial({ color: 0xc81e1e, emissive: 0xff2020, emissiveIntensity: 0.25 });
  M.ind = new THREE.MeshStandardMaterial({ color: 0xb86e10, roughness: 0.5 });
  M.indOn = new THREE.MeshBasicMaterial({ color: 0xffb020 });
  M.box = new THREE.MeshStandardMaterial({ color: 0xf1f1ee, roughness: 0.6 });
  M.shadow = new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.18, depthWrite: false });
  return M;
}
export const TOY_MAT = mats;
const paintCache = {};
const toyPaint = (hex) => paintCache[hex] || (paintCache[hex] = new THREE.MeshStandardMaterial({ color: hex, roughness: 0.45, metalness: 0.05 }));
const ghostCache = {};
function ghostMat(hex, op) {
  const k = hex + op;
  return ghostCache[k] || (ghostCache[k] = new THREE.MeshBasicMaterial({ color: hex, transparent: true, opacity: op, depthWrite: false }));
}

function roundRect(x0, y0, x1, y1, r) {
  const s = new THREE.Shape();
  s.moveTo(x0 + r, y0); s.lineTo(x1 - r, y0); s.quadraticCurveTo(x1, y0, x1, y0 + r);
  s.lineTo(x1, y1 - r); s.quadraticCurveTo(x1, y1, x1 - r, y1); s.lineTo(x0 + r, y1);
  s.quadraticCurveTo(x0, y1, x0, y1 - r); s.lineTo(x0, y0 + r); s.quadraticCurveTo(x0, y0, x0 + r, y0);
  return s;
}
function polyShape(pts) { return new THREE.Shape(pts.map(([x, y]) => new THREE.Vector2(x, y))); }
function extrude(shape, W, bev = 0.07) {
  const g = new THREE.ExtrudeGeometry(shape, { depth: W - 2 * bev, bevelEnabled: true, bevelThickness: bev, bevelSize: bev * 0.8, bevelOffset: -bev * 0.8, bevelSegments: 2, curveSegments: 6 });
  g.translate(0, 0, -(W - 2 * bev) / 2);
  return g;
}
const box = (sx, sy, sz, x, y, z) => new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z);
function quad(a, b, c, d) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute([...a, ...b, ...c, ...d], 3));
  g.setIndex([0, 1, 2, 0, 2, 3]); g.computeVertexNormals();
  g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(8), 2));
  return g;
}
function clean(g) {
  for (const k of Object.keys(g.attributes)) if (!['position', 'normal', 'uv'].includes(k)) g.deleteAttribute(k);
  if (!g.attributes.uv) g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(g.attributes.position.count * 2), 2));
  if (g.index) g = g.toNonIndexed();
  g.clearGroups(); return g;
}

function parts(key) {
  const b = BLUEPRINTS[key], L = b.L, W = b.W, H = b.H, L2 = L / 2;
  const P = { paint: [], glass: [], dark: [], head: [], tail: [], ind: [], box: [], indL: [], indR: [] };
  let r = b.r * 1.08, wheelX;
  if (key === 'truck') {
    const clr = 0.42, cabL = 2.35;
    P.paint.push(extrude(roundRect(L2 - cabL, clr, L2, 3.0, 0.3), W, 0.1));
    P.glass.push(quad([L2 + 0.005, 1.9, -W / 2 + 0.2], [L2 + 0.005, 1.9, W / 2 - 0.2], [L2 + 0.005, 2.75, W / 2 - 0.2], [L2 + 0.005, 2.75, -W / 2 + 0.2]));
    for (const sd of [1, -1]) P.glass.push(quad([L2 - 0.35, 1.9, sd * (W / 2 + 0.005)], [L2 - 1.3, 1.9, sd * (W / 2 + 0.005)], [L2 - 1.3, 2.7, sd * (W / 2 + 0.005)], [L2 - 0.35, 2.7, sd * (W / 2 + 0.005)]));
    P.box.push(extrude(roundRect(-L2, 0.95, L2 - cabL - 0.15, H, 0.12), W, 0.06));
    P.dark.push(box(L - 0.3, 0.35, W * 0.8, 0, 0.75, 0), box(0.12, 0.25, W, L2 + 0.04, 0.6, 0), box(0.05, 0.35, W * 0.7, L2 + 0.01, 1.25, 0));
    wheelX = [L2 - 1.3, -L2 + 1.6, -L2 + 2.95];
    for (const sd of [1, -1]) {
      P.head.push(box(0.05, 0.2, 0.34, L2 + 0.02, 0.95, sd * (W / 2 - 0.32)));
      P.tail.push(box(0.05, 0.16, 0.3, -L2 - 0.02, 1.15, sd * (W / 2 - 0.3)));
      const fi = box(0.08, 0.14, 0.14, L2 + 0.02, 0.95, sd * (W / 2 - 0.08)), ri = box(0.05, 0.14, 0.14, -L2 - 0.02, 1.4, sd * (W / 2 - 0.12));
      P.ind.push(fi.clone(), ri.clone()); (sd > 0 ? P.indR : P.indL).push(box(0.1, 0.16, 0.16, L2 + 0.02, 0.95, sd * (W / 2 - 0.08)), box(0.07, 0.16, 0.16, -L2 - 0.02, 1.4, sd * (W / 2 - 0.12)));
    }
    r = 0.52;
  } else {
    const clr = Math.max(0.16, b.clr + 0.04);
    const bodyTop = clr + (H * BODY_H[key]);
    P.paint.push(extrude(roundRect(-L2, clr, L2, bodyTop, 0.22), W, 0.09));
    const [br, bf, tr, tf] = CAB[key].map(f => f * L);
    const cab = polyShape([[br, bodyTop - 0.05], [bf, bodyTop - 0.05], [tf, H], [tr, H]]);
    P.paint.push(extrude(cab, W * 0.88, 0.07));
    // glass: windscreen, rear window, side windows (slightly outside the cabin)
    const hw = W * 0.88 / 2, y0 = bodyTop + 0.03, y1 = H - 0.07;
    const xAt = (xa, xb, y) => xa + (xb - xa) * (y - (bodyTop - 0.05)) / (H - bodyTop + 0.05);
    const ws0 = xAt(bf, tf, y0), ws1 = xAt(bf, tf, y1), rw0 = xAt(br, tr, y0), rw1 = xAt(br, tr, y1);
    const off = 0.012;
    P.glass.push(quad([ws0 + off, y0, -hw + 0.07], [ws0 + off, y0, hw - 0.07], [ws1 + off, y1, hw - 0.1], [ws1 + off, y1, -hw + 0.1]));
    P.glass.push(quad([rw0 - off, y0, hw - 0.07], [rw0 - off, y0, -hw + 0.07], [rw1 - off, y1, -hw + 0.1], [rw1 - off, y1, hw - 0.1]));
    for (const sd of [1, -1]) {
      const z = sd * (hw + 0.004), a = [rw0 + 0.12, y0], bq = [ws0 - 0.1, y0], c = [ws1 - 0.08, y1 - 0.02], d = [rw1 + 0.1, y1 - 0.02];
      const pts = sd > 0 ? [a, bq, c, d] : [bq, a, d, c];
      P.glass.push(quad([pts[0][0], pts[0][1], z], [pts[1][0], pts[1][1], z], [pts[2][0], pts[2][1], z], [pts[3][0], pts[3][1], z]));
      // B-pillar
      const xb = (ws0 + rw0) / 2 + (key === 'van' ? 0.3 : 0);
      P.paint.push(box(0.09, y1 - y0, 0.02, xb, (y0 + y1) / 2, sd * (hw + 0.01)));
      // lights
      P.head.push(box(0.05, 0.11, 0.36, L2 + 0.01, bodyTop - 0.16, sd * (W / 2 - 0.3)));
      P.tail.push(box(0.05, 0.12, 0.34, -L2 - 0.01, bodyTop - 0.14, sd * (W / 2 - 0.28)));
      const fi = box(0.06, 0.08, 0.12, L2 - 0.01, bodyTop - 0.16, sd * (W / 2 - 0.05)), ri = box(0.06, 0.08, 0.12, -L2 + 0.01, bodyTop - 0.14, sd * (W / 2 - 0.05));
      P.ind.push(fi.clone(), ri.clone()); (sd > 0 ? P.indR : P.indL).push(box(0.07, 0.09, 0.13, L2 - 0.005, bodyTop - 0.16, sd * (W / 2 - 0.05)), box(0.07, 0.09, 0.13, -L2 + 0.005, bodyTop - 0.14, sd * (W / 2 - 0.05)));
      // mirrors
      P.dark.push(box(0.1, 0.09, 0.14, ws0 - 0.05, y0 + 0.05, sd * (W / 2 + 0.06)));
    }
    P.dark.push(box(0.14, 0.18, W * 0.96, L2 - 0.03, clr + 0.1, 0), box(0.14, 0.18, W * 0.96, -L2 + 0.03, clr + 0.1, 0));   // bumpers
    P.dark.push(box(0.03, 0.12, W * 0.42, L2 + 0.01, bodyTop - 0.3, 0));                                                     // grille
    if (key === 'suv') for (const sd of [1, -1]) P.dark.push(box(L * 0.45, 0.04, 0.05, (tr + tf) / 2, H + 0.03, sd * hw * 0.8)); // roof rails
    wheelX = [L2 - b.fo, L2 - b.fo - b.wb];
  }
  return { P, wheelX, r, tw: key === 'truck' ? 0.34 : 0.27 };
}

const cache = {};
function wheelGeo(r, tw) {
  const tyre = new THREE.CylinderGeometry(r, r, tw, 20).rotateX(Math.PI / 2);
  const hub = new THREE.CylinderGeometry(r * 0.55, r * 0.55, 0.04, 16).rotateX(Math.PI / 2).translate(0, 0, tw / 2);
  const cap = new THREE.CylinderGeometry(r * 0.15, r * 0.15, 0.06, 10).rotateX(Math.PI / 2).translate(0, 0, tw / 2 + 0.02);
  return { tyre, hub: mergeGeometries([clean(hub), clean(cap)]) };
}

// Build a toy vehicle. style: 'toy' | 'ghost'. opts.separateWheels for the ego (steering + rolling)
export function buildToy(key, colorHex, style = 'toy', opts = {}) {
  const ck = key;
  if (!cache[ck]) cache[ck] = parts(key);
  const { P, wheelX, r, tw } = cache[ck];
  const b = BLUEPRINTS[key], m = mats();
  const g = new THREE.Group();
  const ghost = style === 'ghost';
  const merged = (list) => mergeGeometries(list.map(x => clean(x.clone())));
  const add = (list, mat, cast = true) => { if (!list.length) return null; const mesh = new THREE.Mesh(merged(list), mat); mesh.castShadow = cast && !ghost; g.add(mesh); return mesh; };
  if (ghost) {
    const body = [...P.paint, ...P.box];
    const geo = merged(body);
    g.add(new THREE.Mesh(geo, ghostMat(colorHex, 0.22)));
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo, 25), new THREE.LineBasicMaterial({ color: colorHex }));
    g.add(edges);
    add(P.glass, ghostMat(colorHex, 0.45), false);
    add([...P.head], ghostMat(0xffffff, 0.8), false); add(P.tail, ghostMat(0xff4040, 0.8), false);
  } else {
    add(P.paint, toyPaint(colorHex)); add(P.box, m.box); add(P.glass, m.glass, false); add(P.dark, m.dark);
    add(P.head, m.head, false); const tl = add(P.tail, opts.ownTail ? (g.userData.tail = m.tail.clone()) : m.tail, false); void tl;
    add(P.ind, m.ind, false);
  }
  // lit indicators (visibility toggled by the blinker logic)
  const indL = new THREE.Mesh(merged(P.indL), m.indOn), indR = new THREE.Mesh(merged(P.indR), m.indOn);
  indL.visible = indR.visible = false; g.add(indL, indR); g.userData.ind = { L: indL, R: indR };
  // wheels
  const wg = wheelGeo(r, tw);
  const wheelPos = []; for (const x of wheelX) for (const sd of [1, -1]) wheelPos.push([x, r, sd * (b.W / 2 - tw / 2 + 0.04), sd]);
  if (ghost) {
    const geos = wheelPos.map(([x, y, z, sd]) => { const t = wg.tyre.clone(); if (sd < 0) t.rotateY(Math.PI); return clean(t.translate(x, y, z)); });
    const geo = mergeGeometries(geos);
    g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo, 30), new THREE.LineBasicMaterial({ color: colorHex, transparent: true, opacity: 0.7 })));
    g.userData.wheels = [];
  } else if (opts.separateWheels) {
    g.userData.wheels = wheelPos.map(([x, y, z, sd]) => {
      const pivot = new THREE.Group(); pivot.position.set(x, y, z);
      const flip = new THREE.Group(); if (sd < 0) flip.rotation.y = Math.PI;
      const spin = new THREE.Group();
      spin.add(new THREE.Mesh(wg.tyre, m.tyre), new THREE.Mesh(wg.hub, m.hub));
      flip.add(spin); pivot.add(flip); g.add(pivot);
      return { pivot, spin, front: x > 0, side: sd };
    });
  } else {
    const tyres = [], hubs = [];
    for (const [x, y, z, sd] of wheelPos) {
      const mtx = new THREE.Matrix4().makeTranslation(x, y, z); if (sd < 0) mtx.multiply(new THREE.Matrix4().makeRotationY(Math.PI));
      tyres.push(clean(wg.tyre.clone().applyMatrix4(mtx))); hubs.push(clean(wg.hub.clone().applyMatrix4(mtx)));
    }
    add(tyres, m.tyre); add(hubs, m.hub, false);
    g.userData.wheels = [];
  }
  // soft contact shadow
  if (!ghost) { const sh = new THREE.Mesh(new THREE.PlaneGeometry(b.L + 0.5, b.W + 0.5).rotateX(-Math.PI / 2).translate(0, 0.02, 0), m.shadow); sh.renderOrder = -1; g.add(sh); }
  g.userData.bp = b;
  return g;
}
