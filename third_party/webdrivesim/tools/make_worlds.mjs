// Regenerates the 4 example worlds in maps/webdrivesim/ with the same code the simulator uses.
// Only needed if you change the simulator code. Run (Node.js 18+):  node third_party/webdrivesim/tools/make_worlds.mjs
import fs from 'fs';
const OUT = new URL('../../../maps/webdrivesim/', import.meta.url).pathname;   // maps/webdrivesim/
import { presetRoad, buildRoad } from '../web/src/road.js';
import { Sim, makeSensors, SENSOR_PRESETS, SENSOR_LIB } from '../web/src/sim.js';
import { BLUEPRINTS } from '../web/src/carModel.js';
import { scenarioObject, toXODR } from '../web/src/export.js';

const base = (preset, vehicles, extra = {}) => {
  const road = { ...presetRoad(preset), wet: false, rails: true, scenery: true };
  return {
    version: 2, name: '', vehicle: { bp: 'sedan', color: '#2563eb' }, sensors: makeSensors('sedan', SENSOR_PRESETS.overtake.ids), noise: 'L1', road,
    env: { style: 'toy', time: 'day', fog: false }, ego: { lane: road.nRight, s: 60, speed: 80 },
    traffic: { vehicles, nSame: 0, nOncoming: 0, mix: { hatch: 25, sedan: 30, suv: 20, van: 10, truck: 15 }, vMin: 70, vMax: 100, behaviour: 'idm', laneChanges: true, seed: 42 },
    drive: { mode: 'ros', setSpeed: 90, timeGap: 1.6, allowOvertake: true, allowOncoming: true, policy: 'normal' },
    ros: { url: 'ws://localhost:9090', camera: true, sync: false, auto: true },
    view: { fov: true, det: true, lidar: true, tracks: true, path: true, pip: true, plots: true, help: true }, ...extra };
};
const worlds = {
  empty_highway: base('oval', [], { ego: { lane: 2, s: 60, speed: 0 } }),
  rural_overtake: (() => { const c = base('rural', [{ type: 'truck', lane: 1, ds: 110, speed: 60, behaviour: 'idm' }, { type: 'sedan', lane: -1, ds: 520, speed: 85, behaviour: 'idm' }, { type: 'van', lane: -1, ds: 1150, speed: 80, behaviour: 'idm' }]); c.traffic.nSame = 4; c.traffic.nOncoming = 6; return c; })(),
  motorway_overtake: (() => { const c = base('motorway', [{ type: 'truck', lane: 3, ds: 110, speed: 80, behaviour: 'idm' }, { type: 'sedan', lane: 2, ds: -90, speed: 135, behaviour: 'idm' }, { type: 'suv', lane: 2, ds: 260, speed: 110, behaviour: 'idm' }]); c.traffic.nSame = 14; c.ego.speed = 100; c.drive.setSpeed = 120; return c; })(),
  superstrada_traffic: (() => { const c = base('superstrada', [{ type: 'truck', lane: 2, ds: 120, speed: 70, behaviour: 'idm' }]); c.traffic.nSame = 12; c.drive.setSpeed = 110; return c; })(),
};
fs.mkdirSync(OUT, { recursive: true });
for (const [name, cfg] of Object.entries(worlds)) {
  cfg.name = name;
  const road = buildRoad(cfg.road), sim = new Sim(cfg, road);
  const obj = scenarioObject(cfg, road, sim, name, { BLUEPRINTS, SENSOR_LIB });
  fs.writeFileSync(`${OUT}${name}.json`, JSON.stringify(obj, null, 2));
  fs.writeFileSync(`${OUT}${name}.xodr`, toXODR(road, name));
  console.log(name, (road.length / 1000).toFixed(1) + ' km', sim.traffic.length + ' vehicles');
}
