// Headless check of the firmware demo on scenes/p13_demo.xml (the Hanoi scene, p13_web.xml, is tested by test_hanoi.mjs)
// Headless check of the browser sim (Node >= 20, no browser): MuJoCo WebAssembly + src/demo.js run one loop of the firmware's
// demo on the light model, compared row by row with the Python replay of the same model (public/reference.json, written by
// sim/export_web.py). Also reports how fast the WebAssembly physics runs here.
import load_mujoco from '@mujoco/mujoco';
import { readFileSync, readdirSync } from 'fs';
import { DemoRunner } from './src/demo.js';

const mj = await load_mujoco();
mj.FS.mkdir('/working'); mj.FS.mkdir('/working/assets');
mj.FS.writeFile('/working/p13_demo.xml', readFileSync('public/scenes/p13_demo.xml'));
for (const f of readdirSync('public/scenes/assets')) mj.FS.writeFile('/working/assets/' + f, readFileSync('public/scenes/assets/' + f));
const model = mj.MjModel.mj_loadXML('/working/p13_demo.xml'), data = new mj.MjData(model);
const demo = JSON.parse(readFileSync('public/demo.json')), ref = JSON.parse(readFileSync('public/reference.json')).rows;
const run = new DemoRunner(mj, model, data, demo);

let fail = false;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) fail = true; };
const ends = [];
const startRow = run.startRow.bind(run);
run.startRow = i => {
  if (run.row >= 0 && run.cycle === 0) {
    const b = run.ball;   // the firmware demo's ball (scenes/p13_demo.xml); the Hanoi scene is p13_web.xml
    ends.push({ row: run.row, t: run.t, q: run.jq.map(a => data.qpos[a]), link1: data.qpos[run.link1q],
                ball: [data.xpos[3 * b], data.xpos[3 * b + 1], data.xpos[3 * b + 2]] });
  }
  startRow(i);
};
const w0 = performance.now();
while (run.cycle === 0) run.step();
const wall = (performance.now() - w0) / 1000;

check(ends.length === ref.length, `${ends.length} rows run, reference has ${ref.length}`);
let dt = 0, dq = 0, dl = 0, db = 0, worst = '';
for (let k = 0; k < Math.min(ends.length, ref.length); k++) {
  const a = ends[k], r = ref[k];
  dt = Math.max(dt, Math.abs(a.t - r.t));
  dq = Math.max(dq, ...a.q.map((v, i) => Math.abs(v - r.q[i])));
  dl = Math.max(dl, Math.abs(a.link1 - r.link1));
  const e = Math.hypot(...a.ball.map((v, i) => v - r.ball[i]));
  if (e > db) { db = e; worst = `row ${k}`; }
}
// the reference is the last 10 ms log sample of each row, so times agree to 10 ms; joints are at rest then
check(dt <= 0.011, `row end times match the Python replay: max difference ${(dt * 1000).toFixed(1)} ms (log step 10 ms)`);
check(dq < 2e-3, `arm joints at each row end: max difference ${(dq * 180 / Math.PI).toFixed(3)} deg`);
check(dl < 0.05, `gripper link 1 at each row end: max difference ${(dl * 180 / Math.PI).toFixed(2)} deg`);
check(db < 0.010, `ball position at each row end: max difference ${(db * 1000).toFixed(1)} mm (${worst})`);
console.log(`one loop = ${run.t.toFixed(1)} s simulated in ${wall.toFixed(1)} s: ${(run.t / wall).toFixed(1)}x real time in Node's WebAssembly`);
process.exit(fail ? 1 : 0);
