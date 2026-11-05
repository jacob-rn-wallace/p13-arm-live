// Headless checks of the ball-sort puzzle (Node >= 20): the solver on every arrangement, then full solves in MuJoCo physics for seeded random
// layouts: the planner's rows are played through DemoRunner and every ball must end on its seat in the sorted order, upright on the seat, none dropped.
// usage: node test_ballsort.mjs [seeds=6 | all]   (all = every one of the 24 arrangements, ~1 hour)
import load_mujoco from '@mujoco/mujoco';
import { readFileSync, readdirSync } from 'fs';
import * as P from './src/planner.js';
import { DemoRunner } from './src/demo.js';
import { pieceBodies, placePieces, readPieces } from './src/puzzle.js';

let fail = false;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) fail = true; };
const bj = JSON.parse(readFileSync('public/ballsort.json')), demo = JSON.parse(readFileSync('public/demo.json'));
const w = P.makeWorld(bj), n = bj.balls.length, slots = w.slotNames;

// ---- solver: all 24 ways to put 3 balls on 4 seats
let all = 0, good = 0, longest = 0;
const perms = []; for (const a of slots) for (const b of slots) for (const c of slots) if (a !== b && a !== c && b !== c) perms.push([a, b, c]);
for (const slotOf of perms) {
  all++; const moves = P.solveSort(slotOf, bj.order, slots), pos = slotOf.slice(); let legal = true;
  for (const m of moves) { if (pos[m.bead] !== m.from || pos.includes(m.to)) legal = false; pos[m.bead] = m.to; }
  if (legal && pos.every((s, i) => s === bj.order[i])) good++;
  longest = Math.max(longest, moves.length);
}
check(good === all, `solver: ${good}/${all} arrangements end sorted by legal moves into empty seats (longest ${longest} moves)`);

// ---- physics
const mj = await load_mujoco(); mj.FS.mkdir('/working'); mj.FS.mkdir('/working/assets');
mj.FS.writeFile('/working/p13_balls.xml', readFileSync('public/scenes/p13_balls.xml'));
for (const f of readdirSync('public/scenes/assets')) mj.FS.writeFile('/working/assets/' + f, readFileSync('public/scenes/assets/' + f));
const model = mj.MjModel.mj_loadXML('/working/p13_balls.xml'), data = new mj.MjData(model);
const pieces = pieceBodies(mj, model, n), want = P.layoutPoses(w, bj.order.slice(0, n)).pos;
const arg = process.argv[2] || '6';
const cases = arg === 'all' ? perms.filter(p => p.join(',') !== bj.order.slice(0, 3).join(',')).map((p, k) => [k + 1, p]) : Array.from({ length: +arg }, (_, k) => [k + 1, P.randomSortLayout(P.mulberry32(k + 1), bj.order, slots)]);
for (const [seed, slotOf] of cases) {
  const moves = P.solveSort(slotOf, bj.order, slots);
  const plan = P.plan(w, slotOf, null, demo.q0, undefined, { moves });
  const minClear = Math.min(...plan.checks.map(c => c.d));
  const run = new DemoRunner(mj, model, data, { ...demo, rows: plan.rows }, { balls: pieces.map((_, i) => 'bead' + i), loop: false });
  run.reset(); placePieces(mj, model, data, w, slotOf, pieces);
  let dropped = false;
  while (!run.done && run.t < 3000) { run.step(); if (run.row % 4 === 0 && readPieces(model, data, w, pieces).some(p => p.z < 40)) dropped = true; }
  const fin = readPieces(model, data, w, pieces, 30);
  const placed = fin.every((p, i) => p.slot === bj.order[i] && Math.hypot(p.x - want[i][0], p.y - want[i][1]) < 15 && Math.abs(p.z - want[i][2]) < 6);
  check(placed && !dropped, `seed ${seed}: ${slotOf.join('')} -> ${bj.order.slice(0, 3).join('')} in ${moves.length} moves (${plan.rows.length} rows, ${run.t.toFixed(0)} s simulated, tightest planned clearance ${minClear.toFixed(1)} mm)` +
    (placed ? '' : ' final ' + fin.map((p, i) => `${i}:${p.slot || '-'}(${p.x.toFixed(0)},${p.y.toFixed(0)},${p.z.toFixed(0)})`).join(' ')));
}
process.exit(fail ? 1 : 0);
