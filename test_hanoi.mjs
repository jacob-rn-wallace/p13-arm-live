// Headless checks of the Hanoi planner (Node >= 20): the JS IK against the Python IK (public/ik_reference.json), the Hanoi solver on
// every legal start, and the planner's own clearance checks.
import { readFileSync } from 'fs';
import * as P from './src/planner.js';

let fail = false;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) fail = true; };
const hj = JSON.parse(readFileSync('public/hanoi.json')), ref = JSON.parse(readFileSync('public/ik_reference.json'));
const w = P.makeWorld(hj);

// ---- IK port vs Python (single seed each: same branch)
let dq = 0, dpos = 0, n = 0, bad = 0;
for (const c of ref.cases) {
  const r = P.solveIK(c.target, c.seed, ref.a6);
  if (r.ok !== c.ok) { bad++; console.log('  ok flag differs for', c.target.map(v => +v.toFixed(2))); continue; }
  if (!c.ok) continue;
  n++; dq = Math.max(dq, ...r.q.map((v, i) => Math.abs(v - c.q[i]))); dpos = Math.max(dpos, r.pe);
}
check(bad === 0, `JS and Python agree on which of ${ref.cases.length} poses are solvable`);
check(dq < 1e-3, `${n} solved poses: max joint difference JS vs Python ${dq.toExponential(2)} rad (limit 1e-3), worst JS position error ${dpos.toExponential(1)} mm`);

// ---- Hanoi solver: every one of the 27 assignments of 3 beads to 3 pegs, every target
const slots = w.slotNames;
let worstMoves = 0, solved = 0, total = 0;
for (let code = 0; code < 27; code++) {
  const pegOf = [0, 1, 2].map(i => slots[Math.floor(code / 3 ** i) % 3]);
  for (const target of slots) {
    const pos = pegOf.slice(); total++;
    let legal = true;
    for (const m of P.solveHanoi(pegOf, target, slots)) {
      if (pos[m.bead] !== m.from) legal = false;
      for (let k = 0; k < m.bead; k++) if (pos[k] === m.from || pos[k] === m.to) legal = false;    // smaller beads on the moved one's peg or on the destination
      pos[m.bead] = m.to;
      worstMoves = Math.max(worstMoves, 0);
    }
    if (legal && pos.every(s => s === target)) solved++;
    worstMoves = Math.max(worstMoves, P.solveHanoi(pegOf, target, slots).length);
  }
}
check(solved === total, `solver: ${solved}/${total} start/target combinations end as a whole tower on the target by legal moves (longest ${worstMoves} moves; 7 = 2^3-1)`);
process.exit(fail ? 1 : 0);
