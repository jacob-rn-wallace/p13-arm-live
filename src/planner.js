// Planner for the Hanoi bead puzzle (sim/hanoi_layout.py, public/hanoi.json): joint-limited IK (a port of
// mae451-arm/arm_firmware_P13/matlab/ik_bounded.py, checked against it by test_hanoi.mjs), the classic Hanoi solver for any legal
// start, a random layout generator, and a move generator that emits rows in demo.json's format
// ({q: 6 rad, grip: 0/1, note}) after checking every straight joint-space segment against the table, pedestals and coins
// with the hand's point cloud (as mae451-arm/.../build_sequence.py does for the ball demo).
// Units: mm and rad in here (the MuJoCo model is in m).

const rad = d => d * Math.PI / 180;
// DH values of the firmware (dh_parameters.cpp); a6 comes from hanoi.json
const D1 = 103, A2 = 40.65, B2 = 191, A3 = 192, B3 = 18.75, A4 = 69.45, A5 = 28.65, D5 = -54.55;
export const LO = [-92, -90, -74, -90, -90, -90].map(rad), HI = [92, 43, 90, 90, 90, 90].map(rad);
export const MARGIN = rad(8);
export const LO_M = LO.map(v => v + MARGIN), HI_M = HI.map(v => v - MARGIN);

// ---------------------------------------------------------------- kinematics
const mul4 = (a, b) => {
  const r = new Array(16).fill(0);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) { let s = 0; for (let k = 0; k < 4; k++) s += a[4 * i + k] * b[4 * k + j]; r[4 * i + j] = s; }
  return r;
};
export function fk(q, a6) {
  const [q1, q2, q3, q4, q5, q6] = q, c = Math.cos, s = Math.sin;
  const A = [
    [c(q1), 0, s(q1), 0, s(q1), 0, -c(q1), 0, 0, 1, 0, D1, 0, 0, 0, 1],
    [c(q2), -s(q2), 0, A2 * c(q2) - B2 * s(q2), s(q2), c(q2), 0, A2 * s(q2) + B2 * c(q2), 0, 0, 1, 0, 0, 0, 0, 1],
    [c(q3), -s(q3), 0, A3 * c(q3) - B3 * s(q3), s(q3), c(q3), 0, A3 * s(q3) + B3 * c(q3), 0, 0, 1, 0, 0, 0, 0, 1],
    [c(q4), 0, -s(q4), A4 * c(q4), s(q4), 0, c(q4), A4 * s(q4), 0, -1, 0, 0, 0, 0, 0, 1],
    [c(q5), -s(q5), 0, A5 * c(q5), s(q5), c(q5), 0, A5 * s(q5), 0, 0, 1, D5, 0, 0, 0, 1],
    [1, 0, 0, a6, 0, c(q6), -s(q6), 0, 0, s(q6), c(q6), 0, 0, 0, 0, 1]];
  let T = A[0];
  for (let i = 1; i < 6; i++) T = mul4(T, A[i]);
  return T;       // row-major 4x4
}
const Rzyx = (y, p, r) => {
  const cy = Math.cos(y), sy = Math.sin(y), cp = Math.cos(p), sp = Math.sin(p), cr = Math.cos(r), sr = Math.sin(r);
  return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr];
};

function solveLinear(M, b, n) {     // Gaussian elimination with partial pivoting; M is n x n row-major (copied)
  const a = M.slice(), x = b.slice();
  for (let i = 0; i < n; i++) {
    let p = i; for (let r = i + 1; r < n; r++) if (Math.abs(a[r * n + i]) > Math.abs(a[p * n + i])) p = r;
    if (p !== i) { for (let k = 0; k < n; k++) { const t = a[i * n + k]; a[i * n + k] = a[p * n + k]; a[p * n + k] = t; } const t = x[i]; x[i] = x[p]; x[p] = t; }
    const d = a[i * n + i] || 1e-12;
    for (let r = i + 1; r < n; r++) { const f = a[r * n + i] / d; if (f) { for (let k = i; k < n; k++) a[r * n + k] -= f * a[i * n + k]; x[r] -= f * x[i]; } }
  }
  for (let i = n - 1; i >= 0; i--) { let s = x[i]; for (let k = i + 1; k < n; k++) s -= a[i * n + k] * x[k]; x[i] = s / (a[i * n + i] || 1e-12); }
  return x;
}

// Position + orientation IK, the same residual as ik_bounded.solve: [p - p*, 100 (R - R*)] over six joints, within [lo, hi].
// Levenberg-Marquardt with a finite-difference Jacobian and the step clamped to the limits. target = [x y z yaw pitch roll] (mm, rad).
export function solveIK(target, seed, a6, lo = LO_M, hi = HI_M, maxIter = 300) {
  const p = target.slice(0, 3), R = Rzyx(target[3], target[4], target[5]);
  const resid = q => {
    const T = fk(q, a6), r = new Array(12);
    r[0] = T[3] - p[0]; r[1] = T[7] - p[1]; r[2] = T[11] - p[2];
    const Rm = [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]];
    for (let i = 0; i < 9; i++) r[3 + i] = 100 * (Rm[i] - R[i]);
    return r;
  };
  const cost = r => r.reduce((s, v) => s + v * v, 0);
  const clampQ = q => q.map((v, i) => Math.min(hi[i] - 1e-6, Math.max(lo[i] + 1e-6, v)));
  let q = clampQ(seed), r = resid(q), f = cost(r), lambda = 1e-2;
  for (let it = 0; it < maxIter; it++) {
    const J = new Array(12 * 6);
    for (let j = 0; j < 6; j++) {
      const h = 1e-6, qj = q.slice(); qj[j] += h; const rj = resid(qj);
      for (let i = 0; i < 12; i++) J[i * 6 + j] = (rj[i] - r[i]) / h;
    }
    const JtJ = new Array(36).fill(0), Jtr = new Array(6).fill(0);
    for (let i = 0; i < 12; i++) for (let a = 0; a < 6; a++) { Jtr[a] += J[i * 6 + a] * r[i]; for (let b = 0; b < 6; b++) JtJ[a * 6 + b] += J[i * 6 + a] * J[i * 6 + b]; }
    let improved = false;
    for (let tries = 0; tries < 12; tries++) {
      const M = JtJ.slice(); for (let a = 0; a < 6; a++) M[a * 6 + a] += lambda * (JtJ[a * 6 + a] + 1e-9);
      const d = solveLinear(M, Jtr.map(v => -v), 6), qn = clampQ(q.map((v, i) => v + d[i])), rn = resid(qn), fn = cost(rn);
      if (fn < f) { const dq = Math.max(...qn.map((v, i) => Math.abs(v - q[i]))); q = qn; r = rn; const df = f - fn; f = fn; lambda = Math.max(lambda / 5, 1e-12); improved = true;
        if (dq < 1e-12 || df < 1e-18 * (1 + f)) return finish(q, f); break; }
      lambda *= 6;
    }
    if (!improved) break;
  }
  return finish(q, f);
  function finish(qq, ff) {
    const T = fk(qq, a6), pe = Math.hypot(T[3] - p[0], T[7] - p[1], T[11] - p[2]);
    const tr = [0, 1, 2].reduce((s, i) => s + [0, 1, 2].reduce((t, j) => t + T[4 * i + j] * R[3 * i + j], 0), 0);
    const oe = Math.acos(Math.min(1, Math.max(-1, (tr - 1) / 2))) * 180 / Math.PI;
    return { q: qq, pe, oe, ok: pe < 0.5 && oe < 0.5 };
  }
}

// ---------------------------------------------------------------- the world
// hanoi.json -> geometry the checks use. The pieces ("beads") are coins: a cylinder (radius R, half-thickness h) with a funnel in its top face
// and a collar (radius collar_r, height collar_h) under its bottom face; the collar stands in the funnel of the piece below (or the pedestal).
export function makeWorld(hj) {
  // ball-sort scene (ballsort.json): spheres on ring seats, one per seat, no collars
  if (hj.balls) hj = { ...hj, beads: hj.balls.map(b => ({ ...b, ball: true })), ped_top: hj.seat_top, ped_r: hj.seat_r, collar_h: 0, collar_r: 0, stack_top: hj.seat_top, mode: 'ball' };
  const pts = (flat) => { const n = flat.length / 3, a = new Float32Array(3 * n); for (let i = 0; i < n; i++) { a[3 * i] = flat[3 * i + 1]; a[3 * i + 1] = flat[3 * i]; a[3 * i + 2] = -flat[3 * i + 2]; } return a; };
  // hand-frame points as (axis, y6, z6): gripper Y -> x6, X -> y6, Z -> -z6 (build_sequence.hand_points)
  const w = { hj, a6: hj.a6, slots: hj.slots, beads: hj.beads, handOpen: pts(hj.cloud_open), handClosed: pts(hj.cloud_closed), rolls: hj.rolls };
  w.slotNames = Object.keys(hj.slots);
  w.stackTop = hj.stack_top;                        // top face of a full tower: nothing is above it
  // surface samples of a coin (relative to its centre): the held piece's collision proxy
  w.beadPts = hj.beads.map(b => {
    if (b.ball) { const out = [], n = 160, ga = Math.PI * (3 - Math.sqrt(5)); for (let i = 0; i < n; i++) { const z = (1 - 2 * (i + 0.5) / n) * b.R, r = Math.sqrt(b.R * b.R - z * z); out.push([r * Math.cos(i * ga), r * Math.sin(i * ga), z]); } return out; }
    const out = [], ring = (r, z) => { const n = Math.max(8, Math.round(r / 3)); for (let k = 0; k < n; k++) { const a = 2 * Math.PI * k / n; out.push([r * Math.cos(a), r * Math.sin(a), z]); } };
    for (const r of [13, 26, b.R]) { ring(r, b.h); ring(r, -b.h); }              // faces
    for (const z of [-b.h / 2, 0, b.h / 2]) ring(b.R, z);                          // rim
    for (const z of [-b.h - hj.collar_h, -b.h - hj.collar_h / 2]) ring(hj.collar_r, z);   // collar
    return out;
  });
  return w;
}

// Bead centre heights for a layout (pegOf[i] = slot of bead i): beads sit on a plinth, largest at the bottom.
export function layoutPoses(w, pegOf) {
  const out = new Array(pegOf.length), stacks = {}, hj = w.hj;
  for (const s of w.slotNames) stacks[s] = [];
  pegOf.forEach((s, i) => stacks[s].push(i));
  if (hj.mode === 'ball') { pegOf.forEach((s, i) => { out[i] = [w.slots[s][0], w.slots[s][1], w.beads[i].seat_z]; }); return { pos: out, stacks }; }
  for (const s of w.slotNames) {
    stacks[s].sort((a, b) => w.beads[b].D - w.beads[a].D);       // bottom first
    let prev = null, z = 0;
    for (const i of stacks[s]) {
      const b = w.beads[i];
      // the collar's bottom stands on the funnel floor of the piece below (or of the pedestal)
      z = prev === null ? hj.ped_floor + hj.collar_h + b.h : z + w.beads[prev].h - hj.funnel_d + hj.collar_h + b.h;
      out[i] = [w.slots[s][0], w.slots[s][1], z]; prev = i;
    }
  }
  return { pos: out, stacks };
}

// ---------------------------------------------------------------- distances from points to obstacles (mm; <= 0 means touching)
function distCyl(x, y, z, cx, cy, r, z0, z1) {
  const dr = Math.max(Math.hypot(x - cx, y - cy) - r, 0), dz = Math.max(z0 - z, z - z1, 0);
  return Math.hypot(dr, dz);
}
function distBead(x, y, z, b, c, hj) {            // coin centred on c, plus its collar: the funnel is ignored (conservative); or a ball
  if (b.ball) return Math.hypot(x - c[0], y - c[1], z - c[2]) - b.R;
  const body = distCyl(x, y, z, c[0], c[1], b.R, c[2] - b.h, c[2] + b.h);
  const collar = distCyl(x, y, z, c[0], c[1], hj.collar_r, c[2] - b.h - hj.collar_h, c[2] - b.h);
  return Math.min(body, collar);
}

// obstacles = {beads: [{i, c:[x,y,z]}]} besides the static table and pedestals. Returns the smallest distance and what it was.
function minClearance(w, P, n, obst, skipTable = false) {
  let best = Infinity, what = '';
  const note = (d, s) => { if (d < best) { best = d; what = s; } };
  const hj = w.hj, ph = hj.ped_top;
  for (let k = 0; k < n; k++) {
    const x = P[3 * k], y = P[3 * k + 1], z = P[3 * k + 2];
    if (!skipTable) note(z, 'table');
    for (const s of w.slotNames) {
      const [sx, sy] = w.slots[s];
      if (!obst.skipPed) note(distCyl(x, y, z, sx, sy, hj.ped_r, 0, ph), 'pedestal ' + s);
    }
    for (const bd of obst.beads) note(distBead(x, y, z, w.beads[bd.i], bd.c, hj), 'piece ' + bd.i);
  }
  return { d: best, what };
}

// hand cloud -> base frame at joint angles q (same transform as build_sequence.hand_points)
export function handPoints(w, q, closed) {
  const T = fk(q, w.a6), src = closed ? w.handClosed : w.handOpen, n = src.length / 3, out = new Float32Array(3 * n);
  const o = [T[3] - w.a6 * T[0], T[7] - w.a6 * T[4], T[11] - w.a6 * T[8]];
  for (let k = 0; k < n; k++) {
    const a = src[3 * k], b = src[3 * k + 1], c = src[3 * k + 2];
    out[3 * k] = T[0] * a + T[1] * b + T[2] * c + o[0];
    out[3 * k + 1] = T[4] * a + T[5] * b + T[6] * c + o[1];
    out[3 * k + 2] = T[8] * a + T[9] * b + T[10] * c + o[2];
  }
  return out;
}

// clearance along the straight joint-space line qa -> qb (what moveJointsTo() drives). `held`: bead index in the hand (closed hand,
// the bead's centre at `grip` mm from the tool point), or null for the open hand.
export function pathClearance(w, qa, qb, state, held = null, samples = 40, grip = [0, 0, 0], touching = []) {
  // `touching`: beads the closed hand is allowed to be next to (the stack the held bead was just taken from: the lower fingers wrap
  // round it by design; whether that works is up to the physics). They still count for the held bead itself.
  const beads = [], handBeads = [];
  state.pos.forEach((c, i) => { if (i !== held) { beads.push({ i, c }); if (!touching.includes(i)) handBeads.push({ i, c }); } });
  let best = { d: Infinity, what: '', f: 0 };
  for (let s = 0; s <= samples; s++) {
    const f = s / samples, q = qa.map((v, i) => v + f * (qb[i] - v));
    const P = handPoints(w, q, held !== null);
    let r = minClearance(w, P, P.length / 3, { beads: handBeads });
    if (held !== null) {                                        // the carried coin against pedestals and the others
      const T = fk(q, w.a6), c = [T[3] + grip[0], T[7] + grip[1], T[11] + grip[2]];
      const bp = w.beadPts[held], B = new Float32Array(3 * bp.length);
      bp.forEach((v, k) => { B[3 * k] = v[0] + c[0]; B[3 * k + 1] = v[1] + c[1]; B[3 * k + 2] = v[2] + c[2]; });
      const rb = minClearance(w, B, bp.length, { beads: handBeads, skipPed: touching.includes('ped') }, true);   // it rests on the touching bead at the start of a lift
      if (rb.d < r.d) r = { d: rb.d, what: 'held bead / ' + rb.what };
    }
    if (r.d < best.d) best = { d: r.d, what: r.what, f };
  }
  return best;
}

// ---------------------------------------------------------------- the Hanoi solver (any legal start -> whole tower on `target`)
// pegOf[i]: slot of bead i (0 smallest). Returns [{bead, from, to}], optimal for the classic goal.
export function solveHanoi(pegOf, target, slotNames) {
  const pos = pegOf.slice(), moves = [];
  const other = (a, b) => slotNames.find(s => s !== a && s !== b);
  (function tower(k, to) {
    if (k < 0) return;
    if (pos[k] === to) { tower(k - 1, to); return; }
    const from = pos[k], via = other(from, to);
    tower(k - 1, via); moves.push({ bead: k, from, to }); pos[k] = to; tower(k - 1, to);
  })(pegOf.length - 1, target);
  return moves;
}

// seeded random layouts: every bead on any peg (stacks come out size-ordered by construction), never already solved for `target`
export function mulberry32(seed) { let a = seed >>> 0; return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
export function randomLayout(rng, nBeads, slotNames, target) {
  for (;;) {
    const pegOf = Array.from({ length: nBeads }, () => slotNames[Math.floor(rng() * slotNames.length)]);
    if (!pegOf.every(s => s === target)) return pegOf;
  }
}

// ---------------------------------------------------------------- the move generator
export const SEEDS = { neg: [-1.2281, -0.3142, -1.1174, 1.4316, 1.2281, 0], pos: [1.2281, -0.3142, -1.1174, 1.4316, -1.2281, 0] };
// IK for a tool point (x, y, z) with the hand along +x and roll `roll`: from `prev`, then from the standard seeds
export function makeIk(w) {
  return (x, y, z, prev, tag, roll) => {
    let r = solveIK([x, y, z, 0, 0, 0], prev, w.a6);
    if (!r.ok) r = solveIK([x, y, z, 0, 0, 0], y < 0 ? SEEDS.neg : SEEDS.pos, w.a6);
    if (!r.ok) throw new Error(`IK failed for ${tag} at (${x.toFixed(0)}, ${y.toFixed(0)}, ${z.toFixed(0)}): ${r.pe.toFixed(1)} mm, ${r.oe.toFixed(1)} deg`);
    const out = r.q.slice(); out[5] = roll; return out;
  };
}
// joint poses down (or up) the vertical line through (x, y) from z0 (pose qa) to z1, excluding qa: moveJointsTo() drives each row on a straight
// joint-space line, so the line is split until it bows less than `tol` mm from the vertical
export function makeVline(w, ik) {
  return (x, y, qa, z0, z1, tol, tag, roll) => {
    const out = [], qb = ik(x, y, z1, qa, tag, roll);
    const rec = (qa_, za, qb_, zb) => {
      const qm = qa_.map((v, i) => (v + qb_[i]) / 2), T = fk(qm, w.a6);
      if (Math.hypot(T[3] - x, T[7] - y) > tol && Math.abs(za - zb) > 12) {
        const zm = (za + zb) / 2, qmid = ik(x, y, zm, qa_, tag, roll); rec(qa_, za, qmid, zm); rec(qmid, zm, qb_, zb);
      } else out.push(qb_);
    };
    rec(qa, z0, qb, z1);
    return out;
  };
}
export const CLEAR_MIN = 4;            // mm: smallest hand/bead/peg gap accepted along any segment
export const PLACE_DROP = 1;           // mm above the resting height where a coin is let go (it is seated before the fingers open)
export const ZMAX = 410;               // highest hover (tool point), mm: A, B and C = (440, 0) reach it

// plan(w, pegOf, target, q0, cal) -> {rows, moves, checks, final}. cal.grip = [dx, dy, dz]: where the carried bead's centre settles
// relative to the tool point (measured in the sim; the fingers pull a bead toward the palm). The hand picks with the bead centred on
// the tool point (best finger clearance) and places with the tool point shifted by -grip so the bead lands over the peg.
// moveJointsTo() drives each row on a straight joint-space line, which is not a vertical line in space: every vertical leg is split
// until the line bows less than TOL_PEG (2 mm, the play between hole and peg is 3 mm) where the bead is on the peg, TOL_AIR above it.
export const TOL_PEG = 2, TOL_AIR = 6;
export function plan(w, pegOf, target, q0, cal = { grip: [0, 0, 0] }, opts = {}) {
  const moves = (opts.moves || solveHanoi(pegOf, target, w.slotNames)).slice(0, opts.maxMoves ?? Infinity), rows = [], checks = [];
  const ballMode = w.hj.mode === 'ball', tolPeg = ballMode ? 8 : TOL_PEG, drop = ballMode ? 3 : PLACE_DROP;
  let state = layoutPoses(w, pegOf), pegs = pegOf.slice(), q = q0.slice();
  let g = cal.grip;      // per move: where the carried coin's collar bottom sits relative to the tool point (cal.gripPer[bead] overrides)
  const ik = makeIk(w), vline = makeVline(w, ik);
  let curMove = 0;
  const push = (qq, grip, note) => rows.push({ q: qq.map(v => Math.round(v * 1e6) / 1e6), grip, note, move: curMove });
  moves.forEach((m, mi) => {
    curMove = mi;
    g = (cal.gripPer && cal.gripPer[m.bead]) || (w.hj.grip_offsets && w.hj.grip_offsets[m.bead]) || cal.grip;
    const [sx, sy] = w.slots[m.from], [tx, ty] = w.slots[m.to], b = w.beads[m.bead];
    const pickZ = state.pos[m.bead][2];
    const after = layoutPoses(w, pegs.map((s, i) => i === m.bead ? m.to : s));
    const placeZ = after.pos[m.bead][2] + drop;
    const zLift = ballMode ? w.hj.seat_top + b.R + 12 : w.stackTop + b.h + w.hj.collar_h + 12;        // the carried coin's collar bottom clear of the tallest stack
    const zSplit = zLift + 6;                      // below this the bead is on (or just above) the peg
    const below = layoutPoses(w, pegs).stacks[m.from].filter(i => i !== m.bead), onTarget = layoutPoses(w, pegs).stacks[m.to].filter(i => i !== m.bead);
    const rest = { ...state, pos: state.pos.slice() }; rest.pos[m.bead] = after.pos[m.bead];
    let chosen = null, tried = [];
    for (let zh = opts.zMax ?? ZMAX; zh >= Math.ceil(zLift / 10) * 10 && !chosen; zh -= 10) {
      try {
        const T0 = fk(q, w.a6), qRise = T0[11] < zh - 1 ? ik(T0[3], T0[7], zh, q, 'rise', q[5]) : null;
        const rollS = w.rolls[m.from], rollT = w.rolls[m.to];
        const qHp = ik(sx, sy, zh, qRise || q, 'hover over ' + m.from, rollS);
        const pickDown = [...vline(sx, sy, qHp, zh, Math.max(zSplit, pickZ), TOL_AIR, 'down to the bead', rollS)];
        if (pickZ < zSplit) pickDown.push(...vline(sx, sy, pickDown[pickDown.length - 1], zSplit, pickZ, tolPeg, 'down to the bead', rollS));
        const qPick = pickDown[pickDown.length - 1];
        const qHt = ik(tx - g[0], ty - g[1], zh - g[2], qHp, 'hover over ' + m.to, rollT);
        const lowZ = Math.max(zSplit - g[2], placeZ - g[2]);
        const placeDown = [...vline(tx - g[0], ty - g[1], qHt, zh - g[2], lowZ, TOL_AIR, 'lower onto the peg', rollT)];
        if (placeZ - g[2] < lowZ - 1) placeDown.push(...vline(tx - g[0], ty - g[1], placeDown[placeDown.length - 1], lowZ, placeZ - g[2], tolPeg, 'lower onto the peg', rollT));
        const qPlace = placeDown[placeDown.length - 1];
        const segs = [];                              // [state, qa, qb, held, what, grip, touching]
        if (qRise) segs.push([state, q, qRise, null, 'rise at the start']);
        segs.push([state, qRise || q, qHp, null, 'transit over the source peg']);
        let prev = qHp; for (const qq of pickDown) { segs.push([state, prev, qq, null, 'descend to the bead']); prev = qq; }
        const upList = [qHp, ...pickDown.slice(0, -1)].reverse(); prev = qPick;
        for (const qq of upList) { segs.push([state, prev, qq, m.bead, 'lift', [0, 0, 0], [...below, 'ped']]); prev = qq; }
        segs.push([state, qHp, qHt, m.bead, 'carry', g]);
        prev = qHt; for (const qq of placeDown) { segs.push([state, prev, qq, m.bead, 'lower', g, [...onTarget, 'ped']]); prev = qq; }
        const risePath = [qHt, ...placeDown.slice(0, -1)].reverse(); prev = qPlace;
        for (const qq of risePath) { segs.push([rest, prev, qq, null, 'rise after release']); prev = qq; }
        let ok = true;
        for (const [st, qa, qb, held, what, grip, touching] of segs) {
          const r = pathClearance(w, qa, qb, st, held, 24, grip || [0, 0, 0], touching || []);
          checks.push({ move: mi + 1, what, hover: zh, d: r.d, hit: r.what });
          if (r.d < (opts.clearMin ?? CLEAR_MIN)) { ok = false; tried.push(`${what} at hover ${zh}: ${r.d.toFixed(1)} mm to ${r.what} (at ${(100 * r.f).toFixed(0)}% of the segment, tool ${fk(qa.map((v, i) => v + r.f * (qb[i] - v)), w.a6).filter((_, k) => [3, 7, 11].includes(k)).map(v => v.toFixed(0)).join(',')})`); break; }
        }
        if (ok) chosen = { zh, qRise, qHp, pickDown, qHt, placeDown, risePath: risePath.slice(1) };
      } catch (e) { tried.push(e.message); }
    }
    if (!chosen) throw new Error(`move ${mi + 1} (bead ${m.bead} ${m.from}->${m.to}) has no clear path; first failures: ${tried.slice(0, 3).join('; ')}`);
    const { zh, qRise, qHp, pickDown, qHt, placeDown, risePath } = chosen, tag = `bead ${m.bead} ${m.from}->${m.to}`;
    if (qRise) push(qRise, 0, `${tag}: rise to hover height`);
    push(qHp, 0, `${tag}: over ${m.from} [${sx}, ${sy}, ${zh}]`);
    pickDown.forEach((qq, k) => push(qq, 0, `${tag}: down to the bead on ${m.from}`));
    const qPick = pickDown[pickDown.length - 1];
    push(qPick, 1, `${tag}: close the gripper`);
    [...pickDown.slice(0, -1)].reverse().forEach(qq => push(qq, 1, `${tag}: lift clear of the peg`));
    push(qHp, 1, `${tag}: lift clear of the peg`);
    push(qHt, 1, `${tag}: carry over ${m.to} [${tx}, ${ty}, ${zh}]`);
    placeDown.forEach(qq => push(qq, 1, `${tag}: lower onto ${m.to}`));
    const qPlace = placeDown[placeDown.length - 1];
    push(qPlace, 0, `${tag}: open the gripper`);
    [...placeDown.slice(0, -1)].reverse().forEach(qq => push(qq, 0, `${tag}: rise clear`));
    push(qHt, 0, `${tag}: rise clear`);
    q = qHt; pegs[m.bead] = m.to; state = layoutPoses(w, pegs);
  });
  return { rows, moves, checks, final: pegs };
}

// ---------------------------------------------------------------- the ball-sort puzzle
// slotOf[i] = seat of ball i (0 smallest); the goal is ball i on order[i] (the first seats in y order). Breadth-first search over the
// arrangements for the fewest moves, each move taking one ball to an empty seat. Returns [{bead, from, to}].
export function solveSort(slotOf, order, slotNames) {
  const n = slotOf.length, goal = order.slice(0, n).join(','), key = a => a.join(',');
  const seen = new Map([[key(slotOf), null]]), queue = [slotOf.slice()];
  for (let h = 0; h < queue.length; h++) {
    const cur = queue[h];
    if (key(cur) === goal) {
      const moves = []; let k = key(cur);
      while (seen.get(k)) { const { prev, move } = seen.get(k); moves.push(move); k = prev; }
      return moves.reverse();
    }
    const used = new Set(cur);
    for (let i = 0; i < n; i++) for (const s of slotNames) {
      if (used.has(s)) continue;
      const next = cur.slice(); next[i] = s; const nk = key(next);
      if (!seen.has(nk)) { seen.set(nk, { prev: key(cur), move: { bead: i, from: cur[i], to: s } }); queue.push(next); }
    }
  }
  throw new Error('no solution');
}
// a random arrangement (balls on distinct seats), never the sorted one
export function randomSortLayout(rng, order, slotNames) {
  const n = order.length > slotNames.length ? slotNames.length : 3;
  for (;;) {
    const pool = slotNames.slice(), slotOf = [];
    for (let i = 0; i < 3; i++) slotOf.push(pool.splice(Math.floor(rng() * pool.length), 1)[0]);
    if (slotOf.join(',') !== order.slice(0, 3).join(',')) return slotOf;
  }
}
