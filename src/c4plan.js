// The arm's move in Connect 4: pick the red ball waiting on the supply seat, carry it over a column, lower it to just above the rim, let go,
// and go back over the seat. Rows are in demo.json's format ({q: 6 rad, grip: 0/1, note, move}) and every straight joint-space segment is
// checked against the table, the seat, the board and the carried ball with the hand's point cloud (as planner.js does for the ball-sort puzzle).
// Units: mm and rad.
import { fk, handPoints, makeIk, makeVline, CLEAR_MIN, TOL_AIR } from './planner.js';

// the board as axis-aligned boxes {c: [x, y, z], h: [hx, hy, hz]}: both panels and the eight dividers (the peaked roof is taken as a flat top: conservative)
export function boardBoxes(b) {
  const out = [], zc = (b.floor_z + b.rim) / 2, hz = (b.rim - b.floor_z) / 2, nl = b.near_lower;
  out.push({ c: [b.x - b.ch_depth / 2 - b.panel_t / 2, 0, zc - nl / 2], h: [b.panel_t / 2, b.y_half, hz - nl / 2], name: 'near panel' });
  out.push({ c: [b.x + b.ch_depth / 2 + b.panel_t / 2, 0, zc], h: [b.panel_t / 2, b.y_half, hz], name: 'far panel' });
  for (let d = 0; d <= b.cols; d++) out.push({ c: [b.x, (d - b.cols / 2) * b.pitch, (b.plate_top + b.rim) / 2], h: [b.ch_depth / 2, b.div_t / 2, (b.rim - b.plate_top) / 2], name: 'divider ' + d });
  return out;
}
const distBox = (x, y, z, bx) => {
  const q = [Math.abs(x - bx.c[0]) - bx.h[0], Math.abs(y - bx.c[1]) - bx.h[1], Math.abs(z - bx.c[2]) - bx.h[2]];
  return Math.hypot(Math.max(q[0], 0), Math.max(q[1], 0), Math.max(q[2], 0)) + Math.min(Math.max(q[0], q[1], q[2]), 0);
};
const distCyl = (x, y, z, cx, cy, r, z0, z1) => Math.hypot(Math.max(Math.hypot(x - cx, y - cy) - r, 0), Math.max(z0 - z, z - z1, 0));

// smallest clearance (mm) along the joint-space line qa -> qb. held: the hand is closed on the ball, whose centre is at `grip` from the tool point.
// ownSeat: the seat the ball is being lifted from (the ball is allowed to touch it).
export function segClearance(w, boxes, qa, qb, held, grip, ownSeat = false, samples = 24) {
  const [sx, sy] = w.slots.S, hj = w.hj, bp = w.beadPts[0];
  let best = { d: Infinity, what: '', f: 0 };
  const note = (d, what, f) => { if (d < best.d) best = { d, what, f }; };
  for (let s = 0; s <= samples; s++) {
    const f = s / samples, q = qa.map((v, i) => v + f * (qb[i] - v)), P = handPoints(w, q, held);
    for (let k = 0; k < P.length; k += 3) {
      const x = P[k], y = P[k + 1], z = P[k + 2];
      note(z, 'table', f);
      note(distCyl(x, y, z, sx, sy, hj.seat_r, 0, hj.seat_top), 'seat', f);
      for (const bx of boxes) note(distBox(x, y, z, bx), bx.name, f);
    }
    if (held) {
      const T = fk(q, w.a6), c = [T[3] + grip[0], T[7] + grip[1], T[11] + grip[2]];
      for (const p of bp) {
        const x = p[0] + c[0], y = p[1] + c[1], z = p[2] + c[2];
        if (!ownSeat) note(distCyl(x, y, z, sx, sy, hj.seat_r, 0, hj.seat_top), 'ball/seat', f);
        for (const bx of boxes) note(distBox(x, y, z, bx), 'ball/' + bx.name, f);
      }
    }
  }
  return best;
}

// The arm's start pose: the hand open, hovering over the supply seat. (The firmware's home pose puts the hand where the board stands.)
export const HOVER = 296;
export function parkPose(w, cj) {
  const ik = makeIk(w), [sx, sy] = w.slots.S;
  return ik(sx, sy, HOVER, [-1.2281, -0.3142, -1.1174, 1.4316, 1.2281, 0], 'park', w.rolls.S);
}

// joint poses along the straight Cartesian line from tool point pa (pose qa) to pb (pose qb), excluding qa: the line is split until the
// joint-space straight line between neighbours stays within `tol` mm of it (moveJointsTo() drives each row on a joint-space line, which sags).
function makeLine(w, ik) {
  const distSeg = (p, a, b) => {
    const ab = [0, 1, 2].map(i => b[i] - a[i]), ap = [0, 1, 2].map(i => p[i] - a[i]), L2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2 || 1e-9;
    const t = Math.min(1, Math.max(0, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / L2));
    return Math.hypot(...[0, 1, 2].map(i => p[i] - a[i] - t * ab[i]));
  };
  return (pa, pb, qa, qb, tol, tag, roll) => {
    const out = [];
    const rec = (a, qa_, b, qb_, depth) => {
      const qm = qa_.map((v, i) => (v + qb_[i]) / 2), T = fk(qm, w.a6), pm = [T[3], T[7], T[11]];
      if (depth < 6 && distSeg(pm, pa, pb) > tol) {
        const m = [0, 1, 2].map(i => (a[i] + b[i]) / 2), qmid = ik(m[0], m[1], m[2], qa_, tag, roll);
        rec(a, qa_, m, qmid, depth + 1); rec(m, qmid, b, qb_, depth + 1);
      } else out.push(qb_);
    };
    rec(pa, qa, pb, qb, 0);
    return out;
  };
}

// plan(w, cj, col, q): rows for one robot move into column `col`, from joint pose q (open hand). Returns { rows, checks, zh, qEnd }.
// grip: where the ball's centre sits relative to the tool point while carried (cj.grip_offsets[0]).
export function planDrop(w, cj, col, q0, opts = {}) {
  const b = cj.board, boxes = boardBoxes(b), g = opts.grip || cj.grip_offsets[0], ball = w.beads[0];
  const ik = makeIk(w), vline = makeVline(w, ik), line = makeLine(w, ik), [sx, sy] = w.slots.S, roll = w.rolls.S;
  const tx = b.x, ty = b.y_col[col];
  const pickZ = ball.seat_z, zLift = cj.seat_top + ball.R + 12, zSplit = zLift + 6;
  const dropZ = b.rim + ball.R + b.drop_above;                       // the ball's centre when the fingers open
  const clearMin = opts.clearMin ?? CLEAR_MIN, tried = [];
  for (let zh = opts.zMax ?? HOVER; zh >= Math.max(dropZ, 200); zh -= 4) {
    try {
      const T0 = fk(q0, w.a6), qRise = T0[11] < zh - 1 ? ik(T0[3], T0[7], zh, q0, 'rise', q0[5]) : null;
      const qHp = ik(sx, sy, zh, qRise || q0, 'hover over the seat', roll);
      const pickDown = [...vline(sx, sy, qHp, zh, Math.max(zSplit, pickZ), TOL_AIR, 'down to the ball', roll)];
      if (pickZ < zSplit) pickDown.push(...vline(sx, sy, pickDown[pickDown.length - 1], zSplit, pickZ, 8, 'down to the ball', roll));
      const qPick = pickDown[pickDown.length - 1];
      const hz = Math.max(zh, dropZ);                                // hover height of the ball's centre over the column
      const qHt = ik(tx - g[0], ty - g[1], hz - g[2], qHp, 'hover over column ' + col, roll);
      const placeDown = hz - dropZ > 1 ? [...vline(tx - g[0], ty - g[1], qHt, hz - g[2], dropZ - g[2], 4, 'lower over the column', roll)] : [qHt];
      const qDrop = placeDown[placeDown.length - 1];
      const pHp = [sx, sy, zh], pHt = [tx - g[0], ty - g[1], hz - g[2]];
      // out to the board's x first (the middle of the workspace nearer the base is out of reach with the hand along +x), then along y over the dividers
      const pVia = [pHt[0], sy, pHt[2]], useVia = Math.abs(sy - pHt[1]) > 20, tol = opts.lineTol ?? 6;
      const qVia = useVia ? ik(pVia[0], pVia[1], pVia[2], qHp, 'via point', roll) : qHt;
      const carry = useVia ? [...line(pHp, pVia, qHp, qVia, tol, 'carry', roll), ...line(pVia, pHt, qVia, qHt, tol, 'carry', roll)] : line(pHp, pHt, qHp, qHt, tol, 'carry', roll);
      const back = useVia ? [...line(pHt, pVia, qHt, qVia, tol, 'back', roll), ...line(pVia, pHp, qVia, qHp, tol, 'back', roll)] : line(pHt, pHp, qHt, qHp, tol, 'back', roll);
      const segs = [];                                               // [qa, qb, held, what, ownSeat]
      if (qRise) segs.push([q0, qRise, false, 'rise at the start']);
      segs.push([qRise || q0, qHp, false, 'transit over the seat']);
      let prev = qHp; for (const qq of pickDown) { segs.push([prev, qq, false, 'descend to the ball']); prev = qq; }
      const upList = [qHp, ...pickDown.slice(0, -1)].reverse(); prev = qPick;
      for (const qq of upList) { segs.push([prev, qq, true, 'lift', true]); prev = qq; }
      prev = qHp; for (const qq of carry) { segs.push([prev, qq, true, 'carry']); prev = qq; }
      prev = qHt; for (const qq of placeDown.slice(qHt === qDrop ? 1 : 0)) { segs.push([prev, qq, true, 'lower']); prev = qq; }
      const rise = [qHt, ...placeDown.slice(0, -1)].reverse(); prev = qDrop;
      for (const qq of (qHt === qDrop ? [] : rise)) { segs.push([prev, qq, false, 'rise after release']); prev = qq; }
      prev = qHt; for (const qq of back) { segs.push([prev, qq, false, 'back over the seat']); prev = qq; }
      const checks = []; let ok = true;
      for (const [qa, qb, held, what, ownSeat] of segs) {
        const r = segClearance(w, boxes, qa, qb, held, held ? (ownSeat ? [0, 0, 0] : g) : [0, 0, 0], !!ownSeat);
        checks.push({ what, hover: zh, d: r.d, hit: r.what });
        if (r.d < clearMin) { ok = false; const tp = q => { const T = fk(q, w.a6); return `(${T[3].toFixed(0)},${T[7].toFixed(0)},${T[11].toFixed(0)})`; }; tried.push(`${what} at hover ${zh}: ${r.d.toFixed(1)} mm to ${r.what} at ${(100 * r.f).toFixed(0)}% of ${tp(qa)} -> ${tp(qb)}`); break; }
      }
      if (!ok) continue;
      const rows = [], tag = `column ${col + 1}`, push = (qq, grip, note) => rows.push({ q: qq.map(v => Math.round(v * 1e6) / 1e6), grip, note: `${tag}: ${note}`, move: opts.move ?? 0 });
      if (qRise) push(qRise, 0, 'rise to hover height');
      push(qHp, 0, 'over the supply seat');
      pickDown.forEach(qq => push(qq, 0, 'down to the ball on the seat'));
      push(qPick, 1, 'close the gripper');
      [...pickDown.slice(0, -1)].reverse().forEach(qq => push(qq, 1, 'lift the ball clear'));
      push(qHp, 1, 'lift the ball clear');
      carry.forEach((qq, k) => push(qq, 1, k === carry.length - 1 ? 'carry over the column' : 'carry'));
      if (qHt !== qDrop) placeDown.forEach(qq => push(qq, 1, 'lower to the rim'));
      push(qDrop, 0, 'open the gripper: the ball drops');
      if (qHt !== qDrop) [...placeDown.slice(0, -1)].reverse().forEach(qq => push(qq, 0, 'rise clear'));
      push(qHt, 0, 'rise clear');
      back.forEach(qq => push(qq, 0, 'back over the supply seat'));
      return { rows, checks, zh, qEnd: qHp, qDrop, dropZ };
    } catch (e) { tried.push(`hover ${zh}: ${e.message}`); }
  }
  throw new Error(`column ${col + 1}: no clear path; first failures: ${tried.slice(0, 4).join('; ')}`);
}
