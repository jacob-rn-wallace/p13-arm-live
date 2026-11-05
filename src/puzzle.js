// Glue between the planner (planner.js, mm) and the MuJoCo scene (m): put the pieces of a layout where they stand, read where they are.
import { layoutPoses } from './planner.js';

export function pieceBodies(mj, model, n) {
  const id = (type, name) => { const i = mj.mj_name2id(model, mj.mjtObj[type].value, name); if (i < 0) throw new Error('not in the model: ' + name); return i; };
  return Array.from({ length: n }, (_, i) => ({ body: id('mjOBJ_BODY', 'bead' + i), qadr: model.jnt_qposadr[id('mjOBJ_JOINT', 'bead' + i)], vadr: model.jnt_dofadr[id('mjOBJ_JOINT', 'bead' + i)] }));
}

// write the free joints of the pieces (position m, upright) and clear their velocities; call after DemoRunner.reset()
export function placePieces(mj, model, data, world, pegOf, pieces) {
  const { pos } = layoutPoses(world, pegOf);
  pieces.forEach((p, i) => {
    const c = pos[i];
    data.qpos.set([c[0] / 1000, c[1] / 1000, c[2] / 1000 + 0.0003, 1, 0, 0, 0], p.qadr);
    data.qvel.fill(0, p.vadr, p.vadr + 6);
  });
  mj.mj_forward(model, data);
}

// where each piece is: slot name if its centre is over a peg (within `tol` mm), its centre height, and how far it is tilted from upright (deg)
export function readPieces(model, data, world, pieces, tol = 25) {
  return pieces.map(p => {
    const b = p.body, x = data.xpos[3 * b] * 1000, y = data.xpos[3 * b + 1] * 1000, z = data.xpos[3 * b + 2] * 1000;
    let slot = null;
    for (const s of world.slotNames) if (Math.hypot(x - world.slots[s][0], y - world.slots[s][1]) < tol) slot = s;
    const R = data.xmat.slice(9 * b, 9 * b + 9), tilt = Math.acos(Math.min(1, Math.max(-1, R[8]))) * 180 / Math.PI;
    return { slot, x, y, z, tilt };
  });
}
