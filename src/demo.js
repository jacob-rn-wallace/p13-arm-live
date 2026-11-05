// The arm firmware's demo, run against MuJoCo the way sim/run_demo.py runs it (the reference for test_demo.mjs):
// settle 0.5 s at q_0, then forever (like loop()): moveJointsTo(row) on one shared trapezoidal profile -> 300 ms pause ->
// if the gripper bit changed, CLOSE for 4 s / OPEN for 3 s (the gripper Uno as specified: drive link 1 at ~21 deg/s until it
// stalls at the current limit or reaches 45 deg, then hold). Every number comes from demo.json (export_web.py, sim/firmware.py).

export function trapezoid(diff, vmax, amax) {
  let vs = Infinity, as = Infinity;
  for (let i = 0; i < diff.length; i++) {
    const a = Math.abs(diff[i]);
    if (a > 1e-6) { vs = Math.min(vs, vmax[i] / a); as = Math.min(as, amax[i] / a); }
  }
  if (!Number.isFinite(vs)) return null;
  let ta = vs / as, T;
  if (vs * ta >= 1) { ta = Math.sqrt(1 / as); vs = as * ta; T = 2 * ta; } else T = 1 / vs + ta;
  return { ta, T, vs, as };
}

export function profileS(tau, p) {
  if (tau >= p.T) return 1;
  if (tau < p.ta) return 0.5 * p.as * tau * tau;
  if (tau <= p.T - p.ta) return 0.5 * p.as * p.ta * p.ta + p.vs * (tau - p.ta);
  return 1 - 0.5 * p.as * (p.T - tau) * (p.T - tau);
}

export class DemoRunner {
  // opts.balls: names of the free bodies the page tracks (default ['obj1'], the firmware demo's ball); opts.loop: go back to row 0 at the
  // end like loop() (default true) or stop ('done')
  constructor(mj, model, data, demo, opts = {}) {
    this.mj = mj; this.model = model; this.data = data; this.demo = demo; this.loop = opts.loop !== false;
    const id = (type, name) => { const i = mj.mj_name2id(model, mj.mjtObj[type].value, name); if (i < 0) throw new Error('not in the model: ' + name); return i; };
    this.jq = [1, 2, 3, 4, 5, 6].map(i => model.jnt_qposadr[id('mjOBJ_JOINT', 'j' + i)]);
    this.link1q = model.jnt_qposadr[id('mjOBJ_JOINT', 'F1_link1')];
    this.aPos = [1, 2, 3, 4, 5, 6].map(i => id('mjOBJ_ACTUATOR', 'j' + i));
    this.aVel = [1, 2, 3, 4, 5, 6].map(i => id('mjOBJ_ACTUATOR', 'v' + i));
    this.aGrip = id('mjOBJ_ACTUATOR', 'gripper');
    this.balls = (opts.balls || ['obj1']).map(n => id('mjOBJ_BODY', n));
    this.ball = this.balls[0];
    const kp = model.actuator_gainprm[10 * this.aGrip], fmax = model.actuator_forcerange[2 * this.aGrip + 1];
    this.stallErr = demo.stall_factor * fmax / kp;
    this.dt = model.opt.timestep;
    this.drag = null;      // optional external force: (mj, model, data) => void, called every step
    this.reset();
  }

  reset() {
    const { mj, model, data, demo } = this;
    mj.mj_resetData(model, data);
    const q = data.qpos;
    for (let i = 0; i < 6; i++) q[this.jq[i]] = demo.q0[i];
    this.cmd = demo.q0.slice(); this.prev = demo.q0.slice(); this.grip = 0; this.gripping = 0;
    this.applyCtrl();
    mj.mj_forward(model, data);
    this.t = 0; this.cycle = 0; this.row = -1;
    this.phase = 'settle'; this.timer = 0; this.done = false;
  }

  applyCtrl() {
    const c = this.data.ctrl;
    for (let i = 0; i < 6; i++) { c[this.aPos[i]] = this.cmd[i]; c[this.aVel[i]] = (this.cmd[i] - this.prev[i]) / this.dt; this.prev[i] = this.cmd[i]; }
    c[this.aGrip] = this.grip;
  }

  startRow(i) {
    const rows = this.demo.rows;
    if (i >= rows.length) {
      if (!this.loop) { this.phase = 'done'; this.done = true; return; }
      i = 0; this.cycle++;
    }
    this.row = i;
    this.start = this.cmd.slice(); this.target = rows[i].q;
    this.diff = this.target.map((v, k) => v - this.start[k]);
    this.prof = trapezoid(this.diff, this.demo.vmax, this.demo.amax);
    this.tau = 0;
    if (this.prof) this.phase = 'move'; else this.beginPause();
  }
  // start a new list of rows from wherever the arm is now, without touching the rest of the world (Connect 4 plays one robot move at a time)
  begin(rows) { this.demo.rows = rows; this.done = false; this.cycle = 0; this.startRow(0); }
  beginPause() { this.phase = 'pause'; this.timer = 0; }

  // one physics step: advance the firmware state machine, set the actuators, step MuJoCo
  step() {
    const dt = this.dt, demo = this.demo;
    switch (this.phase) {
      case 'settle':
        if ((this.timer += dt) >= demo.settle - 1e-9) this.startRow(0);
        break;
      case 'move': {
        this.tau = Math.min(this.tau + dt, this.prof.T);
        const s = profileS(this.tau, this.prof);
        for (let i = 0; i < 6; i++) this.cmd[i] = this.start[i] + s * this.diff[i];
        break; }
      case 'pause':
      case 'done':
        break;
      case 'grip': {
        const q = this.data.qpos[this.link1q];
        if (this.closing) { if (this.grip - q < this.stallErr && this.grip < demo.grip_max) this.grip = Math.min(this.grip + demo.grip_rate * dt, demo.grip_max); }
        else this.grip = Math.max(this.grip - demo.grip_rate * dt, 0);
        break; }
    }
    this.applyCtrl();
    this.data.qfrc_applied.fill(0);
    if (this.drag) this.drag(this.mj, this.model, this.data);
    this.mj.mj_step(this.model, this.data);
    this.t += dt;
    // transitions that happen after the step (as in run_demo.py's loops)
    if (this.phase === 'move' && this.tau >= this.prof.T) { this.cmd = this.target.slice(); this.beginPause(); }
    else if (this.phase === 'pause' && (this.timer += dt) >= demo.pause - 1e-9) {
      const g = demo.rows[this.row].grip;
      if (g !== this.gripping) {
        this.phase = 'grip'; this.closing = g === 1; this.timer = 0; this.gripDur = g ? demo.close : demo.open;
        this.grip = this.data.qpos[this.link1q];
      } else this.startRow(this.row + 1);
    } else if (this.phase === 'grip' && (this.timer += dt) >= this.gripDur - 1e-9) { this.gripping = this.closing ? 1 : 0; this.startRow(this.row + 1); }
  }

  // what the page shows
  status() {
    if (this.phase === 'settle') return { row: 0, of: this.demo.rows.length, text: 'starting at the home pose' };
    const r = this.demo.rows[this.row], note = r.note.replace(/\s*\[.*?\]/g, '').replace(/: from home, over the ball/, '').replace(/\s*\(.*\)$/, '');
    const what = this.phase === 'grip' ? (this.closing ? 'closing the gripper' : 'opening the gripper')
      : this.phase === 'pause' ? note + ' (pause)' : note;
    return { row: this.row + 1, of: this.demo.rows.length, text: what };
  }
  gripperState() {
    if (this.phase === 'grip') return this.closing ? 'closing' : 'opening';
    return this.gripping ? 'holding' : 'open';
  }
}
