// Connect 4 in the MuJoCo scene (scenes/p13_connect4.xml): the pool of 42 balls (bead0-20 red = the arm, bead21-41 yellow = the person),
// feeding the arm's supply seat, dropping the person's ball, and reading where the balls came to rest. Units: m in MuJoCo, mm here.
import { pieceBodies } from './puzzle.js';
import { ROWS, COLS } from './connect4.js';

export class C4Sim {
  constructor(mj, model, data, cj) {
    this.mj = mj; this.model = model; this.data = data; this.cj = cj; this.b = cj.board;
    this.pieces = pieceBodies(mj, model, 2 * cj.n_balls);
    this.reset();
  }
  // all balls back in their parking spots (the caller has reset MuJoCo's data, or this writes them)
  reset() {
    this.used = new Array(this.pieces.length).fill(false);
    this.nextRed = 0; this.nextYellow = this.cj.n_balls; this.seatBall = -1; this.onBoard = [];     // onBoard: [{idx, who, col, row}]
    this.pieces.forEach((p, i) => this.put(i, this.cj.park[i]));
    this.mj.mj_forward(this.model, this.data);
  }
  put(i, [x, y, z]) {
    const p = this.pieces[i];
    this.data.qpos.set([x / 1000, y / 1000, z / 1000, 1, 0, 0, 0], p.qadr);
    this.data.qvel.fill(0, p.vadr, p.vadr + 6);
  }
  pos(i) { const b = this.pieces[i].body, x = this.data.xpos; return [x[3 * b] * 1000, x[3 * b + 1] * 1000, x[3 * b + 2] * 1000]; }
  // a new red ball on the supply seat; `above` mm above its resting place (0: placed on the seat)
  feedSeat(above = 0) {
    if (this.nextRed >= this.cj.n_balls) return -1;
    const i = this.nextRed++, [sx, sy] = this.cj.slots.S, sz = this.cj.balls[0].seat_z;
    this.put(i, [sx, sy, sz + 0.3 + above]); this.seatBall = i; this.used[i] = true;
    this.mj.mj_forward(this.model, this.data); return i;
  }
  // a ball goes back to its parking spot (a lost ball: it left the seat or the board)
  parkBall(i) { this.put(i, this.cj.park[i]); this.used[i] = false; if (this.seatBall === i) this.seatBall = -1; this.mj.mj_forward(this.model, this.data); }
  // the supply ball is on the seat, at rest, where the arm will look for it
  seatOk() {
    const i = this.seatBall; if (i < 0) return false;
    const [x, y, z] = this.pos(i), [sx, sy] = this.cj.slots.S;
    return Math.hypot(x - sx, y - sy) < 12 && Math.abs(z - this.cj.balls[0].seat_z) < 6 && this.speed(i) < 0.05;
  }
  // the person's ball, released just above column `col` (a ball already sitting there would make no difference: it falls)
  dropHuman(col, above = 60) {
    const i = this.nextYellow++, b = this.b;
    this.put(i, [b.x, b.y_col[col], b.rim + this.cj.ball_r + above]); this.used[i] = true;
    this.mj.mj_forward(this.model, this.data); return i;
  }
  speed(i) { const v = this.pieces[i].vadr, q = this.data.qvel; return Math.hypot(q[v], q[v + 1], q[v + 2]); }
  // where a ball is relative to the board: { col, row, inside } (col -1 / row -1 when it's not in a channel)
  cellOf(i) {
    const [x, y, z] = this.pos(i), b = this.b, half = b.pitch / 2 - b.div_t / 2;
    let col = -1; b.y_col.forEach((yc, c) => { if (Math.abs(y - yc) < half) col = c; });
    const row = Math.round((z - b.row_z[0]) / 80), inside = col >= 0 && Math.abs(x - b.x) < b.ch_depth / 2 && z < b.rim + 20;
    return { col, row, inside, x, y, z };
  }
  // every ball in use that is on the board, as the logic board would have it: Int8Array(COLS*ROWS) of 0/1/2, plus a list of strays
  readBoard() {
    const cells = new Int8Array(COLS * ROWS), strays = [];
    this.pieces.forEach((p, i) => {
      if (!this.used[i] || i === this.seatBall) return;
      const c = this.cellOf(i);
      if (c.inside && c.row >= 0 && c.row < ROWS) { if (cells[c.col * ROWS + c.row]) strays.push(i); cells[c.col * ROWS + c.row] = i < this.cj.n_balls ? 2 : 1; }
      else strays.push(i);
    });
    return { cells, strays };
  }
  // the balls on the board have stopped moving (speeds in m/s)
  settled(thresh = 0.01) { return this.pieces.every((p, i) => !this.used[i] || this.speed(i) < thresh); }
}
