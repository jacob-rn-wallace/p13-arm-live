// The Connect 4 game as a state machine over the MuJoCo scene: the person drops yellow balls, the arm answers with red ones.
// Used by the page (main.js) and by the headless test (test_connect4.mjs), so both run the same turns. The caller steps the physics through
// game.step() (which calls DemoRunner.step()) and renders; the game decides what happens next.
import { COLS, ROWS, newBoard, canPlay, play, winFrom, findWin, isFull, chooseMove, legalMoves } from './connect4.js';
import { planDrop, parkPose } from './c4plan.js';
import { C4Sim } from './c4sim.js';

// phases: 'human' (waiting for the person's column), 'drop' (their ball is falling), 'think' (the arm is choosing and planning: call think()),
//         'robot' (the arm is moving), 'settle' (the ball is coming to rest), 'over' (win or draw), 'fault' (the balls are not where the game says)
export class C4Game {
  constructor({ mj, model, data, cj, demo, world, DemoRunner }) {
    this.cj = cj; this.world = world; this.mj = mj; this.model = model; this.data = data;
    this.park = parkPose(world, cj);
    this.run = new DemoRunner(mj, model, data, { ...demo, q0: this.park, rows: [] }, { balls: ['bead0'], loop: false });
    this.sim = new C4Sim(mj, model, data, cj);
    this.level = 'medium'; this.humanFirst = true; this.rng = Math.random;
    this.newGame();
  }
  newGame(opts = {}) {
    if (opts.level) this.level = opts.level;
    if (opts.humanFirst !== undefined) this.humanFirst = opts.humanFirst;
    if (opts.rng) this.rng = opts.rng;
    this.run.reset(); this.sim.reset(); this.sim.feedSeat();
    this.run.demo.rows = []; this.run.done = true; this.run.phase = 'done';
    this.board = newBoard(); this.moves = []; this.win = null; this.fault = null; this.plan = null; this.fed = true;
    this.lastCol = -1; this.quiet = 0; this.waited = 0; this.log = [];
    this.phase = this.humanFirst ? 'human' : 'think';
    this.turns = 0; this.refeeds = 0;
  }
  get robotMoves() { return this.moves.filter(m => m.who === 2).length; }

  // the person picks a column; returns false when it is not their turn or the column is full
  humanMove(col) {
    if (this.phase !== 'human' || !canPlay(this.board, col)) return false;
    play(this.board, col, 1); this.moves.push({ who: 1, col }); this.lastCol = col;
    this.sim.dropHuman(col); this.phase = 'drop'; this.quiet = 0; this.waited = 0;
    return true;
  }

  // call once when phase === 'think' (it takes up to a second: the page lets the status text paint first)
  think() {
    if (this.phase !== 'think') return;
    if (!this.sim.seatOk()) {                                    // the supply ball got knocked off the seat: put it away and bring another
      if (this.sim.seatBall >= 0) this.sim.parkBall(this.sim.seatBall);
      if (this.sim.feedSeat(0) < 0) { this.phase = 'fault'; this.fault = { strays: [], cells: this.board.cells, expected: this.board.cells, noBalls: true }; return; }
      this.refeeds++;
    }
    const col = chooseMove(this.board, 2, this.level, this.rng), t0 = performance.now();
    const q = this.run.cmd.slice();
    const plan = planDrop(this.world, this.cj, col, q, { move: this.moves.length });
    plan.col = col; plan.thinkMs = performance.now() - t0;
    plan.feedRow = plan.rows.findIndex(r => /ball drops/.test(r.note));
    play(this.board, col, 2); this.moves.push({ who: 2, col }); this.lastCol = col;
    this.plan = plan; this.fed = false;
    this.run.begin(plan.rows); this.phase = 'robot';
  }

  // one physics step and whatever follows from it
  step() {
    this.run.step();
    const dt = this.run.dt;
    switch (this.phase) {
      case 'drop': this.restCheck(dt, 0.5, () => this.landed()); break;
      case 'robot':
        if (!this.fed && this.run.row >= this.plan.feedRow) { this.sim.feedSeat(this.cj.feed_above); this.fed = true; }
        if (this.run.done) { this.phase = 'settle'; this.quiet = 0; this.waited = 0; }
        break;
      case 'settle': this.restCheck(dt, 0.8, () => this.landed()); break;
    }
  }
  // the balls have been still for `need` seconds (or we have waited 8 s): act
  restCheck(dt, need, then) {
    this.waited += dt;
    this.quiet = this.sim.settled() ? this.quiet + dt : 0;
    if (this.quiet >= need || this.waited > 8) then();
  }
  // a ball has come to rest: check it against the game, then the win, then whose turn
  landed() {
    const { cells, strays } = this.sim.readBoard();
    let same = strays.length === 0;
    for (let i = 0; i < cells.length; i++) if (cells[i] !== this.board.cells[i]) same = false;
    if (!same) { this.phase = 'fault'; this.fault = { strays, cells, expected: this.board.cells.slice() }; return; }
    const w = this.lastCol >= 0 ? winFrom(this.board, this.lastCol, this.board.h[this.lastCol] - 1) : null;
    if (w) { this.win = { who: this.board.cells[w[0][0] * ROWS + w[0][1]], cells: w }; this.phase = 'over'; return; }
    if (isFull(this.board)) { this.phase = 'over'; return; }
    this.phase = this.phase === 'drop' ? 'think' : 'human';
  }
  // the ball sitting in cell (col, row), as a pool index, or -1
  ballAt(col, row) {
    for (let i = 0; i < this.sim.pieces.length; i++) {
      if (!this.sim.used[i] || i === this.sim.seatBall) continue;
      const c = this.sim.cellOf(i); if (c.inside && c.col === col && c.row === row) return i;
    }
    return -1;
  }
}
