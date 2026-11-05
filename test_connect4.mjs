// Headless checks of the Connect 4 demo (Node >= 20): the rules and the opponent, the arm's planner for every column, then games in MuJoCo
// physics against a scripted person: after every move the balls that came to rest must be exactly where the game says (nothing stray).
// usage: node test_connect4.mjs [robotMoves=3 | full] [seed=1] [level=medium]     (full: play until someone wins; ~10 min)
import load_mujoco from '@mujoco/mujoco';
import { readFileSync, readdirSync } from 'fs';
import * as P from './src/planner.js';
import * as C from './src/connect4.js';
import { DemoRunner } from './src/demo.js';
import { planDrop, parkPose } from './src/c4plan.js';
import { C4Game } from './src/c4game.js';

let fail = false;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) fail = true; };
const cj = JSON.parse(readFileSync('public/connect4.json')), demo = JSON.parse(readFileSync('public/demo.json')), w = P.makeWorld(cj);

// ---- rules
{
  const b = C.newBoard();
  for (const c of [0, 1, 0, 1, 0, 1]) C.play(b, c, C.cellAt(b, 0, 0) === 0 ? 1 : (b.h[0] + b.h[1]) % 2 ? 2 : 1);
  const t = (cells, who) => { const bb = C.newBoard(); for (const [c, r] of cells) { bb.cells[c * C.ROWS + r] = who; bb.h[c] = Math.max(bb.h[c], r + 1); } return bb; };
  check(!!C.findWin(t([[0, 0], [1, 0], [2, 0], [3, 0]], 1)), 'rules: four in a row (horizontal)');
  check(!!C.findWin(t([[2, 0], [2, 1], [2, 2], [2, 3]], 2)), 'rules: four in a column');
  check(!!C.findWin(t([[0, 0], [1, 1], [2, 2], [3, 3]], 1)) && !!C.findWin(t([[3, 0], [4, 1], [5, 2], [6, 3]], 1)), 'rules: both diagonals');
  check(!C.findWin(t([[0, 0], [1, 0], [2, 0], [4, 0]], 1)) && !C.findWin(t([[0, 0], [1, 1], [2, 2]], 1)), 'rules: three in a row is not a win');
  const full = C.newBoard(); for (let k = 0; k < C.ROWS; k++) C.play(full, 3, 1);
  check(!C.canPlay(full, 3) && C.legalMoves(full).length === 6, 'rules: a full column is not playable');
  const rng = P.mulberry32(5); let wins = 0, games = 30;
  for (let g = 0; g < games; g++) {            // the medium level against a random player, alternating who starts
    const bb = C.newBoard(); let who = g % 2 ? 2 : 1;
    for (;;) {
      const m = C.legalMoves(bb), c = who === 2 ? C.chooseMove(bb, 2, 'medium', rng) : m[Math.floor(rng() * m.length)], r = C.play(bb, c, who);
      if (C.winFrom(bb, c, r)) { if (who === 2) wins++; break; }
      if (C.isFull(bb)) break; who = 3 - who;
    }
  }
  check(wins >= games - 1, `opponent: medium beats a random player in ${wins}/${games} games`);
  const bb = t([[0, 0], [0, 1], [0, 2]], 1); bb.cells[1 * C.ROWS] = 2; bb.cells[2 * C.ROWS] = 2; bb.h[1] = bb.h[2] = 1;
  check(C.chooseMove(bb, 2, 'hard', rng) === 0, 'opponent: blocks a vertical three');
  const b3 = t([[2, 0], [3, 0], [4, 0]], 2); b3.cells[0] = 1; b3.h[0] = 1; b3.cells[6 * C.ROWS] = 1; b3.h[6] = 1;
  check([1, 5].includes(C.chooseMove(b3, 2, 'easy', P.mulberry32(1))), 'opponent: takes a win in one, even at the easy level');
}

// ---- the arm's planner: every column, from the start pose and from where the previous move ended
{
  let q = parkPose(w, cj), worst = Infinity, rows = 0;
  for (const col of [0, 6, 3, 1, 5, 2, 4]) {
    try { const p = planDrop(w, cj, col, q); worst = Math.min(worst, ...p.checks.map(c => c.d)); rows += p.rows.length; q = p.qEnd; }
    catch (e) { check(false, `plan: column ${col + 1}: ${e.message}`); }
  }
  check(worst >= 8, `plan: all 7 columns planned (${rows} rows); tightest clearance along any segment ${worst.toFixed(1)} mm`);
}

// ---- physics: games
const mj = await load_mujoco(); mj.FS.mkdir('/working'); mj.FS.mkdir('/working/assets');
mj.FS.writeFile('/working/p13_connect4.xml', readFileSync('public/scenes/p13_connect4.xml'));
for (const f of readdirSync('public/scenes/assets')) mj.FS.writeFile('/working/assets/' + f, readFileSync('public/scenes/assets/' + f));
const model = mj.MjModel.mj_loadXML('/working/p13_connect4.xml'), data = new mj.MjData(model);
const arg = process.argv[2] || '3', seed = +(process.argv[3] || 1), level = process.argv[4] || 'medium';
const maxRobot = arg === 'full' ? 21 : +arg;
const game = new C4Game({ mj, model, data, cj, demo, world: w, DemoRunner });
const rng = P.mulberry32(seed);
game.newGame({ level, humanFirst: seed % 2 === 1, rng });
let guard = 0; const flung = new Set();
while (game.phase !== 'over' && game.phase !== 'fault' && !(game.robotMoves >= maxRobot && game.phase === 'human') && guard < 6e6) {
  if (game.phase === 'human') {                                          // a scripted person: a random column, but a win in one or a block when there is one
    const legal = C.legalMoves(game.board), pick = legal.find(c => { C.play(game.board, c, 1); const ok = !!C.winFrom(game.board, c, game.board.h[c] - 1); C.undo(game.board, c); return ok; })
      ?? legal.find(c => { C.play(game.board, c, 2); const ok = !!C.winFrom(game.board, c, game.board.h[c] - 1); C.undo(game.board, c); return ok; }) ?? legal[Math.floor(rng() * legal.length)];
    game.humanMove(pick);
  }
  if (game.phase === 'think') { const before = game.robotMoves; game.think(); console.log(`  robot move ${before + 1}: column ${game.plan.col + 1}, ${game.plan.rows.length} rows, planned in ${game.plan.thinkMs.toFixed(0)} ms, t = ${game.run.t.toFixed(0)} s`); }
  game.step(); guard++;
  if (process.env.TRACE && game.run.t > +(process.env.FROM || 0) && guard % 200 === 0 && game.sim.used[+process.env.TRACE]) {
    const i = +process.env.TRACE, c = game.sim.cellOf(i), r = game.plan && game.run.row >= 0 ? game.plan.rows[game.run.row] : null;
    console.log(`  trace t ${game.run.t.toFixed(2)} ball ${i} ${c.x.toFixed(0)},${c.y.toFixed(0)},${c.z.toFixed(0)} v ${game.sim.speed(i).toFixed(2)} phase ${game.phase}/${game.run.phase} row ${game.run.row} ${r ? r.note : ''}`);
  }
  if (guard % 10 === 0) for (let i = 0; i < game.sim.pieces.length; i++) if (game.sim.used[i] && !flung.has(i) && game.sim.speed(i) > 4.5) {   // a ball moving this fast was thrown: a drop from the rim reaches 2.5 m/s
    flung.add(i); const r = game.plan && game.run.row >= 0 ? game.plan.rows[game.run.row] : null, c = game.sim.cellOf(i);
    console.log(`  FAST ball ${i}: ${game.sim.speed(i).toFixed(1)} m/s at t = ${game.run.t.toFixed(2)} s, phase ${game.phase}, row ${game.run.row} (${r ? r.note : '-'}), run phase ${game.run.phase}, pos ${c.x.toFixed(0)},${c.y.toFixed(0)},${c.z.toFixed(0)}`);
  }
}
const rb = game.sim.readBoard();
const text = [...Array(C.ROWS).keys()].reverse().map(r => [...Array(C.COLS).keys()].map(c => '.YR'[rb.cells[c * C.ROWS + r]]).join(' ')).join('\n');
console.log(text);
if (game.fault) for (const i of game.fault.strays) { const c = game.sim.cellOf(i); console.log(`  stray ball ${i} (${i < cj.n_balls ? 'red' : 'yellow'}): x ${c.x.toFixed(0)} y ${c.y.toFixed(0)} z ${c.z.toFixed(0)}, over column ${c.col + 1}, row ${c.row}, speed ${game.sim.speed(i).toFixed(3)}`); }
if (game.refeeds) console.log(`  the supply ball had to be replaced ${game.refeeds} time(s)`);
check(game.phase !== 'fault', `physics game (seed ${seed}, ${level}): ${game.moves.length} moves (${game.robotMoves} by the arm), ${game.run.t.toFixed(0)} s simulated, phase ${game.phase}` +
  (game.fault ? ` strays ${game.fault.strays} board ${[...game.fault.cells].join('')} wanted ${[...game.fault.expected].join('')}` : ''));
if (game.win) check(true, `${game.win.who === 2 ? 'the arm' : 'the person'} won with four in a row: ${game.win.cells.map(c => c.map(v => v + 1).join(',')).join(' ')}`);
process.exit(fail ? 1 : 0);
