// Connect 4 rules and the arm's opponent. Pure logic (no MuJoCo, no DOM), tested by test_connect4.mjs.
// A board is { cells: Int8Array(COLS * ROWS), h: Int8Array(COLS) }: cells[c * ROWS + r] is 0 (empty), 1 (the person) or 2 (the arm); r = 0 is the bottom row.

export const COLS = 7, ROWS = 6;
export const newBoard = () => ({ cells: new Int8Array(COLS * ROWS), h: new Int8Array(COLS) });
export const cloneBoard = b => ({ cells: b.cells.slice(), h: b.h.slice() });
export const canPlay = (b, c) => c >= 0 && c < COLS && b.h[c] < ROWS;
export const legalMoves = b => { const m = []; for (let c = 0; c < COLS; c++) if (b.h[c] < ROWS) m.push(c); return m; };
export const isFull = b => b.h.every(v => v === ROWS);
export const cellAt = (b, c, r) => b.cells[c * ROWS + r];

// drop a piece; returns the row it lands in
export function play(b, c, who) {
  const r = b.h[c]++; b.cells[c * ROWS + r] = who; return r;
}
export function undo(b, c) { const r = --b.h[c]; b.cells[c * ROWS + r] = 0; }

const DIRS = [[1, 0], [0, 1], [1, 1], [1, -1]];
// the four (or more) in a row through (c, r), as a list of [c, r], or null
export function winFrom(b, c, r) {
  const who = b.cells[c * ROWS + r]; if (!who) return null;
  for (const [dc, dr] of DIRS) {
    const line = [[c, r]];
    for (const s of [1, -1]) for (let k = 1; ; k++) {
      const cc = c + s * k * dc, rr = r + s * k * dr;
      if (cc < 0 || cc >= COLS || rr < 0 || rr >= ROWS || b.cells[cc * ROWS + rr] !== who) break;
      line.push([cc, rr]);
    }
    if (line.length >= 4) return line;
  }
  return null;
}
// any win on the board: { who, cells } or null
export function findWin(b) {
  for (let c = 0; c < COLS; c++) for (let r = 0; r < b.h[c]; r++) { const w = winFrom(b, c, r); if (w) return { who: b.cells[c * ROWS + r], cells: w }; }
  return null;
}

// ---------------------------------------------------------------- the opponent: negamax with alpha-beta
// All 69 lines of four, as index lists
const LINES = [];
for (let c = 0; c < COLS; c++) for (let r = 0; r < ROWS; r++) for (const [dc, dr] of DIRS) {
  const cc = c + 3 * dc, rr = r + 3 * dr;
  if (cc < 0 || cc >= COLS || rr < 0 || rr >= ROWS) continue;
  LINES.push([0, 1, 2, 3].map(k => (c + k * dc) * ROWS + r + k * dr));
}
const ORDER = [3, 2, 4, 1, 5, 0, 6];
const WIN = 1e6;

// score for `me`: lines holding only my pieces count more the fuller they are, minus the opponent's; the centre column is worth a little
function evaluate(b, me) {
  const opp = 3 - me; let s = 0;
  for (const ln of LINES) {
    let m = 0, o = 0;
    for (let k = 0; k < 4; k++) { const v = b.cells[ln[k]]; if (v === me) m++; else if (v === opp) o++; }
    if (m && o) continue;
    if (m) s += m === 3 ? 50 : m === 2 ? 6 : 1; else if (o) s -= o === 3 ? 55 : o === 2 ? 6 : 1;
  }
  for (let r = 0; r < ROWS; r++) { const v = b.cells[3 * ROWS + r]; if (v === me) s += 4; else if (v === opp) s -= 4; }
  return s;
}

// negamax from `who`'s side to move; lastC: the column just played (for the win test)
function negamax(b, who, depth, alpha, beta, lastC, stats) {
  stats.nodes++;
  if (lastC >= 0 && winFrom(b, lastC, b.h[lastC] - 1)) return -(WIN + depth);       // the player who just moved has won
  if (isFull(b)) return 0;
  if (depth === 0) return evaluate(b, who);
  let best = -Infinity;
  for (const c of ORDER) {
    if (b.h[c] >= ROWS) continue;
    play(b, c, who);
    const v = -negamax(b, 3 - who, depth - 1, -beta, -alpha, c, stats);
    undo(b, c);
    if (v > best) best = v;
    if (best > alpha) alpha = best;
    if (alpha >= beta) break;
  }
  return best;
}

// scores of every legal column for `who`: { c, score }[]; depth plies of look-ahead
export function scoreMoves(b, who, depth) {
  const stats = { nodes: 0 }, out = [];
  for (const c of legalMoves(b)) {
    play(b, c, who);
    const v = -negamax(b, 3 - who, depth - 1, -Infinity, Infinity, c, stats);
    undo(b, c);
    out.push({ c, score: v });
  }
  out.nodes = stats.nodes;
  return out;
}

export const LEVELS = {
  easy:   { depth: 2, blunder: 0.30, name: 'Easy' },       // sees one move ahead, and 30 % of the time plays a random column
  medium: { depth: 4, blunder: 0.0, name: 'Medium' },
  hard:   { depth: 8, blunder: 0.0, name: 'Hard' },
};
// the arm's column. rng: () => [0,1). Ties between equal scores are broken at random (centre first for the hard level).
export function chooseMove(b, who, level = 'medium', rng = Math.random) {
  const L = LEVELS[level], moves = legalMoves(b);
  if (moves.length === 1) return moves[0];
  const w = moves.find(c => { play(b, c, who); const ok = !!winFrom(b, c, b.h[c] - 1); undo(b, c); return ok; });
  if (w !== undefined) return w;                                                 // a win in one is always taken
  if (L.blunder && rng() < L.blunder) return moves[Math.floor(rng() * moves.length)];
  const sc = scoreMoves(b, who, L.depth), top = Math.max(...sc.map(s => s.score));
  const best = sc.filter(s => s.score >= top - (level === 'easy' ? 3 : 0)).map(s => s.c);
  if (level === 'hard') { best.sort((a, c) => Math.abs(a - 3) - Math.abs(c - 3)); return best[0]; }
  return best[Math.floor(rng() * best.length)];
}
