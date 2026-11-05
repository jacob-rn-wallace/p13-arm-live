// P13 gripper on Group 1's arm: live MuJoCo (WebAssembly) physics + three.js. Default: Connect 4, the person against the arm (src/c4game.js:
// the person clicks a column, the arm picks a red ball and drops it into one). ?demo=sort: the ball-sort puzzle (Randomize, Solve: the arm plans
// and sorts three balls by size live; src/planner.js, src/puzzle.js). ?demo=firmware: the arm firmware's pick-and-place demo (src/demo.js).
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import load_mujoco from '@mujoco/mujoco';
import { buildScene, syncBodies, fromMj, toMj } from './render.js';
import { DemoRunner } from './demo.js';
import { makeWorld, solveSort, randomSortLayout, plan } from './planner.js';
import { pieceBodies, placePieces, readPieces } from './puzzle.js';
import { C4Game } from './c4game.js';
import { LEVELS } from './connect4.js';

const MODE = new URLSearchParams(location.search).get('demo') || 'connect4';          // connect4 | sort | firmware
const SORT = MODE === 'sort', C4 = MODE === 'connect4';
document.body.dataset.mode = C4 ? 'connect4' : SORT ? 'sort' : 'firmware';

const base = new URL('.', import.meta.url).href;
const say = m => { const el = document.getElementById('loadmsg'); if (el) el.textContent = m; };

say('Loading MuJoCo (WebAssembly)…');
const mj = await load_mujoco({ locateFile: p => p.endsWith('.wasm') ? new URL(p, base).href : p });
mj.FS.mkdir('/working'); mj.FS.mkdir('/working/assets');
say('Loading the arm and the gripper…');
const get = async (p, kind) => { const r = await fetch(new URL(p, base)); if (!r.ok) throw new Error(`${p}: HTTP ${r.status}`); return r[kind](); };
const sceneName = C4 ? 'p13_connect4' : SORT ? 'p13_balls' : 'p13_demo';
const xml = await get(`scenes/${sceneName}.xml`, 'text');
mj.FS.writeFile(`/working/${sceneName}.xml`, xml);
await Promise.all([...xml.matchAll(/file="([^"]+)"/g)].map(async m =>
  mj.FS.writeFile('/working/assets/' + m[1], new Uint8Array(await get('scenes/assets/' + m[1], 'arrayBuffer')))));
const demo = await get('demo.json', 'json');
const model = mj.MjModel.mj_loadXML(`/working/${sceneName}.xml`), data = new mj.MjData(model);
let sort = null;                                    // ball-sort state: the world, the pieces, the layout and what the arm is doing
if (SORT) {
  const bj = await get('ballsort.json', 'json'), world = makeWorld(bj), n = bj.balls.length;
  const run0 = new DemoRunner(mj, model, data, { ...demo, rows: [] }, { balls: bj.balls.map((_, i) => 'bead' + i), loop: false });
  sort = { bj, world, n, run: run0, pieces: pieceBodies(mj, model, n), layout: null, rows: [], moves: [], phase: 'idle' };
}
let game = null, cj = null;                         // Connect 4: the game (turns, the arm's plans, the ball pool) and its geometry
if (C4) {
  cj = await get('connect4.json', 'json');
  game = new C4Game({ mj, model, data, cj, demo, world: makeWorld(cj), DemoRunner });
}
const run = C4 ? game.run : SORT ? sort.run : new DemoRunner(mj, model, data, demo);
const palm = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'palm');

// ---------------- scene
say('Starting the renderer…');
const scene = new THREE.Scene(); scene.background = new THREE.Color(0xe6ebf0);
const camera = new THREE.PerspectiveCamera(40, innerWidth / innerHeight, 0.01, 30);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFShadowMap; renderer.shadowMap.autoUpdate = false;   // shadows are redrawn every other frame (below)
document.body.prepend(renderer.domElement);
scene.add(new THREE.HemisphereLight(0xf4f8fc, 0x8c96a0, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 2.0); sun.position.copy(fromMj(0.6, -0.8, 1.6)); sun.target.position.copy(fromMj(0.2, 0, 0));
sun.castShadow = true; sun.shadow.mapSize.set(1536, 1536); sun.shadow.bias = -0.0005;
Object.assign(sun.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.2, far: 4 });
scene.add(sun, sun.target);
const bodies = buildScene(mj, model, scene);
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });

// ---------------- camera views (MuJoCo coordinates, m)
const VIEWS = C4 ? { overview: [[1.6, -1.3, 0.55], [0.30, 0.0, -0.10]], board: [[1.95, 0.0, 0.0], [0.44, 0.0, -0.06]], side: [[0.38, -1.8, 0.0], [0.38, 0.0, -0.06]], hand: [[0.42, -0.42, 0.24], [0, 0, 0]] }
  : SORT ? { overview: [[1.15, -1.05, 0.80], [0.30, 0.0, 0.10]], side: [[0.30, -1.45, 0.35], [0.30, 0.0, 0.12]], hand: [[0.42, -0.42, 0.24], [0, 0, 0]] }
  : { overview: [[0.95, -0.85, 0.62], [0.22, 0.0, 0.12]], side: [[0.25, -1.25, 0.30], [0.25, 0.0, 0.17]], hand: [[0.42, -0.42, 0.24], [0, 0, 0]] };
let view = 'overview';
const palmPos = () => fromMj(data.xpos[3 * palm], data.xpos[3 * palm + 1], data.xpos[3 * palm + 2]);
function setView(v) {
  view = v;
  const [eye, at] = VIEWS[v], o = v === 'hand' ? palmPos() : new THREE.Vector3();
  controls.target.copy(fromMj(...at).add(o)); camera.position.copy(fromMj(...eye).add(o)); controls.update();
  document.querySelectorAll('#view button').forEach(b => b.classList.toggle('on', b.dataset.v === v));
}

// ---------------- shift-drag: a capped spring from the grabbed point to the cursor (in the plane facing the camera)
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2(), K = 40, FMAX = 6;   // N/m, N
let grab = null;
const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
  new THREE.LineBasicMaterial({ color: 0xc0392b })); line.visible = false; line.frustumCulled = false; scene.add(line);
function pick(e) {
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1); ray.setFromCamera(ndc, camera);
}
renderer.domElement.addEventListener('pointerdown', e => {
  if (!e.shiftKey || e.button !== 0) return;
  pick(e);
  const hit = ray.intersectObjects(scene.children, true).find(h => h.object.bodyID > 0);
  if (!hit) return;
  const b = hit.object.bodyID, p = toMj(hit.point), x = [0, 1, 2].map(i => data.xpos[3 * b + i]), R = data.xmat.slice(9 * b, 9 * b + 9);
  const d = [p[0] - x[0], p[1] - x[1], p[2] - x[2]];            // grabbed point in the body frame: R^T (p - x)
  grab = { b, local: [0, 1, 2].map(j => R[j] * d[0] + R[3 + j] * d[1] + R[6 + j] * d[2]),
           plane: new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()).negate(), hit.point), target: hit.point.clone() };
  controls.enabled = false; line.visible = true; e.preventDefault();
}, true);
addEventListener('pointermove', e => { if (!grab) return; pick(e); ray.ray.intersectPlane(grab.plane, grab.target); });
addEventListener('pointerup', () => { grab = null; controls.enabled = true; line.visible = false; });
const grabPoint = () => {
  const b = grab.b, R = data.xmat.slice(9 * b, 9 * b + 9), l = grab.local;
  return [0, 1, 2].map(i => data.xpos[3 * b + i] + R[3 * i] * l[0] + R[3 * i + 1] * l[1] + R[3 * i + 2] * l[2]);
};
run.drag = () => {
  if (!grab) return;
  const p = grabPoint(), t = toMj(grab.target);
  let f = [0, 1, 2].map(i => K * (t[i] - p[i])); const n = Math.hypot(...f); if (n > FMAX) f = f.map(c => c * FMAX / n);
  mj.mj_applyFT(model, data, f, [0, 0, 0], p, grab.b, data.qfrc_applied);
};

// ---------------- controls
let paused = false, rate = 1;
const playBtn = document.getElementById('play');
const setRate = v => { rate = v; document.querySelectorAll('#speed button').forEach(x => x.classList.toggle('on', +x.dataset.v === v)); };
setRate(C4 ? 2 : 1);
const setPaused = p => { paused = p; playBtn.textContent = p ? 'Play' : 'Pause'; playBtn.classList.toggle('on', p); };
playBtn.onclick = () => setPaused(!paused);
const randomizeBtn = document.getElementById('randomize'), solveBtn = document.getElementById('solve');
function randomize() {
  if (!SORT) { run.reset(); setPaused(false); return; }
  sort.layout = randomSortLayout(Math.random, sort.bj.order, sort.world.slotNames);
  sort.moves = solveSort(sort.layout, sort.bj.order, sort.world.slotNames); sort.rows = []; sort.phase = 'idle';
  run.demo.rows = []; run.reset(); placePieces(mj, model, data, sort.world, sort.layout, sort.pieces); setPaused(false);
}
function solve() {
  if (!SORT || !sort.layout || sort.phase === 'planning') return;
  sort.phase = 'planning'; statusEl.innerHTML = 'Planning the moves…';
  setTimeout(() => {                                // let the page paint first: the planner takes ~1-2 s
    try {
      const moves = solveSort(sort.layout, sort.bj.order, sort.world.slotNames);
      const p = plan(sort.world, sort.layout, null, demo.q0, undefined, { moves });
      sort.moves = moves; sort.rows = p.rows; run.demo.rows = p.rows; sort.phase = 'solving';
      run.reset(); placePieces(mj, model, data, sort.world, sort.layout, sort.pieces); setPaused(false);
    } catch (e) { sort.phase = 'idle'; statusEl.textContent = 'Could not plan: ' + e.message; }
  }, 30);
}
randomizeBtn.onclick = randomize; solveBtn.onclick = solve;
document.getElementById('restart').onclick = () => { if (SORT && sort.layout) { run.demo.rows = sort.rows; run.reset(); placePieces(mj, model, data, sort.world, sort.layout, sort.pieces); sort.phase = sort.rows.length ? 'solving' : 'idle'; setPaused(false); } else { run.reset(); setPaused(false); } };
document.querySelectorAll('#speed button').forEach(b => b.onclick = () => setRate(+b.dataset.v));
document.querySelectorAll('#view button').forEach(b => b.onclick = () => setView(b.dataset.v));
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if (e.code === 'Space') { setPaused(!paused); e.preventDefault(); }
  else if (C4) {
    if (/^[1-7]$/.test(e.key)) humanClick(+e.key - 1);
    else if (e.key === 'n' || e.key === 'N') newGame();
    else if (e.key === 'v' || e.key === 'V') { const vs = ['overview', 'board', 'hand', 'side']; setView(vs[(vs.indexOf(view) + 1) % vs.length]); }
  }
  else if (e.key === 'r' || e.key === 'R') randomize();
  else if (e.key === 's' || e.key === 'S' || e.key === 'Enter') solve();
  else if (e.key === '1') setView('overview'); else if (e.key === '2') setView('hand'); else if (e.key === '3') setView('side');
});


// ---------------- Connect 4: columns to click, the ghost ball, new game, status
const tally = { you: 0, arm: 0, draw: 0 };
let hoverCol = -1, thinkPending = false, counted = false, winBalls = [], ghost = null;
const colBoxes = [];
if (C4) {
  const b = cj.board, R = cj.ball_r / 1000;
  ghost = new THREE.Mesh(new THREE.SphereGeometry(R, 32, 20), new THREE.MeshStandardMaterial({ color: 0xf2c818, transparent: true, opacity: 0.5, roughness: 0.5 }));
  ghost.visible = false; scene.add(ghost);
  b.y_col.forEach((yc, c) => {                      // invisible click targets: the whole column, and the space above it
    const z0 = b.plate_top, z1 = b.rim + 260, h = (z1 - z0) / 1000, m = new THREE.Mesh(new THREE.BoxGeometry((b.ch_depth + 12) / 1000, h, (b.pitch - 6) / 1000),
      new THREE.MeshBasicMaterial({ color: 0xf2c818, transparent: true, opacity: 0, depthWrite: false }));
    m.position.copy(fromMj(b.x / 1000, yc / 1000, (z0 + z1) / 2000)); m.col = c; scene.add(m); colBoxes.push(m);
  });
}
function hoverFrom(e) {
  pick(e);
  const hit = ray.intersectObjects(colBoxes, false)[0];
  hoverCol = hit ? hit.object.col : -1;
}
function humanClick(c) { if (game.humanMove(c)) hoverCol = -1; }
function newGame() {
  if (!C4) return;
  game.newGame({ level: levelKey, humanFirst: youFirst });
  winBalls = []; counted = false; thinkPending = false; setPaused(false);
  for (const p of game.sim.pieces) bodies[p.body].traverse(o => { if (o.material && o.material.emissive) o.material.emissive.copy(o.material.color).multiplyScalar(0.3); });   // a little self-light: the channels are deep in shadow
}
let levelKey = 'medium', youFirst = true;
if (C4) {
  renderer.domElement.addEventListener('pointermove', e => { if (!grab && !e.buttons) hoverFrom(e); });
  renderer.domElement.addEventListener('pointerleave', () => { hoverCol = -1; });
  let down = null;
  renderer.domElement.addEventListener('pointerdown', e => { down = { x: e.clientX, y: e.clientY, shift: e.shiftKey }; });
  renderer.domElement.addEventListener('pointerup', e => {
    if (!down || down.shift || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5) { down = null; return; }
    down = null; hoverFrom(e); if (hoverCol >= 0) humanClick(hoverCol);
  });
  document.getElementById('newgame').onclick = newGame;
  document.querySelectorAll('#first button').forEach(b => b.onclick = () => {
    youFirst = b.dataset.v === 'you'; document.querySelectorAll('#first button').forEach(x => x.classList.toggle('on', x === b)); });
  document.querySelectorAll('#level button').forEach(b => b.onclick = () => {
    levelKey = b.dataset.v; game.level = levelKey; document.querySelectorAll('#level button').forEach(x => x.classList.toggle('on', x === b)); });
}
const disc = k => `<span class="disc ${k}"></span>`;
function c4Status() {
  const g = game, last = g.moves.length ? g.moves[g.moves.length - 1] : null;
  switch (g.phase) {
    case 'human': return `${g.moves.length ? '<span class="step">Move ' + (g.moves.length + 1) + '</span>' : ''}${disc('y')}Your turn: click a column <span class="grip">${LEVELS[g.level].name} · you are yellow</span>`;
    case 'drop': return `<span class="step">Move ${g.moves.length}</span>${disc('y')}Your ball is dropping into column ${last.col + 1}`;
    case 'think': return `<span class="step">Move ${g.moves.length + 1}</span>${disc('r')}The arm is thinking…`;
    case 'robot': {
      const r = g.run.phase === 'settle' ? null : g.plan.rows[Math.max(g.run.row, 0)];
      const what = g.run.phase === 'grip' ? (g.run.closing ? 'closing the gripper' : 'opening the gripper') : r ? r.note.replace(/^column \d+: /, '') : 'starting';
      return `<span class="step">Move ${g.moves.length}</span>${disc('r')}The arm plays column ${g.plan.col + 1}: ${what}<span class="grip">gripper ${g.run.gripperState()}</span>`;
    }
    case 'settle': return `<span class="step">Move ${g.moves.length}</span>${disc('r')}The ball is coming to rest in column ${last.col + 1}`;
    case 'over': {
      if (!g.win) return `<span class="win">Draw</span>The board is full <span class="grip">press New game</span>`;
      return `<span class="win">${g.win.who === 1 ? 'You win!' : 'The arm wins!'}</span>four in a row <span class="grip">press New game</span>`;
    }
    default: return 'A ball did not land where the game expects it. Press New game.';
  }
}
function c4Frame(t) {
  const g = game;
  g.sim.pieces.forEach((p, i) => { bodies[p.body].visible = g.sim.used[i]; });
  if (g.phase === 'think' && !thinkPending) {
    thinkPending = true;                                       // let the status text paint first: planning takes ~0.5 s
    setTimeout(() => { try { g.think(); } catch (err) { g.phase = 'fault'; console.error(err); statusEl.textContent = 'The arm could not plan that move: ' + err.message; } thinkPending = false; }, 40);
  }
  const showGhost = g.phase === 'human' && hoverCol >= 0 && !paused;
  ghost.visible = showGhost;
  if (showGhost) ghost.position.copy(fromMj(cj.board.x / 1000, cj.board.y_col[hoverCol] / 1000, (cj.board.rim + cj.ball_r + 30) / 1000));
  colBoxes.forEach(m => { m.material.opacity = g.phase === 'human' && m.col === hoverCol ? 0.16 : 0; });
  if (g.phase === 'over') {
    if (!counted) { counted = true; if (!g.win) tally.draw++; else if (g.win.who === 1) tally.you++; else tally.arm++;
      winBalls = g.win ? g.win.cells.map(([c, r]) => g.ballAt(c, r)).filter(i => i >= 0) : []; }
    const k = 0.6 + 0.3 * Math.sin(t / 150);                   // the winning four glow
    for (const i of winBalls) bodies[g.sim.pieces[i].body].traverse(o => { if (o.material && o.material.emissive) o.material.emissive.copy(o.material.color).multiplyScalar(k); });
  }
  document.getElementById('tally').textContent = `you ${tally.you} · arm ${tally.arm} · draws ${tally.draw}`;
}

// ---------------- loop: step the physics to keep up with the clock (x rate), within a per-frame time budget
syncBodies(data, bodies); setView('overview');
document.getElementById('loading').remove();
const statusEl = document.getElementById('status'), clockEl = document.getElementById('clock');
const NAMES = ['small', 'medium', 'large'], SEAT = { D: '1', A: '2', B: '3', E: '4' };
function sortStatus() {
  if (!sort.layout) return 'Press <b>Randomize</b> to scramble the balls';
  if (sort.phase === 'planning') return 'Planning the moves…';
  if (sort.phase === 'idle') return `<span class="step">${sort.moves.length} moves needed</span>Press <b>Solve</b> and the arm sorts them: small, medium, large from the left`;
  if (run.done && sort.phase === 'solving') {
    const fin = readPieces(model, data, sort.world, sort.pieces, 30), ok = fin.every((p, i) => p.slot === sort.bj.order[i]);
    sort.phase = ok ? 'done' : 'failed';
  }
  if (sort.phase === 'done') return `<span class="step">Sorted</span>${sort.moves.length} moves, small → medium → large from the left <span class="grip">press Randomize to go again</span>`;
  if (sort.phase === 'failed') return 'The arm finished, but the balls are not sorted (a ball slipped)';
  const r = sort.rows[Math.max(run.row, 0)], m = sort.moves[r ? r.move : 0];
  if (!r || run.phase === 'settle') return 'Starting…';
  const clean = t => t.replace(/\s*\[.*?\]/g, '').replace(/over ([DABE])$/, (_, k) => 'over seat ' + SEAT[k]).replace(/down to the bead on ([DABE])/, (_, k) => 'down onto the ball on seat ' + SEAT[k])
    .replace('lift clear of the peg', 'lift the ball clear').replace(/lower onto ([DABE])/, (_, k) => 'lower onto seat ' + SEAT[k]).replace('carry over', 'carry over seat').replace(/seat ([DABE])/, (_, k) => 'seat ' + SEAT[k]);
  const what = run.phase === 'grip' ? (run.closing ? 'closing the gripper' : 'opening the gripper') : clean(r.note.split(': ')[1]);
  return `<span class="step">Move ${r.move + 1} / ${sort.moves.length}</span>${NAMES[m.bead]} ball, seat ${SEAT[m.from]} → ${SEAT[m.to]}: ${what}<span class="grip">gripper ${run.gripperState()}</span>`;
}
let slow = 16, lastFrame = performance.now(), checked = 0, ratio = Math.min(devicePixelRatio, 2);
let ahead = 0, last = performance.now(), acc = { sim: 0, wall: 0 }, shown = 1, frame = 0, lastPalm = palmPos();
renderer.setAnimationLoop(() => {
  const now = performance.now(), dt = Math.min((now - last) / 1000, 0.1); last = now;
  if (!paused) {
    ahead += dt * rate; let n = 0; const budget = now + 12;
    while (ahead >= run.dt && performance.now() < budget) { if (C4) game.step(); else run.step(); ahead -= run.dt; n++; }
    if (ahead > 0.05) ahead = 0.05;                          // can't keep up: run slower rather than fall further behind
    acc.sim += n * run.dt; acc.wall += dt;
    if (acc.wall > 0.5) { shown = acc.sim / acc.wall; acc.sim = acc.wall = 0; }
  }
  syncBodies(data, bodies);
  if (C4) c4Frame(now);
  if (view === 'hand') { const p = palmPos(), d = p.clone().sub(lastPalm); camera.position.add(d); controls.target.add(d); lastPalm = p; }
  else lastPalm = palmPos();
  if (grab) { const p = grabPoint(); line.geometry.setFromPoints([fromMj(...p), grab.target]); }
  controls.update(); renderer.shadowMap.needsUpdate = frame % 2 === 0; renderer.render(scene, camera);
  // slow machine (software rendering, big screen): lower the pixel ratio step by step, never back up
  slow = 0.95 * slow + 0.05 * (now - lastFrame); lastFrame = now;
  if (++checked > 90 && slow > 24 && ratio > 1) { ratio = Math.max(1, ratio - 0.25); renderer.setPixelRatio(ratio); checked = 0; }
  if (++frame % 10 === 0) {
    const s = SORT || C4 ? null : run.status();
    if (C4) statusEl.innerHTML = c4Status(); else if (SORT) statusEl.innerHTML = sortStatus(); else
    statusEl.innerHTML = `<span class="step">Step ${s.row} / ${s.of}</span>${s.text}<span class="grip">gripper ${run.gripperState()}</span>`;
    clockEl.textContent = paused ? `t ${run.t.toFixed(1)} s · paused`
      : `t ${run.t.toFixed(1)} s · ${shown.toFixed(1)}× real time${shown < rate * 0.9 ? ' (this computer can’t keep up with ' + rate + '×)' : ''}`;
  }
});
if (SORT) randomize();
if (C4) newGame();
window.__p13 = { mj, model, data, run, sort, randomize, solve, game, newGame, humanClick: C4 ? humanClick : null };
