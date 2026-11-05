// Bundle src/main.js -> dist/main.js and copy the page, the scene, the demo and MuJoCo's wasm next to it.
import { build } from 'esbuild';
import { cpSync, mkdirSync, rmSync } from 'fs';
rmSync('dist', { recursive: true, force: true }); mkdirSync('dist', { recursive: true });
await build({ entryPoints: ['src/main.js'], bundle: true, minify: true, format: 'esm', outfile: 'dist/main.js', logLevel: 'info',
  external: ['module', 'fs', 'path', 'url', 'worker_threads', 'os'] });
cpSync('index.html', 'dist/index.html');
cpSync('public', 'dist', { recursive: true });
cpSync('node_modules/@mujoco/mujoco/mujoco.wasm', 'dist/mujoco.wasm');
