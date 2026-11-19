#!/usr/bin/env node
// Start each level from randomly disturbed initial conditions and report how
// often each solution still completes it, and how long it takes.
//   node agent/harness/robustness.js [nameFilter] [runs=20] [which=new,old,sample]
'use strict';
const fs = require('fs'), path = require('path'), game = require('./game'), { OLD } = require('./old');
const [filter = '', runs = '20', which = 'new'] = process.argv.slice(2);

// Size of the disturbance for each state, by level family (uniform, plus or minus).
const NOISE = [
  [/^Tutorial/, { x: 1, dx: 1 }],
  [/^CruiseControl/, { speed: 10 }],
  [/^StabilizeSingle/, { theta: 0.1, dtheta: 0.2, x: 0.5, dx: 0.3 }],
  [/^SwingUpSingle/, { theta: 0.5, dtheta: 0.5, x: 0.5, dx: 0.3 }],
  [/^StabilizeDouble/, { theta1: 0.03, dtheta1: 0.03, theta2: 0.03, dtheta2: 0.03, x: 0.3, dx: 0.1 }],
  [/^SwingUpDouble/, { theta1: 0.3, dtheta1: 0.2, theta2: 0.3, dtheta2: 0.2, x: 0.3, dx: 0.1 }],
  [/^Multirotor/, { x: 0.3, y: 0.3, dx: 0.5, dy: 0.5, theta: 0.3, dtheta: 0.5 }],
  [/^RocketLanding/, { x: 30, y: 30, dx: 8, dy: 8, theta: 0.2, dtheta: 0.1 }],
  [/^BallOnPlatform/, { ball_x: 0.15, ball_y: 0.3, ball_dx: 0.3, ball_dy: 0.5 }],
  [/^Vehicle/, { y: 1.5, heading: 0.15 }],
];

// Small deterministic random number generator, so every solution sees the same starts.
function rng(seed) { return () => { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }

const baselines = JSON.parse(fs.readFileSync(path.join(__dirname, 'baselines.json'), 'utf8'));
function source(level, kind) {
  if (kind === 'sample') return game.sampleSolution(level);
  const f = kind === 'new' ? path.join(__dirname, '..', 'solutions', level + '.js') : OLD[level] && path.join(__dirname, '..', '..', OLD[level] + '.js');
  return f && fs.existsSync(f) ? fs.readFileSync(f, 'utf8') : null;
}

console.log('level'.padEnd(30) + which.split(',').map(k => (k + ': solved, median s, worst s').padStart(34)).join(''));
for (const level of game.levelNames()) {
  if (!level.includes(filter)) continue;
  const noise = NOISE.find(([re]) => re.test(level))[1];
  let line = level.padEnd(30);
  for (const kind of which.split(',')) {
    const code = source(level, kind);
    if (!code) { line += '-'.padStart(34); continue; }
    const times = [];
    for (let i = 0; i < +runs; i++) {
      const rand = rng(1000 + i);
      const r = game.runLevel(level, code, { maxT: kind === "old" ? 150 : 3 * Math.max(baselines[level] && baselines[level].sample[1] || 20, 20), perturb(L) {
        for (const m of L.models || [L.model]) for (const k in noise) m[k] += (2 * rand() - 1) * noise[k];
      } });
      if (r.status === 'SOLVED') times.push(r.time);
    }
    times.sort((a, b) => a - b);
    line += (times.length + '/' + runs + (times.length ? ', ' + times[times.length >> 1].toFixed(2) + ', ' + times[times.length - 1].toFixed(2) : '')).padStart(34);
  }
  console.log(line);
}
