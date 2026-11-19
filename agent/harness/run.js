#!/usr/bin/env node
// Run the solutions in agent/solutions against the real game physics.
//   node agent/harness/run.js            all levels
//   node agent/harness/run.js Rocket     only levels whose name contains "Rocket"
'use strict';
const fs = require('fs'), path = require('path');
const game = require('./game');
const SOL = path.join(__dirname, '..', 'solutions');
const baselines = fs.existsSync(path.join(__dirname, 'baselines.json')) ? JSON.parse(fs.readFileSync(path.join(__dirname, 'baselines.json'))) : {};
const filter = process.argv[2];
const fmt = b => b ? (b[0] === 'SOLVED' ? b[1].toFixed(2) : b[0]) : '-';
console.log('level'.padEnd(30), 'new'.padStart(16), 'old'.padStart(8), 'sample'.padStart(8), ' max ms/step');
let ok = true;
for (const level of game.levelNames()) {
  if (filter && !level.includes(filter)) continue;
  const f = path.join(SOL, level + '.js');
  const b = baselines[level] || {};
  if (!fs.existsSync(f)) { console.log(level.padEnd(30), 'missing'.padStart(16), fmt(b.old).padStart(8), fmt(b.sample).padStart(8)); ok = false; continue; }
  const r = game.runLevel(level, fs.readFileSync(f, 'utf8'), { filename: f });
  if (r.status !== 'SOLVED') ok = false;
  console.log(level.padEnd(30), (r.status + ' ' + r.time.toFixed(2)).padStart(16), fmt(b.old).padStart(8), fmt(b.sample).padStart(8), r.maxStepMs.toFixed(1).padStart(8));
  if (r.error) console.log(r.error.split('\n').slice(0, 4).join('\n'));
}
process.exit(ok ? 0 : 1);
