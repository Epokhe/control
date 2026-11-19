#!/usr/bin/env node
// Print selected model fields while a solution runs.
//   node agent/harness/trace.js <Level> <field,field,...> [everyNSteps] [maxT] [solutionFile]
'use strict';
const fs = require('fs'), path = require('path'), game = require('./game');
const [level, fields = 'T', every = '10', maxT = '20', file] = process.argv.slice(2);
const keys = fields.split(',');
const f = file || path.join(__dirname, '..', 'solutions', level + '.js');
const r = game.runLevel(level, fs.readFileSync(f, 'utf8'), {
  maxT: +maxT, verbose: true, filename: f,
  trace(L, step) {
    if (step % +every) return;
    const ms = L.models || [L.model];
    console.log(ms.map(m => keys.map(k => k + '=' + (typeof m[k] === 'number' ? m[k].toFixed(4) : m[k])).join(' ')).join(' | '));
  },
});
console.log(r.status, r.time.toFixed(2), r.error || '');
