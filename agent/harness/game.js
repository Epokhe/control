// Headless loader for the ControlChallenges game (github.com/janismac/ControlChallenges).
// It evaluates the game's own model and level files in a Node vm context, so a
// controller runs against exactly the physics the browser runs.
'use strict';
const fs = require('fs'), vm = require('vm'), path = require('path'), zlib = require('zlib');

const GAME_DIR = process.env.GAME_DIR || path.join(__dirname, '.game');
const DT = 0.02; // the game's fixed control period (ui.js)

// Minimal PNG decoder (8-bit RGBA, non-interlaced), enough for img/track.png.
function decodePNG(file) {
  const buf = fs.readFileSync(file);
  let pos = 8, width, height, idat = [];
  while (pos < buf.length) {
    const len = buf.readUInt32BE(pos), type = buf.toString('ascii', pos + 4, pos + 8);
    const data = buf.subarray(pos + 8, pos + 8 + len);
    if (type === 'IHDR') {
      width = data.readUInt32BE(0); height = data.readUInt32BE(4);
      if (data[8] !== 8 || data[9] !== 6 || data[12] !== 0) throw new Error('unsupported PNG format');
    } else if (type === 'IDAT') idat.push(data);
    pos += 12 + len;
  }
  const raw = zlib.inflateSync(Buffer.concat(idat)), bpp = 4, stride = width * bpp;
  const out = new Uint8Array(width * height * bpp);
  for (let y = 0; y < height; y++) {
    const f = raw[y * (stride + 1)], rowIn = y * (stride + 1) + 1, rowOut = y * stride;
    for (let i = 0; i < stride; i++) {
      const a = i >= bpp ? out[rowOut + i - bpp] : 0;
      const b = y > 0 ? out[rowOut - stride + i] : 0;
      const c = i >= bpp && y > 0 ? out[rowOut - stride + i - bpp] : 0;
      let p = 0;
      if (f === 1) p = a; else if (f === 2) p = b; else if (f === 3) p = (a + b) >> 1;
      else if (f === 4) { const q = a + b - c, pa = Math.abs(q - a), pb = Math.abs(q - b), pc = Math.abs(q - c); p = pa <= pb && pa <= pc ? a : pb <= pc ? b : c; }
      out[rowOut + i] = (raw[rowIn + i] + p) & 255;
    }
  }
  return { width, height, data: out };
}

const sources = [];
let images = null;
function loadSources() {
  if (sources.length) return;
  if (!fs.existsSync(path.join(GAME_DIR, 'js'))) throw new Error('Game source not found at ' + GAME_DIR + '. Run agent/harness/fetch-game.sh first.');
  const add = f => sources.push(new vm.Script(fs.readFileSync(f, 'utf8').replace(/'use strict';/, ''), { filename: f }));
  add(path.join(GAME_DIR, 'js/numeric-1.2.6.js'));
  for (const d of ['models', 'levels']) for (const f of fs.readdirSync(path.join(GAME_DIR, 'js', d)).sort()) add(path.join(GAME_DIR, 'js', d, f));
  images = { 'img/track.png': decodePNG(path.join(GAME_DIR, 'img/track.png')) };
}

function levelNames() {
  loadSources();
  return fs.readdirSync(path.join(GAME_DIR, 'js/levels')).sort().map(f => f.replace('.js', ''));
}

// The sample solution that ships inside each level file (used only as a baseline).
function sampleSolution(level) {
  const src = fs.readFileSync(path.join(GAME_DIR, 'js/levels', level + '.js'), 'utf8');
  const m = src.match(/sampleSolution = (".*");\s*$/m);
  return m ? vm.runInNewContext(m[1]) : null;
}

// Run one level. opts.perturb(level, ctx) may modify the freshly built level
// before the first step. Returns {status, time, steps, error}.
function runLevel(levelName, code, opts = {}) {
  loadSources();
  const maxT = opts.maxT || 120;
  const ctx = {
    console: opts.verbose ? console : { log() {} }, Math, alert() {}, monitor() {},
    ImageDataCache: { load() {}, get: u => images[u], at(u, x, y) { const im = images[u]; const i = (y * im.width + x) * 4; return [im.data[i], im.data[i + 1], im.data[i + 2], im.data[i + 3]]; } },
    round: (x, d) => { const s = Math.pow(10, d); return Math.round(x * s) / s; },
    padSpaces: (s, c) => String('                                 ' + s).slice(-c),
  };
  vm.createContext(ctx);
  for (const s of sources) s.runInContext(ctx);
  // Same wrapping the game's UI applies to the editor contents.
  vm.runInContext("controlFunction=undefined;controlFunction = (function(){\n'use strict';\n" + code + "\nreturn controlFunction;\n})();", ctx, { filename: opts.filename || 'solution.js' });
  vm.runInContext('var L = new Levels.' + levelName + '();', ctx);
  if (opts.perturb) opts.perturb(ctx.L, ctx);
  const trace = opts.trace;
  let status = 'TIMEOUT', error = null, steps = 0;
  const t0 = Date.now();
  let cpuMax = 0;
  try {
    const n = Math.round(maxT / DT);
    for (; steps < n;) {
      const c0 = process.hrtime.bigint();
      ctx.L.simulate(DT, ctx.controlFunction);
      const c = Number(process.hrtime.bigint() - c0) / 1e6; if (c > cpuMax) cpuMax = c;
      steps++;
      if (trace) trace(ctx.L, steps);
      if (ctx.L.levelFailed()) { status = 'FAILED'; break; }
      if (ctx.L.levelComplete()) { status = 'SOLVED'; break; }
    }
  } catch (e) { status = 'ERROR'; error = String(e && e.stack || e); }
  return { status, time: ctx.L.getSimulationTime(), steps, error, wallMs: Date.now() - t0, maxStepMs: cpuMax, level: ctx.L };
}

module.exports = { runLevel, levelNames, sampleSolution, GAME_DIR, DT };
