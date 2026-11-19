#!/usr/bin/env node
// Render side-by-side videos: an earlier solution (or, where there is none,
// the game's own sample solution) on the left, the new solution on the right,
// drawn by the game's own drawing code.
//   node agent/harness/video.js [nameFilter] [outDir=agent/videos]
// Needs Playwright (with Chromium) and ffmpeg.
'use strict';
const fs = require('fs'), path = require('path'), http = require('http'), { spawn } = require('child_process');
const game = require('./game'), { OLD } = require('./old');
const GAME = path.join(__dirname, '.game'), ROOT = path.join(__dirname, '..', '..');
const [filter = '', outDir = path.join(__dirname, '..', 'videos')] = process.argv.slice(2);
const FPS = 25, W = 1280, H = 620, STEP = 0.02;
const baselines = JSON.parse(fs.readFileSync(path.join(__dirname, 'baselines.json'), 'utf8'));

const PAGE = `<!doctype html><meta charset="utf-8"><body style="margin:0;background:#fff">
<canvas id="out" width="${W}" height="${H}"></canvas>
<script src="js/jquery-1.11.2.js"></script><script src="js/numeric-1.2.6.js"></script>
${fs.readdirSync(path.join(GAME, 'js', 'models')).map(f => `<script src="js/models/${f}"></script>`).join('')}
${fs.readdirSync(path.join(GAME, 'js', 'levels')).map(f => `<script src="js/levels/${f}"></script>`).join('')}
<script src="js/ImageDataCache.js"></script><script src="js/utils.js"></script>
<script>
function monitor() {}
var sides = [], out = document.getElementById('out'), octx = out.getContext('2d'), title = '', speed = 1;
function setup(level, entries) {
  title = new Levels[level]().title; speed = 1;
  sides = entries.map(function (e) {
    var canvas = document.createElement('canvas'); canvas.width = ${W / 2}; canvas.height = 480;
    return {label: e.label, mine: e.mine, level: new Levels[level](), canvas: canvas, ctx: canvas.getContext('2d'),
            control: new Function("var controlFunction;\\ncontrolFunction = (function(){\\n'use strict';\\n" + e.code + "\\nreturn controlFunction;\\n})();\\nreturn controlFunction;")(),
            done: null, failed: null, error: null};
  });
}
function advance(steps) {
  sides.forEach(function (s) {
    for (var i = 0; i < steps && s.done === null && s.failed === null; i++) {
      try { s.level.simulate(${STEP}, s.control); } catch (e) { s.failed = s.level.getSimulationTime(); s.error = String(e); break; }
      if (s.level.levelFailed()) s.failed = s.level.getSimulationTime();
      else if (s.level.levelComplete()) s.done = s.level.getSimulationTime();
    }
  });
  return sides.map(function (s) { return {time: s.level.getSimulationTime(), done: s.done, failed: s.failed, error: s.error}; });
}
function frame() {
  octx.fillStyle = '#fff'; octx.fillRect(0, 0, ${W}, ${H});
  octx.fillStyle = '#1f2430'; octx.fillRect(0, 0, ${W}, 56);
  octx.fillStyle = '#fff'; octx.font = '600 24px sans-serif'; octx.textAlign = 'center'; octx.textBaseline = 'middle';
  octx.fillText(title, ${W / 2}, 29);
  if (speed > 1) { octx.textAlign = 'right'; octx.font = '600 18px sans-serif'; octx.fillStyle = '#ffd166'; octx.fillText('fast-forward \\u00d7' + speed, ${W - 20}, 29); }
  sides.forEach(function (s, i) {
    var x = i * ${W / 2};
    s.ctx.save(); s.level.draw(s.ctx, s.canvas); s.ctx.restore();
    octx.drawImage(s.canvas, x, 56);
    octx.fillStyle = s.mine ? '#e8f1ff' : '#f3f3f3'; octx.fillRect(x, 536, ${W / 2}, 84);
    octx.fillStyle = '#1f2430'; octx.textAlign = 'left'; octx.font = '600 22px sans-serif';
    octx.fillText(s.label, x + 20, 564);
    octx.font = '20px sans-serif';
    var t = s.level.getSimulationTime().toFixed(2) + ' s';
    if (s.done !== null) { octx.fillStyle = '#12833b'; octx.fillText('\\u2714 solved in ' + s.done.toFixed(2) + ' s', x + 20, 596); }
    else if (s.failed !== null) { octx.fillStyle = '#c0392b'; octx.fillText('\\u2718 failed at ' + s.failed.toFixed(2) + ' s', x + 20, 596); }
    else { octx.fillStyle = '#555'; octx.fillText(t, x + 20, 596); }
  });
  octx.fillStyle = '#1f2430'; octx.fillRect(${W / 2 - 1}, 56, 2, ${H - 56});
  return out.toDataURL('image/jpeg', 0.92);
}
</script>`;

const TYPES = { '.js': 'text/javascript', '.png': 'image/png', '.css': 'text/css', '.html': 'text/html' };
function serve() {
  return new Promise(resolve => {
    const server = http.createServer((req, res) => {
      const url = req.url.split('?')[0];
      if (url === '/video.html') { res.writeHead(200, { 'content-type': 'text/html' }); return res.end(PAGE); }
      const file = path.join(GAME, path.normalize(url));
      if (!file.startsWith(GAME) || !fs.existsSync(file)) { res.writeHead(404); return res.end(); }
      res.writeHead(200, { 'content-type': TYPES[path.extname(file)] || 'application/octet-stream' });
      fs.createReadStream(file).pipe(res);
    }).listen(0, '127.0.0.1', () => resolve(server));
  });
}

(async () => {
  const { chromium } = require('playwright');
  fs.mkdirSync(outDir, { recursive: true });
  const server = await serve(), base = 'http://127.0.0.1:' + server.address().port;
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: W, height: H } });
  page.on('pageerror', e => console.error('page error:', e.message));
  for (const level of game.levelNames()) {
    if (!level.includes(filter)) continue;
    const mine = fs.readFileSync(path.join(__dirname, '..', 'solutions', level + '.js'), 'utf8');
    const old = OLD[level] && fs.readFileSync(path.join(ROOT, OLD[level] + '.js'), 'utf8');
    const b = baselines[level] || {}, other = old ? b.old : b.sample;
    const otherTime = other && other[0] === 'SOLVED' ? other[1] : 60;
    const entries = [
      old ? { label: 'Barış', code: old } : { label: "Game's sample solution (none from Barış)", code: game.sampleSolution(level) },
      { label: 'Fable', code: mine, mine: true },
    ];
    await page.goto(base + '/video.html');
    await page.evaluate(([l, e]) => setup(l, e), [level, entries]);
    await page.evaluate(() => frame());              // let the game start loading its images
    await page.waitForTimeout(700);
    const file = path.join(outDir, level + '.mp4');
    const ff = spawn('ffmpeg', ['-y', '-loglevel', 'error', '-f', 'image2pipe', '-framerate', String(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '24', '-movflags', '+faststart', file], { stdio: ['pipe', 'inherit', 'inherit'] });
    const write = async url => { if (!ff.stdin.write(Buffer.from(url.split(',')[1], 'base64'))) await new Promise(r => ff.stdin.once('drain', r)); };
    for (let i = 0; i < FPS / 2; i++) await write(await page.evaluate(() => frame()));     // hold the start for a moment
    let after = 0, speed = 1, state;
    for (let n = 0; n < 20000; n++) {
      // Real time until the new solution has finished; then fast-forward what is left.
      const r = await page.evaluate(([steps, sp]) => { speed = sp; var st = advance(steps); return { st: st, url: frame() }; }, [speed * Math.round(1 / (FPS * STEP)), speed]);
      state = r.st; await write(r.url);
      const over = s => s.done !== null || s.failed !== null;
      if (state.every(over) || state[0].time > Math.max(2.5 * otherTime, otherTime + 20)) { if (++after > 2 * FPS) break; }
      else if (over(state[1]) && speed === 1 && state[0].time > (state[1].done || state[1].failed) + 1.5) {
        speed = Math.max(1, Math.ceil((otherTime - state[0].time) / 6));
      }
    }
    ff.stdin.end();
    await new Promise(r => ff.on('close', r));
    console.log(level.padEnd(30), 'left', JSON.stringify(state[0]), 'right', JSON.stringify(state[1]));
  }
  await browser.close(); server.close();
})();
