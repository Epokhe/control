// Ball on platform, bounce: keep the ball bouncing straight up and down with
// its highest point exactly at the origin.
//
// Every bounce is aimed (see "Aimed bounces" below). The ball starts off to
// the side, so the first bounce is a low hop that brings it down over the
// centre, and the next one sends it straight up. The same rule covers any
// later disturbance: a ball that is not coming down over the centre gets
// hopped there.

// ---------------------------------------------------------------------------
// Ball and paddle: the machine's dimensions and a model of it.
//
// The paddle is a bar on a hinge on top of a piston. The ball bounces off it
// without friction and keeps 90 % of its speed relative to the paddle.
// ---------------------------------------------------------------------------
var PADDLE = {
  g: 9.81, radius: 0.2, baseY: -5, halfWidth: 1.5, thickness: 0.15, restitution: 0.9,
  lengthMin: 1.5, lengthMax: 2.3, angleMax: 0.7, pistonAccel: 10, hingeAccel: 15,
  dt: 0.02, substeps: 5
};

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function paddleState(ball, piston, hinge)
{
  return {x: ball.x, y: ball.y, vx: ball.vx, vy: ball.vy, L: piston.length, v: piston.speed, th: hinge.angle, om: hinge.speed};
}

// Where the ball is relative to the paddle. along: position along the bar from
// its centre; gap: clearance between ball and bar; (nx, ny): direction from
// the nearest point of the bar to the ball; closing: speed at which the gap
// shrinks; (px, py), (pvx, pvy): that nearest point and its velocity.
function paddleContact(s)
{
  var P = PADDLE, C = Math.cos(s.th), S = Math.sin(s.th);
  var cx = -S * P.thickness, cy = P.baseY + s.L + C * P.thickness;
  var along = C * (s.x - cx) + S * (s.y - cy);
  var a = clamp(along, -P.halfWidth, P.halfWidth);
  var px = cx + C * a, py = cy + S * a;
  var pvx = -C * s.om * P.thickness - S * s.om * a, pvy = s.v - S * s.om * P.thickness + C * s.om * a;
  var nx = s.x - px, ny = s.y - py, d = Math.sqrt(nx * nx + ny * ny);
  nx /= d; ny /= d;
  return {along: along, gap: d - P.radius, nx: nx, ny: ny, px: px, py: py, pvx: pvx, pvy: pvy,
          closing: -(nx * (s.vx - pvx) + ny * (s.vy - pvy))};
}

// Advance the model by one control period with constant accelerations, in the
// same small steps as the machine. Returns true if the ball bounced.
function paddleAdvance(s, pistonAccel, hingeAccel)
{
  var P = PADDLE, h = P.dt / P.substeps, bounced = false;
  pistonAccel = clamp(pistonAccel, -P.pistonAccel, P.pistonAccel);
  hingeAccel = clamp(hingeAccel, -P.hingeAccel, P.hingeAccel);
  for (var i = 0; i < P.substeps; i++) {
    s.x += h * s.vx;
    s.y += h * s.vy - 0.5 * h * h * P.g;
    s.vy -= h * P.g;
    if (s.y - P.radius < P.baseY) s.vy = Math.abs(s.vy);
    s.om += h * hingeAccel; s.th += h * s.om;
    s.v += h * pistonAccel; s.L += h * s.v;
    if (s.L > P.lengthMax) { s.L = P.lengthMax; s.v = Math.min(0, s.v); }
    if (s.L < P.lengthMin) { s.L = P.lengthMin; s.v = Math.max(0, s.v); }
    if (s.th > P.angleMax) { s.th = P.angleMax; s.om = Math.min(0, s.om); }
    if (s.th < -P.angleMax) { s.th = -P.angleMax; s.om = Math.max(0, s.om); }
    var c = paddleContact(s);
    if (c.closing > 0 && c.gap < 0) {
      var kick = (1 + P.restitution) * c.closing;
      s.vx += c.nx * kick; s.vy += c.ny * kick;
      s.x = c.px + c.nx * P.radius; s.y = c.py + c.ny * P.radius;
      bounced = true;
    }
  }
  return bounced;
}

// Time until a ball in free flight comes down to height y.
function fallTime(s, y)
{
  var disc = s.vy * s.vy + 2 * PADDLE.g * (s.y - y);
  return disc > 0 ? (s.vy + Math.sqrt(disc)) / PADDLE.g : 0;
}

// Height of the ball's centre when it rests on a level paddle.
function restHeight(length) { return PADDLE.baseY + length + PADDLE.thickness + PADDLE.radius; }

// ---------------------------------------------------------------------------
// Aimed bounces.
//
// Before a bounce the controller picks what the paddle should be doing at the
// moment of contact (piston length, speed and acceleration, and the tilt),
// drives piston and hinge there along smooth curves, and checks the aim by
// running the model forward to the bounce. Newton's method corrects speed and
// tilt until the predicted outcome is the wanted one. This is repeated every
// control period, so errors are corrected for as long as the ball is in the
// air.
// ---------------------------------------------------------------------------

// Shortly before contact the plan is no longer changed.
var AIM_COMMIT = 3.5 * PADDLE.dt;

// Piston and hinge accelerations over time that reach the goal {length, speed,
// accel, angle} (hinge at rest) after time tau, starting from state s.
function aimCurves(s, goal, tau)
{
  tau = Math.max(tau, PADDLE.dt);
  var D = goal.length - s.L - s.v * tau, E = goal.speed - s.v;
  var a0 = (12 * D - 6 * E * tau + goal.accel * tau * tau) / (tau * tau);
  var snap = (6 * (a0 + goal.accel) * tau - 12 * E) / (tau * tau * tau);
  var jerk = (goal.accel - a0 - 0.5 * snap * tau * tau) / tau;
  var Dh = goal.angle - s.th - s.om * tau, Eh = -s.om;
  var h0 = (6 * Dh - 2 * Eh * tau) / (tau * tau), h1 = 2 * (Eh - h0 * tau) / (tau * tau);
  return function (t) {
    if (t >= tau) return [goal.accel, 0];
    return [a0 + jerk * t + 0.5 * snap * t * t, h0 + h1 * t];
  };
}

// Whether those curves stay within what piston and hinge can do.
function aimFeasible(s, goal, tau, margin)
{
  var c = aimCurves(s, goal, tau), L = s.L, v = s.v, dt = PADDLE.dt;
  for (var t = 0; t < tau; t += dt) {
    var u = c(Math.min(t + 0.5 * dt, tau - 1e-9));
    if (Math.abs(u[0]) > PADDLE.pistonAccel * margin || Math.abs(u[1]) > PADDLE.hingeAccel * margin) return false;
    v += u[0] * dt; L += v * dt;
    if (L < PADDLE.lengthMin || L > PADDLE.lengthMax) return false;
  }
  return true;
}

// Run the model forward until the ball bounces. The curves are redrawn from
// the current state every period, as the controller itself does, until
// shortly before contact. Returns the state just after the bounce.
function aimShoot(s0, goal, tau)
{
  var s = {}, k, c, first, since = 0; for (k in s0) s[k] = s0[k];
  for (var i = 0; i < 150; i++) {
    var left = tau - i * PADDLE.dt;
    if (i === 0 || left > AIM_COMMIT) { c = aimCurves(s, goal, left); since = 0; if (i === 0) first = c; }
    var u = c((since + 0.5) * PADDLE.dt);
    since++;
    if (paddleAdvance(s, u[0], u[1])) return {s: s, curve: first, time: (i + 1) * PADDLE.dt};
  }
  return null;
}

// Rough paddle speed and tilt that send a ball, arriving at (vx, vIn), straight
// up to the height apogee from a paddle at the given piston length.
function roughAim(vx, vIn, length, apogee)
{
  var vOut = Math.sqrt(Math.max(0, 2 * PADDLE.g * (apogee - restHeight(length))));
  return {speed: (vOut + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(vx / (vOut - vIn))};
}

// One aimer plans one bounce at a time. Call step every control period with
// the current state and the target:
//   {length, accel: piston length and acceleration at contact,
//    guess(tau): first guess {speed, angle},
//    miss(after): two numbers that are zero when the bounce is as wanted,
//    fixed: true to use the guess as it is}
function makeAimer()
{
  var aim = null, plan = null, left = null;
  function goalOf(target) { return {length: target.length, accel: target.accel, speed: aim.speed, angle: aim.angle}; }
  return {
    reset: function () { aim = null; plan = null; left = null; },
    step: function (s, target) {
      var dt = PADDLE.dt, u;
      if (plan && plan.left <= AIM_COMMIT) {
        plan.step++; plan.left -= dt;
        u = plan.curve((plan.step + 0.5) * dt);
        return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
      }
      // Time to contact: counted down from the first estimate, which is
      // corrected by what the model says.
      var tau = left === null ? fallTime(s, restHeight(target.length)) : left - dt, fresh = left === null ? 2 : 0;
      if (!aim || target.fixed) aim = target.guess(tau);
      var out = null;
      for (var iter = 0; iter < 8; iter++) {
        out = aimShoot(s, goalOf(target), tau);
        if (!out) break;
        if (fresh > 0 && Math.abs(out.time - tau) > 0.5 * dt) { tau = out.time; fresh--; continue; }
        if (target.fixed) break;
        var f = target.miss(out.s);
        if (Math.abs(f[0]) + Math.abs(f[1]) < 1e-10) break;
        var eps = 1e-5, g = goalOf(target);
        g.speed += eps; var a = aimShoot(s, g, tau);
        g.speed -= eps; g.angle += eps; var b = aimShoot(s, g, tau);
        if (!a || !b) break;
        var fa = target.miss(a.s), fb = target.miss(b.s);
        var j00 = (fa[0] - f[0]) / eps, j10 = (fa[1] - f[1]) / eps, j01 = (fb[0] - f[0]) / eps, j11 = (fb[1] - f[1]) / eps;
        var det = j00 * j11 - j01 * j10;
        if (Math.abs(det) < 1e-12) break;
        aim.speed -= clamp((j11 * f[0] - j01 * f[1]) / det, -1, 1);
        aim.angle = clamp(aim.angle - clamp((-j10 * f[0] + j00 * f[1]) / det, -0.2, 0.2), -0.6, 0.6);
      }
      if (!out) return null;
      left = tau;
      plan = {curve: out.curve, step: 0, left: tau};
      u = out.curve(0.5 * dt);
      return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
    }
  };
}

var HOP_APOGEE = -2.0;                // top of a hop
var LENGTH = {hop: 1.75, afterHop: 2.1, steady: 2.1};   // piston length to bounce at
var CENTRED = 0.008;                  // sideways error small enough to trim without a hop
// Paddle acceleration at contact. With this value a bounce that happens a
// little earlier or later than expected sends the ball equally high.
var CONTACT_ACCEL = -PADDLE.restitution / (1 + PADDLE.restitution) * PADDLE.g;

var aimer = makeAimer(), target = null, last = null, hopped = false;

// Bounce the ball to the centre: the hop has a given height and must come
// down, at the following bounce, over the centre.
function hop(s)
{
  return {
    hop: true, length: LENGTH.hop, accel: CONTACT_ACCEL,
    guess: function (tau) { return {speed: roughAim(0, s.vy - PADDLE.g * tau, LENGTH.hop, HOP_APOGEE).speed, angle: 0}; },
    miss: function (after) {
      var landing = nextBounce(after, LENGTH.afterHop);
      return [after.y + after.vy * after.vy / (2 * PADDLE.g) - HOP_APOGEE, landing ? landing.s.x : after.x];
    }
  };
}

// Bounce the ball straight up to the origin. A small sideways offset that is
// left is removed slowly.
function straightUp(s, length)
{
  return {
    length: length, accel: CONTACT_ACCEL,
    guess: function (tau) { return roughAim(s.vx, s.vy - PADDLE.g * tau, length, 0); },
    miss: function (after) {
      var drift = clamp(-after.x / (2 * after.vy / PADDLE.g), -0.005, 0.005);
      return [after.y + after.vy * after.vy / (2 * PADDLE.g), after.vx - drift];
    }
  };
}

// Where the ball will next bounce if it is then sent straight up.
function nextBounce(s, length)
{
  var tau = fallTime(s, restHeight(length)), rough = roughAim(s.vx, s.vy - PADDLE.g * tau, length, 0);
  return aimShoot(s, {length: length, accel: CONTACT_ACCEL, speed: rough.speed, angle: rough.angle}, tau);
}

function controlFunction(ball, piston, hinge, T)
{
  var s = paddleState(ball, piston, hinge);
  // A jump in the ball's velocity means it has bounced: aim the next one.
  if (last !== null && Math.abs(ball.vy - (last - PADDLE.g * PADDLE.dt)) > 1e-6) { hopped = target.hop; target = null; aimer.reset(); }
  last = ball.vy;
  if (!target) {
    var length = hopped ? LENGTH.afterHop : LENGTH.steady, landing = nextBounce(s, length);
    target = !landing || Math.abs(landing.s.x) > CENTRED ? hop(s) : straightUp(s, length);
  }
  return aimer.step(s, target) || {pistonAcceleration: -4 * piston.speed, hingeAcceleration: -4 * hinge.speed};
}
