// Ball on platform, balance: catch the falling ball and bring it to rest in
// the middle of the paddle. All of the work is in "Catching the ball" below.

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

// ---------------------------------------------------------------------------
// Catching the ball.
//
// A ball cannot simply be caught: the piston is hardly quicker than gravity,
// so it cannot match the speed of a fast ball, and every bounce keeps 90 % of
// the speed between ball and paddle. The catcher therefore plays three aimed
// bounces (see "Aimed bounces"):
//   hop    a low bounce that brings the ball down over the wanted spot,
//   pop    a bounce there with a tilted paddle that removes the sideways
//          speed and leaves just enough height to level the paddle again,
//   drop   a bounce off a paddle moving away that takes out most of the rest.
// What is left is a small bounce: the paddle chases the ball upwards and then
// falls with it, so that they meet gently, and finally comes to a stop with
// the ball lying on top. Which step comes next is decided from where the ball
// is heading, so the same rules deal with any start.
//
// makeCatcher(P) returns step(ball, piston, hinge) -> {pistonAcceleration,
// hingeAcceleration, resting}. P.x is where the ball should come to rest.
// step.settle is the piston rule for a ball that is on the paddle.
// ---------------------------------------------------------------------------
var CATCH_DEFAULTS = {
  x: 0,
  hop: 0.5, pop: 0.1,            // heights of the first two aimed bounces above the paddle (the hop at least)
  popTilt: 0.32,                 // largest tilt planned for the pop
  dropSpeed: 0.4,                // upward speed left to the ball by the third
  hopLength: 1.75, popLength: 1.9, dropLength: 1.8,   // piston lengths to bounce at
  small: 0.06,                   // bounces lower than this are not aimed
  near: 0.05,                    // a ball coming down this close to the spot needs no hop
  slow: 0.05,                    // sideways speed below which a bounce with a level paddle will do
  touch: 0.05,                   // closing speed kept until the ball touches
  brake: 8,                      // piston deceleration planned for stopping
  carry: 6,                      // strongest downward piston acceleration with the ball on top
  restGap: 0.004, restSpeed: 0.3,   // the ball counts as lying on the paddle below these
  chase: 0.4,                    // closing speed on small bounces, as a share of free-fall speed over the gap
  slide: 2, soft: 0.5, lead: 0.2, tiltMax: 0.3,       // sliding the resting ball to the spot
  hinge: 12, hingeSoft: 0.8
};

// Speed to approach a target with: brakes at a constant rate, eases off close by.
function approach(err, decel, soft) { return -Math.sign(err) * (Math.sqrt(soft * soft + 2 * decel * Math.abs(err)) - soft); }

function makeCatcher(options)
{
  var P = {}, key;
  for (key in CATCH_DEFAULTS) P[key] = options && key in options ? options[key] : CATCH_DEFAULTS[key];
  var aimer = makeAimer(), mode = null, last = null;
  var CONTACT_ACCEL = -PADDLE.restitution / (1 + PADDLE.restitution) * PADDLE.g;


  function riseOf(after) { return after.vy * after.vy / (2 * PADDLE.g); }

  // The ball's state at its next bounce, if that is aimed at the given target.
  function nextBounce(s, t)
  {
    var tau = fallTime(s, restHeight(t.length)), rough = t.guess(tau);
    return aimShoot(s, {length: t.length, accel: t.accel, speed: rough.speed, angle: rough.angle}, tau);
  }

  // Hop: a low bounce that comes down over the wanted spot. The further the
  // ball has to travel, the higher the hop, so that the tilt needed to stop
  // its sideways motion at the next bounce stays moderate.
  function hopTarget(s)
  {
    var g = PADDLE.g, fall = fallTime(s, restHeight(P.hopLength)), far = Math.abs(s.x + s.vx * fall - P.x);
    var height = P.hop, popSpeed = Math.sqrt(2 * g * P.pop);
    while (height < 2 && far / (2 * Math.sqrt(2 * height / g)) > Math.tan(P.popTilt) * (popSpeed + Math.sqrt(2 * g * height))) height += 0.05;
    return {
      length: P.hopLength, accel: CONTACT_ACCEL,
      guess: function (tau) { return {speed: roughAim(0, s.vy - g * tau, P.hopLength, restHeight(P.hopLength) + height).speed, angle: 0}; },
      miss: function (after) {
        var landing = nextBounce(after, popTarget(after));
        return [riseOf(after) - height, (landing ? landing.s.x : after.x) - P.x];
      }
    };
  }

  // Pop: take out the sideways speed, but for the little that carries the ball
  // exactly to the centre by the next bounce.
  function popTarget(s)
  {
    return {
      length: P.popLength, accel: CONTACT_ACCEL,
      guess: function (tau) { return roughAim(s.vx, s.vy - PADDLE.g * tau, P.popLength, restHeight(P.popLength) + P.pop); },
      miss: function (after) {
        var landing = nextBounce(after, dropTarget(after));
        return [riseOf(after) - P.pop, (landing ? landing.s.x : after.x) - P.x];
      }
    };
  }

  // Drop: with the paddle level again, take nearly all the speed out of the ball.
  function dropTarget(s)
  {
    return {
      length: P.dropLength, accel: CONTACT_ACCEL,
      guess: function (tau) {
        var vIn = s.vy - PADDLE.g * tau;
        return {speed: (P.dropSpeed + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(s.vx / (P.dropSpeed - vIn))};
      },
      miss: function (after) { return [after.vy - P.dropSpeed, after.vx]; }
    };
  }

  function choose(s)
  {
    var landing = nextBounce(s, popTarget(s));
    if (!landing || Math.abs(landing.s.x - P.x) > P.near) return {name: 'hop', target: hopTarget(s)};
    return Math.abs(s.vx) < P.slow ? {name: 'drop', target: dropTarget(s)} : {name: 'pop', target: popTarget(s)};
  }

  // Piston rule for a ball lying on the paddle or bouncing a little.
  // Lying: move at the speed wanted (zero to stop), without throwing the ball
  // off. Bouncing: move with the ball, closing in slowly, so that it lands softly.
  function settle(ball, piston, c, wanted)
  {
    var M = PADDLE, gap = Math.max(0, c.gap), lowest = -M.pistonAccel, want = wanted;
    if (gap < P.restGap && Math.abs(c.closing) < P.restSpeed) lowest = -P.carry;
    else want = ball.vy - (c.pvy - piston.speed) - M.g * M.dt + P.touch + P.chase * Math.sqrt(2 * (M.g + M.pistonAccel) * gap);
    var up = Math.sqrt(2 * P.brake * Math.max(0, M.lengthMax - 0.01 - piston.length));
    var down = -Math.sqrt(2 * P.brake * Math.max(0, piston.length - M.lengthMin - 0.01));
    return clamp((clamp(want, down, up) - piston.speed) / M.dt, lowest, M.pistonAccel);
  }

  function step(ball, piston, hinge)
  {
    var M = PADDLE, s = paddleState(ball, piston, hinge), c = paddleContact(s);
    if (last !== null && Math.abs(ball.vy - (last - M.g * M.dt)) > 1e-6) { mode = null; aimer.reset(); }
    last = ball.vy;
    var gap = Math.max(0, c.gap), resting = gap < P.restGap && Math.abs(c.closing) < P.restSpeed;

    if (!resting) {
      // Bounces too small to aim are followed with the simple rules further down.
      if (!mode) mode = s.y + s.vy * s.vy / (2 * M.g) - (c.py + M.radius) < P.small ? {name: 'small'} : choose(s);
      if (mode.name !== 'small') {
        var u = aimer.step(s, mode.target);
        if (u) { u.resting = false; return u; }
      }
    }

    var pistonAcceleration = settle(ball, piston, c, 0);

    // Hinge. The ball only changes its sideways speed when it presses on a
    // tilted paddle: in one go when it bounces, steadily when it rests.
    var vxWant = approach(ball.x - P.x, P.slide, P.soft);
    var impact = Math.sqrt(Math.max(0, c.closing) * Math.max(0, c.closing) + 2 * M.g * gap);
    var push = Math.max((1 + M.restitution) * impact, M.g * P.lead);
    var tilt = clamp((ball.vx - vxWant) / push, -P.tiltMax, P.tiltMax);
    var rate = approach(hinge.angle - tilt, P.hinge, P.hingeSoft);
    var hingeAcceleration = clamp((rate - hinge.speed) / M.dt, -M.hingeAccel, M.hingeAccel);
    return {pistonAcceleration: pistonAcceleration, hingeAcceleration: hingeAcceleration, resting: resting};
  }

  step.settle = settle;
  return step;
}

var catcher = makeCatcher({x: 0});

function controlFunction(ball, piston, hinge, T)
{
  var u = catcher(ball, piston, hinge);
  return {pistonAcceleration: u.pistonAcceleration, hingeAcceleration: u.hingeAcceleration};
}
