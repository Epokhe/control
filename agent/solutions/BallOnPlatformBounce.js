// Ball on platform, bounce: keep the ball bouncing straight up and down with
// its highest point exactly at the origin.
//
// Each bounce is aimed. Before a bounce the controller picks what the paddle
// should be doing at the moment of contact (its upward speed and its tilt),
// drives piston and hinge there along smooth curves, and checks the aim by
// running the model forward to the bounce. Newton's method corrects speed and
// tilt until the predicted outcome is exactly the wanted one; this is repeated
// every control period, so errors are corrected as long as the ball is in the
// air.
//
// The ball starts off to the side, so the first bounce is a low hop that brings
// it down over the centre, and the second sends it straight up.

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

var HOP_APOGEE = -2.0;                // top of the first hop
var LENGTH = {hop: 1.75, afterHop: 2.1, steady: 2.1};   // piston length to bounce at
var CENTRED = 0.008;                  // sideways error small enough to trim without a hop
// Paddle acceleration at contact. With this value a bounce that happens a
// little earlier or later than expected sends the ball equally high.
var CONTACT_ACCEL = -PADDLE.restitution / (1 + PADDLE.restitution) * PADDLE.g;

var COMMIT = 3 * PADDLE.dt;           // this close to contact the plan is no longer changed
var aim = null, last = null, plan = null, hopping = false, hopped = false;

// Piston and hinge accelerations over time that reach the wanted contact
// state (length, speed, CONTACT_ACCEL; angle, at rest) after time tau.
function curves(s, length, speed, angle, tau)
{
  tau = Math.max(tau, PADDLE.dt);
  var D = length - s.L - s.v * tau, E = speed - s.v;
  var a0 = (12 * D - 6 * E * tau + CONTACT_ACCEL * tau * tau) / (tau * tau);
  var snap = (6 * (a0 + CONTACT_ACCEL) * tau - 12 * E) / (tau * tau * tau);
  var jerk = (CONTACT_ACCEL - a0 - 0.5 * snap * tau * tau) / tau;
  var Dh = angle - s.th - s.om * tau, Eh = -s.om;
  var h0 = (6 * Dh - 2 * Eh * tau) / (tau * tau), h1 = 2 * (Eh - h0 * tau) / (tau * tau);
  return function (t) {
    if (t >= tau) return [CONTACT_ACCEL, 0];
    return [a0 + jerk * t + 0.5 * snap * t * t, h0 + h1 * t];
  };
}

// Run the model forward until the ball bounces. The curves are redrawn from
// the current state every period, as the controller itself does, until
// shortly before contact.
function shoot(s0, length, speed, angle, tau)
{
  var s = {}, k, c, first, since = 0; for (k in s0) s[k] = s0[k];
  for (var i = 0; i < 150; i++) {
    var left = tau - i * PADDLE.dt;
    if (i === 0 || left > COMMIT) { c = curves(s, length, speed, angle, left); since = 0; if (i === 0) first = c; }
    var u = c((since + 0.5) * PADDLE.dt);
    since++;
    if (paddleAdvance(s, u[0], u[1])) return {s: s, curve: first, time: (i + 1) * PADDLE.dt};
  }
  return null;
}

// Rough paddle speed and tilt that send a ball, arriving at (vx, vIn), straight
// up to the height target from a paddle at the given piston length.
function roughAim(vx, vIn, length, target)
{
  var vOut = Math.sqrt(Math.max(0, 2 * PADDLE.g * (target - restHeight(length))));
  return {speed: (vOut + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(vx / (vOut - vIn))};
}

// How far the bounce is from what we want: [height error, sideways error].
function miss(s)
{
  var apogee = s.y + s.vy * s.vy / (2 * PADDLE.g);
  if (hopping) {
    // Follow the hop down to the next bounce, which should be over the centre.
    var flight = fallTime(s, restHeight(LENGTH.afterHop));
    var then = roughAim(s.vx, s.vy - PADDLE.g * flight, LENGTH.afterHop, 0);
    var next = shoot(s, LENGTH.afterHop, then.speed, then.angle, flight);
    return [apogee - HOP_APOGEE, next ? next.s.x : s.x + s.vx * flight];
  }
  // Straight up to the origin; a remaining sideways offset is removed slowly.
  var drift = clamp(-s.x / (2 * s.vy / PADDLE.g), -0.005, 0.005);
  return [apogee, s.vx - drift];
}

function controlFunction(ball, piston, hinge, T)
{
  var P = PADDLE, s = paddleState(ball, piston, hinge);
  // A jump in the ball's velocity means it has bounced.
  if (last !== null && Math.abs(ball.vy - (last - P.g * P.dt)) > 1e-6) { aim = null; plan = null; hopped = hopping; }
  last = ball.vy;
  if (plan && plan.left <= COMMIT) {
    plan.step++; plan.left -= P.dt;
    var held = plan.curve((plan.step + 0.5) * P.dt);
    return {pistonAcceleration: held[0], hingeAcceleration: held[1]};
  }
  if (!aim) {
    // If the ball is not coming down over the centre, hop there first.
    var len = hopped ? LENGTH.afterHop : LENGTH.steady, fall = fallTime(s, restHeight(len));
    var rough = roughAim(s.vx, s.vy - P.g * fall, len, 0);
    var landing = shoot(s, len, rough.speed, rough.angle, fall);
    hopping = !landing || Math.abs(landing.s.x) > CENTRED;
  }
  var length = hopping ? LENGTH.hop : hopped ? LENGTH.afterHop : LENGTH.steady;
  var tau = fallTime(s, restHeight(length));    // time to contact
  if (!aim) {
    // First guess from the bounce law for a level paddle at the wanted height.
    aim = hopping ? {speed: roughAim(0, s.vy - P.g * tau, length, HOP_APOGEE).speed, angle: 0} : roughAim(s.vx, s.vy - P.g * tau, length, 0);
  }
  var out = null;
  for (var iter = 0; iter < 6; iter++) {
    out = shoot(s, length, aim.speed, aim.angle, tau);
    if (!out) break;
    tau = out.time;                                  // contact time as the model sees it
    var f = miss(out.s);
    if (Math.abs(f[0]) + Math.abs(f[1]) < 1e-10) break;
    var eps = 1e-5;
    var a = shoot(s, length, aim.speed + eps, aim.angle, tau), b = shoot(s, length, aim.speed, aim.angle + eps, tau);
    if (!a || !b) break;
    var fa = miss(a.s), fb = miss(b.s);
    var j00 = (fa[0] - f[0]) / eps, j10 = (fa[1] - f[1]) / eps, j01 = (fb[0] - f[0]) / eps, j11 = (fb[1] - f[1]) / eps;
    var det = j00 * j11 - j01 * j10;
    if (Math.abs(det) < 1e-12) break;
    aim.speed -= clamp((j11 * f[0] - j01 * f[1]) / det, -1, 1);
    aim.angle = clamp(aim.angle - clamp((-j10 * f[0] + j00 * f[1]) / det, -0.2, 0.2), -0.6, 0.6);
  }
  if (out) plan = {curve: out.curve, step: 0, left: tau};
  if (!out) return {pistonAcceleration: -4 * piston.speed, hingeAcceleration: -4 * hinge.speed};
  var u = out.curve(0.5 * P.dt);
  return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
}
