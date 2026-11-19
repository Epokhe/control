// Ball on platform, edge balance: balance the ball on the raised left end of
// the paddle.
//
// 1. The ball is caught on the flat of the paddle ("Catching the ball" below).
// 2. Slide. The ball slides without friction, driven only by tilting the bar.
//    A planner finds the fastest tilting motion that sends it towards the left
//    end at a moderate speed with the left end raised, and feedback around
//    the plan follows it.
// 3. Climb. The last bit is uphill and around the rounded end. The bar is
//    tilted so that the ball's speed is exactly what it takes to reach the top
//    of the end and stop there: steeper if the ball is too fast, flatter if
//    it is too slow.
// 4. Balance. On top of the end the ball is an upside-down pendulum on a
//    pivot that the hinge moves sideways. A linear regulator holds it there,
//    while the piston takes out the up and down motion of the bar's end.

//#include lib/mpc.js

//#include lib/paddle.js

//#include lib/aim.js

//#include lib/catch.js

var CATCH_X = -0.5;        // where to catch the ball
var ANGLE = -0.4;          // paddle angle to settle at when balancing (left end raised)
var LAUNCH = [0.03, -0.6, -0.3, 0];      // end of the slide: just before the end, moving towards it, left end raised
var TOL = [0.01, 0.05, 0.03, 0.1];
var W = {q: 0.3, dq: 0.05, th: 0.5, dth: 0.05, u: 0.004, limit: 30, press: 3};
var HOLD = {top: 1000, dq: 1, th: 0.01, dth: 100, u: 0.01};   // weights of the regulator that balances the ball
var MID_LENGTH = 1.9;
var MIN_PRESS = 6;         // keep the ball pressed on with at least this acceleration
var CLIMB = {hinge: 12, hingeSoft: 0.8, lost: 0.6};

var catcher = makeCatcher({x: CATCH_X}), drive = null, sliding = false, balancing = false;

// The ball's path in the paddle's own frame (origin at the hinge), by distance
// q travelled from the left end of the flat: along the flat for q > 0, around
// the rounded end for q < 0. Returns position c and unit direction t.
function path(q, o)
{
  var M = PADDLE;
  if (q >= 0) { o.cx = q - M.halfWidth; o.cy = M.thickness + M.radius; o.tx = 1; o.ty = 0; o.curve = 0; }
  else {
    var psi = -q / M.radius, s = Math.sin(psi), c = Math.cos(psi);
    o.cx = -M.halfWidth - M.radius * s; o.cy = M.thickness + M.radius * c; o.tx = c; o.ty = s; o.curve = 1 / M.radius;
  }
  return o;
}

// Sliding ball on the tilting bar. State [q, dq, angle, rate], input: hinge
// acceleration. press: how hard the ball is pressed onto the bar.
var scratch = {};
function slide(x, u, out)
{
  var p = path(x[0], scratch), S = Math.sin(x[2]), C = Math.cos(x[2]), g = PADDLE.g;
  out[0] = x[1];
  out[1] = -g * (p.tx * S + p.ty * C) - u[0] * (-p.tx * p.cy + p.ty * p.cx) + x[3] * x[3] * (p.tx * p.cx + p.ty * p.cy);
  out[2] = x[3];
  out[3] = u[0];
}
function press(x, u)
{
  var p = path(x[0], scratch), S = Math.sin(x[2]), C = Math.cos(x[2]), nx = -p.ty, ny = p.tx;
  return u[0] * (-nx * p.cy + ny * p.cx) - x[3] * x[3] * (nx * p.cx + ny * p.cy) + 2 * x[3] * x[1] - x[1] * x[1] * p.curve + PADDLE.g * (nx * S + ny * C);
}

// Measure the sliding state from ball and paddle.
function measure(s)
{
  var M = PADDLE, S = Math.sin(s.th), C = Math.cos(s.th);
  var px = s.x, py = s.y - (M.baseY + s.L);
  var lx = C * px + S * py, ly = -S * px + C * py;                 // ball in the paddle's frame
  var vx = C * s.vx + S * (s.vy - s.v) + s.om * ly, vy = -S * s.vx + C * (s.vy - s.v) - s.om * lx;   // and its velocity there
  var q = lx + M.halfWidth;
  if (q < 0) q = -M.radius * Math.atan2(-q, ly - M.thickness);
  var p = path(q, scratch);
  return [q, p.tx * vx + p.ty * vy, s.th, s.om];
}

function hingeLimit(v, lo, hi) { return v < lo ? v - lo : v > hi ? v - hi : 0; }

// Angle of the ball's contact normal from vertical: zero on top of the end.
function lean(x) { return x[2] - Math.min(0, x[0]) / PADDLE.radius; }

function build()
{
  var M = PADDLE, h = M.dt;
  var mpc = makeMPC({
    n: 4, m: 1, N: 200, h: h, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [-M.hingeAccel], uMax: [M.hingeAccel],
    f: slide,
    nr: 8,
    residual: function (x, u, r) {
      r[0] = W.q * (x[0] - LAUNCH[0]); r[1] = W.dq * x[1]; r[2] = W.th * (x[2] - LAUNCH[2]); r[3] = W.dth * x[3]; r[4] = W.u * u[0];
      r[5] = W.limit * hingeLimit(x[2], -0.65, 0.65);                       // stay clear of the hinge's end stops
      r[6] = W.limit * hingeLimit(x[0], 0.01, 2 * M.halfWidth - 0.1);       // stay on the flat of the bar
      r[7] = W.press * Math.min(0, press(x, u) - MIN_PRESS);                // and keep the ball pressed on
    },
    nrT: 4,
    terminalResidual: function (x, r) { for (var i = 0; i < 4; i++) r[i] = (x[i] - LAUNCH[i]) / TOL[i]; }
  });

  // Balancing regulators for a range of tilts. The ball can rest on top of
  // the end at any tilt; the regulator brings it to the top and to rest, and
  // steers the tilt itself back only slowly.
  var gains = [], top = [1, 0, -M.radius, 0];
  for (var a = -0.05; a > -0.7; a -= 0.05) {
    var lin = mpc.linearAt([M.radius * a, 0, a, 0], [0]), Q = numeric.diag([0, HOLD.dq, HOLD.th, HOLD.dth]);
    for (var i = 0; i < 4; i++) for (var j = 0; j < 4; j++) Q[i][j] += HOLD.top * top[i] * top[j];
    gains.push(dlqr(lin.A, lin.B, Q, [[HOLD.u]])[0]);
  }
  function balance(x)
  {
    var K = gains[clamp(Math.round(-x[2] / 0.05) - 1, 0, gains.length - 1)];
    return clamp(-(K[0] * (x[0] - M.radius * ANGLE) + K[1] * x[1] + K[2] * (x[2] - ANGLE) + K[3] * x[3]), -M.hingeAccel, M.hingeAccel);
  }
  // After the slide: keep the tilt while the ball runs up to the end, then balance.
  function finish(x)
  {
    balancing = x[0] < 0;
    if (balancing) return [balance(x)];
    var rate = approach(x[2] - LAUNCH[2], CLIMB.hinge, CLIMB.hingeSoft);
    return [clamp((rate - x[3]) / h, -M.hingeAccel, M.hingeAccel)];
  }

  // Slide times to try: scaled with the distance to cover, shortest first.
  var STRETCH = [1, 1.15, 1.4, 1.8];
  function seed(x0, attempt)
  {
    if (attempt >= STRETCH.length) return false;
    var time = 0.45 + 0.75 * Math.sqrt(Math.max(0.05, Math.abs(x0[0] - LAUNCH[0])));
    mpc.setHorizon(Math.round(STRETCH[attempt] * time / h));
    mpc.clear();
  }
  drive = makeManeuver({
    mpc: mpc,
    plan: {iters: 40, outer: 10, shrink: 0, seeds: [seed]},
    reached: function (x) { for (var i = 0; i < 4; i++) if (Math.abs(x[i] - LAUNCH[i]) > 0.5 * TOL[i]) return false; return true; },
    strayed: function (x, xPlan) { return Math.abs(x[0] - xPlan[0]) > 0.08 || Math.abs(x[1] - xPlan[1]) > 0.5 || Math.abs(x[2] - xPlan[2]) > 0.2; },
    // The climb is on as long as the ball is close to the end and still moving up, or on top.
    canHold: function (x) { return x[0] < 0 ? Math.abs(lean(x)) < CLIMB.lost : x[0] < 0.2 && x[1] < -0.05; },
    hold: finish,
    fallback: function (x) { return [clamp(-40 * x[3] - 100 * x[2], -M.hingeAccel, M.hingeAccel)]; }
  });
}

function controlFunction(ball, piston, hinge, T)
{
  var M = PADDLE, s = paddleState(ball, piston, hinge);
  if (!sliding) {
    var u = catcher(ball, piston, hinge);
    sliding = u.resting && Math.abs(ball.x - CATCH_X) < 0.1 && Math.abs(ball.vx) < 0.1 && Math.abs(piston.speed) < 0.05;
    if (!sliding) return {pistonAcceleration: u.pistonAcceleration, hingeAcceleration: u.hingeAcceleration};
  }
  if (!drive) build();
  var x = measure(s);
  var hingeAcceleration = drive(x)[0], pistonAcceleration;
  if (balancing && drive.phase() === 'hold') {
    // Balancing: tilting the bar moves its end sideways, which is what steers
    // the ball, but also up and down, which would make the ball hop. The
    // piston takes out the up and down part.
    var S = Math.sin(hinge.angle), C = Math.cos(hinge.angle);
    var lift = -M.thickness * S - M.halfWidth * C, liftRate = -M.thickness * C + M.halfWidth * S;   // end height per angle, and its slope
    var follow = -lift * hinge.speed;
    pistonAcceleration = -(lift * hingeAcceleration + liftRate * hinge.speed * hinge.speed) + (follow - piston.speed) / 0.1;
  } else {
    // Sliding: drift gently to mid-stroke, to have room for the above.
    pistonAcceleration = clamp((approach(piston.length - MID_LENGTH, 0.8, 0.3) - piston.speed) / 0.1, -1, 1);
  }
  if (typeof DEBUG !== 'undefined') console.log(T.toFixed(2), drive.phase(), 'meas', x.map(function (v) { return v.toFixed(3); }).join(' '), 'gap', paddleContact(s).gap.toFixed(4), 'L', piston.length.toFixed(3), piston.speed.toFixed(3));
  return {pistonAcceleration: pistonAcceleration, hingeAcceleration: hingeAcceleration};
}
