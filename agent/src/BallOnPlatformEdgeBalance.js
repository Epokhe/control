// Ball on platform, edge balance: balance the ball on the raised left end of
// the paddle.
//
// The ball is first caught on the flat of the paddle ("Catching the ball"
// below). From there it is one smooth problem: the ball slides without
// friction along the bar and around its rounded end, driven only by tilting
// the bar, and has to arrive on top of the end exactly at rest - the same kind
// of problem as swinging up a pendulum, and solved the same way: plan the
// fastest tilting motion that gets there, follow it with feedback, and hold
// the ball on top with a linear regulator.

//#include lib/mpc.js

//#include lib/paddle.js

//#include lib/aim.js

//#include lib/catch.js

var CATCH_X = -0.5;        // where to catch the ball
var ANGLE = -0.4;          // paddle angle to balance at (left end raised)
var TOL = [0.004, 0.04, 0.02, 0.04];
var W = {q: 0.3, dq: 0.05, th: 0.5, dth: 0.05, u: 0.004, limit: 30, press: 3};
var MIN_PRESS = 6;         // keep the ball pressed on with at least this acceleration

var catcher = makeCatcher({x: CATCH_X}), drive = null, sliding = false;
var REST = [PADDLE.radius * ANGLE, 0, ANGLE, 0];   // ball on top of the end: as far round the end as the bar is tilted

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

function build()
{
  var M = PADDLE, h = M.dt;
  var mpc = makeMPC({
    n: 4, m: 1, N: 200, h: h, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [-M.hingeAccel], uMax: [M.hingeAccel],
    f: slide,
    nr: 8,
    residual: function (x, u, r) {
      r[0] = W.q * (x[0] - REST[0]); r[1] = W.dq * x[1]; r[2] = W.th * (x[2] - REST[2]); r[3] = W.dth * x[3]; r[4] = W.u * u[0];
      r[5] = W.limit * hingeLimit(x[2], -0.65, 0.65);                       // stay clear of the hinge's end stops
      r[6] = W.limit * hingeLimit(x[0], -1.2 * M.radius, 2 * M.halfWidth - 0.1);   // and of both ends of the bar
      r[7] = W.press * Math.min(0, press(x, u) - MIN_PRESS);                // and keep the ball on the bar
    },
    nrT: 4,
    terminalResidual: function (x, r) { for (var i = 0; i < 4; i++) r[i] = (x[i] - REST[i]) / TOL[i]; }
  });
  if (typeof DEBUG !== 'undefined') DEBUG.mpc = mpc;
  var lin = mpc.linearAt(REST, [0]);
  var K = dlqr(lin.A, lin.B, numeric.diag([100, 1, 10, 1]), [[0.01]])[0];
  function hold(x) { var u = 0; for (var i = 0; i < 4; i++) u -= K[i] * (x[i] - REST[i]); return [clamp(u, -M.hingeAccel, M.hingeAccel)]; }
  var TIMES = [1.5, 2, 2.6];
  function seed(x0, attempt)
  {
    if (attempt >= TIMES.length) return false;
    mpc.setHorizon(Math.round(TIMES[attempt] / h));
    mpc.clear();
  }
  drive = makeManeuver({
    mpc: mpc,
    plan: {iters: 40, outer: 10, shrink: 0, seeds: [seed]},
    reached: function (x) { for (var i = 0; i < 4; i++) if (Math.abs(x[i] - REST[i]) > 0.5 * TOL[i]) return false; return true; },
    strayed: function (x, xPlan) { return Math.abs(x[0] - xPlan[0]) > 0.08 || Math.abs(x[1] - xPlan[1]) > 0.5 || Math.abs(x[2] - xPlan[2]) > 0.2; },
    canHold: function (x) { return Math.abs(x[0] - REST[0]) < 0.06 && Math.abs(x[1]) < 0.4 && Math.abs(x[2] - REST[2]) < 0.25; },
    hold: hold,
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
  if (typeof DEBUG !== 'undefined') { var first = !DEBUG.done; DEBUG.done = 1; var uu = drive(x); if (first) { var X = DEBUG.mpc.states(), U = DEBUG.mpc.plan(), N = DEBUG.mpc.horizon(); for (var k = 0; k < N; k += 2) console.log('plan', k, X[k].map(function (v) { return v.toFixed(3); }).join(' '), 'u', U[k][0].toFixed(2), 'press', press(X[k], U[k]).toFixed(2)); } console.log(T.toFixed(2), 'meas', x.map(function (v) { return v.toFixed(3); }).join(' '), 'u', uu[0].toFixed(2)); return {pistonAcceleration: clamp(-piston.speed / M.dt, -P_CARRY, M.pistonAccel), hingeAcceleration: uu[0]}; }
  // The piston only has to stand still.
  return {pistonAcceleration: clamp(-piston.speed / M.dt, -P_CARRY, M.pistonAccel), hingeAcceleration: drive(x)[0]};
}
var P_CARRY = 6;
