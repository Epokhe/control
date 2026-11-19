// Double pendulum on a cart (both levels): get both links upright and balance at x = 0.
//
// One controller handles the stabilisation level and the swing-up level. It
// plans a minimum-time trajectory to the upright rest state from wherever it
// starts, follows it with model-predictive control and finally holds with LQR.
//
// Swing-up of a double pendulum is a hard search problem for a local
// optimiser: the motion is chaotic and the goal is unstable. The planner
// therefore gets its first guess from physics. Cart and pendulum have no
// friction, so their motion is time-reversible. Letting the pendulum fall from
// upright and steering it into a given state near the bottom is an easy
// problem (the bottom is stable). Played backwards, that fall is a swing-up
// that starts exactly where the pendulum is now.

//#include lib/mpc.js

var F_MAX = 50;
var TOL = [0.01, 0.01, 0.001, 0.01, 0.001, 0.01];   // "arrived" for x, dx, theta1, dtheta1, theta2, dtheta2
var SUB = 3;                                         // plan in steps of 3 control periods
var drive = null;

function wrap(a) { return a - 2 * Math.PI * Math.round(a / (2 * Math.PI)); }
// Is the state within `scale` times the arrival tolerances of the goal?
function within(x, scale) { var e = err(x); for (var i = 0; i < e.length; i++) if (Math.abs(e[i]) > scale * TOL[i]) return false; return true; }
function err(x) { return [x[0], x[1], wrap(x[2]), x[3], wrap(x[4]), x[5]]; }

// Equations of motion (Lagrange, point masses on massless rods, angles from
// the upward vertical). State: [x, dx, theta1, dtheta1, theta2, dtheta2].
function dynamics(p)
{
  var M11 = p.m0 + p.m1 + p.m2, a = (p.m1 + p.m2) * p.L1, b = p.m2 * p.L2;
  var M22 = (p.m1 + p.m2) * p.L1 * p.L1, M33 = p.m2 * p.L2 * p.L2, d = p.m2 * p.L1 * p.L2;
  return function (x, u, out) {
    var s1 = Math.sin(x[2]), c1 = Math.cos(x[2]), s2 = Math.sin(x[4]), c2 = Math.cos(x[4]);
    var s12 = Math.sin(x[2] - x[4]), c12 = Math.cos(x[2] - x[4]);
    var w1 = x[3] * x[3], w2 = x[5] * x[5];
    var M12 = a * c1, M13 = b * c2, M23 = d * c12;
    var r1 = u[0] + a * s1 * w1 + b * s2 * w2;
    var r2 = -d * s12 * w2 + a * p.g * s1;
    var r3 = d * s12 * w1 + b * p.g * s2;
    // Solve the symmetric 3x3 system M q'' = r by Cramer's rule.
    var det = M11 * (M22 * M33 - M23 * M23) - M12 * (M12 * M33 - M23 * M13) + M13 * (M12 * M23 - M22 * M13);
    out[0] = x[1];
    out[1] = (r1 * (M22 * M33 - M23 * M23) - M12 * (r2 * M33 - M23 * r3) + M13 * (r2 * M23 - M22 * r3)) / det;
    out[2] = x[3];
    out[3] = (M11 * (r2 * M33 - M23 * r3) - r1 * (M12 * M33 - M23 * M13) + M13 * (M12 * r3 - r2 * M13)) / det;
    out[4] = x[5];
    out[5] = (M11 * (M22 * r3 - r2 * M23) - M12 * (M12 * r3 - r2 * M13) + r1 * (M12 * M23 - M22 * M13)) / det;
  };
}

function build(p, h)
{
  var f = dynamics(p);
  var hPlan = SUB * h;
  var period = 2 * Math.PI * Math.sqrt((p.L1 + p.L2) / p.g);   // time scale of the hanging pendulum

  // Planner towards the upright rest state. The running cost is small and
  // only shapes the motion; arriving is a terminal constraint.
  var up = makeMPC({
    n: 6, m: 1, N: 200, h: h, sub: SUB, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [-F_MAX], uMax: [F_MAX], f: f,
    nr: 7,
    residual: function (x, u, r) {
      r[0] = 0.3 * x[0]; r[1] = 0.05 * x[1]; r[2] = 0.6 * Math.sin(x[2] / 2); r[3] = 0.05 * x[3];
      r[4] = 0.6 * Math.sin(x[4] / 2); r[5] = 0.05 * x[5]; r[6] = 0.002 * u[0];
    },
    nrT: 6,
    terminalResidual: function (x, r) { var e = err(x); for (var i = 0; i < 6; i++) r[i] = e[i] / TOL[i]; }
  });

  // Planner for the reverse problem: start (almost) upright at x = 0 and end
  // exactly in the time-reverse of the state we are actually in.
  var target = [0, 0, 0, 0, 0, 0];
  function errDown(x) { return [x[0] - target[0], x[1] - target[1], wrap(x[2] - target[2]), x[3] - target[3], wrap(x[4] - target[4]), x[5] - target[5]]; }
  function reachedDown(x) { var e = errDown(x); for (var i = 0; i < 6; i++) if (Math.abs(e[i]) > 0.5 * TOL[i]) return false; return true; }
  var down = makeMPC({
    n: 6, m: 1, N: 200, h: h, sub: SUB, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [-F_MAX], uMax: [F_MAX], f: f,
    nr: 7,
    residual: function (x, u, r) {
      var e = errDown(x);
      r[0] = 0.3 * e[0]; r[1] = e[1]; r[2] = 6 * Math.sin(e[2] / 2); r[3] = e[3]; r[4] = 6 * Math.sin(e[4] / 2); r[5] = e[5]; r[6] = 0.01 * u[0];
    },
    nrT: 6,
    terminalResidual: function (x, r) { var e = errDown(x); for (var i = 0; i < 6; i++) r[i] = e[i] / TOL[i]; }
  });

  var one = makeMPC({n: 6, m: 1, N: 1, h: h, f: f, uMin: [-F_MAX], uMax: [F_MAX], nr: 1, nrT: 1}).linearAt([0, 0, 0, 0, 0, 0], [0]);
  var K = dlqr(one.A, one.B, numeric.diag([100, 10, 100, 10, 100, 10]), [[0.01]])[0];
  function hold(x) { var e = err(x), u = 0; for (var i = 0; i < 6; i++) u -= K[i] * e[i]; return [Math.max(-F_MAX, Math.min(F_MAX, u))]; }

  // Seed 1: simply try to hold (good when already near upright).
  var HOLD_TIMES = [0.6, 0.75, 0.95];   // horizons to try, in pendulum periods
  function seedHold(x0, attempt)
  {
    if (attempt >= HOLD_TIMES.length || Math.cos(x0[2]) < 0.5 || Math.cos(x0[4]) < 0.5) return false;
    up.setHorizon(Math.round(HOLD_TIMES[attempt] * period / hPlan));
    up.guess(x0, hold);
  }

  // Seed 2: a controlled fall from upright into (the reverse of) the current
  // state, played backwards. Tried for a few durations and for both
  // directions in which the links can fold when they leave the top.
  var FALLS = [[0.75, 1], [0.75, -1], [0.9, 1], [0.9, -1], [1.1, 1], [1.1, -1]];
  function seedReverse(x0, attempt)
  {
    if (attempt >= FALLS.length) return false;
    var n = Math.round(FALLS[attempt][0] * period / hPlan), tilt = 0.05 * FALLS[attempt][1], k;
    target = [x0[0], -x0[1], x0[2], -x0[3], x0[4], -x0[5]];
    down.setHorizon(n);
    down.clear();
    down.resetMultipliers();
    down.optimise([0, 0, tilt, 0, -tilt, 0], 40, 8, reachedDown);
    if (!reachedDown(down.final())) return 'skip';
    var fall = down.plan(), rise = [];
    for (k = 0; k < n; k++) rise.push([fall[n - 1 - k][0]]);
    up.loadPlan(rise);
  }

  drive = makeManeuver({
    mpc: up, sub: SUB,
    plan: {iters: 40, outer: 8, margin: 1.0, shrink: 0, seeds: [seedHold, seedReverse]},
    reached: function (x) { return within(x, 0.5); },
    strayed: function (x, xPlan) { for (var i = 0; i < x.length; i++) if (Math.abs(x[i] - xPlan[i]) > 30 * TOL[i]) return true; return false; },
    canHold: function (x) { var e = err(x); return Math.abs(e[2]) < 0.2 && Math.abs(e[4]) < 0.2 && Math.abs(e[3]) < 1 && Math.abs(e[5]) < 1 && Math.abs(e[0]) < 1; },
    hold: hold,
    // No plan found: take energy out through the cart for a moment, then try again.
    fallback: function (x) { return [Math.max(-F_MAX, Math.min(F_MAX, -60 * x[1] - 5 * x[0]))]; }
  });
}

function controlFunction(pendulum)
{
  if (!drive) build(pendulum, 0.02);
  return drive([pendulum.x, pendulum.dx, pendulum.theta1, pendulum.dtheta1, pendulum.theta2, pendulum.dtheta2])[0];
}
