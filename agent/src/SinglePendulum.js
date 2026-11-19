// Single pendulum on a cart (both levels): get upright and balance at x = 0.

//#include lib/mpc.js

var F_MAX = 50;
var TOL = [0.01, 0.01, 0.001, 0.01];   // what "arrived" means for x, dx, theta, dtheta
var W = {x: 0.3, dx: 0.05, th: 0.3, dth: 0.05, u: 0.002};
var drive = null, K = null;

function wrap(a) { return a - 2 * Math.PI * Math.round(a / (2 * Math.PI)); }
// Is the state within `scale` times the arrival tolerances of the goal?
function within(x, scale) { var e = err(x); for (var i = 0; i < e.length; i++) if (Math.abs(e[i]) > scale * TOL[i]) return false; return true; }
function err(x) { return [x[0], x[1], wrap(x[2]), x[3]]; }

function build(p, h)
{
  var period = 2 * Math.PI * Math.sqrt(p.L / p.g);   // time scale of the hanging pendulum
  var HORIZONS = [1.5, 2, 2.5];                      // horizons to try, in pendulum periods
  var mpc = makeMPC({
    n: 4, m: 1, N: 400, h: h, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [-F_MAX], uMax: [F_MAX],
    f: function (x, u, out) {
      var s = Math.sin(x[2]), c = Math.cos(x[2]);
      var ddx = (u[0] + p.m1 * p.L * x[3] * x[3] * s - p.m1 * p.g * s * c) / (p.m0 + p.m1 * s * s);
      out[0] = x[1]; out[1] = ddx; out[2] = x[3]; out[3] = (p.g * s - ddx * c) / p.L;
    },
    nr: 5,
    residual: function (x, u, r) {
      r[0] = W.x * x[0]; r[1] = W.dx * x[1]; r[2] = W.th * 2 * Math.sin(x[2] / 2); r[3] = W.dth * x[3]; r[4] = W.u * u[0];
    },
    nrT: 4,
    terminalResidual: function (x, r) { var e = err(x); for (var i = 0; i < 4; i++) r[i] = e[i] / TOL[i]; }
  });
  var lin = mpc.linearAt([0, 0, 0, 0], [0]);
  K = dlqr(lin.A, lin.B, numeric.diag([100, 10, 100, 10]), [[0.01]])[0];
  function hold(x) { var e = err(x), u = 0; for (var i = 0; i < 4; i++) u -= K[i] * e[i]; return [Math.max(-F_MAX, Math.min(F_MAX, u))]; }

  // Seed 1: simply try to hold (good when already near upright).
  function seedHold(x0, attempt)
  {
    if (attempt >= HORIZONS.length || Math.cos(x0[2]) < 0.5) return false;
    mpc.setHorizon(Math.round(HORIZONS[attempt] * period / h));
    mpc.guess(x0, hold);
  }
  // Seed 2: start from "do nothing" and let the optimiser find the swing-up.
  function seedSwing(x0, attempt)
  {
    if (attempt >= HORIZONS.length) return false;
    mpc.setHorizon(Math.round(HORIZONS[attempt] * period / h));
    mpc.clear();
  }

  drive = makeManeuver({
    mpc: mpc,
    plan: {iters: 30, outer: 8, margin: 1.06, seeds: [seedHold, seedSwing]},
    reached: function (x) { return within(x, 0.5); },
    strayed: function (x, xPlan) { for (var i = 0; i < x.length; i++) if (Math.abs(x[i] - xPlan[i]) > 30 * TOL[i]) return true; return false; },
    canHold: function (x) { var e = err(x); return Math.abs(e[2]) < 0.3 && Math.abs(e[3]) < 1.5 && Math.abs(e[0]) < 1; },
    hold: hold,
    fallback: function (x) { return [Math.max(-F_MAX, Math.min(F_MAX, -60 * x[1] - 5 * x[0]))]; }
  });
}

function controlFunction(pendulum)
{
  if (!drive) build(pendulum, 0.02);
  return drive([pendulum.x, pendulum.dx, pendulum.theta, pendulum.dtheta])[0];
}
