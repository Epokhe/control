// Rocket landing (all four levels): touch down upright, slowly, near x = 0.
//
// The same controller lands the rocket from any of the levels' starting
// states, including upside down and with an engine that cannot throttle below
// 60 % (it reads the throttle limit and the other parameters from the model).
// It plans a minimum-time descent to a gentle touchdown, follows that plan
// with feedback, and falls back to a conventional cascaded landing law if it
// ever has no plan.

//#include lib/mpc.js

var SUB = 5;                 // plan in steps of 5 control periods (0.1 s)
var GIMBAL_MAX = 0.2;
// Touchdown aimed for, and how exactly the plan has to hit it. The level
// allows |x| < 30, |dx| < 5, |dy| < 5, |sin theta| < 0.05, |dtheta| < 0.1.
var TOUCH = {dy: -2, sink: 0.2};                 // arrive descending at 2 m/s, aiming just below ground level
var X_REACH = 80;                                // sideways error the fallback law reacts to at most
var SAFE_HEIGHT = 50;                            // where the fallback law waits until it is over the pad
var TOL = [2, 0.6, 0.3, 0.6, 0.01, 0.02];        // x, dx, y, dy, theta, dtheta

function wrap(a) { return a - 2 * Math.PI * Math.round(a / (2 * Math.PI)); }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function makeRocket(r, h)
{
  var g = r.g, twr = r.TWR, thrMin = r.throttleLimit, len = r.Length, wid = r.Width;
  var legY = len / 2 + wid, legX = 1.8 * wid;   // landing legs relative to the centre
  var yTouch = legY;                            // centre height at upright touchdown

  function f(x, u, out)
  {
    var a = g * twr * u[0];
    out[0] = x[1]; out[1] = a * Math.sin(x[4] + u[1]);
    out[2] = x[3]; out[3] = a * Math.cos(x[4] + u[1]) - g;
    out[4] = x[5]; out[5] = -a * 6 / len * Math.sin(u[1]);
  }
  // Height of the lowest point of the hull above the ground.
  function clearance(x)
  {
    var s = Math.sin(x[4]), c = Math.cos(x[4]);
    return x[2] + Math.min(len / 2 * c, -legY * c - legX * Math.abs(s));
  }
  function goodTouchdown(x, slack)
  {
    return Math.abs(x[0]) < 30 * slack && Math.abs(x[1]) < 5 * slack && Math.abs(x[3]) < 5 * slack &&
           Math.abs(x[5]) < 0.1 * slack && Math.abs(Math.sin(x[4])) < 0.05 * slack && Math.cos(x[4]) > 0;
  }
  function err(x) { return [x[0], x[1], x[2] - (yTouch - TOUCH.sink), x[3] - TOUCH.dy, wrap(x[4]), x[5]]; }

  // LQR gain for the sideways motion near hover: states (x, dx, theta, dtheta), input gimbal.
  function sidewaysGain()
  {
    var hoverThrottle = clamp(1 / twr, thrMin, 1);
    var lin = makeMPC({n: 6, m: 2, N: 1, h: h, f: f, uMin: [0, 0], uMax: [1, 1], nr: 1, nrT: 1}).linearAt([0, 0, 100, 0, 0, 0], [hoverThrottle, 0]);
    var idx = [0, 1, 4, 5], A = [], B = [], i, j;
    for (i = 0; i < 4; i++) { A.push([]); for (j = 0; j < 4; j++) A[i].push(lin.A[idx[i]][idx[j]]); B.push([lin.B[idx[i]][1]]); }
    var K = dlqr(A, B, numeric.diag([1 / 100, 1 / 50, 1 / 0.04, 1 / 0.04]), [[1 / 0.04]])[0];
    return [-K[0], -K[1], -K[2], -K[3]];
  }

  // Conventional landing law, used as the planner's starting guess and as the
  // fallback. Sideways: tilt the rocket to accelerate towards the pad.
  // Vertically: follow a braking curve down, but only once it is over the pad
  // and upright; until then stay at a safe height. A rocket that cannot hover
  // has no such choice and always follows the braking curve.
  var canHover = thrMin * twr < 0.95;
  var Kside = sidewaysGain();
  var spinUp = g * twr * 6 / len * Math.sin(GIMBAL_MAX);   // angular acceleration at full throttle and gimbal
  function cascade(x)
  {
    var th = wrap(x[4]), height = Math.max(0, x[2] - yTouch), upright = Math.cos(th);
    // Sideways and attitude: near upright, an LQR law on (x, dx, theta, dtheta)
    // drives the gimbal; the position error it sees is limited so that a far-off
    // pad asks for a steady tilt, not a somersault. When tumbling, turn upright
    // first, along a braking curve in angle.
    var tumbling = Math.abs(th) > 0.8 || Math.abs(x[5]) > 0.7;
    var gimbal;
    if (tumbling) {
      var spinWant = -(th > 0 ? 1 : -1) * Math.min(Math.sqrt(2 * 0.5 * spinUp * Math.abs(th)), 2 * Math.abs(th));
      gimbal = 6 * (x[5] - spinWant);
    } else {
      var reach = Math.min(X_REACH, 0.3 * height);   // close to the ground, staying upright matters more than position
      gimbal = Kside[0] * clamp(x[0], -reach, reach) + Kside[1] * x[1] + Kside[2] * th + Kside[3] * x[5];
    }
    gimbal = clamp(gimbal, -GIMBAL_MAX, GIMBAL_MAX);
    // Height: braking curve down to the touchdown speed, with its deceleration fed forward.
    var aBrake = 0.45 * g;                                             // more than minimum throttle ever gives
    var vBrake = -Math.min(Math.sqrt(2 * aBrake * height), 0.8 * height) + TOUCH.dy;   // eases off for the last metres
    var overPad = Math.abs(x[0]) < 10 && Math.abs(x[1]) < 1.5 && Math.abs(th) < 0.03 && Math.abs(x[5]) < 0.03;
    var vWant = vBrake, aWant = (x[3] < TOUCH.dy && height > 2 * aBrake / 0.64) ? aBrake : 0;
    if (canHover && !overPad) { var vSafe = clamp(0.4 * (SAFE_HEIGHT - height), -15, 15); if (vSafe > vBrake) { vWant = vSafe; aWant = 0; } }
    var throttle = (g + aWant + 3 * (vWant - x[3])) / (g * twr * Math.max(upright, 0.3));
    if (tumbling) throttle = upright > 0 ? 1 : 0.6;                    // thrust is what turns the rocket
    return [clamp(throttle, thrMin, 1), gimbal];
  }

  var mpc = makeMPC({
    n: 6, m: 2, N: 400, h: h, sub: SUB, rho: 1e-4, rhoGrow: 10, rhoMax: 10,
    uMin: [thrMin, -GIMBAL_MAX], uMax: [1, GIMBAL_MAX], f: f,
    nr: 5,
    // Small running cost: stay roughly over the pad, upright, with calm inputs.
    residual: function (x, u, res) {
      res[0] = 0.002 * x[0]; res[1] = 0.3 * Math.sin(x[4] / 2); res[2] = 0.05 * x[5]; res[3] = 0.05 * u[0]; res[4] = 0.5 * u[1];
    },
    nrT: 6,
    terminalResidual: function (x, res) { var e = err(x); for (var i = 0; i < 6; i++) res[i] = e[i] / TOL[i]; }
  });

  // Starting guess: the conventional law, simulated until it touches down.
  var STRETCH = [1, 1.3, 1.7, 0.8];
  function seed(x0, attempt)
  {
    if (attempt >= STRETCH.length) return false;
    var x = x0.slice(), xn = [0, 0, 0, 0, 0, 0], steps = 0;
    while (steps < 380 && clearance(x) > 0) { mpc.step(x, cascade(x), xn); x = xn.slice(); steps++; }
    mpc.setHorizon(Math.max(10, Math.round(STRETCH[attempt] * steps)));
    mpc.guess(x0, cascade);
  }

  var drive = makeManeuver({
    mpc: mpc, sub: SUB,
    plan: {iters: 40, outer: 8, margin: 1.03, seeds: [seed],
      shrink: [0.6, 0.8, 0.9, 0.95],
      // Accept a plan only if it stays off the ground until its very end and
      // the contact there is a clean landing.
      valid: function (X, N) {
        for (var k = 0; k <= N; k++) if (clearance(X[k]) <= 0) return k >= N - 2 && goodTouchdown(X[k], 0.7);
        return false;
      }},
    reached: function (x) { var e = err(x); for (var i = 0; i < 6; i++) if (Math.abs(e[i]) > 0.5 * TOL[i]) return false; return true; },
    strayed: function (x, xPlan) { return Math.abs(x[0] - xPlan[0]) > 3 || Math.abs(x[2] - xPlan[2]) > 3 || Math.abs(x[4] - xPlan[4]) > 0.05; },
    canHold: function () { return true; },
    hold: cascade,
    fallback: cascade
  });

  return {
    control: drive,
    // Where the rocket will be one control period after applying u.
    predict: (function () {
      var one = makeMPC({n: 6, m: 2, N: 1, h: h, f: f, uMin: [0, 0], uMax: [1, 1], nr: 1, nrT: 1});
      return function (x, u) { var out = [0, 0, 0, 0, 0, 0]; one.step(x, [clamp(u[0], thrMin, 1), clamp(u[1], -GIMBAL_MAX, GIMBAL_MAX)], out); return out; };
    })()
  };
}

// One level flies three rockets through the same controlFunction, so the
// controller keeps one planner per rocket and recognises each rocket by where
// its model said it would be next.
var rockets = [];

function controlFunction(rocket)
{
  var x = [rocket.x, rocket.dx, rocket.y, rocket.dy, rocket.theta, rocket.dtheta];
  var mine = null, i, j;
  for (i = 0; i < rockets.length && !mine; i++) {
    var e = rockets[i].expect, d = 0;
    for (j = 0; j < 6; j++) d = Math.max(d, Math.abs(e[j] - x[j]));
    if (d < 1e-3 && rockets[i].T < rocket.T) mine = rockets[i];
  }
  if (!mine) { mine = makeRocket(rocket, 0.02); rockets.push(mine); }
  var u = mine.control(x);
  mine.expect = mine.predict(x, u);
  mine.T = rocket.T;
  return {throttle: u[0], gimbalAngle: u[1]};
}
