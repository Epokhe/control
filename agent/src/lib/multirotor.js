// ---------------------------------------------------------------------------
// Multirotor: fly to the origin and hover there, as fast as possible.
// Shared by the three multirotor levels; a level adds its own keep-out
// penalties (walls) and starting guesses for the planner.
// State: [x, dx, y, dy, theta, dtheta], inputs: [thrustLeft, thrustRight].
// ---------------------------------------------------------------------------
var TOL = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01];   // the level's own completion tolerances

// level: {nPenalty, penalty(x, r, offset), valid(states, N), stiffen(more), seeds(tools) -> [seed functions],
//         fallback(x, tools): what to do while no plan has been found (default: hover at the goal)}
function makeMultirotorController(level)
{
  var drive = null;

  function build(v, h)
  {
    var hover = v.mass * v.g / 2, tMax = v.maxThrust;
    var nP = level.nPenalty || 0;
    function f(x, u, out)
    {
      var T = (u[0] + u[1]) / v.mass;
      out[0] = x[1]; out[1] = Math.sin(x[4]) * T;
      out[2] = x[3]; out[3] = Math.cos(x[4]) * T - v.g;
      out[4] = x[5]; out[5] = (u[0] - u[1]) * v.Length / v.I;
    }
    var mpc = makeMPC({
      n: 6, m: 2, N: 300, h: h, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [0, 0], uMax: [tMax, tMax], f: f,
      nr: 8 + nP,
      // Small running cost: prefer being near the goal and near hover thrust.
      residual: function (x, u, r) {
        r[0] = 0.5 * x[0]; r[1] = 0.05 * x[1]; r[2] = 0.5 * x[2]; r[3] = 0.05 * x[3]; r[4] = 0.2 * x[4]; r[5] = 0.02 * x[5];
        r[6] = 0.01 * (u[0] - hover); r[7] = 0.01 * (u[1] - hover);
        if (nP) level.penalty(x, r, 8);
      },
      nrT: 6,
      terminalResidual: function (x, r) { for (var i = 0; i < 6; i++) r[i] = x[i] / TOL[i]; }
    });

    var lin = mpc.linearAt([0, 0, 0, 0, 0, 0], [hover, hover]);
    var K = dlqr(lin.A, lin.B, numeric.diag([400, 20, 400, 20, 100, 1]), numeric.diag([0.02, 0.02]));
    // LQR towards a hover point (default: the goal).
    function hoverAt(x, px, py)
    {
      var e = [x[0] - (px || 0), x[1], x[2] - (py || 0), x[3], x[4], x[5]], u = [hover, hover];
      for (var q = 0; q < 2; q++) { for (var i = 0; i < 6; i++) u[q] -= K[q][i] * e[i]; u[q] = Math.max(0, Math.min(tMax, u[q])); }
      return u;
    }
    function hold(x) { return hoverAt(x, 0, 0); }
    // Fly towards a point from any state: a position loop asks for an
    // acceleration, limited so that the tilt it takes stays moderate, and a
    // fast attitude loop turns the vehicle to deliver it. Slower than the
    // regulator above near the point, but it does not mind a bad start.
    function flyTo(x, px, py, vx, vy)   // vx, vy: velocity of the point, if it moves
    {
      var ax = Math.max(-6, Math.min(6, 4 * (px - x[0]) + 4 * ((vx || 0) - x[1])));
      var ay = Math.max(-0.5 * v.g, Math.min(6, 4 * (py - x[2]) + 4 * ((vy || 0) - x[3])));
      var tilt = Math.atan2(ax, ay + v.g);
      var thrust = v.mass * (ay + v.g) / Math.max(0.5, Math.cos(x[4]));
      var turn = (64 * (tilt - x[4]) - 16 * x[5]) * v.I / v.Length / 2;
      return [Math.max(0, Math.min(tMax, thrust / 2 + turn)), Math.max(0, Math.min(tMax, thrust / 2 - turn))];
    }
    function within(x, scale) { for (var i = 0; i < 6; i++) if (Math.abs(x[i]) > scale * TOL[i]) return false; return true; }

    var tools = {mpc: mpc, hold: hold, hoverAt: hoverAt, flyTo: flyTo, hover: hover, v: v, h: h};
    drive = makeManeuver({
      mpc: mpc,
      plan: {iters: 40, outer: 8, margin: 1.05, shrink: level.shrink, stiffen: level.stiffen, valid: level.valid, seeds: level.seeds(tools)},
      reached: function (x) { return within(x, 0.5); },
      strayed: function (x, xPlan) { for (var i = 0; i < 6; i++) if (Math.abs(x[i] - xPlan[i]) > 30 * TOL[i]) return true; return false; },
      canHold: function (x) { return Math.abs(x[4]) < 0.5 && Math.abs(x[0]) < 0.5 && Math.abs(x[2]) < 0.5; },
      hold: hold,
      fallback: level.fallback ? function (x) { return level.fallback(x, tools); } : hold
    });
  }

  return function (vehicle)
  {
    if (!drive) build(vehicle, 0.02);
    var u = drive([vehicle.x, vehicle.dx, vehicle.y, vehicle.dy, vehicle.theta, vehicle.dtheta]);
    return {thrustLeft: u[0], thrustRight: u[1]};
  };
}
