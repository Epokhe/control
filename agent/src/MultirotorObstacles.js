// Multirotor obstacles: fly to the origin without touching the two walls.

//#include lib/mpc.js

//#include lib/multirotor.js

// The walls, from the level description. Each blocks a 0.2 wide strip of x
// either above or below a given height.
var WALLS = [
  {x: -1.5, halfWidth: 0.1, edge: 0.8, blocks: 'above'},
  {x: -0.5, halfWidth: 0.1, edge: 1.2, blocks: 'below'}
];
var CLEARANCE = 0.12;   // distance the planner keeps from a wall
var WEIGHT = 40;
var parked = null;

// How deep a point is inside a wall grown by `margin` (0 when outside).
function intrusion(wall, x, y, margin)
{
  var across = wall.halfWidth + margin - Math.abs(x - wall.x);
  var along = wall.blocks === 'above' ? y - (wall.edge - margin) : (wall.edge + margin) - y;
  return across > 0 && along > 0 ? Math.min(across, along) : 0;
}

var controller = makeMultirotorController({
  nPenalty: WALLS.length,
  penalty: function (x, r, offset) {
    for (var i = 0; i < WALLS.length; i++) r[offset + i] = WEIGHT * intrusion(WALLS[i], x[0], x[2], CLEARANCE);
  },
  // A plan is only accepted if it stays clear of every wall, also between plan steps.
  valid: function (X, N) {
    for (var k = 0; k < N; k++) for (var s = 0; s < 4; s++) {
      var a = s / 4, x = X[k][0] + a * (X[k + 1][0] - X[k][0]), y = X[k][2] + a * (X[k + 1][2] - X[k][2]);
      for (var i = 0; i < WALLS.length; i++) if (intrusion(WALLS[i], x, y, 0.03) > 0) return false;
    }
    return true;
  },
  // Without a plan: hover on the spot rather than fly at a wall, and plan again.
  fallback: function (x, t) {
    if (!parked) parked = {x: x[0], y: x[2]};
    return t.hoverAt(x, parked.x, parked.y);
  },
  // Starting guess: fly the hover regulator through the open side of each
  // wall still ahead, then to the goal. The optimiser rounds off the corners.
  seeds: function (t) {
    var FACTORS = [1, 1.2, 1.4, 1.7, 2, 2.5];
    function route(cautious) { return function (x0, attempt) {
      if (attempt >= FACTORS.length) return false;
      var gates = [], i, w, length = 0, px = x0[0], py = x0[2];
      for (i = 0; i < WALLS.length; i++) {
        w = WALLS[i];
        if (w.x > x0[0]) gates.push({x: w.x, y: w.blocks === 'above' ? w.edge - 0.4 : w.edge + 0.4, wall: w});
      }
      gates.push({x: 0, y: 0});
      for (i = 0; i < gates.length; i++) { length += Math.sqrt(Math.pow(gates[i].x - px, 2) + Math.pow(gates[i].y - py, 2)); px = gates[i].x; py = gates[i].y; }
      var next = 0;
      t.mpc.setHorizon(Math.round(FACTORS[attempt] * (2 * Math.sqrt(length / t.v.g) + 0.6) / t.h));
      t.mpc.guess(x0, function (x) {
        while (next < gates.length - 1 && x[0] > gates[next].x - 0.15) next++;
        // Aim a little past the gate so the guess keeps moving.
        var g = gates[next], ahead = next < gates.length - 1 ? 0.3 : 0;
        // Level with the blocked part of the wall ahead: get to the open side first.
        if (cautious && g.wall && x[0] < g.wall.x - g.wall.halfWidth && intrusion(g.wall, g.wall.x, x[2], CLEARANCE) > 0) return t.hoverAt(x, Math.min(x[0], g.wall.x - 0.35), g.y);
        return t.hoverAt(x, g.x + ahead, g.y);
      });
    }; }
    return [route(false), route(true)];
  }
});

function controlFunction(vehicle) { return controller(vehicle); }
