// Multirotor obstacles: fly to the origin without touching the two walls.

//#include lib/mpc.js

//#include lib/multirotor.js

// The walls as the game draws them. Each fills a 0.2 wide strip of x, either
// above or below a given height.
var WALLS = [
  {x: -1.5, halfWidth: 0.1, edge: 0.4, blocks: 'above'},
  {x: -0.5, halfWidth: 0.1, edge: 1.6, blocks: 'below'}
];
// The game itself only fails the level when the vehicle's centre is well
// inside a wall, which lets the vehicle fly visibly through the wall ends.
// Here the whole vehicle has to stay clear of the walls as drawn.
var BODY = 0.18;        // half the vehicle's span (as the game draws it)
var KEEP = 0.25;        // least distance from a wall, centre to wall, that a plan may have:
                        // the body plus an allowance for not following the plan exactly
var CLEARANCE = 0.27;   // distance from a wall the planner aims to keep
var WEIGHT = 60, weight = WEIGHT;   // penalty weight: where an attempt starts, and where it is now
var parked = null;

// Distance from a point to a wall; inside the wall, minus the depth.
function distance(wall, x, y)
{
  var across = Math.abs(x - wall.x) - wall.halfWidth, along = wall.blocks === 'above' ? wall.edge - y : y - wall.edge;
  if (across < 0 && along < 0) return Math.max(across, along);
  across = Math.max(0, across); along = Math.max(0, along);
  return Math.sqrt(across * across + along * along);
}

// Corner points of a route around the walls still ahead: past the end of
// each wall on its open side, then to the goal.
function route(x0, y0)
{
  var pts = [{x: x0, y: y0}], gap = CLEARANCE + 0.05, i;
  for (i = 0; i < WALLS.length; i++) {
    var w = WALLS[i];
    var y = w.blocks === 'above' ? w.edge - gap : w.edge + gap, side = w.halfWidth + 0.15;
    if (x0 < w.x - side) pts.push({x: w.x - side, y: y});
    if (x0 < w.x + side) pts.push({x: w.x + side, y: y});
  }
  pts.push({x: 0, y: 0});
  return pts;
}

var controller = makeMultirotorController({
  nPenalty: WALLS.length,
  shrink: 0,
  penalty: function (x, r, offset) {
    // Near or inside a wall the plan is pushed towards the wall's open end, not
    // towards its nearest face: a plan pushed out sideways can end up on the
    // wrong side and stay there.
    for (var i = 0; i < WALLS.length; i++) {
      var w = WALLS[i], beside = Math.min(1, Math.max(0, (w.halfWidth + CLEARANCE - Math.abs(x[0] - w.x)) / CLEARANCE));
      var past = (w.blocks === 'above' ? x[2] - w.edge : w.edge - x[2]) + CLEARANCE;
      r[offset + i] = weight * beside * Math.min(1, Math.max(0, past));
    }
  },
  stiffen: function (more) { weight = more ? 2 * weight : WEIGHT; },
  // A plan is only accepted if the whole vehicle stays clear of every wall,
  // also between plan steps. (A start that is already closer than that must at
  // least not get any closer.)
  valid: function (X, N) {
    for (var i = 0; i < WALLS.length; i++) {
      var least = Math.min(KEEP, distance(WALLS[i], X[0][0], X[0][2])) - 1e-9;
      for (var k = 0; k < N; k++) for (var s = 0; s < 4; s++) {
        var a = s / 4;
        if (distance(WALLS[i], X[k][0] + a * (X[k + 1][0] - X[k][0]), X[k][2] + a * (X[k + 1][2] - X[k][2])) < least) return false;
      }
    }
    return true;
  },
  // Without a plan: hover on the spot rather than fly at a wall, and plan again.
  fallback: function (x, t) {
    if (!parked) parked = {x: x[0], y: x[2]};
    return t.flyTo(x, parked.x, parked.y);
  },
  // Starting guess: the vehicle follows a point that moves along the route at
  // a steady pace. It cuts the corners a little; the planner straightens that
  // out, and then shortens the flight.
  seeds: function (t) {
    // Speeds of the moving point to try, m/s, fastest first. The first that
    // works is kept: squeezing a slower plan afterwards tends to lose the way
    // over the second wall.
    var PACES = [[4, 3.5, 3, 2.6, 2.3, 2], [1.7, 1.5, 1.3, 1.1, 0.9, 0.7]];
    return PACES.map(function (PACE) { return function (x0, attempt) {
      if (attempt >= PACE.length) return false;
      var pts = route(x0[0], x0[2]), lengths = [], total = 0, i;
      for (i = 1; i < pts.length; i++) { lengths.push(Math.sqrt(Math.pow(pts[i].x - pts[i - 1].x, 2) + Math.pow(pts[i].y - pts[i - 1].y, 2))); total += lengths[i - 1]; }
      var pace = PACE[attempt], k = 0;
      t.mpc.setHorizon(Math.round((total / pace + 1.2) / t.h));   // and time to settle at the end
      t.mpc.guess(x0, function (x) {
        var along = pace * t.h * k++, seg = 0;
        if (along >= total) return t.flyTo(x, 0, 0);
        while (seg < lengths.length - 1 && along > lengths[seg]) along -= lengths[seg++];
        var a = along / lengths[seg], ux = (pts[seg + 1].x - pts[seg].x) / lengths[seg], uy = (pts[seg + 1].y - pts[seg].y) / lengths[seg];
        return t.flyTo(x, pts[seg].x + a * lengths[seg] * ux, pts[seg].y + a * lengths[seg] * uy, pace * ux, pace * uy);
      });
    }; });
  }
});

function controlFunction(vehicle) { return controller(vehicle); }
