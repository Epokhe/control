// Multirotor flip: starting at theta = 2 pi, unwind to theta = 0 (one full
// flip) and hover at the origin, without leaving the box.

//#include lib/mpc.js

//#include lib/multirotor.js

// The game fails the level when the vehicle's centre is 0.92 from the middle,
// by which point the rotors would be inside the wall (the box's inner faces
// are at 1). Here the whole vehicle has to stay inside.
var BODY = 0.2;            // half the vehicle's span, with a little to spare
var BOX = 1 - BODY;        // limit for the centre
var CLEARANCE = 0.1;       // distance from that limit the planner aims to keep
var WEIGHT = 40;

var controller = makeMultirotorController({
  nPenalty: 2,
  penalty: function (x, r, offset) {
    r[offset] = WEIGHT * Math.max(0, Math.abs(x[0]) - (BOX - CLEARANCE));
    r[offset + 1] = WEIGHT * Math.max(0, Math.abs(x[2]) - (BOX - CLEARANCE));
  },
  valid: function (X, N) {
    for (var k = 0; k <= N; k++) if (Math.abs(X[k][0]) > BOX || Math.abs(X[k][2]) > BOX) return false;
    return true;
  },
  // Without a plan: hover upright where the vehicle is, whichever way up it
  // counts as, and plan again from there.
  fallback: function (x, t) {
    var turns = 2 * Math.PI * Math.round(x[4] / (2 * Math.PI));
    return t.hoverAt([x[0], x[1], x[2], x[3], x[4] - turns, x[5]], 0, 0);
  },
  // Starting guess: the hover regulator, which already spins the vehicle
  // towards theta = 0. The optimiser adds the climb that makes room for it.
  seeds: function (t) {
    var TIMES = [1.2, 1.6, 2.2, 3];
    return [function (x0, attempt) {
      if (attempt >= TIMES.length) return false;
      t.mpc.setHorizon(Math.round(TIMES[attempt] / t.h));
      t.mpc.guess(x0, t.hold);
    }];
  }
});

function controlFunction(vehicle) { return controller(vehicle); }
