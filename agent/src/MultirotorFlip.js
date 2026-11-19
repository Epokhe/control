// Multirotor flip: starting at theta = 2 pi, unwind to theta = 0 (one full
// flip) and hover at the origin, without leaving the box.

//#include lib/mpc.js

//#include lib/multirotor.js

var BOX = 1 - 0.8 * 0.1;   // the level fails outside |x|, |y| < boxRadius - 0.8 * Length
var CLEARANCE = 0.15;
var WEIGHT = 40;

var controller = makeMultirotorController({
  nPenalty: 2,
  penalty: function (x, r, offset) {
    r[offset] = WEIGHT * Math.max(0, Math.abs(x[0]) - (BOX - CLEARANCE));
    r[offset + 1] = WEIGHT * Math.max(0, Math.abs(x[2]) - (BOX - CLEARANCE));
  },
  valid: function (X, N) {
    for (var k = 0; k <= N; k++) if (Math.abs(X[k][0]) > BOX - 0.03 || Math.abs(X[k][2]) > BOX - 0.03) return false;
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
