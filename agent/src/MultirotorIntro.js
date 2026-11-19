// Multirotor intro: fly into the box at the origin and hover there.

//#include lib/mpc.js

//#include lib/multirotor.js

var controller = makeMultirotorController({
  // Starting guess: simulate the hover regulator; try a few flight times,
  // scaled from the distance to cover and the acceleration available.
  seeds: function (t) {
    var FACTORS = [1.5, 2.2, 3.2];
    return [function (x0, attempt) {
      if (attempt >= FACTORS.length) return false;
      var dist = Math.sqrt(x0[0] * x0[0] + x0[2] * x0[2]);
      var time = 2 * Math.sqrt(dist / t.v.g) + 0.3;      // bang-bang at about 1 g, plus time to turn
      t.mpc.setHorizon(Math.round(FACTORS[attempt] * time / t.h));
      t.mpc.guess(x0, t.hold);
    }];
  }
});

function controlFunction(vehicle) { return controller(vehicle); }
