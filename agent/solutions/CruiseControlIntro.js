// Cruise control (both levels): hold the target speed on a sloped road.
//
// The car is a first-order system:
//     speed' = maxAcceleration * throttle - drag * speed * |speed| - g * sin(inclination)
// so the controller is model inversion plus a one-step (deadbeat) correction:
//   1. feedforward: the throttle that cancels drag and gravity right now,
//   2. tracking:    the extra acceleration that closes the speed error within
//                   one control period, aimed at where the target will be at
//                   the end of that period (the target's rate is estimated
//                   from its last two values, nothing is assumed about how it
//                   moves).
// The throttle saturates at +-1 while the error is large, which is the
// time-optimal thing to do for a first-order plant.

var prev = null; // {T, target}

function controlFunction(vehicle)
{
  var h = prev ? vehicle.T - prev.T : 0.02;
  var targetRate = (prev && h > 0) ? (vehicle.targetSpeed - prev.target) / h : 0;
  prev = {T: vehicle.T, target: vehicle.targetSpeed};

  var v = vehicle.speed;
  var resistance = vehicle.drag * v * Math.abs(v) + vehicle.g * Math.sin(vehicle.inclination);
  var targetNext = vehicle.targetSpeed + targetRate * h;
  var acceleration = (targetNext - v) / h;
  return Math.max(-1, Math.min(1, (acceleration + resistance) / vehicle.maxAcceleration));
}
