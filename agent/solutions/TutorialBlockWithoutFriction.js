// Tutorials 1-3: move a block to x = 0 and stop it there.
//
// The same controller solves all three tutorials. It never reads the friction,
// gravity or slope parameters. Instead it treats the block as a double
// integrator  x'' = F + w  with an unknown lumped disturbance w (friction,
// slope, anything else) and estimates w from how the velocity actually changed
// over the last step. With w known, the fastest way to the target is
// bang-bang: full force towards it, then full force against the motion, timed
// by the braking curve v = sqrt(2 * a_brake * distance).
//
// Each step the controller picks the force that puts the block on that curve
// at the end of the step. Far away this saturates (bang-bang). In the last
// centimetres the curve becomes the line v = -2x/h, which is exactly the
// two-step deadbeat law for a sampled double integrator, so the block lands on
// the target and stays there instead of chattering.

var F_MAX = 50;          // actuator limit (the game clips to +-50)
var BRAKE_SHARE = 0.9;   // plan to brake with 90 % of what is available
var prev = null;         // {T, dx, F} from the previous call
var w = 0;               // disturbance estimate [m/s^2]

// Velocity the block should have at position x (sampled braking curve).
function brakingCurve(x, h)
{
  var dir = x > 0 ? -1 : 1;                                 // direction of travel towards the target
  var a = BRAKE_SHARE * Math.max(1, F_MAX - dir * w);       // deceleration available when moving that way
  var c = a * h / 2;
  return dir * (Math.sqrt(c * c + 2 * a * Math.abs(x)) - c);
}

function controlFunction(block)
{
  var h = prev ? block.T - prev.T : 0.02;
  if (prev && h > 0) w = (block.dx - prev.dx) / h - prev.F;

  // Mismatch between the velocity after this step and the curve at the
  // position after this step. It grows with F, so bisection finds its root.
  function miss(F)
  {
    var acc = F + w;
    return block.dx + acc * h - brakingCurve(block.x + block.dx * h + 0.5 * acc * h * h, h);
  }
  var lo = -F_MAX, hi = F_MAX, F;
  if (miss(lo) >= 0) F = lo;
  else if (miss(hi) <= 0) F = hi;
  else {
    for (var i = 0; i < 40; i++) {
      F = 0.5 * (lo + hi);
      if (miss(F) > 0) hi = F; else lo = F;
    }
  }
  prev = {T: block.T, dx: block.dx, F: F};
  return F;
}
