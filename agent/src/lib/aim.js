// ---------------------------------------------------------------------------
// Aimed bounces.
//
// Before a bounce the controller picks what the paddle should be doing at the
// moment of contact (piston length, speed and acceleration, and the tilt),
// drives piston and hinge there along smooth curves, and checks the aim by
// running the model forward to the bounce. Newton's method corrects speed and
// tilt until the predicted outcome is the wanted one. This is repeated every
// control period, so errors are corrected for as long as the ball is in the
// air.
// ---------------------------------------------------------------------------

// Shortly before contact the plan is no longer changed.
var AIM_COMMIT = 3.5 * PADDLE.dt;

// Piston and hinge accelerations over time that reach the goal {length, speed,
// accel, angle} (hinge at rest) after time tau, starting from state s.
function aimCurves(s, goal, tau)
{
  tau = Math.max(tau, PADDLE.dt);
  var D = goal.length - s.L - s.v * tau, E = goal.speed - s.v;
  var a0 = (12 * D - 6 * E * tau + goal.accel * tau * tau) / (tau * tau);
  var snap = (6 * (a0 + goal.accel) * tau - 12 * E) / (tau * tau * tau);
  var jerk = (goal.accel - a0 - 0.5 * snap * tau * tau) / tau;
  var Dh = goal.angle - s.th - s.om * tau, Eh = -s.om;
  var h0 = (6 * Dh - 2 * Eh * tau) / (tau * tau), h1 = 2 * (Eh - h0 * tau) / (tau * tau);
  return function (t) {
    if (t >= tau) return [goal.accel, 0];
    return [a0 + jerk * t + 0.5 * snap * t * t, h0 + h1 * t];
  };
}

// Whether those curves stay within what piston and hinge can do.
function aimFeasible(s, goal, tau, margin)
{
  var c = aimCurves(s, goal, tau), L = s.L, v = s.v, dt = PADDLE.dt;
  for (var t = 0; t < tau; t += dt) {
    var u = c(Math.min(t + 0.5 * dt, tau - 1e-9));
    if (Math.abs(u[0]) > PADDLE.pistonAccel * margin || Math.abs(u[1]) > PADDLE.hingeAccel * margin) return false;
    v += u[0] * dt; L += v * dt;
    if (L < PADDLE.lengthMin || L > PADDLE.lengthMax) return false;
  }
  return true;
}

// Run the model forward until the ball bounces. The curves are redrawn from
// the current state every period, as the controller itself does, until
// shortly before contact. Returns the state just after the bounce.
function aimShoot(s0, goal, tau)
{
  var s = {}, k, c, first, since = 0; for (k in s0) s[k] = s0[k];
  for (var i = 0; i < 150; i++) {
    var left = tau - i * PADDLE.dt;
    if (i === 0 || left > AIM_COMMIT) { c = aimCurves(s, goal, left); since = 0; if (i === 0) first = c; }
    var u = c((since + 0.5) * PADDLE.dt);
    since++;
    if (paddleAdvance(s, u[0], u[1])) return {s: s, curve: first, time: (i + 1) * PADDLE.dt};
  }
  return null;
}

// Rough paddle speed and tilt that send a ball, arriving at (vx, vIn), straight
// up to the height apogee from a paddle at the given piston length.
function roughAim(vx, vIn, length, apogee)
{
  var vOut = Math.sqrt(Math.max(0, 2 * PADDLE.g * (apogee - restHeight(length))));
  return {speed: (vOut + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(vx / (vOut - vIn))};
}

// One aimer plans one bounce at a time. Call step every control period with
// the current state and the target:
//   {length, accel: piston length and acceleration at contact,
//    guess(tau): first guess {speed, angle},
//    miss(after): two numbers that are zero when the bounce is as wanted,
//    fixed: true to use the guess as it is}
function makeAimer()
{
  var aim = null, plan = null, left = null;
  function goalOf(target) { return {length: target.length, accel: target.accel, speed: aim.speed, angle: aim.angle}; }
  return {
    reset: function () { aim = null; plan = null; left = null; },
    step: function (s, target) {
      var dt = PADDLE.dt, u;
      if (plan && plan.left <= AIM_COMMIT) {
        plan.step++; plan.left -= dt;
        u = plan.curve((plan.step + 0.5) * dt);
        return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
      }
      // Time to contact: counted down from the first estimate, which is
      // corrected by what the model says.
      var tau = left === null ? fallTime(s, restHeight(target.length)) : left - dt, fresh = left === null ? 2 : 0;
      if (!aim || target.fixed) aim = target.guess(tau);
      var out = null;
      for (var iter = 0; iter < 8; iter++) {
        out = aimShoot(s, goalOf(target), tau);
        if (!out) break;
        if (fresh > 0 && Math.abs(out.time - tau) > 0.5 * dt) { tau = out.time; fresh--; continue; }
        if (target.fixed) break;
        var f = target.miss(out.s);
        if (Math.abs(f[0]) + Math.abs(f[1]) < 1e-10) break;
        var eps = 1e-5, g = goalOf(target);
        g.speed += eps; var a = aimShoot(s, g, tau);
        g.speed -= eps; g.angle += eps; var b = aimShoot(s, g, tau);
        if (!a || !b) break;
        var fa = target.miss(a.s), fb = target.miss(b.s);
        var j00 = (fa[0] - f[0]) / eps, j10 = (fa[1] - f[1]) / eps, j01 = (fb[0] - f[0]) / eps, j11 = (fb[1] - f[1]) / eps;
        var det = j00 * j11 - j01 * j10;
        if (Math.abs(det) < 1e-12) break;
        aim.speed -= clamp((j11 * f[0] - j01 * f[1]) / det, -1, 1);
        aim.angle = clamp(aim.angle - clamp((-j10 * f[0] + j00 * f[1]) / det, -0.2, 0.2), -0.6, 0.6);
      }
      if (!out) return null;
      left = tau;
      plan = {curve: out.curve, step: 0, left: tau};
      u = out.curve(0.5 * dt);
      return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
    }
  };
}
