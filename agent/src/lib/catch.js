// ---------------------------------------------------------------------------
// Catching the ball.
//
// A ball cannot simply be caught: the piston is hardly quicker than gravity,
// so it cannot match the speed of a fast ball, and every bounce keeps 90 % of
// the speed between ball and paddle. The catcher therefore plays three aimed
// bounces (see "Aimed bounces"):
//   hop    a low bounce that brings the ball down over the wanted spot,
//   pop    a bounce there with a tilted paddle that removes the sideways
//          speed and leaves just enough height to level the paddle again,
//   drop   a bounce off a paddle moving away that takes out most of the rest.
// What is left is a small bounce: the paddle chases the ball upwards and then
// falls with it, so that they meet gently, and finally comes to a stop with
// the ball lying on top. Which step comes next is decided from where the ball
// is heading, so the same rules deal with any start.
//
// makeCatcher(P) returns step(ball, piston, hinge) -> {pistonAcceleration,
// hingeAcceleration, resting}. P.x is where the ball should come to rest.
// step.settle is the piston rule for a ball that is on the paddle.
// ---------------------------------------------------------------------------
var CATCH_DEFAULTS = {
  x: 0,
  hop: 0.5, pop: 0.1,            // heights of the first two aimed bounces above the paddle (the hop at least)
  popTilt: 0.32,                 // largest tilt planned for the pop
  dropSpeed: 0.4,                // upward speed left to the ball by the third
  hopLength: 1.75, popLength: 1.9, dropLength: 1.8,   // piston lengths to bounce at
  small: 0.06,                   // bounces lower than this are not aimed
  near: 0.05,                    // a ball coming down this close to the spot needs no hop
  slow: 0.05,                    // sideways speed below which a bounce with a level paddle will do
  touch: 0.05,                   // closing speed kept until the ball touches
  brake: 8,                      // piston deceleration planned for stopping
  carry: 6,                      // strongest downward piston acceleration with the ball on top
  restGap: 0.004, restSpeed: 0.3,   // the ball counts as lying on the paddle below these
  chase: 0.4,                    // closing speed on small bounces, as a share of free-fall speed over the gap
  slide: 2, soft: 0.5, lead: 0.2, tiltMax: 0.3,       // sliding the resting ball to the spot
  hinge: 12, hingeSoft: 0.8
};

// Speed to approach a target with: brakes at a constant rate, eases off close by.
function approach(err, decel, soft) { return -Math.sign(err) * (Math.sqrt(soft * soft + 2 * decel * Math.abs(err)) - soft); }

function makeCatcher(options)
{
  var P = {}, key;
  for (key in CATCH_DEFAULTS) P[key] = options && key in options ? options[key] : CATCH_DEFAULTS[key];
  var aimer = makeAimer(), mode = null, last = null;
  var CONTACT_ACCEL = -PADDLE.restitution / (1 + PADDLE.restitution) * PADDLE.g;


  function riseOf(after) { return after.vy * after.vy / (2 * PADDLE.g); }

  // The ball's state at its next bounce, if that is aimed at the given target.
  function nextBounce(s, t)
  {
    var tau = fallTime(s, restHeight(t.length)), rough = t.guess(tau);
    return aimShoot(s, {length: t.length, accel: t.accel, speed: rough.speed, angle: rough.angle}, tau);
  }

  // Hop: a low bounce that comes down over the wanted spot. The further the
  // ball has to travel, the higher the hop, so that the tilt needed to stop
  // its sideways motion at the next bounce stays moderate.
  function hopTarget(s)
  {
    var g = PADDLE.g, fall = fallTime(s, restHeight(P.hopLength)), far = Math.abs(s.x + s.vx * fall - P.x);
    var height = P.hop, popSpeed = Math.sqrt(2 * g * P.pop);
    while (height < 2 && far / (2 * Math.sqrt(2 * height / g)) > Math.tan(P.popTilt) * (popSpeed + Math.sqrt(2 * g * height))) height += 0.05;
    return {
      length: P.hopLength, accel: CONTACT_ACCEL,
      guess: function (tau) { return {speed: roughAim(0, s.vy - g * tau, P.hopLength, restHeight(P.hopLength) + height).speed, angle: 0}; },
      miss: function (after) {
        var landing = nextBounce(after, popTarget(after));
        return [riseOf(after) - height, (landing ? landing.s.x : after.x) - P.x];
      }
    };
  }

  // Pop: take out the sideways speed, but for the little that carries the ball
  // exactly to the centre by the next bounce.
  function popTarget(s)
  {
    return {
      length: P.popLength, accel: CONTACT_ACCEL,
      guess: function (tau) { return roughAim(s.vx, s.vy - PADDLE.g * tau, P.popLength, restHeight(P.popLength) + P.pop); },
      miss: function (after) {
        var landing = nextBounce(after, dropTarget(after));
        return [riseOf(after) - P.pop, (landing ? landing.s.x : after.x) - P.x];
      }
    };
  }

  // Drop: with the paddle level again, take nearly all the speed out of the ball.
  function dropTarget(s)
  {
    return {
      length: P.dropLength, accel: CONTACT_ACCEL,
      guess: function (tau) {
        var vIn = s.vy - PADDLE.g * tau;
        return {speed: (P.dropSpeed + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(s.vx / (P.dropSpeed - vIn))};
      },
      miss: function (after) { return [after.vy - P.dropSpeed, after.vx]; }
    };
  }

  function choose(s)
  {
    var landing = nextBounce(s, popTarget(s));
    if (!landing || Math.abs(landing.s.x - P.x) > P.near) return {name: 'hop', target: hopTarget(s)};
    return Math.abs(s.vx) < P.slow ? {name: 'drop', target: dropTarget(s)} : {name: 'pop', target: popTarget(s)};
  }

  // Piston rule for a ball lying on the paddle or bouncing a little.
  // Lying: move at the speed wanted (zero to stop), without throwing the ball
  // off. Bouncing: move with the ball, closing in slowly, so that it lands softly.
  function settle(ball, piston, c, wanted)
  {
    var M = PADDLE, gap = Math.max(0, c.gap), lowest = -M.pistonAccel, want = wanted;
    if (gap < P.restGap && Math.abs(c.closing) < P.restSpeed) lowest = -P.carry;
    else want = ball.vy - (c.pvy - piston.speed) - M.g * M.dt + P.touch + P.chase * Math.sqrt(2 * (M.g + M.pistonAccel) * gap);
    var up = Math.sqrt(2 * P.brake * Math.max(0, M.lengthMax - 0.01 - piston.length));
    var down = -Math.sqrt(2 * P.brake * Math.max(0, piston.length - M.lengthMin - 0.01));
    return clamp((clamp(want, down, up) - piston.speed) / M.dt, lowest, M.pistonAccel);
  }

  function step(ball, piston, hinge)
  {
    var M = PADDLE, s = paddleState(ball, piston, hinge), c = paddleContact(s);
    if (last !== null && Math.abs(ball.vy - (last - M.g * M.dt)) > 1e-6) { mode = null; aimer.reset(); }
    last = ball.vy;
    var gap = Math.max(0, c.gap), resting = gap < P.restGap && Math.abs(c.closing) < P.restSpeed;

    if (!resting) {
      // Bounces too small to aim are followed with the simple rules further down.
      if (!mode) mode = s.y + s.vy * s.vy / (2 * M.g) - (c.py + M.radius) < P.small ? {name: 'small'} : choose(s);
      if (mode.name !== 'small') {
        var u = aimer.step(s, mode.target);
        if (u) { u.resting = false; return u; }
      }
    }

    var pistonAcceleration = settle(ball, piston, c, 0);

    // Hinge. The ball only changes its sideways speed when it presses on a
    // tilted paddle: in one go when it bounces, steadily when it rests.
    var vxWant = approach(ball.x - P.x, P.slide, P.soft);
    var impact = Math.sqrt(Math.max(0, c.closing) * Math.max(0, c.closing) + 2 * M.g * gap);
    var push = Math.max((1 + M.restitution) * impact, M.g * P.lead);
    var tilt = clamp((ball.vx - vxWant) / push, -P.tiltMax, P.tiltMax);
    var rate = approach(hinge.angle - tilt, P.hinge, P.hingeSoft);
    var hingeAcceleration = clamp((rate - hinge.speed) / M.dt, -M.hingeAccel, M.hingeAccel);
    return {pistonAcceleration: pistonAcceleration, hingeAcceleration: hingeAcceleration, resting: resting};
  }

  step.settle = settle;
  return step;
}
