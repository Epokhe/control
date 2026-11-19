// Ball on platform, bounce: keep the ball bouncing straight up and down with
// its highest point exactly at the origin.
//
// Every bounce is aimed (see "Aimed bounces" below). The ball starts off to
// the side, so the first bounce is a low hop that brings it down over the
// centre, and the next one sends it straight up. The same rule covers any
// later disturbance: a ball that is not coming down over the centre gets
// hopped there.

//#include lib/paddle.js

//#include lib/aim.js

var HOP_APOGEE = -2.0;                // top of a hop
var LENGTH = {hop: 1.75, afterHop: 2.1, steady: 2.1};   // piston length to bounce at
var CENTRED = 0.008;                  // sideways error small enough to trim without a hop
// Paddle acceleration at contact. With this value a bounce that happens a
// little earlier or later than expected sends the ball equally high.
var CONTACT_ACCEL = -PADDLE.restitution / (1 + PADDLE.restitution) * PADDLE.g;

var aimer = makeAimer(), target = null, last = null, hopped = false;

// Bounce the ball to the centre: the hop has a given height and must come
// down, at the following bounce, over the centre.
function hop(s)
{
  return {
    hop: true, length: LENGTH.hop, accel: CONTACT_ACCEL,
    guess: function (tau) { return {speed: roughAim(0, s.vy - PADDLE.g * tau, LENGTH.hop, HOP_APOGEE).speed, angle: 0}; },
    miss: function (after) {
      var landing = nextBounce(after, LENGTH.afterHop);
      return [after.y + after.vy * after.vy / (2 * PADDLE.g) - HOP_APOGEE, landing ? landing.s.x : after.x];
    }
  };
}

// Bounce the ball straight up to the origin. A small sideways offset that is
// left is removed slowly.
function straightUp(s, length)
{
  return {
    length: length, accel: CONTACT_ACCEL,
    guess: function (tau) { return roughAim(s.vx, s.vy - PADDLE.g * tau, length, 0); },
    miss: function (after) {
      var drift = clamp(-after.x / (2 * after.vy / PADDLE.g), -0.005, 0.005);
      return [after.y + after.vy * after.vy / (2 * PADDLE.g), after.vx - drift];
    }
  };
}

// Where the ball will next bounce if it is then sent straight up.
function nextBounce(s, length)
{
  var tau = fallTime(s, restHeight(length)), rough = roughAim(s.vx, s.vy - PADDLE.g * tau, length, 0);
  return aimShoot(s, {length: length, accel: CONTACT_ACCEL, speed: rough.speed, angle: rough.angle}, tau);
}

function controlFunction(ball, piston, hinge, T)
{
  var s = paddleState(ball, piston, hinge);
  // A jump in the ball's velocity means it has bounced: aim the next one.
  if (last !== null && Math.abs(ball.vy - (last - PADDLE.g * PADDLE.dt)) > 1e-6) { hopped = target.hop; target = null; aimer.reset(); }
  last = ball.vy;
  if (!target) {
    var length = hopped ? LENGTH.afterHop : LENGTH.steady, landing = nextBounce(s, length);
    target = !landing || Math.abs(landing.s.x) > CENTRED ? hop(s) : straightUp(s, length);
  }
  return aimer.step(s, target) || {pistonAcceleration: -4 * piston.speed, hingeAcceleration: -4 * hinge.speed};
}
