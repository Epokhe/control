// ---------------------------------------------------------------------------
// Ball and paddle: the machine's dimensions and a model of it.
//
// The paddle is a bar on a hinge on top of a piston. The ball bounces off it
// without friction and keeps 90 % of its speed relative to the paddle.
// ---------------------------------------------------------------------------
var PADDLE = {
  g: 9.81, radius: 0.2, baseY: -5, halfWidth: 1.5, thickness: 0.15, restitution: 0.9,
  lengthMin: 1.5, lengthMax: 2.3, angleMax: 0.7, pistonAccel: 10, hingeAccel: 15,
  dt: 0.02, substeps: 5
};

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function paddleState(ball, piston, hinge)
{
  return {x: ball.x, y: ball.y, vx: ball.vx, vy: ball.vy, L: piston.length, v: piston.speed, th: hinge.angle, om: hinge.speed};
}

// Where the ball is relative to the paddle. along: position along the bar from
// its centre; gap: clearance between ball and bar; (nx, ny): direction from
// the nearest point of the bar to the ball; closing: speed at which the gap
// shrinks; (px, py), (pvx, pvy): that nearest point and its velocity.
function paddleContact(s)
{
  var P = PADDLE, C = Math.cos(s.th), S = Math.sin(s.th);
  var cx = -S * P.thickness, cy = P.baseY + s.L + C * P.thickness;
  var along = C * (s.x - cx) + S * (s.y - cy);
  var a = clamp(along, -P.halfWidth, P.halfWidth);
  var px = cx + C * a, py = cy + S * a;
  var pvx = -C * s.om * P.thickness - S * s.om * a, pvy = s.v - S * s.om * P.thickness + C * s.om * a;
  var nx = s.x - px, ny = s.y - py, d = Math.sqrt(nx * nx + ny * ny);
  nx /= d; ny /= d;
  return {along: along, gap: d - P.radius, nx: nx, ny: ny, px: px, py: py, pvx: pvx, pvy: pvy,
          closing: -(nx * (s.vx - pvx) + ny * (s.vy - pvy))};
}

// Advance the model by one control period with constant accelerations, in the
// same small steps as the machine. Returns true if the ball bounced.
function paddleAdvance(s, pistonAccel, hingeAccel)
{
  var P = PADDLE, h = P.dt / P.substeps, bounced = false;
  pistonAccel = clamp(pistonAccel, -P.pistonAccel, P.pistonAccel);
  hingeAccel = clamp(hingeAccel, -P.hingeAccel, P.hingeAccel);
  for (var i = 0; i < P.substeps; i++) {
    s.x += h * s.vx;
    s.y += h * s.vy - 0.5 * h * h * P.g;
    s.vy -= h * P.g;
    if (s.y - P.radius < P.baseY) s.vy = Math.abs(s.vy);
    s.om += h * hingeAccel; s.th += h * s.om;
    s.v += h * pistonAccel; s.L += h * s.v;
    if (s.L > P.lengthMax) { s.L = P.lengthMax; s.v = Math.min(0, s.v); }
    if (s.L < P.lengthMin) { s.L = P.lengthMin; s.v = Math.max(0, s.v); }
    if (s.th > P.angleMax) { s.th = P.angleMax; s.om = Math.min(0, s.om); }
    if (s.th < -P.angleMax) { s.th = -P.angleMax; s.om = Math.max(0, s.om); }
    var c = paddleContact(s);
    if (c.closing > 0 && c.gap < 0) {
      var kick = (1 + P.restitution) * c.closing;
      s.vx += c.nx * kick; s.vy += c.ny * kick;
      s.x = c.px + c.nx * P.radius; s.y = c.py + c.ny * P.radius;
      bounced = true;
    }
  }
  return bounced;
}

// Time until a ball in free flight comes down to height y.
function fallTime(s, y)
{
  var disc = s.vy * s.vy + 2 * PADDLE.g * (s.y - y);
  return disc > 0 ? (s.vy + Math.sqrt(disc)) / PADDLE.g : 0;
}

// Height of the ball's centre when it rests on a level paddle.
function restHeight(length) { return PADDLE.baseY + length + PADDLE.thickness + PADDLE.radius; }
