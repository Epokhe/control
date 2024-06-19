
function controlHinge(hinge, angleGoal) {
  // control hinge angle
  const err = hinge.angle - angleGoal;
  
  const p = -10*err;
  const d = -5*hinge.speed;
  return p+d;
}

function controlPiston(piston, speedGoal) {
  const err = piston.speed - speedGoal;
  
  const p = -10*err;
  //console.log(p);
  return p;
}

let hitSpeed = 0.4;

function controlFunction(ball, piston, hinge, T)
{
  const err = ball.x;
  
  const p = 0.1*err;
  const d = 0.1*ball.vx;
  
  const angleGoal = p+d;
  
  var apogee = ball.y + ball.vy**2/(2*9.81);
  monitor('apogee', apogee);
  
  hitSpeed -= 0.0001 * apogee;
  
  let pistonAcceleration;
  if (ball.vy < 0 && ball.y < 1) {
    pistonAcceleration = controlPiston(piston, hitSpeed);
  }
  else {
    pistonAcceleration = controlPiston(piston, -10);
  }
  
  
  
  return {pistonAcceleration, hingeAcceleration: controlHinge(hinge, angleGoal)};
}

