function normalize(angle) {
    return (angle + Math.PI) % (2*Math.PI) - Math.PI;
  }

function controlPosition(rocket, targetX) {
  const err = rocket.x - targetX;
  const p = -err;
  const d = -10*rocket.dx;
  const targetAngle = 0.003*(p+d);
  return controlGimbal(rocket, targetAngle);
}

function controlGimbal(rocket, targetAngle) {
  const err = rocket.theta - targetAngle;
  const p = err;
  const d = rocket.dtheta;
  return p+d;
}

function controlThrottle(rocket, targetSpeed) {
  const err = rocket.dy-targetSpeed;
  const p = -err;
  return 0.5+p;
}

function controlFunction(rocket)
{
  
  let targetSpeed;
  if (rocket.y > 35) targetSpeed = -10;
  else targetSpeed = -4;
  
  const gimbalAngle = controlPosition(rocket, 0);
  const throttle = controlThrottle(rocket, targetSpeed);
  
  return {throttle,gimbalAngle};
}