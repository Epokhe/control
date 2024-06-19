function normalize(angle) {
    return (angle + Math.PI) % (2*Math.PI) - Math.PI;
  }



function controlGimbal(rocket, targetAngle) {
  const err = rocket.theta - targetAngle;
  const err2 = rocket.dx;
  const p = err + 0.01*err2;
  const d = 2*rocket.dtheta;
  return p+d;
}

function controlPosition(rocket, targetX) {
  const err = rocket.x - targetX;
  const p = -err;
  const d = -10*rocket.dx;
  const targetAngle = 0.003*(p+d);
  return controlGimbal(rocket, targetAngle);
}


function controlThrottle(rocket, targetSpeed) {
  if (rocket.theta > Math.PI/2 || rocket.theta < -Math.PI/2) return 1;
  const err = rocket.dy-targetSpeed;
  const p = -err;
  return 0.5+p;
}

var state = 0;

function controlFunction(rocket)
{
  
  // change state
  if (rocket.dy > -0.1 && Math.abs(rocket.x) < 20 && Math.abs(rocket.dx) < 5 &&
     Math.abs(rocket.theta) < Math.PI/4 && Math.abs(rocket.dtheta) < 0.2) {
    state = 1;
  }
  
  if (state == 0) {
    // first case
        console.log('first');
    return {
      gimbalAngle: controlPosition(rocket, 0), // 0 target angle
      throttle: controlThrottle(rocket, 0), // 0 target speed
    }

  } else {
    let targetSpeed;
    if (rocket.y > 50) targetSpeed = -20;
    else targetSpeed = -4;
    return {
      gimbalAngle: controlPosition(rocket, 0), // 0 target angle
      throttle: controlThrottle(rocket, targetSpeed), // 0 target speed
    }
  }
}