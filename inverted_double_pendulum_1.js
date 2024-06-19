function normalize(angle) {
    return (angle + Math.PI) % (2*Math.PI) - Math.PI;
  }
  
  
  function controlAngleLower(pendulum, targetAngle) {
    const err = normalize(pendulum.theta1) - targetAngle;
    const p = 200*err;
    const d = 100*pendulum.dtheta1
      
    const force = p+d;
    return force;
  }
  
  
  // i think we will control the upper one with the lower one
  function controlAngleUpper(pendulum, targetAngle) {
    const err = normalize(pendulum.theta2) - targetAngle;
    const p = 1*err;
    const d = 1*pendulum.dtheta2;
      
    const targetAngleLower = p+d;
    return controlAngleLower(pendulum, targetAngleLower);
  }
  
  
  function controlPosition(pendulum, targetX) {
    const err = pendulum.x - targetX;
    const p = -0.005*err;
  
    const d = -0.05*pendulum.dx;
    const targetAngle = p+d;
  
    const force = controlAngleUpper(pendulum, targetAngle);
    return force;
  }
  
  
  function controlFunction(pendulum)
  {
    const force = controlPosition(pendulum, 0);
    monitor('force', force);
    return force;
  }
  

  