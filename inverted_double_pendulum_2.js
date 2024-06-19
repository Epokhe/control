function normalize(angle) {
    return (angle + Math.PI) % (2*Math.PI) - Math.PI;
  }
  
  
  function controlAngleLower(pendulum, targetAngle) {
    const err = normalize(pendulum.theta1) - targetAngle;
    const p = 300*err;
    const d = 150*pendulum.dtheta1;
          
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
  
    const d = -0.075*pendulum.dx;
    const targetAngle = p+d;
  
    const force = controlAngleUpper(pendulum, targetAngle);
    return force;
  }
  
  
  function controlFunction(pendulum)
  {
    // copied swing up part from the sample solution
  var s = [[0.295, 49.5], [0.81, -49.5], [0.7, 0.0], [0.52, 50.0], [1.2, 0.0]];
  while(s.length != 0)
  {
    var [dt,F] = s.shift();
    pendulum.T -= dt;
    if(pendulum.T <= 0) return F;
  }


    const force = controlPosition(pendulum, 0);
    monitor('force', force);
    return force;
  }
  

  