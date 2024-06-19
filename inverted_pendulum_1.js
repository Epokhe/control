function controlAngle(pendulum, angleGoal) {
    const err = pendulum.theta - angleGoal;
    const p = 200*err;
    const d = 50*pendulum.dtheta
      
    const force = p+d;
    return force;
  }
  
  
  function controlPosition(pendulum, targetX) {
    const err = pendulum.x - targetX;
    const p = -0.02*err;
    
    const d = -0.05*pendulum.dx;
    const targetAngle = p+d;
    
    const force = controlAngle(pendulum, targetAngle);
    return force;
  }
  
  function controlFunction(pendulum)
  {
    const force = controlPosition(pendulum, 0);
    monitor('force', force);
    return force;
  }
  