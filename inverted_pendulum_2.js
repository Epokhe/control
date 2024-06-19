function normalize(angle) {
    return (angle + Math.PI) % (2*Math.PI) - Math.PI;
  }
  
  function controlAngle(pendulum, angleGoal) {
    console.log(
      pendulum.theta, 
      normalize(pendulum.theta)
    );
  
    const err = normalize(pendulum.theta) - angleGoal;
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
    // shame
    if (pendulum.T < 0.5) return 50;
    if (pendulum.T < 1.5) return -50;
    if (pendulum.T < 2.5) return 50;
    
    const force = controlPosition(pendulum, 0);
  
    
    
    return force;
  }
  