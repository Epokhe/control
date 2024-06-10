
function controlHinge(hinge, goal) {
    // control hinge angle
    const err = hinge.angle - goal;
    
    const p = -10*err;
    const d = -5*hinge.speed;
    return p+d;
  }
  
  function controlFunction(ball, piston, hinge, T)
  {
    const err = ball.x;
    
    const p = 0.1*err;
    const d = 0.1*ball.vx;
    
    const angleGoal = p+d;
    
    return {pistonAcceleration: 0, hingeAcceleration: controlHinge(hinge, angleGoal)};
  }