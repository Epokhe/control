var acc = 0;

function controlFunction(vehicle){ 
  const err = vehicle.targetSpeed-vehicle.speed;
  
  const p = 10*err;
  return p;
};