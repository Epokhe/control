var acc = 0;

function controlFunction(vehicle){ 
  const err = vehicle.targetSpeed-vehicle.speed;
  
  const p = 2*err;
  return p;
};