// Vehicle racing: steer and choose the speed, using only the five LIDAR distances.

var P = {gain: 2, power: 2, vCorner: 9, brake: 5.5, margin: 6, grip: 0.8};

function controlFunction(vehicle)
{
  var pts = vehicle.lidarPoints, sum = 0, aim = 0, i;
  for (i = 0; i < pts.length; i++) { var w = Math.pow(pts[i].distance, P.power); sum += w; aim += w * pts[i].direction; }
  var steering = P.gain * aim / sum;

  var v = vehicle.speed, wheelbase = vehicle.Lf + vehicle.Lr;
  // Speed from which the car can still slow to cornering speed in the free road ahead.
  var ahead = Math.max(0, pts[2].distance - P.margin);
  var vBrake = Math.sqrt(P.vCorner * P.vCorner + 2 * P.brake * ahead);
  // Speed at which the wanted steering angle is still within the lateral grip limit.
  var vGrip = Math.sqrt(P.grip * vehicle.lateralAccelerationLimit * wheelbase / Math.max(1e-3, Math.tan(Math.min(1.2, Math.abs(steering)))));
  var vWant = Math.min(vBrake, vGrip);
  var acceleration = Math.max(-vehicle.accelerationLimit, Math.min(vehicle.accelerationLimit, (vWant - v) / 0.1));
  return {steering: steering, acceleration: acceleration};
}
