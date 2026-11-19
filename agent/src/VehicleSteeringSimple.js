// Vehicle steering at constant speed, using only the five LIDAR distances.

var GAIN = 2, POWER = 2;

function controlFunction(vehicle)
{
  var pts = vehicle.lidarPoints, sum = 0, aim = 0, i;
  for (i = 0; i < pts.length; i++) { var w = Math.pow(pts[i].distance, POWER); sum += w; aim += w * pts[i].direction; }
  return GAIN * aim / sum;
}
