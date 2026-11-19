// Ball on platform, balance: catch the falling ball and bring it to rest in
// the middle of the paddle. All of the work is in "Catching the ball" below.

//#include lib/paddle.js

//#include lib/aim.js

//#include lib/catch.js

var catcher = makeCatcher({x: 0});

function controlFunction(ball, piston, hinge, T)
{
  var u = catcher(ball, piston, hinge);
  return {pistonAcceleration: u.pistonAcceleration, hingeAcceleration: u.hingeAcceleration};
}
