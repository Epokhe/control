var acc = 0;

function controlFunction(block)
{
  acc += 0.005*block.x;
  return -block.x-block.dx-acc;
}