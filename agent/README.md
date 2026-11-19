# All 21 levels, solved from scratch by an agent

A second set of solutions for [ControlChallenges](https://github.com/janismac/ControlChallenges)
(game commit `027b7d0`, the same one the solutions in the repository root were written for).
They were written from scratch, without looking at the earlier solutions or at the game's sample
solutions, and with a few house rules:

- every controller is feedback on the measured state; there are no timed input schedules,
- nothing relies on a quirk of the simulation,
- one controller per kind of machine: the four rocket levels share one file, the pendulum pairs
  share one each, the three tutorials share one,
- every level is also tested from randomly disturbed starting states, not only the one the game uses.

All 21 levels are solved. The time shown is the game's own clock when it declares the level complete.

## Head to head

"Barış" is the solution in the repository root, "sample" is the solution that ships with the game.
Watch any of them side by side in [`videos/`](videos) (`all-levels.mp4` plays them back to back).

| Level | Barış | Fable | sample | |
|---|---:|---:|---:|---|
| Tutorial: block with friction | 10.08 | **0.46** | 10.98 | 22x faster |
| Tutorial: block without friction | 10.02 | **0.46** | 7.50 | 22x faster |
| Tutorial: block on slope | 15.00 | **0.46** | 17.42 | 33x faster |
| Cruise control intro | 15.54 | 15.54 | 15.54 | tie: the level only ends at 15.5 s |
| Cruise control 2 | 15.30 | 15.30 | 15.30 | tie, same reason |
| Stabilise single pendulum | 5.92 | **2.72** | 3.90 | 2.2x faster |
| Swing up single pendulum | 11.32 | **3.64** | 14.78 | 3.1x faster |
| Stabilise double pendulum | 33.10 | **4.26** | 8.14 | 7.8x faster |
| Swing up double pendulum | 86.38 | **4.26** | 13.74 | 20x faster |
| Rocket landing | 17.80 | **10.42** | 18.58 | 1.7x faster |
| Rocket landing, upside down | 27.64 | **11.22** | 21.42 | 2.5x faster |
| Rocket landing, three rockets | 31.56 | **12.50** | 25.72 | 2.5x faster |
| Rocket landing, hoverslam | not solved | **9.62** | 12.62 | |
| Ball on platform: balance | 20.00 | **2.02** | 4.48 | 9.9x faster |
| Ball on platform: bounce | 112.88 | **6.18** | 17.42 | 18x faster |
| Ball on platform: edge balance | not solved | **6.24** | 14.50 | |
| Multirotor intro | not solved | **1.26** | 7.02 | |
| Multirotor obstacles | not solved | **1.88** | 8.92 | |
| Multirotor flip | not solved | **1.78** | 8.30 | |
| Vehicle steering | not solved | 24.64 | 24.64 | fixed speed, so the lap takes what it takes |
| Vehicle racing | not solved | **35.82** | 39.90 | |

Fourteen levels had a solution from Barış: twelve are faster now, by a factor of 2 to 33, and the
two cruise-control levels are a tie that nobody can break. The other seven are new.

## Starting somewhere else

`harness/robustness.js` starts each level from randomly disturbed states (the same twelve for every
solution) and counts how often the level is still completed. The size of the disturbance is in the
table at the top of that file, for example a rocket up to 30 m and 8 m/s away from its usual start
and tilted by up to 0.2 rad, or a ball up to 15 cm to the side.

| Level | solved | median s | worst s |
|---|---:|---:|---:|
| BallOnPlatformBalance | 12/12 | 2.02 | 2.92 |
| BallOnPlatformBounce | 12/12 | 6.22 | 6.28 |
| BallOnPlatformEdgeBalance | 12/12 | 6.48 | 6.94 |
| CruiseControl2 | 12/12 | 15.52 | 26.66 |
| CruiseControlIntro | 12/12 | 15.82 | 17.04 |
| MultirotorFlip | 12/12 | 1.86 | 2.38 |
| MultirotorIntro | 12/12 | 1.26 | 1.42 |
| MultirotorObstacles | 12/12 | 1.90 | 2.56 |
| RocketLandingHoverslam | 12/12 | 9.72 | 10.22 |
| RocketLandingMulti | 12/12 | 12.64 | 14.42 |
| RocketLandingNormal | 12/12 | 10.12 | 16.28 |
| RocketLandingUpsideDown | 12/12 | 11.82 | 16.82 |
| StabilizeDoublePendulum | 12/12 | 4.26 | 20.92 |
| StabilizeSinglePendulum | 12/12 | 2.56 | 2.88 |
| SwingUpDoublePendulum | 12/12 | 7.24 | 9.98 |
| SwingUpSinglePendulum | 12/12 | 3.60 | 4.06 |
| TutorialBlockOnSlope | 12/12 | 0.48 | 0.54 |
| TutorialBlockWithFriction | 12/12 | 0.46 | 0.54 |
| TutorialBlockWithoutFriction | 12/12 | 0.48 | 0.54 |
| VehicleRacing | 12/12 | 35.82 | 36.02 |
| VehicleSteeringSimple | 12/12 | 24.64 | 24.66 |

That is 252 of 252 runs. The two slow outliers are worth knowing about: one disturbed start of the stabilise-double-pendulum level takes 20.9 s where the others take about 4
(the planner settles for a much longer manoeuvre there), and cruise control simply takes longer when the car starts 10 m/s off the target speed.
For comparison, with the same starts the game's sample solutions land 6 of 12 on the three-rocket
level, 8 of 12 on the hoverslam and 10 of 12 on the multirotor flip.

## How each one works

**Tutorials.** The block is treated as `x'' = F + w` with an unknown push `w` (friction, slope) that
is estimated from how the speed changed over the last step. Full force towards the target, then full
force against the motion, timed by the braking curve; in the last centimetres that curve turns into
the two-step deadbeat law, so the block stops exactly on the target. The controller never reads the
friction or slope parameters.

**Cruise control.** Model inversion (the throttle that cancels drag and slope) plus a one-step
correction of the speed error. The level cannot be completed before 15.3 s, whatever the controller.

**Pendulums, rockets, multirotor.** One small trajectory optimiser (`src/lib/mpc.js`, iterative LQR
with an augmented-Lagrangian terminal constraint, about 450 lines) does the heavy lifting. For each
machine it searches for the shortest plan that reaches the goal from the measured state, follows the
plan with the time-varying feedback gains that fall out of the optimiser, plans again if the state is
pushed off the plan, and holds with LQR at the end. What differs per machine is the equations of
motion and where the first guess comes from:

- *Double pendulum swing-up*: the system has no friction, so its motion is time-reversible. Letting
  the pendulum fall from upright into the current state is easy; played backwards it is a swing-up.
- *Rocket*: a conventional cascaded landing law (also the fallback when there is no plan) gives the
  guess. The plan may not touch the ground before its end, and must end in a clean landing. The same
  file lands all four levels; it reads the minimum throttle from the model, which is all that makes
  the hoverslam level different.
- *Multirotor*: a hover regulator flown through the open side of each wall gives the guess; the walls
  are penalties in the plan and a plan that touches one is rejected. The flip is found by the
  optimiser on its own from a guess that just unwinds the angle.

**Ball on platform.** The piston is hardly quicker than gravity and every bounce keeps 90 % of the
speed, so a ball cannot simply be caught. Everything is built from *aimed bounces* (`src/lib/aim.js`):
the paddle is steered to meet the ball at a chosen height, speed and tilt, and those are solved for
with Newton's method on an exact copy of the game's bounce, so that the ball comes off the paddle
with the wanted velocity.

- *Bounce*: one low hop that comes down over the centre, then straight up with the top of the flight
  at the origin. The paddle accelerates at contact by an amount that makes the height insensitive to
  the bounce happening a little early or late.
- *Balance*: three aimed bounces take the energy out (hop to the spot, pop with a tilted paddle to
  remove the sideways speed, drop off a paddle that is moving away), then the paddle follows the last
  small bounces and comes to rest under the ball.
- *Edge balance*: catch the ball as above, plan a slide along the tilted bar that never lets the ball
  lift off, and balance it on the rounded end with a gain-scheduled LQR while the piston cancels the
  end's up-and-down motion.

**Vehicles.** Only the five LIDAR distances are used. Steering aims at the average LIDAR direction, weighted by the square of
each distance, so the car heads for open road. The racing level adds a speed: the highest from which
it can still brake to cornering speed within the free distance ahead, capped by what the tyres' grip
allows for the current steering angle.

## Honest notes

- The planners think before they act. In the game this shows as a pause when the level starts
  (it does not count on the game clock): around a second for pendulums and multirotor, 4 s for a
  rocket, 15 s for the level with three rockets, measured in Node on a cloud machine.
- A plan is only as good as the optimiser's luck. From disturbed starts the rocket planner sometimes
  settles for a slower landing (worst case in the table above), and falls back to the conventional
  landing law if it finds nothing.
- The speeds are near what the actuators allow but are not proven optimal.

## Running it

```
agent/harness/fetch-game.sh            # clone the game at the right commit (needs git)
node agent/harness/build.js            # src/ -> solutions/ (one self-contained file per level)
node agent/harness/run.js              # every level, with times
node agent/harness/robustness.js "" 12 new,old,sample
node agent/harness/video.js            # side-by-side videos (needs Playwright and ffmpeg)
```

To try one in the game, paste the file from `solutions/` into the level's editor.

- `src/` is the code to read; `src/lib/` holds what several levels share.
- `solutions/` is generated from `src/` by `build.js`.
- `harness/` loads the game's own model and level files in Node, so the physics and the completion
  checks are the game's, not a copy.
