# Initial Webots 3D Results

## Objective

Connect RoboBench's grid planners to a simulated wheeled robot
and investigate how planned route structure affects execution.

## Setup

- Simulator: Webots R2025a.
- Robot: E-puck with differential wheel control.
- Position feedback: exact simulated pose from the Supervisor API.
- Start: (-0.25, 0) metres, heading 0 degrees.
- Goal: (0.25, 0) metres.
- Obstacle: unrotated 0.2 × 0.2 × 0.2 metre WoodenBox,
  centred at (0, 0, 0.1).
- Planner scale: 1 pixel represents 1 mm.
- Grid spacing: 20 pixels, or 20 mm.
- Conservative planning radius: 40 mm.
- Waypoint tolerance: 8 mm.
- Intermediate collinear grid points are removed before execution.

The physical robot model is unchanged by the planning radius.

The controller turns in place when heading error exceeds 0.15 radians.
Otherwise, it drives towards the next waypoint, reducing forward
speed as waypoint distance decreases.

The current planner obstacle coordinates are entered explicitly
and checked against the box's world fields.

## Physical contact validation

Box collisions are detected by matching contact-point locations
reported for the robot, including descendant bodies, and the named box.

A separate slow approach test deliberately made contact:

- Normal robot floor contacts produced no shared box contacts.
- Box contact produced two shared contact points.
- The controller stopped both wheels and reported PASS.

The contact detector currently monitors the named box, not every
object in the world.

## Three-planner execution comparison

Each planner ran once from a reset starting pose and physical motion.

| Planner | Planned distance | Travelled distance | Execution time | Box contact |
|---|---:|---:|---:|---|
| A-star | 0.820 m | 0.782 m | 83.62 s | Not detected |
| Dijkstra | 0.820 m | 0.790 m | 74.37 s | Not detected |
| BFS | 0.820 m | 0.790 m | 72.16 s | Not detected |

All three reached the goal.

The planners returned equal-length grid paths but selected different
routes among the available shortest paths.

Travelled distance is not directly equivalent to planned distance.
Waypoint tolerance allows early turns and stopping short of exact
waypoint centres.

## Execution breakdown

Times describe controller command states, not isolated physical
translation and rotation.

| Planner | Segments | Turn episodes | Turn commands | Move commands | Total |
|---|---:|---:|---:|---:|---:|
| A-star | 5 | 4 | 8.74 s | 74.85 s | 83.62 s |
| Dijkstra | 4 | 4 | 8.80 s | 65.54 s | 74.37 s |
| BFS | 4 | 3 | 6.59 s | 65.54 s | 72.16 s |

Approximately 0.03 seconds of initial idle time is included in
each total. Displayed values are rounded.

A-star's additional execution time occurred mainly during move
commands. This is consistent with the controller slowing near an
additional waypoint.

BFS and Dijkstra had equal displayed move-command time. Their
difference occurred mainly during turn commands.

## Same-route control

One A-star route was calculated once and executed three times with
the same controller, resetting robot pose and physical motion
between trials.

All three reported matching values at the displayed precision:

- Goal reached.
- No detected box contact.
- Travelled distance: 0.782 m.
- Turn-command time: 8.74 s.
- Move-command time: 74.85 s.
- Total execution time: 83.62 s.

This supports repeatability for this route and setup. It does not
establish repeatability across all environments or initial conditions.

## Interpretation

Equal-length grid paths can have different execution times because
route turns, waypoint placement, and the tracking controller affect
robot motion.

These experiments demonstrate successful 3D integration and suggest
that route structure contributed to the observed timing differences.

They do not establish a universally fastest planner.

Planning computation time and physical execution time are separate
measurements and should not be combined without explanation.

## Limitations

- One static obstacle layout.
- One execution trial per planner in the initial comparison.
- Three repetitions of one route in the control test.
- Perfect simulated position feedback.
- No added localisation or actuator noise in these 3D trials.
- No dynamic obstacles or online replanning.
- Fixed algorithm order.
- Robot resets do not reset the entire simulator random state.
- Waypoint simplification and tolerance affect executed trajectories.
- The collision monitor covers the named box only.

## Reproduce in Webots

Open webots/worlds/robobench_3d.wbt.

Ensure E-puck has supervisor TRUE and the WoodenBox has
DEF ROBOBENCH_OBSTACLE.

For the instrumented three-planner comparison, select controller:

robobench_compare_3d

For the repeated identical-route control, select:

robobench_same_route

Save the world after selecting the controller, reload it, and run
the simulation. These controllers reset the robot pose automatically.

Results append to:

- results/webots_execution_breakdown.csv
- results/webots_same_route.csv

Generated CSV files are excluded from Git.