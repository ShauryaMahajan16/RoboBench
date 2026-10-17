# Multi-Route Waypoint Cleanup Experiment

## Question

Does the observed benefit of removing redundant collinear waypoints
hold across three tested route polylines?

## Method

Three routes around the same centred box were tested:

- Bottom: five original segments.
- Top: the mirrored Bottom route, with five original segments.
- Early bottom turn: four original segments.

Every route had a planned length of 0.820 metres.

Four conditions were tested per route:

1. Original waypoints, unchanged.
2. Added midpoints, unchanged.
3. Original waypoints, with collinear-point cleanup.
4. Added midpoints, with collinear-point cleanup.

Each combination was repeated three times, giving 36 trials.
Route/repetition blocks and condition order within each block were
shuffled using seed 42.

The controller, nominal starting pose, reset procedure, and 0.008-metre
waypoint tolerance were held fixed. No motion noise was deliberately
added. Pose feedback came from the Webots Supervisor.

Run ID: `2026-10-17T19:40:44.225188+00:00`

## Results

All 36 trials reached the goal without detected contact with the named
box. Maximum reported goal errors were below 0.008 metres.

Mean execution durations in simulation seconds:

| Route | Original unchanged | Midpoints unchanged | Original cleanup | Midpoints cleanup |
|---|---:|---:|---:|---:|
| Bottom | 83.616 | 123.232 | 83.616 | 83.616 |
| Top | 83.616 | 123.232 | 83.616 | 83.616 |
| Early bottom turn | 74.368 | 107.584 | 74.368 | 74.368 |

Relative to each unchanged midpoint condition:

- Bottom cleanup reduced execution time by 39.616 seconds,
  approximately 32.1%.
- Top cleanup reduced execution time by 39.616 seconds,
  approximately 32.1%.
- Early bottom turn cleanup reduced execution time by 33.216 seconds,
  approximately 30.9%.

Cleanup preserved each original route. The cleaned midpoint route
recovered the original waypoint list and matched its execution duration
at the displayed precision.

## Interpretation

Adding midpoints introduced extra slowdown targets along straight
segments. Removing those points eliminated the added delay in all
three tested polylines.

The Early bottom turn route remained faster than Bottom and Top despite
equal planned length. Collinear-point cleanup does not remove differences
in actual corners, segment lengths, or controller behaviour.

This is a demonstration of route-representation sensitivity and a
standard preprocessing remedy, not a new planning algorithm.

## Limitations

All routes use one box placement, one start and goal, and one controller
configuration. Top and Bottom are mirror images, so the route set has
limited geometric diversity.

Repetitions check consistency under the reset procedure. They are not
independent environments or robustness trials.

The controller uses perfect pose feedback and no deliberately added
motion noise. Contact monitoring covers the named box.

Small goal-error differences do not establish an accuracy improvement.
The findings should not be generalized to other controllers without
further testing.

## Data and controller

Archived trials: `data/webots_waypoint_routes.csv`

Controller:
`webots/controllers/robobench_waypoint_routes/robobench_waypoint_routes.py`