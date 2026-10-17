# Controlled Waypoint Experiment

## Question

Does adding collinear waypoints change execution time when the planned
geometric route and route-following controller remain the same?

## Method

An E-puck followed the same 0.820-metre planned polyline around a static
box in Webots.

Two conditions were tested:

- Original waypoints: five segments.
- Extra midpoints: ten segments, created by inserting one midpoint
  into every original segment.

Both conditions used the same start, goal, controller parameters,
8-millimetre waypoint tolerance, and reset procedure. Each condition
was executed three times, with condition order shuffled within each
repetition using seed 42.

No motion noise was deliberately added. The controller used perfect
Supervisor position and orientation feedback.

Run ID: `2026-10-17T18:22:47.502789+00:00`

## Results

| Condition | Collision-free goals | Mean total time | Moving time | Turning time |
|---|---:|---:|---:|---:|
| Original waypoints | 3/3 | 83.616 s | 74.848 s | 8.736 s |
| Extra midpoints | 3/3 | 123.232 s | 114.624 s | 8.576 s |

Each trial also included approximately 0.032 seconds of idle time.
Times are simulation durations.

Adding midpoints increased total execution time by 39.616 seconds,
approximately 47.4%.

Moving-command time increased by 39.776 seconds, while turning-command
time decreased by 0.160 seconds. All three repetitions within each
condition had identical durations at the displayed precision.

## Interpretation

The controller reduces forward speed as it approaches its next
waypoint. Inserting collinear waypoints introduces additional
slowdown targets even though the planned geometric path is unchanged.

The result supports waypoint handling as a cause of execution-time
differences under this controller. It does not quantify how much of
the earlier difference between planners was caused by waypoint count,
because those planners also selected different route geometries.

## Limitations

This experiment tests one route, one obstacle placement, two waypoint
representations, and one controller configuration.

Identical planned geometry does not guarantee identical executed
trajectories. Waypoint tolerance and steering corrections can change
actual motion.

Repetitions check consistency under the reset procedure. They are not
independent environments or evidence of robustness to uncertainty.

Collision monitoring covers the named obstacle box.

## Data and controller

Archived trials: `data/webots_waypoint_test.csv`

Controller:
`webots/controllers/robobench_waypoint_test/robobench_waypoint_test.py`