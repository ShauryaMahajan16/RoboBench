# Controlled Waypoint Cleanup Experiment

## Question

Can removing redundant collinear waypoints eliminate the execution
delay caused by inserting midpoints into the same planned route?

## Method

Four conditions were tested on the same 0.820-metre planned polyline:

1. Original route, unchanged.
2. Route with extra midpoints, unchanged.
3. Original route, with collinear-point cleanup.
4. Route with extra midpoints, with collinear-point cleanup.

Each condition was executed three times in Webots. Condition order
was shuffled within each repetition using seed 42.

Cleanup retained corners and endpoints. In this experiment, it
converted the ten-segment midpoint route back into the original
five-segment route. It did not change the planned geometric polyline.

All conditions used the same movement controller, nominal starting
pose, reset procedure, and 0.008-metre waypoint tolerance. No motion
noise was deliberately added. Pose feedback came from the Supervisor.

Run ID: `2026-10-17T19:20:04.814044+00:00`

## Results

| Condition | Collision-free goals | Mean total | Moving | Turning | Maximum goal error |
|---|---:|---:|---:|---:|---:|
| Original / unchanged | 3/3 | 83.616 s | 74.848 s | 8.736 s | 0.007963 m |
| Midpoints / unchanged | 3/3 | 123.232 s | 114.624 s | 8.576 s | 0.007970 m |
| Original / cleanup | 3/3 | 83.616 s | 74.848 s | 8.736 s | 0.007962 m |
| Midpoints / cleanup | 3/3 | 83.616 s | 74.848 s | 8.736 s | 0.007963 m |

Times are simulation durations. Each trial also included approximately
0.032 seconds of idle time.

On the midpoint route, cleanup reduced execution time by 39.616 seconds,
approximately 32.1% relative to the unchanged midpoint condition.

Cleanup left the original route unchanged and produced the same
execution duration at the displayed precision.

## Interpretation

This controller slows as it approaches each waypoint. Removing
redundant points avoids additional slowdown targets along straight
segments.

The result demonstrates sensitivity to waypoint representation and
the benefit of a simple preprocessing step in this setup. It is not
a new path-planning algorithm or a general controller improvement.

The planner-comparison experiments already removed collinear grid
points. Therefore, this cleanup does not explain or eliminate all
execution-time differences between A-star, Dijkstra, and BFS.

## Limitations

The experiment uses one geometric route, one box placement, one
controller configuration, and three repetitions per condition.

Repetitions assess consistency under the reset procedure. They are
not independent environments or tests of robustness to uncertainty.

Contact monitoring covers the named box. Small differences in goal
error do not establish an accuracy improvement.

## Data and reproduction

Archived data: `data/webots_waypoint_cleanup.csv`

Controller:
`webots/controllers/robobench_waypoint_test/robobench_waypoint_test.py`

The current controller runs this 12-trial cleanup experiment.
The earlier six-trial waypoint experiment remains documented in
WAYPOINT_RESULTS.md, with its data archived separately.
Its original controller version is available in Git history.