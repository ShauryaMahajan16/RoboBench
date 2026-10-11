# Motion Uncertainty and Planning Clearance

## Questions

1. How does movement error affect execution of a planned route?
2. Does extra planning clearance improve success under that error?

## Motion model

The robot moves towards successive route waypoints.

Each movement command receives independent Gaussian displacement
error on the x and y axes. Noise sigma is measured in pixels per
axis per movement step.

The controller knows the robot's actual position perfectly and
corrects towards its next waypoint.

- Physical robot radius: 12 pixels.
- Commanded movement: up to 2 pixels per step.
- Waypoint tolerance: 1 pixel.
- Step limit: 10,000.
- Touching an obstacle counts as collision.
- Collision checks cover the entire movement segment.

This is a simplified holonomic movement model, not wheel physics
or sensor uncertainty.

## Experiment 1: Movement error

A-star planned routes on generated map seeds 0–19 using a 20-pixel
grid. Nineteen maps had routes; seed 10 was excluded.

Each noise level used the same 19 maps and 20 motion seeds per map.

| Noise sigma | Successes / 380 | Collisions | Mean distance among successes |
|---|---:|---:|---:|
| 0 px | 380 | 0 | 650.5 px |
| 0.25 px | 380 | 0 | 655.8 px |
| 0.5 px | 380 | 0 | 682.4 px |
| 1 px | 380 | 0 | 853.4 px |
| 2 px | 193 | 187 | 1952.3 px |

No timeouts occurred.

At sigma 1, every trial succeeded, but mean travelled distance
increased by approximately 31% over zero noise.

At sigma 2, success fell to 50.8%. Its distance average includes
only successful trials and is not a comparison across all outcomes.

## Experiment 2: Extra planning clearance

Movement noise was fixed at sigma 2. The physical radius stayed
12 pixels, while planning used radius 12 plus extra clearance.

Planning clearance also applied to map boundaries.

All 20 maps were included, with 20 trials per map per setting.
Unavailable routes were recorded separately.

| Extra clearance | Maps with routes | Successes / 400 | Collisions | No-route cases |
|---|---:|---:|---:|---:|
| 0 px | 19 | 193 | 187 | 20 |
| 4 px | 19 | 193 | 187 | 20 |
| 8 px | 14 | 256 | 24 | 120 |
| 12 px | 12 | 240 | 0 | 160 |

No timeouts occurred.

Eight pixels gave the highest overall success among tested settings:
64.0%. Twelve pixels eliminated observed collisions but rejected
more routes, reducing overall success to 60.0%.

## Matched-map comparison

Twelve maps had routes at every clearance setting:

1, 2, 4, 8, 9, 11, 13, 14, 15, 16, 17, 18.

| Extra clearance | Successes / 240 | Mean planned distance |
|---|---:|---:|
| 0 px | 124 | 650.0 px |
| 4 px | 124 | 650.0 px |
| 8 px | 240 | 713.3 px |
| 12 px | 240 | 713.3 px |

On these same maps, extra clearance improved observed success,
while mean planned distance increased by approximately 9.7%.

## Interpretation

A collision-free planned route does not guarantee collision-free
execution when movement is uncertain.

Extra planning clearance can improve execution reliability, but
may increase route distance or eliminate otherwise available routes.

The best tested clearance depends on the objective: avoiding
collisions alone differs from maximizing successful completion
across every map.

## Limitations

- Perfect position feedback is assumed.
- Noise is independent Gaussian displacement error.
- There is no heading, wheel dynamics, sensor noise, or replanning.
- The map set is small and contains static rectangular obstacles.
- Trials sharing a map are not independent environment samples.
- Observed 100% success does not guarantee future reliability.
- The 20-pixel grid can make small clearance changes leave routes unchanged.
- Conservative rectangular safety margins affect available routes.

## Reproduce

Run from the RoboBench folder:

.\.venv\Scripts\python.exe test_motion.py

.\.venv\Scripts\python.exe benchmark_motion.py

.\.venv\Scripts\python.exe benchmark_clearance.py

To inspect execution visually:

.\.venv\Scripts\python.exe noise_gui.py