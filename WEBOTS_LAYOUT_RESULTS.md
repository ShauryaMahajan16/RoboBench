# RoboBench: Three-layout Webots experiment

Run: `2026-10-12T11:20:51.460891+00:00`

Three planners were tested on three box placements, giving nine trials. Each planner used the same route-following controller. Obstacle footprints were calculated from the Webots box geometry.

## Individual trials

| Layout | Planner | Outcome | Box collision | Planned (m) | Travelled (m) | Segments | Execution (simulation s) |
|---|---|---|---|---:|---:|---:|---:|
| Centre | A-star | GOAL REACHED | False | 0.820 | 0.782 | 5 | 83.62 |
| Centre | Dijkstra | GOAL REACHED | False | 0.820 | 0.790 | 4 | 74.37 |
| Centre | BFS | GOAL REACHED | False | 0.820 | 0.790 | 4 | 72.16 |
| Right-up | A-star | GOAL REACHED | False | 0.740 | 0.703 | 5 | 79.81 |
| Right-up | Dijkstra | GOAL REACHED | False | 0.740 | 0.710 | 4 | 70.24 |
| Right-up | BFS | GOAL REACHED | False | 0.740 | 0.710 | 4 | 69.54 |
| Left-down | A-star | GOAL REACHED | False | 0.740 | 0.702 | 5 | 79.81 |
| Left-down | Dijkstra | GOAL REACHED | False | 0.740 | 0.710 | 4 | 71.81 |
| Left-down | BFS | GOAL REACHED | False | 0.740 | 0.710 | 4 | 68.00 |

## Comparison on common successful layouts

Means below include only layouts where all three planners reached the goal without a detected box collision.

| Planner | Goals reached / trials | Detected box collisions | Common layouts | Mean execution (simulation s) |
|---|---:|---:|---:|---:|
| A-star | 3/3 | 0 | 3 | 81.08 |
| Dijkstra | 3/3 | 0 | 3 | 72.14 |
| BFS | 3/3 | 0 | 3 | 69.90 |

## Interpretation and limitations

All three planners had equal planned distances on 3 of the three layouts.

Equal planned distance does not imply equal execution time. Planners can select different shortest routes, with different segment lengths and turn requirements. The controller slows near waypoints, so route structure can affect execution time.

Execution times measure simulated robot motion, not planning computation time or elapsed time on the computer.

The robot uses perfect position and orientation feedback from the Webots Supervisor. This experiment adds no motion noise. Collision monitoring covers the named box.

There is one trial per planner per layout. These results describe this small experiment and do not establish general performance rankings or reliability.

## Reproduce the analysis

Run the nine-layout experiment using the `robobench_layout_trials` controller, then run:

```powershell
.\.venv\Scripts\python.exe analyze_webots_layouts.py
```

This report summarizes the latest run in `results/webots_layout_trials.csv`.
