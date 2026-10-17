# RoboBench: Extended Webots Results

Run: `2026-10-17T17:56:23.295831+00:00`

The experiment tested nine placements of one static box, three planners, and three repetitions per combination: 81 trials.

Repetitions used the same nominal starting pose and controller. No motion noise was deliberately added. They assess consistency and are not independent environment samples.

## Outcomes

| Planner | Goal reached | Box contact | Obstacle stop | Timeout | No route |
|---|---:|---:|---:|---:|---:|
| A-star | 27/27 | 0 | 0 | 0 | 0 |
| Dijkstra | 27/27 | 0 | 0 | 0 | 0 |
| BFS | 27/27 | 0 | 0 | 0 | 0 |

## Execution by layout

Each cell reports median execution duration and the full minimum-to-maximum range across three repetitions. Only layouts where every trial succeeded without detected box contact are included.

| Layout | A-star: median [min, max] s | Dijkstra: median [min, max] s | BFS: median [min, max] s |
|---|---:|---:|---:|
| Centre | 83.616 [83.616, 83.616] | 74.368 [74.368, 74.368] | 72.160 [72.160, 72.160] |
| Right | 82.720 [82.720, 82.720] | 73.152 [73.152, 73.152] | 72.480 [72.480, 72.480] |
| Left | 82.752 [82.752, 82.752] | 74.752 [74.752, 74.752] | 70.912 [70.912, 70.912] |
| Up | 80.672 [80.672, 80.672] | 71.424 [71.424, 71.424] | 69.216 [69.216, 69.216] |
| Down | 80.672 [80.672, 80.672] | 71.424 [71.424, 71.424] | 69.216 [69.216, 69.216] |
| Right-up | 79.776 [79.776, 79.808] | 70.240 [70.240, 70.240] | 69.536 [69.536, 69.536] |
| Right-down | 79.776 [79.776, 79.808] | 70.240 [70.240, 70.240] | 69.536 [69.536, 69.536] |
| Left-up | 79.808 [79.808, 79.808] | 71.808 [71.808, 71.808] | 68.000 [68.000, 68.000] |
| Left-down | 79.808 [79.808, 79.808] | 71.808 [71.808, 71.808] | 68.000 [68.000, 68.000] |

## Paired-layout summary

Common successful layouts: 9/9.

Each layout receives equal weight. The reported mean averages the three-repetition median from each common successful layout.

| Planner | Mean layout median (s) | Largest within-layout repeat range (s) |
|---|---:|---:|
| A-star | 81.067 | 0.032000 |
| Dijkstra | 72.135 | 0.000000 |
| BFS | 69.895 | 0.000000 |

## Interpretation and limitations

Planned distances matched across planners and repetitions on 9/9 layouts.

Execution times are simulation durations, not planning computation times or wall-clock durations.

A small repeat range indicates consistency under the tested reset procedure. It does not establish robustness to uncertainty.

The layouts differ only in the position of one box. The study does not represent diverse obstacle shapes, multiple obstacles, sensor uncertainty, or physical robot trials.

The controller uses perfect Supervisor pose feedback. Contact monitoring covers the named obstacle box. Proximity-triggered stops are recorded separately from physical box contacts.

Execution comparisons conditioned on common successful layouts must be read alongside the all-trial outcome counts.

No confidence intervals or significance tests are reported.

## Reproduce

Read archived trials and regenerate this report:

```powershell
.\.venv\Scripts\python.exe analyze_webots_extended.py
```

Input: `data/webots_extended_trials.csv`.

This report describes a separate experiment from the earlier nine-trial study; it does not overwrite those historical results.
