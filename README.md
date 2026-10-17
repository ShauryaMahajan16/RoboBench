# RoboBench

A Python project for comparing A-star, Dijkstra, and breadth-first search
and studying how planning choices affect simulated robot navigation.

RoboBench includes:

- An interactive 2D map editor and animated navigation simulator.
- Reproducible planning, grid-resolution, and motion-noise experiments.
- Webots E-puck execution experiments.
- Automated correctness checks.
- Archived experiment data, plotting scripts, and a working paper.

This project evaluates established algorithms. It does not currently
claim a new planning algorithm or real-world robot validation.

## Quick start: Windows

Clone the repository and create a Python environment:

```powershell
git clone https://github.com/ShauryaMahajan16/RoboBench.git
cd RoboBench
python -m venv .venv
```

Launch the latest 2D interface:

```powershell
.\.venv\Scripts\python.exe noise_gui.py
```

The interface supports map selection and editing, planner comparisons,
route animation, movement noise, and extra planning clearance.

Alternatively, double-click `Launch_RoboBench.bat` after creating
the environment.

## Requirements

The core planners, checks, and 2D experiments use Python's standard
library. Visual tools require Tkinter.

Matplotlib is required only for plotting:

```powershell
.\.venv\Scripts\python.exe -m pip install matplotlib
```

The recorded plotting environment used Python 3.14.7 and
Matplotlib 3.11.2. See [REPRODUCING.md](REPRODUCING.md).

Webots is required to run the 3D controllers. It is not required to
analyze archived Webots data or regenerate figures.

GitHub Actions runs the configured checks on Python 3.10, 3.12,
and 3.14. This does not establish compatibility of every GUI,
plotting script, or Webots controller with those versions.

## Paper and reports

- [Working paper](PAPER.md)
- [Research background](RELATED_WORK.md)
- [Grid-resolution results](RESULTS.md)
- [Resolution-retry results](ADAPTIVE_RESULTS.md)
- [Motion uncertainty and clearance results](MOTION_RESULTS.md)
- [Initial Webots layout study](WEBOTS_LAYOUT_RESULTS.md)
- [Extended 81-trial Webots study](WEBOTS_EXTENDED_RESULTS.md)
- [Controlled waypoint experiment](WAYPOINT_RESULTS.md)
- [Figure reproduction guide](REPRODUCING.md)

The paper is a working draft, not a peer-reviewed publication.

## Regenerate figures from archived data

Install Matplotlib, then run:

```powershell
.\.venv\Scripts\python.exe plot_archived_results.py
.\.venv\Scripts\python.exe plot_waypoint_results.py
```

These commands read archived files in `data/` and write PNG and PDF
figures into `figures/`.

The first command generates Figures 1–5. The second generates Figure 6,
the controlled waypoint comparison.

Existing figure files with the same names are overwritten.

## Analyze archived Webots trials

For the extended experiment:

```powershell
.\.venv\Scripts\python.exe analyze_webots_extended.py
```

This reads `data/webots_extended_trials.csv` and regenerates
`WEBOTS_EXTENDED_RESULTS.md`.

The older command below analyzes fresh local nine-trial results:

```powershell
.\.venv\Scripts\python.exe analyze_webots_layouts.py
```

It requires `results/webots_layout_trials.csv`, which is not included
in a fresh clone. It describes the initial nine-trial study rather
than the extended experiment.

## Correctness checks

Run from the project root:

```powershell
.\.venv\Scripts\python.exe test_planner.py
.\.venv\Scripts\python.exe scenarios.py
.\.venv\Scripts\python.exe test_maps.py
.\.venv\Scripts\python.exe test_bfs.py
.\.venv\Scripts\python.exe test_adaptive.py
.\.venv\Scripts\python.exe test_motion.py
.\.venv\Scripts\python.exe test_world_geometry.py
```

These cover input handling, endpoints, designed passages, generated
maps, planner agreement, resolution retry, swept movement collisions,
and conversion of box geometry into planner coordinates.

Agreement between planners is a consistency check, not proof that
every implementation or modelling choice is correct.

## Run fresh 2D experiments

### Fixed-map benchmark

```powershell
.\.venv\Scripts\python.exe benchmark.py
```

Compares three planners on three fixed maps with repeated timing runs.

### Generated-map search effort

```powershell
.\.venv\Scripts\python.exe exploration.py
```

### Generated-map timing and paired comparison

```powershell
.\.venv\Scripts\python.exe benchmark_maps.py
.\.venv\Scripts\python.exe compare_results.py
```

### Grid resolution

```powershell
.\.venv\Scripts\python.exe resolution_experiment.py
.\.venv\Scripts\python.exe analyze_resolution.py
```

Optional visual comparison:

```powershell
.\.venv\Scripts\python.exe view_resolution.py
```

### Offset passages and resolution retry

```powershell
.\.venv\Scripts\python.exe offset_passages.py
.\.venv\Scripts\python.exe benchmark_adaptive.py
```

### Movement uncertainty and planning clearance

```powershell
.\.venv\Scripts\python.exe benchmark_motion.py
.\.venv\Scripts\python.exe benchmark_clearance.py
```

Fresh experiment outputs are written to `results/`. They do not
automatically replace archived files in `data/`.

## Webots experiments

World:

`webots/worlds/robobench_3d.wbt`

Controllers:

| Controller | Purpose |
|---|---|
| `robobench_layout_trials` | Nine box placements, three planners, three repetitions: 81 trials |
| `robobench_waypoint_test` | Same planned polyline with original waypoints or extra midpoints: six trials |

To run an experiment:

1. Open the world in Webots and pause the simulation.
2. Select the E-puck and set its controller to the desired name.
3. Confirm `supervisor` is enabled.
4. Confirm the box DEF is `ROBOBENCH_OBSTACLE`.
5. Confirm box size is `0.2 0.2 0.2` and mass is `0`.
6. Save and reload the world, then run the simulation.

The controllers set the tested box placements and reset the robot.
They append results to separate CSV files:

- `results/webots_extended_trials.csv`
- `results/webots_waypoint_test.csv`

The current extended controller differs from the historical
nine-trial controller. Historical nine-trial data and Figure 5
remain available separately.

## Planning and motion models

The 2D planning region measures 600 by 400 pixels. Grid movement is
horizontal or vertical, with equal edge costs within a resolution.

A-star uses Manhattan distance. Dijkstra uses the same priority-queue
search with a zero heuristic. BFS uses a separate FIFO queue.

Planning uses conservative rectangular obstacle margins, which can
exclude physically valid routes near corners.

The 2D motion model uses a circular robot with radius 12 pixels,
perfect position feedback, and optional independent Gaussian displacement
noise on each axis per movement step. It is a simplified holonomic model.

The Webots experiments use an E-puck with perfect Supervisor pose
feedback and no deliberately added motion noise. Their planning radius
is 40 pixels, with 0.001 metres per planner pixel and an 8-millimetre
waypoint tolerance.

These models differ. Their execution measurements should not be pooled.

## Selected observations

- On 100 generated maps, 10- and 20-pixel grids found routes on 95 maps;
  the 40-pixel grid found routes on 89.
- Resolution retry preserved tested fine-grid reachability across
  244 passage cases but did not reduce average planning time.
- At movement-noise sigma 2, eight pixels of extra clearance increased
  successful completion from 193/400 to 256/400 cases. Greater clearance
  also made some maps unplannable.
- All 81 extended Webots trials reached the goal without detected
  contact with the named box.
- In a separate same-polyline experiment, adding collinear midpoints
  increased execution duration from 83.616 to 123.232 simulation seconds,
  approximately 47.4%.

These findings describe the tested maps, controllers, and settings.
They do not establish a universal planner ranking.

## Measurement interpretation

**Planning time** measures the planning call, including its internal
checks and route reconstruction. External validation, animation, and
CSV writing are excluded.

**Planned distance** is the sum of planned segment lengths. No-route
cases have no planned distance.

**Nodes expanded** counts positions where neighbours are examined,
excluding the final goal and stale priority-queue entries.

**Webots execution time** measures simulated robot motion duration,
not planning computation time or wall-clock time.

**Moving and turning durations** describe controller command modes.
Moving commands can include steering corrections.

The extended Webots summary gives each layout equal weight by averaging
its three-repetition median. Repetitions are not independent layouts.

## Data and reproducibility

- `results/`: fresh local outputs, excluded from Git.
- `data/`: archived data used by reports and plotting scripts.
- `figures/`: generated PNG and PDF figures.
- `maps/`: saved example custom maps.

The archived 2D files contain summaries rather than every individual
timing or motion trial. Archived Webots files contain individual trials.

Some experiments overwrite their local CSV outputs; Webots controllers
append trials with run IDs. Check each script before rerunning.

Timings can change with hardware, system load, software versions,
and implementation changes. Reruns should be identified separately
from historical results.

## Limitations

- Static rectangular obstacles and known maps.
- Generated maps align to a 20-pixel grid.
- Conservative planning margins.
- Perfect position feedback in both motion models.
- No physical robot validation.
- Webots layouts vary the position of one box.
- Webots contact monitoring covers the named box.
- Repeated trials do not establish robustness to uncertainty.
- No confidence intervals or significance tests are reported.
- The literature review remains preliminary.

## Development assistance

AI assistance was used in developing code and drafting documentation.
The project includes automated checks and recorded experiments, but
independent technical review remains valuable.