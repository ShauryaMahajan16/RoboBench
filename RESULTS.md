# RoboBench: Initial Results

## Research question

How does grid resolution affect route finding, route distance,
planning time, and search effort in a static 2D environment?

## Method

Three planners were evaluated:

- A-star with Manhattan distance.
- Dijkstra using the same priority-queue search with a zero heuristic.
- BFS using a separately implemented queue-based search.

The experiment used 100 reproducible obstacle maps, generated with
seeds 0–99.

Map dimensions were 600 × 400 pixels. Robot radius was 12 pixels.
Start was (60, 200), and goal was (540, 200).

Each map was tested at grid spacings of 10, 20, and 40 pixels.
Movement was restricted to horizontal and vertical steps.

Each algorithm received two warm-up runs and ten recorded runs per
map and grid spacing. Algorithm and grid order were shuffled using
a fixed seed.

Timing excluded route validation and file writing. Reported times
are averages of per-map median planning times, including no-route cases.

Returned paths were checked for endpoint correctness, valid steps,
map boundaries, and circular robot clearance. Algorithm reachability
and route distance were compared against Dijkstra.

All 9,000 recorded planner checks passed. Grid refinement checks
also passed on every map.

## Grid resolution results

| Grid spacing | Maps with routes | A-star time | Dijkstra time | BFS time |
|---|---:|---:|---:|---:|
| 10 px | 95/100 | 4.308 ms | 12.332 ms | 3.461 ms |
| 20 px | 95/100 | 1.309 ms | 3.333 ms | 1.047 ms |
| 40 px | 89/100 | 0.416 ms | 0.748 ms | 0.291 ms |

These timings describe one local experiment run and may change
with hardware, system load, and subsequent implementation changes.

## Reachability and route distance

The 10- and 20-pixel grids found routes on the same 95 maps.
Their shortest route distances were identical on all 95.

The 20-pixel grid found routes on six maps where the 40-pixel grid
found none:

37, 42, 58, 61, 65, 81.

Among the 89 maps reachable at both 20 and 40 pixels:

- The 20-pixel grid produced shorter routes on 39 maps.
- Distances were equal on 50 maps.
- Mean distance reduction across the 89 paired maps was 24.3 pixels.
- Maximum distance reduction was 280 pixels.

## Interpretation

The coarse grid reduced computation but lost routes and sometimes
required longer paths.

For this generated-map set, the 20-pixel grid preserved the
reachability and distances obtained at 10 pixels with less computation.

This does not establish 20 pixels as universally optimal. A separate
designed offset-passage test found a route at 10 pixels that the
20-pixel grid missed.

All three planners agreed on reachability and shortest distance,
but they differed in search effort and runtime.

A-star generally expanded fewer nodes. That did not consistently
make it the fastest implementation.

## Earlier paired timing experiment

In a separate experiment at 20-pixel spacing, with 20 recorded
runs per algorithm per map:

- A-star had a lower median than BFS on 48 reachable maps.
- BFS had a lower median on 47 reachable maps.
- BFS had the lower average of map median times.
- The median per-map A-star/BFS time ratio was 0.959.

Aggregate averages and per-map rankings describe different aspects
of performance. Neither established a consistent runtime winner.

Timings from separate sessions should not be treated as a controlled
comparison of implementation changes.

## Limitations

- Maps contain static rectangles, and the full map is known.
- There is no sensor noise, motion uncertainty, or dynamic obstacle model.
- Obstacle safety margins are rectangular and conservative near corners.
- No-route cases can include blocked endpoints or grid-resolution effects.
- Generated obstacles align to a 20-pixel grid, so the map generator
  may favour some resolutions. Offset-passage tests supplement it.
- A-star and Dijkstra share search code.
- BFS is separate but uses the same planning assumptions.
- Agreement and geometric checks do not rule out every possible bug.
- Small timing differences may reflect measurement noise.
- Pixels are simulation units, not calibrated physical distances.

## Reproduce

Run inside the project folder on Windows:

python -m venv .venv

.\.venv\Scripts\python.exe resolution_experiment.py

.\.venv\Scripts\python.exe analyze_resolution.py

.\.venv\Scripts\python.exe view_resolution.py

Generated CSV files are written to the results folder and are
overwritten when the corresponding experiment is rerun.