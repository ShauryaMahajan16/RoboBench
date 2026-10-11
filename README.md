# RoboBench

A reproducible Python prototype for comparing A-star, Dijkstra,
and Breadth-First Search (BFS) on two-dimensional obstacle maps.

Includes an animated simulation, a map viewer, correctness checks,
timing experiments, and measurements of search effort.

## Requirements

- Python 3.10 or newer
- Tkinter for the visual tools
- Git for version control

Only Python's standard library is used. No pip packages are required.

Automated checks run on GitHub with Python 3.10, 3.12, and 3.14.

## Windows setup

Open a terminal inside the project folder:

```powershell
python -m venv .venv
```

In VS Code, use **Python: Select Interpreter** to select:

```text
.venv\Scripts\python.exe
```

The commands below run that environment directly.

## Correctness checks

### Input and endpoint handling

```powershell
.\.venv\Scripts\python.exe test_planner.py
```

Expected: 18 checks pass.

### Robot clearance, detours, and grid resolution

```powershell
.\.venv\Scripts\python.exe scenarios.py
```

Expected: 12/12 checks pass.

### Generated maps

```powershell
.\.venv\Scripts\python.exe test_maps.py
```

Expected: 100/100 map checks pass.

### Independent BFS comparisons

```powershell
.\.venv\Scripts\python.exe test_bfs.py
```

Compares BFS with A-star and Dijkstra on 100 generated maps and
six designed scenarios. Also checks identical start/goal positions.

Expected: all BFS checks pass.

## Experiments

### Three fixed maps

```powershell
.\.venv\Scripts\python.exe benchmark.py
```

Runs 100 recorded trials per algorithm per map, after warm-up.

Expected: 900/900 checks pass.

Outputs:

- `results/trials.csv`
- `results/summary.csv`

### Search effort across generated maps

```powershell
.\.venv\Scripts\python.exe exploration.py
```

Expected: 100 three-algorithm map comparisons pass.

Output: `results/exploration.csv`

### Timing across generated maps

```powershell
.\.venv\Scripts\python.exe benchmark_maps.py
```

Runs 20 recorded trials per algorithm on each of 100 maps,
after warm-up. Algorithm order is shuffled with a fixed seed.

Expected: 6000/6000 checks pass.

Outputs:

- `results/map_trials.csv`
- `results/map_summary.csv`

### Paired timing analysis

Run this after `benchmark_maps.py`:

```powershell
.\.venv\Scripts\python.exe compare_results.py
```

Compares median timings for algorithms on the same map.

Output: `results/paired_comparison.csv`

## Visual tools

```powershell
.\.venv\Scripts\python.exe main.py
```

Animates a robot following A-star's route on the map defined in
`main.py`. If no route exists, the robot remains still.

```powershell
.\.venv\Scripts\python.exe view_map.py
```

Displays generated maps for seeds 0–99 and routes from A-star
or Dijkstra. The viewer does not animate movement.

## Planning model

- Map size: 600 × 400 pixels.
- Robot: circular body with configurable radius.
- Movement: horizontal and vertical grid steps.
- All steps within an experiment have equal cost.
- A-star uses Manhattan distance.
- Dijkstra uses the same priority-queue search with a zero heuristic.
- BFS uses a separately implemented first-in, first-out queue.
- Obstacles and map boundaries are known before planning.
- Touching an obstacle counts as a collision.

Each planner uses rectangular obstacle safety margins. This is
conservative near corners and can exclude some physically valid
routes for a circular robot.

Start and goal must align on the same grid. Blocked endpoints return
no route; malformed inputs raise `ValueError`.

## Measurements

**Planning time:** planner execution, including input checks and route
reconstruction. Validation, animation, and CSV writing are excluded.

**Planned distance:** sum of route-segment lengths in pixels.
No-route cases have no reported distance.

**Nodes expanded:** positions where neighbours are examined.
The final goal and outdated priority-queue entries are excluded.

The generated-map timing summary averages one median timing per map.
The paired analysis compares algorithms within each map.

## Initial findings

In the initial generated-map experiment:

- All three algorithms found routes on 95 maps and no route on five.
- On reachable maps, mean expanded nodes were 124.4 for A-star,
  372.0 for Dijkstra, and 366.6 for BFS.
- A-star had a lower median time than BFS on 48 reachable maps;
  BFS had a lower median on 47.
- BFS had the lower average of map median times in that run.

A-star explored fewer nodes, but runtime depended on map geometry
and implementation overhead. These results do not establish a
universal runtime winner.

In a designed offset passage, a 20-pixel grid missed an opening that
a 10-pixel grid represented successfully.

Re-run experiments locally; timings vary with hardware and system load.

## Limitations

- This is a static path-planning prototype, not a physical robot benchmark.
- There is no sensor noise, motion uncertainty, or dynamic obstacle model.
- Algorithms share the same planning assumptions; agreement cannot
  rule out every modelling error.
- Separate geometric checks validate returned route clearance.
- No-route cases can include blocked endpoints or grid-resolution effects,
  rather than physically disconnected environments.
- Repeated trials measure timing variation, not independent map diversity.
- Strict timing rankings are descriptive, not statistical significance tests.
- Pixels are simulation units, not calibrated physical distances.

## Results and version control

Experiments overwrite their corresponding CSV files when rerun.
Generated files in `results/` are excluded from Git.

GitHub Actions runs the core correctness checks, search-effort
comparison, and fixed-map benchmark. The larger timing experiment
and paired analysis are run locally.