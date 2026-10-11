# RoboBench

A Python prototype for comparing A-star and Dijkstra path planning
on reproducible two-dimensional obstacle maps.

The project includes an animated simulation, a map viewer,
automated correctness checks, repeated timing measurements,
and measurements of search effort.

## Requirements

- Python 3.10 or newer
- Tkinter for the simulation and viewer
- Git for version control

Only Python's standard library is used. No pip packages are required.

## Setup on Windows

Open a terminal inside the RoboBench folder:

```powershell
python -m venv .venv
```

Run commands using the environment's Python directly:

```powershell
.\.venv\Scripts\python.exe test_planner.py
```

In VS Code, select `.venv\Scripts\python.exe` using
**Python: Select Interpreter**.

## Run the checks and experiments

### Input and endpoint checks

```powershell
.\.venv\Scripts\python.exe test_planner.py
```

Expected: all 18 checks pass.

### Repeated timing benchmark

```powershell
.\.venv\Scripts\python.exe benchmark.py
```

Tests three fixed maps with 100 recorded runs per algorithm per map,
after five warm-up runs. Algorithm order is shuffled with a fixed seed.

Expected: 600/600 checks pass.

Outputs:

- `results/trials.csv`
- `results/summary.csv`

### Generated-map checks

```powershell
.\.venv\Scripts\python.exe test_maps.py
```

Tests 100 maps generated using seeds 0–99. Checks returned route
geometry and compares algorithm reachability and path distance.

Expected: 100/100 map checks pass.

Output: `results/generated_maps.csv`

### Search-effort comparison

```powershell
.\.venv\Scripts\python.exe exploration.py
```

Measures nodes expanded on the same 100 generated maps.

Expected: all 100 map comparisons pass.

Output: `results/exploration.csv`

### Designed scenarios

```powershell
.\.venv\Scripts\python.exe scenarios.py
```

Tests narrow passages, robot size, a detour, and an opening that
a coarse grid misses.

Expected: 12/12 scenario checks pass.

## Visual tools

### Animated simulation

```powershell
.\.venv\Scripts\python.exe main.py
```

Animates the robot following a route on the map defined in `main.py`.
A blocked map produces no movement.

### Generated-map viewer

```powershell
.\.venv\Scripts\python.exe view_map.py
```

Select a seed from 0–99, switch algorithms, or find a blocked map.
This viewer displays planned routes without animating movement.

## Planning model

- Map dimensions: 600 × 400 pixels.
- Robot: circular body with a configurable radius.
- Movement: horizontal and vertical grid steps.
- Step cost: distance in pixels.
- A-star heuristic: Manhattan distance.
- Dijkstra: the same search engine with a zero heuristic.
- Obstacles and map boundaries are known before planning.
- Touching an obstacle counts as a collision.

The planner expands each obstacle into a rectangular safety margin.
This is conservative near corners and may exclude some routes that
a circular robot could physically follow.

Start and goal must align on the same grid. Blocked endpoints return
no route; malformed inputs raise `ValueError`.

## Measurements

**Planning time:** time spent calling the planner, including its input
checks and route reconstruction. Animation and file writing are excluded.

**Planned distance:** sum of route-segment lengths. No-route cases have
no reported distance.

**Nodes expanded:** positions where the search examines neighbours.
The final goal and outdated priority-queue entries are excluded.

Timing depends on hardware and system load. Search counts also depend
on grid resolution, map geometry, and tie-breaking.

## Initial observations

On the initial 100 generated maps:

- Both algorithms found routes on 95 maps.
- Both reported no route on five maps.
- Mean expanded nodes on reachable maps were 124.4 for A-star
  and 372.0 for Dijkstra.

In the designed offset-passage scenario:

- A 20-pixel grid missed the opening.
- A 10-pixel grid found a 500-pixel route.
- A-star expanded 112 nodes; Dijkstra expanded 1,401.

These observations apply to this implementation and these maps.
Regenerate results locally using the commands above.

## Limitations

- This is a static path-planning prototype, not a physical robot benchmark.
- There is no sensor noise, motion uncertainty, or dynamic obstacle model.
- Both algorithms share search code, so agreement alone cannot establish
  correctness. Separate geometric checks provide additional validation.
- No-route cases may include blocked endpoints or openings missed by
  the grid; they are not all physically disconnected environments.
- Repeated timings on one map are not independent navigation tasks.
- Pixels are simulation units, not calibrated physical distances.

## Project files

| File | Purpose |
|---|---|
| `planner.py` | A-star and Dijkstra search |
| `main.py` | Animated simulation |
| `benchmark.py` | Repeated timing measurements |
| `test_planner.py` | Input and endpoint checks |
| `test_maps.py` | Generated maps and geometric checks |
| `exploration.py` | Search-effort measurements |
| `scenarios.py` | Designed passage and resolution tests |
| `view_map.py` | Interactive map inspection |

Generated CSV files are overwritten when experiments are rerun.
The `results` folder is excluded from Git; regenerate it locally.