# Coarse-to-Fine Planning Experiment

## Question

Does trying coarse grids first reduce planning time while preserving
the reachability of fixed 10-pixel A-star planning?

## Strategy

Coarse-to-fine A-star tries grid spacings of 40, 20, and 10 pixels.
It retries only when a search finds no route and stops at the first
successful resolution.

This is resolution retry, not a spatially adaptive grid.

## Method

The experiment used 244 passage layouts:

- Passage widths: 20, 30, 40, and 60 pixels.
- Passage centres: every integer y-coordinate from 160 through 220.
- Robot diameter: 24 pixels.
- Map size: 600 × 400 pixels.

Each method received two warm-up runs and ten recorded runs per case.
Case order and method order were shuffled with a fixed seed.

Timing included the complete planning call, input checks, and all
retry attempts. External geometry validation and CSV writing were excluded.

All 4,880 recorded checks passed.

## Reachability and selected resolution

Both methods found routes in 153 of 244 cases.

| Coarse-to-fine outcome | Cases |
|---|---:|
| Route found at 40 pixels | 84 |
| Route found at 20 pixels | 39 |
| Route found at 10 pixels | 30 |
| No route | 91 |

## Timing and search effort

Times are averages of per-case median timings from one local run.

| Case group | Fixed 10px time | Retry time | Fixed nodes | Retry total nodes |
|---|---:|---:|---:|---:|
| Reachable: 153 cases | 0.761 ms | 0.792 ms | 126.1 | 132.5 |
| No route: 91 cases | 5.104 ms | 6.880 ms | 925.0 | 1235.0 |

Retry node counts include all attempted resolutions.

The reachable-case timing difference was small. In no-route cases,
the retry strategy's average timing was approximately 35% higher.

## Route quality

Coarse-to-fine produced longer routes in 41 successful cases.

- Mean extra distance across all 153 successful cases: 8.0 pixels.
- Maximum extra distance: 60 pixels.

Stopping at a successful coarse resolution can sacrifice the shortest
route available on a finer grid.

## Conclusion

In this passage experiment, coarse-to-fine retry preserved the tested
fine-grid reachability but did not reduce average planning time.
It also sometimes produced longer routes.

These results do not establish that resolution retry is ineffective
for every environment. They show that it was not an improvement on
the cases and implementation tested here.

## Limitations

- The environment is a simple static wall with one passage.
- All planners know the complete obstacle map.
- Timing depends on hardware and system load.
- No statistical significance analysis was performed.
- The strategy performs additional input checks.
- Reachability preservation relies on these nested grids and aligned endpoints.
- Conservative obstacle margins may exclude physically valid corner routes.

## Reproduce

Run from the RoboBench folder:

python -m venv .venv

.\.venv\Scripts\python.exe offset_passages.py

.\.venv\Scripts\python.exe test_adaptive.py

.\.venv\Scripts\python.exe benchmark_adaptive.py

Outputs are saved in results/offset_passages.csv,
results/adaptive_trials.csv, and results/adaptive_summary.csv.