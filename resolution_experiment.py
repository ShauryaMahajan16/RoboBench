import csv
import math
import random
import statistics
import time
from pathlib import Path

from bfs import find_path_bfs
from planner import find_path
from scenarios import validate_geometry
from test_maps import (
    START,
    GOAL,
    RADIUS,
    MAP_COUNT,
    generate_obstacles,
    route_distance,
)

GRID_SIZES = [10, 20, 40]
ALGORITHMS = ["A-star", "Dijkstra", "BFS"]
REPEATS = 10
WARMUP_RUNS = 2


def run_planner(algorithm, obstacles, grid_size, stats):
    if algorithm == "BFS":
        return find_path_bfs(
            START, GOAL, obstacles, RADIUS, grid_size,
            stats=stats,
        )

    return find_path(
        START, GOAL, obstacles, RADIUS, grid_size,
        use_heuristic=(algorithm == "A-star"),
        stats=stats,
    )


def save_csv(filename, rows):
    with filename.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    rng = random.Random(42)
    trials = []
    summaries = []
    passed = 0

    for seed in range(MAP_COUNT):
        obstacles = generate_obstacles(seed)

        # Vary grid order to reduce systematic order effects.
        grid_order = GRID_SIZES.copy()
        rng.shuffle(grid_order)

        references = {}

        for grid_size in grid_order:
            reference = run_planner(
                "Dijkstra", obstacles, grid_size, {}
            )

            validate_geometry(
                reference, obstacles, RADIUS, grid_size
            )

            expected_success = bool(reference)
            expected_distance = (
                route_distance(reference) if reference else None
            )
            references[grid_size] = expected_distance

            timings = {name: [] for name in ALGORITHMS}
            node_counts = {name: [] for name in ALGORITHMS}

            for algorithm in ALGORITHMS:
                for _ in range(WARMUP_RUNS):
                    run_planner(algorithm, obstacles, grid_size, {})

            for trial in range(1, REPEATS + 1):
                order = ALGORITHMS.copy()
                rng.shuffle(order)

                for algorithm in order:
                    stats = {}

                    started = time.perf_counter_ns()
                    path = run_planner(
                        algorithm, obstacles, grid_size, stats
                    )
                    elapsed_ms = (
                        time.perf_counter_ns() - started
                    ) / 1_000_000

                    validate_geometry(
                        path, obstacles, RADIUS, grid_size
                    )

                    success = bool(path)
                    distance = route_distance(path) if path else None

                    if success != expected_success:
                        raise RuntimeError(
                            f"Reachability mismatch: seed {seed}, "
                            f"grid {grid_size}, {algorithm}"
                        )

                    if success and not math.isclose(
                        distance, expected_distance
                    ):
                        raise RuntimeError(
                            f"Distance mismatch: seed {seed}, "
                            f"grid {grid_size}, {algorithm}"
                        )

                    passed += 1
                    timings[algorithm].append(elapsed_ms)
                    node_counts[algorithm].append(
                        stats["nodes_expanded"]
                    )

                    trials.append({
                        "seed": seed,
                        "grid_size_px": grid_size,
                        "algorithm": algorithm,
                        "trial": trial,
                        "route_found": success,
                        "distance_px": distance if success else "",
                        "planning_time_ms": elapsed_ms,
                        "nodes_expanded": stats["nodes_expanded"],
                    })

            for algorithm in ALGORITHMS:
                summaries.append({
                    "seed": seed,
                    "grid_size_px": grid_size,
                    "algorithm": algorithm,
                    "route_found": expected_success,
                    "distance_px": (
                        expected_distance if expected_success else ""
                    ),
                    "median_planning_ms": statistics.median(
                        timings[algorithm]
                    ),
                    "mean_nodes_expanded": statistics.mean(
                        node_counts[algorithm]
                    ),
                })

        # These grids are nested relative to the same start.
        # A finer grid should preserve any valid coarse-grid route.
        for fine, coarse in [(10, 20), (20, 40)]:
            fine_distance = references[fine]
            coarse_distance = references[coarse]

            if coarse_distance is not None:
                if (
                    fine_distance is None
                    or fine_distance > coarse_distance + 0.000001
                ):
                    raise RuntimeError(
                        f"Grid refinement check failed on seed {seed}."
                    )

        if (seed + 1) % 10 == 0:
            print(f"Completed {seed + 1}/{MAP_COUNT} maps")

    for grid_size in GRID_SIZES:
        print(f"\nGrid spacing: {grid_size} pixels")

        for algorithm in ALGORITHMS:
            selected = [
                row for row in summaries
                if row["grid_size_px"] == grid_size
                and row["algorithm"] == algorithm
            ]

            found = sum(row["route_found"] for row in selected)
            mean_time = statistics.mean(
                row["median_planning_ms"] for row in selected
            )
            mean_nodes = statistics.mean(
                row["mean_nodes_expanded"] for row in selected
            )

            print(
                f"  {algorithm:8} | Routes: {found}/{MAP_COUNT} "
                f"| Mean map median: {mean_time:.3f} ms "
                f"| Mean nodes: {mean_nodes:.1f}"
            )

    folder = Path(__file__).resolve().parent / "results"
    folder.mkdir(exist_ok=True)

    save_csv(folder / "resolution_trials.csv", trials)
    save_csv(folder / "resolution_summary.csv", summaries)

    total = MAP_COUNT * len(GRID_SIZES) * len(ALGORITHMS) * REPEATS

    print(f"\nPlanner checks passed: {passed}/{total}")
    print("Grid refinement checks passed on all maps.")
    print("Timing and node averages include maps with no route.")
    print(f"Results saved in: {folder}")


if __name__ == "__main__":
    main()