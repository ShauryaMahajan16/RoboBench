import csv
import math
import random
import statistics
import time
from pathlib import Path

from bfs import find_path_bfs
from planner import find_path
from test_maps import (
    START,
    GOAL,
    RADIUS,
    CELL_SIZE,
    MAP_COUNT,
    generate_obstacles,
    route_distance,
    validate_path,
)

ALGORITHMS = ["A-star", "Dijkstra", "BFS"]
REPEATS = 20
WARMUP_RUNS = 3


def run_planner(algorithm, obstacles, stats):
    if algorithm == "BFS":
        return find_path_bfs(
            START, GOAL, obstacles, RADIUS, CELL_SIZE,
            stats=stats,
        )

    return find_path(
        START, GOAL, obstacles, RADIUS, CELL_SIZE,
        use_heuristic=(algorithm == "A-star"),
        stats=stats,
    )


def save_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
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

        # Establish the expected result for this map.
        reference = run_planner("Dijkstra", obstacles, {})
        validate_path(reference, obstacles)

        expected_success = bool(reference)
        expected_distance = (
            route_distance(reference) if reference else None
        )

        timings = {name: [] for name in ALGORITHMS}
        expansions = {name: [] for name in ALGORITHMS}

        for algorithm in ALGORITHMS:
            for _ in range(WARMUP_RUNS):
                run_planner(algorithm, obstacles, {})

        for trial in range(1, REPEATS + 1):
            order = ALGORITHMS.copy()
            rng.shuffle(order)

            for algorithm in order:
                stats = {}

                started = time.perf_counter_ns()
                path = run_planner(algorithm, obstacles, stats)
                elapsed_ms = (
                    time.perf_counter_ns() - started
                ) / 1_000_000

                # Validation is outside the timed section.
                validate_path(path, obstacles)

                success = bool(path)
                distance = route_distance(path) if path else None

                correct = success == expected_success

                if success:
                    correct = correct and math.isclose(
                        distance, expected_distance
                    )

                if not correct:
                    raise RuntimeError(
                        f"{algorithm} failed on seed {seed}, "
                        f"trial {trial}."
                    )

                passed += 1
                timings[algorithm].append(elapsed_ms)
                expansions[algorithm].append(
                    stats["nodes_expanded"]
                )

                trials.append({
                    "seed": seed,
                    "algorithm": algorithm,
                    "trial": trial,
                    "route_found": success,
                    "distance_px": distance if success else "",
                    "planning_time_ms": elapsed_ms,
                    "nodes_expanded": stats["nodes_expanded"],
                    "check_passed": correct,
                })

        for algorithm in ALGORITHMS:
            summaries.append({
                "seed": seed,
                "algorithm": algorithm,
                "route_found": expected_success,
                "distance_px": (
                    expected_distance if expected_success else ""
                ),
                "trials": REPEATS,
                "median_planning_ms": statistics.median(
                    timings[algorithm]
                ),
                "mean_planning_ms": statistics.mean(
                    timings[algorithm]
                ),
                "mean_nodes_expanded": statistics.mean(
                    expansions[algorithm]
                ),
            })

        if (seed + 1) % 10 == 0:
            print(f"Completed {seed + 1}/{MAP_COUNT} maps")

    for reachable, title in [
        (True, "Maps with a route"),
        (False, "Maps with no route"),
    ]:
        print(f"\n{title}")

        for algorithm in ALGORITHMS:
            selected = [
                row for row in summaries
                if row["algorithm"] == algorithm
                and row["route_found"] == reachable
            ]

            if not selected:
                print(f"  {algorithm}: No maps in this group")
                continue

            # Each map contributes one median timing.
            map_medians = [
                row["median_planning_ms"] for row in selected
            ]
            node_counts = [
                row["mean_nodes_expanded"] for row in selected
            ]

            print(
                f"  {algorithm:8} | Maps: {len(selected)} "
                f"| Mean of map medians: "
                f"{statistics.mean(map_medians):.3f} ms "
                f"| Mean nodes: "
                f"{statistics.mean(node_counts):.1f}"
            )

    output_folder = Path(__file__).resolve().parent / "results"
    output_folder.mkdir(exist_ok=True)

    save_csv(output_folder / "map_trials.csv", trials)
    save_csv(output_folder / "map_summary.csv", summaries)

    total = MAP_COUNT * len(ALGORITHMS) * REPEATS

    print(f"\nChecks passed: {passed}/{total}")
    print(f"Results saved in: {output_folder}")


if __name__ == "__main__":
    main()