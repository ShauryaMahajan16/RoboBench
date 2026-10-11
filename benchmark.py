import csv
import math
import random
import statistics
import time
from pathlib import Path

from bfs import find_path_bfs
from planner import find_path

START = (60, 200)
GOAL = (540, 200)
ROBOT_RADIUS = 12
CELL_SIZE = 20

REPEATS = 100
WARMUP_RUNS = 5

TESTS = [
    ("Clear map", [], True, 480),
    (
        "Two obstacles",
        [(200, 100, 260, 280), (360, 220, 440, 300)],
        True,
        680,
    ),
    ("Blocking wall", [(280, 0, 320, 400)], False, None),
]

ALGORITHMS = ["A-star", "Dijkstra", "BFS"]


def run_planner(algorithm, obstacles, stats):
    if algorithm == "BFS":
        return find_path_bfs(
            START, GOAL, obstacles, ROBOT_RADIUS, CELL_SIZE,
            stats=stats,
        )

    return find_path(
        START, GOAL, obstacles, ROBOT_RADIUS, CELL_SIZE,
        use_heuristic=(algorithm == "A-star"),
        stats=stats,
    )


def path_distance(path):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(path, path[1:])
    )


def check_result(path, expected_success, expected_distance):
    if bool(path) != expected_success:
        return False

    if not path:
        return True

    return (
        path[0] == START
        and path[-1] == GOAL
        and math.isclose(path_distance(path), expected_distance)
    )


def main():
    rng = random.Random(42)
    rows = []
    summary_rows = []
    passed = 0

    for map_name, obstacles, expected_success, expected_distance in TESTS:
        timings = {name: [] for name in ALGORITHMS}
        expansions = {name: [] for name in ALGORITHMS}
        routes_found = {name: 0 for name in ALGORITHMS}

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

                success = bool(path)
                distance = path_distance(path) if success else None
                correct = check_result(
                    path, expected_success, expected_distance
                )

                timings[algorithm].append(elapsed_ms)
                expansions[algorithm].append(stats["nodes_expanded"])
                routes_found[algorithm] += int(success)
                passed += int(correct)

                rows.append({
                    "map": map_name,
                    "algorithm": algorithm,
                    "trial": trial,
                    "route_found": success,
                    "planned_distance_px": (
                        distance if distance is not None else ""
                    ),
                    "planning_time_ms": elapsed_ms,
                    "nodes_expanded": stats["nodes_expanded"],
                    "check_passed": correct,
                })

        print(f"\nMap: {map_name}")

        for algorithm in ALGORITHMS:
            median_ms = statistics.median(timings[algorithm])
            mean_ms = statistics.mean(timings[algorithm])
            mean_nodes = statistics.mean(expansions[algorithm])

            distance_text = (
                f"{expected_distance:.1f} px"
                if expected_success else "N/A"
            )

            print(
                f"  {algorithm:8} | "
                f"Routes: {routes_found[algorithm]}/{REPEATS} | "
                f"Median: {median_ms:.3f} ms | "
                f"Mean nodes: {mean_nodes:.1f} | "
                f"Distance: {distance_text}"
            )

            summary_rows.append({
                "map": map_name,
                "algorithm": algorithm,
                "trials": REPEATS,
                "routes_found": routes_found[algorithm],
                "median_planning_ms": median_ms,
                "mean_planning_ms": mean_ms,
                "mean_nodes_expanded": mean_nodes,
            })

    output_folder = Path(__file__).resolve().parent / "results"
    output_folder.mkdir(exist_ok=True)

    for filename, data in [
        ("trials.csv", rows),
        ("summary.csv", summary_rows),
    ]:
        with (output_folder / filename).open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)

    total = len(TESTS) * len(ALGORITHMS) * REPEATS

    print(f"\nChecks passed: {passed}/{total}")
    print(f"Results saved in: {output_folder}")

    if passed != total:
        raise SystemExit("Some checks failed. Review the results.")


if __name__ == "__main__":
    main()