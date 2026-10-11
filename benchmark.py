import csv
import math
import random
import statistics
import time
from pathlib import Path

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
        [
            (200, 100, 260, 280),
            (360, 220, 440, 300),
        ],
        True,
        680,
    ),
    ("Blocking wall", [(280, 0, 320, 400)], False, None),
]

ALGORITHMS = [
    ("A-star", True),
    ("Dijkstra", False),
]


def run_planner(obstacles, use_heuristic):
    return find_path(
        START,
        GOAL,
        obstacles,
        ROBOT_RADIUS,
        CELL_SIZE,
        use_heuristic=use_heuristic,
    )


def path_distance(path):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(path, path[1:])
    )


def check_result(path, expected_success, expected_distance):
    success = bool(path)

    if success != expected_success:
        return False

    if not success:
        return True

    return (
        path[0] == START
        and path[-1] == GOAL
        and math.isclose(path_distance(path), expected_distance)
    )


def main():
    # A fixed seed makes the order of trials reproducible.
    rng = random.Random(42)

    rows = []
    summary_rows = []
    passed = 0

    for map_name, obstacles, expected_success, expected_distance in TESTS:
        timings = {name: [] for name, _ in ALGORITHMS}
        routes_found = {name: 0 for name, _ in ALGORITHMS}

        # Warm up both algorithms before recording timings.
        for _, use_heuristic in ALGORITHMS:
            for _ in range(WARMUP_RUNS):
                run_planner(obstacles, use_heuristic)

        for trial in range(1, REPEATS + 1):
            # Vary which algorithm runs first.
            order = ALGORITHMS.copy()
            rng.shuffle(order)

            for algorithm, use_heuristic in order:
                started = time.perf_counter_ns()
                path = run_planner(obstacles, use_heuristic)
                elapsed_ms = (
                    time.perf_counter_ns() - started
                ) / 1_000_000

                success = bool(path)
                distance = path_distance(path) if success else None
                correct = check_result(
                    path, expected_success, expected_distance
                )

                timings[algorithm].append(elapsed_ms)
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
                    "check_passed": correct,
                })

        print(f"\nMap: {map_name}")

        for algorithm, _ in ALGORITHMS:
            samples = timings[algorithm]
            median_ms = statistics.median(samples)
            mean_ms = statistics.mean(samples)
            distance_text = (
                f"{expected_distance:.1f} px"
                if expected_success else "N/A"
            )

            print(
                f"  {algorithm:8} | "
                f"Routes found: {routes_found[algorithm]}/{REPEATS} | "
                f"Median: {median_ms:.3f} ms | "
                f"Mean: {mean_ms:.3f} ms | "
                f"Distance: {distance_text}"
            )

            summary_rows.append({
                "map": map_name,
                "algorithm": algorithm,
                "trials": REPEATS,
                "routes_found": routes_found[algorithm],
                "median_planning_ms": median_ms,
                "mean_planning_ms": mean_ms,
            })

    # Save beside this script, regardless of terminal location.
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