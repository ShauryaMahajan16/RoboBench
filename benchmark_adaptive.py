import csv
import math
import random
import statistics
import time
from pathlib import Path

from adaptive import find_path_adaptive
from planner import find_path
from offset_passages import PASSAGE_WIDTHS, CENTRES, make_passage
from scenarios import validate_geometry
from test_maps import START, GOAL, RADIUS, route_distance

METHODS = ["Fixed 10px", "Coarse-to-fine"]
REPEATS = 10
WARMUP_RUNS = 2


def plan(method, obstacles, stats):
    if method == "Coarse-to-fine":
        return find_path_adaptive(
            START, GOAL, obstacles, RADIUS, stats=stats
        )

    path = find_path(
        START, GOAL, obstacles, RADIUS, 10, stats=stats
    )
    stats["selected_grid"] = 10 if path else None
    stats["attempt_count"] = 1
    return path


def save_csv(filename, rows):
    with filename.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    rng = random.Random(42)
    cases = [
        (width, centre)
        for width in PASSAGE_WIDTHS
        for centre in CENTRES
    ]
    rng.shuffle(cases)

    trials = []
    summaries = []
    passed = 0

    for index, (width, centre) in enumerate(cases, start=1):
        obstacles = make_passage(centre, width)

        reference = find_path(
            START, GOAL, obstacles, RADIUS, 10
        )
        expected_success = bool(reference)
        fine_distance = route_distance(reference) if reference else None

        timings = {method: [] for method in METHODS}
        results = {}

        for method in METHODS:
            for _ in range(WARMUP_RUNS):
                plan(method, obstacles, {})

        for trial in range(1, REPEATS + 1):
            order = METHODS.copy()
            rng.shuffle(order)

            for method in order:
                stats = {}

                started = time.perf_counter_ns()
                path = plan(method, obstacles, stats)
                elapsed_ms = (
                    time.perf_counter_ns() - started
                ) / 1_000_000

                success = bool(path)
                grid = stats["selected_grid"]
                distance = route_distance(path) if path else None

                if success != expected_success:
                    raise RuntimeError(
                        f"Reachability mismatch: {width}px, centre {centre}"
                    )

                if path:
                    validate_geometry(
                        path, obstacles, RADIUS, grid
                    )

                    if distance < fine_distance - 0.000001:
                        raise RuntimeError("Unexpected distance improvement.")

                    if method == "Fixed 10px" and not math.isclose(
                        distance, fine_distance
                    ):
                        raise RuntimeError("Fixed-grid distance mismatch.")

                attempts = (
                    len(stats["attempts"])
                    if method == "Coarse-to-fine" else 1
                )

                timings[method].append(elapsed_ms)
                results[method] = {
                    "distance": distance,
                    "grid": grid,
                    "nodes": stats["nodes_expanded"],
                    "attempts": attempts,
                }

                trials.append({
                    "passage_width_px": width,
                    "passage_centre_y": centre,
                    "method": method,
                    "trial": trial,
                    "route_found": success,
                    "distance_px": distance if success else "",
                    "selected_grid_px": grid if success else "",
                    "planning_time_ms": elapsed_ms,
                    "total_nodes_expanded": stats["nodes_expanded"],
                    "attempts": attempts,
                })

                passed += 1

        for method in METHODS:
            result = results[method]

            summaries.append({
                "passage_width_px": width,
                "passage_centre_y": centre,
                "method": method,
                "route_found": expected_success,
                "distance_px": (
                    result["distance"] if expected_success else ""
                ),
                "fine_distance_px": (
                    fine_distance if expected_success else ""
                ),
                "selected_grid_px": (
                    result["grid"] if expected_success else ""
                ),
                "median_planning_ms": statistics.median(timings[method]),
                "total_nodes_expanded": result["nodes"],
                "attempts": result["attempts"],
            })

        if index % 40 == 0 or index == len(cases):
            print(f"Completed {index}/{len(cases)} passage cases")

    for reachable, title in [
        (True, "Cases with a route"),
        (False, "Cases with no route"),
    ]:
        print(f"\n{title}")

        for method in METHODS:
            selected = [
                row for row in summaries
                if row["method"] == method
                and row["route_found"] == reachable
            ]

            if not selected:
                continue

            mean_time = statistics.mean(
                row["median_planning_ms"] for row in selected
            )
            mean_nodes = statistics.mean(
                row["total_nodes_expanded"] for row in selected
            )

            print(
                f"  {method:14} | Cases: {len(selected)} "
                f"| Mean case median: {mean_time:.3f} ms "
                f"| Mean total nodes: {mean_nodes:.1f}"
            )

    successful = [
        row for row in summaries
        if row["method"] == "Coarse-to-fine" and row["route_found"]
    ]

    extra_distances = [
        row["distance_px"] - row["fine_distance_px"]
        for row in successful
    ]

    print(
        "\nCoarse-to-fine routes longer than fixed 10px: "
        f"{sum(value > 0.000001 for value in extra_distances)}"
    )
    print(
        "Mean extra distance across successful cases: "
        f"{statistics.mean(extra_distances):.1f} px"
    )
    print(
        "Maximum extra distance: "
        f"{max(extra_distances):.1f} px"
    )

    folder = Path(__file__).resolve().parent / "results"
    folder.mkdir(exist_ok=True)

    save_csv(folder / "adaptive_trials.csv", trials)
    save_csv(folder / "adaptive_summary.csv", summaries)

    total = len(cases) * len(METHODS) * REPEATS
    print(f"\nChecks passed: {passed}/{total}")
    print(f"Results saved in: {folder}")


if __name__ == "__main__":
    main()