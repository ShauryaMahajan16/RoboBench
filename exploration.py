import csv
import math
import statistics
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


def main():
    rows = []
    counts = {
        name: {"reachable": [], "no_route": []}
        for name in ALGORITHMS
    }

    for seed in range(MAP_COUNT):
        obstacles = generate_obstacles(seed)
        paths = {}

        for algorithm in ALGORITHMS:
            stats = {}
            path = run_planner(algorithm, obstacles, stats)

            validate_path(path, obstacles)
            paths[algorithm] = path

            group = "reachable" if path else "no_route"
            counts[algorithm][group].append(stats["nodes_expanded"])

            rows.append({
                "seed": seed,
                "algorithm": algorithm,
                "route_found": bool(path),
                "distance_px": route_distance(path) if path else "",
                "nodes_expanded": stats["nodes_expanded"],
            })

        reference = paths["Dijkstra"]

        for algorithm in ALGORITHMS:
            path = paths[algorithm]

            if bool(path) != bool(reference):
                raise RuntimeError(
                    f"{algorithm}: reachability mismatch on seed {seed}."
                )

            if path and not math.isclose(
                route_distance(path), route_distance(reference)
            ):
                raise RuntimeError(
                    f"{algorithm}: distance mismatch on seed {seed}."
                )

    for group, title in [
        ("reachable", "Maps with a route"),
        ("no_route", "Maps with no route"),
    ]:
        print(f"\n{title}")

        for algorithm in ALGORITHMS:
            samples = counts[algorithm][group]

            if not samples:
                print(f"  {algorithm}: No maps in this group")
                continue

            print(
                f"  {algorithm:8} | Maps: {len(samples)} "
                f"| Mean nodes expanded: {statistics.mean(samples):.1f} "
                f"| Minimum: {min(samples)} "
                f"| Maximum: {max(samples)}"
            )

    output_folder = Path(__file__).resolve().parent / "results"
    output_folder.mkdir(exist_ok=True)
    output_file = output_folder / "exploration.csv"

    with output_file.open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nAll {MAP_COUNT} three-algorithm map comparisons passed.")
    print(f"Results saved to: {output_file}")


if __name__ == "__main__":
    main()