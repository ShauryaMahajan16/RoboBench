import csv
import math
import statistics
from pathlib import Path

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

ALGORITHMS = [
    ("A-star", True),
    ("Dijkstra", False),
]


def main():
    rows = []
    counts = {
        name: {"reachable": [], "no_route": []}
        for name, _ in ALGORITHMS
    }

    for seed in range(MAP_COUNT):
        obstacles = generate_obstacles(seed)
        paths = {}

        for algorithm, use_heuristic in ALGORITHMS:
            stats = {}

            path = find_path(
                START,
                GOAL,
                obstacles,
                RADIUS,
                CELL_SIZE,
                use_heuristic=use_heuristic,
                stats=stats,
            )

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

        astar = paths["A-star"]
        dijkstra = paths["Dijkstra"]

        if bool(astar) != bool(dijkstra):
            raise RuntimeError(f"Reachability mismatch on seed {seed}.")

        if astar and not math.isclose(
            route_distance(astar), route_distance(dijkstra)
        ):
            raise RuntimeError(f"Distance mismatch on seed {seed}.")

    for group, title in [
        ("reachable", "Maps with a route"),
        ("no_route", "Maps with no route"),
    ]:
        print(f"\n{title}")

        for algorithm, _ in ALGORITHMS:
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

    print(f"\nAll {MAP_COUNT} map comparisons passed.")
    print(f"Results saved to: {output_file}")


if __name__ == "__main__":
    main()