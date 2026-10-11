import csv
import math
from pathlib import Path

from bfs import find_path_bfs
from planner import find_path
from scenarios import validate_geometry
from test_maps import START, GOAL, RADIUS, route_distance

GRID_SIZES = [10, 20, 40]
ALGORITHMS = ["A-star", "Dijkstra", "BFS"]

PASSAGE_WIDTHS = [20, 30, 40, 60]
CENTRES = range(160, 221)


def make_passage(centre, width):
    top_edge = centre - width // 2
    bottom_edge = top_edge + width

    return [
        (280, 0, 320, top_edge),
        (280, bottom_edge, 320, 400),
    ]


def run_planner(algorithm, obstacles, grid_size):
    stats = {}

    if algorithm == "BFS":
        path = find_path_bfs(
            START, GOAL, obstacles, RADIUS, grid_size,
            stats=stats,
        )
    else:
        path = find_path(
            START, GOAL, obstacles, RADIUS, grid_size,
            use_heuristic=(algorithm == "A-star"),
            stats=stats,
        )

    return path, stats


def main():
    rows = []
    passed = 0

    for width in PASSAGE_WIDTHS:
        found_counts = {grid: 0 for grid in GRID_SIZES}

        for centre in CENTRES:
            obstacles = make_passage(centre, width)

            for grid_size in GRID_SIZES:
                # A centre must lie strictly between these limits
                # to avoid touching the upper and lower walls.
                lowest = centre - width // 2 + RADIUS
                highest = centre - width // 2 + width - RADIUS

                # Grid points are anchored at the starting position.
                grid_y_values = range(
                    START[1] % grid_size,
                    401,
                    grid_size,
                )

                expected_success = any(
                    lowest < y < highest
                    and RADIUS <= y <= 400 - RADIUS
                    for y in grid_y_values
                )

                distances = []

                for algorithm in ALGORITHMS:
                    path, stats = run_planner(
                        algorithm, obstacles, grid_size
                    )

                    validate_geometry(
                        path, obstacles, RADIUS, grid_size
                    )

                    if bool(path) != expected_success:
                        raise RuntimeError(
                            f"Unexpected result: width {width}, "
                            f"centre {centre}, grid {grid_size}, "
                            f"{algorithm}"
                        )

                    distance = route_distance(path) if path else None

                    if path:
                        distances.append(distance)

                    rows.append({
                        "passage_width_px": width,
                        "passage_centre_y": centre,
                        "grid_size_px": grid_size,
                        "algorithm": algorithm,
                        "route_found": bool(path),
                        "expected_route_found": expected_success,
                        "distance_px": distance if path else "",
                        "nodes_expanded": stats["nodes_expanded"],
                    })

                    passed += 1

                if distances and not all(
                    math.isclose(value, distances[0])
                    for value in distances
                ):
                    raise RuntimeError(
                        "Algorithms disagree on shortest distance."
                    )

                found_counts[grid_size] += int(expected_success)

        print(f"\nPassage width: {width} pixels")

        for grid_size in GRID_SIZES:
            print(
                f"  Grid {grid_size:2}px: "
                f"route found at {found_counts[grid_size]}/"
                f"{len(CENTRES)} passage positions"
            )

    folder = Path(__file__).resolve().parent / "results"
    folder.mkdir(exist_ok=True)
    destination = folder / "offset_passages.csv"

    with destination.open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nPlanner checks passed: {passed}/{len(rows)}")
    print("All route-distance comparisons passed.")
    print(f"Results saved to: {destination}")


if __name__ == "__main__":
    main()