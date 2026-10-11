import csv
import json
import math
import random
from pathlib import Path

from planner import find_path

START = (60, 200)
GOAL = (540, 200)
RADIUS = 12
CELL_SIZE = 20
MAP_COUNT = 100


def generate_obstacles(seed):
    rng = random.Random(seed)
    obstacles = []

    for _ in range(rng.randint(3, 8)):
        left = rng.randrange(100, 480, 20)
        top = rng.randrange(0, 340, 20)
        right = min(left + rng.choice([40, 60, 80]), 580)
        bottom = min(top + rng.choice([60, 100, 140]), 400)

        obstacles.append((left, top, right, bottom))

    return obstacles


def route_distance(path):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(path, path[1:])
    )


def validate_path(path, obstacles):
    # No returned route has no geometry to validate.
    if not path:
        return

    if path[0] != START or path[-1] != GOAL:
        raise ValueError("Route has incorrect start or goal.")

    for x, y in path:
        if not (
            RADIUS <= x <= 600 - RADIUS
            and RADIUS <= y <= 400 - RADIUS
        ):
            raise ValueError("Robot leaves the map boundary.")

    for a, b in zip(path, path[1:]):
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])

        if (dx, dy) not in [(CELL_SIZE, 0), (0, CELL_SIZE)]:
            raise ValueError("Route contains an invalid grid step.")

        min_x, max_x = sorted([a[0], b[0]])
        min_y, max_y = sorted([a[1], b[1]])

        for left, top, right, bottom in obstacles:
            # Exact minimum distance from this axis-aligned
            # movement segment to the obstacle rectangle.
            gap_x = max(left - max_x, min_x - right, 0)
            gap_y = max(top - max_y, min_y - bottom, 0)

            if math.hypot(gap_x, gap_y) <= RADIUS:
                raise ValueError("Robot touches an obstacle.")


def main():
    rows = []
    reachable = 0
    passed = 0

    for seed in range(MAP_COUNT):
        obstacles = generate_obstacles(seed)

        astar_path = find_path(
            START, GOAL, obstacles, RADIUS, CELL_SIZE,
            use_heuristic=True,
        )

        dijkstra_path = find_path(
            START, GOAL, obstacles, RADIUS, CELL_SIZE,
            use_heuristic=False,
        )

        try:
            # Check geometry separately from the planner.
            validate_path(astar_path, obstacles)
            validate_path(dijkstra_path, obstacles)

            if bool(astar_path) != bool(dijkstra_path):
                raise ValueError("Algorithms disagree on reachability.")

            if astar_path and not math.isclose(
                route_distance(astar_path),
                route_distance(dijkstra_path),
            ):
                raise ValueError("Algorithms disagree on shortest distance.")

        except ValueError as error:
            raise RuntimeError(
                f"Map seed {seed} failed: {error}"
            ) from error

        passed += 1
        reachable += int(bool(astar_path))

        for algorithm, path in [
            ("A-star", astar_path),
            ("Dijkstra", dijkstra_path),
        ]:
            rows.append({
                "seed": seed,
                "algorithm": algorithm,
                "obstacles": json.dumps(obstacles),
                "route_found": bool(path),
                "distance_px": route_distance(path) if path else "",
                "geometry_valid": True if path else "",
                "comparison_passed": True,
            })

    output_folder = Path(__file__).resolve().parent / "results"
    output_folder.mkdir(exist_ok=True)
    output_file = output_folder / "generated_maps.csv"

    with output_file.open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"Different maps tested: {MAP_COUNT}")
    print(f"Maps with a route found: {reachable}")
    print(f"Maps with no route found: {MAP_COUNT - reachable}")
    print(f"Map checks passed: {passed}/{MAP_COUNT}")
    print(f"Results saved to: {output_file}")


if __name__ == "__main__":
    main()