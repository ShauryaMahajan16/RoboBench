import math

from bfs import find_path_bfs
from planner import find_path
from scenarios import SCENARIOS, validate_geometry
from test_maps import (
    START,
    GOAL,
    RADIUS,
    CELL_SIZE,
    MAP_COUNT,
    generate_obstacles,
    route_distance,
)


def compare_algorithms(obstacles, radius, cell_size):
    bfs_path = find_path_bfs(
        START, GOAL, obstacles, radius, cell_size
    )

    validate_geometry(bfs_path, obstacles, radius, cell_size)

    for name, use_heuristic in [
        ("A-star", True),
        ("Dijkstra", False),
    ]:
        other_path = find_path(
            START,
            GOAL,
            obstacles,
            radius,
            cell_size,
            use_heuristic=use_heuristic,
        )

        validate_geometry(
            other_path, obstacles, radius, cell_size
        )

        if bool(bfs_path) != bool(other_path):
            raise AssertionError(
                f"BFS and {name} disagree on reachability."
            )

        if bfs_path and not math.isclose(
            route_distance(bfs_path),
            route_distance(other_path),
        ):
            raise AssertionError(
                f"BFS and {name} disagree on distance."
            )

    return bfs_path


def main():
    assert find_path_bfs(
        START, START, [], RADIUS, CELL_SIZE
    ) == [START]

    assert find_path_bfs(
        START,
        START,
        [(40, 180, 80, 220)],
        RADIUS,
        CELL_SIZE,
    ) == []

    print("PASS: Clear and blocked identical endpoints")

    for scenario in SCENARIOS:
        path = compare_algorithms(
            scenario["obstacles"],
            scenario["radius"],
            scenario["cell_size"],
        )

        expected = scenario["expected_distance"]

        if expected is None:
            if path:
                raise AssertionError("Unexpected route found.")
        elif not path or not math.isclose(
            route_distance(path), expected
        ):
            raise AssertionError("Incorrect scenario distance.")

        print(f"PASS: {scenario['name']}")

    for seed in range(MAP_COUNT):
        try:
            compare_algorithms(
                generate_obstacles(seed),
                RADIUS,
                CELL_SIZE,
            )
        except (AssertionError, ValueError) as error:
            raise AssertionError(
                f"Generated map seed {seed}: {error}"
            ) from error

    print(
        f"\nGenerated-map comparisons passed: "
        f"{MAP_COUNT}/{MAP_COUNT}"
    )
    print("Designed-scenario comparisons passed: 6/6")
    print("All BFS checks passed.")


if __name__ == "__main__":
    main()