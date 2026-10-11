import math

from planner import find_path

START = (60, 200)
GOAL = (540, 200)

# Each wall has a gap between its upper and lower rectangles.
gap_40 = [
    (280, 0, 320, 180),
    (280, 220, 320, 400),
]

gap_20 = [
    (280, 0, 320, 190),
    (280, 210, 320, 400),
]

# This gap allows the robot centre near y=190.
# The coarse grid visits y=180 and y=200, missing that opening.
offset_gap_30 = [
    (280, 0, 320, 175),
    (280, 205, 320, 400),
]

SCENARIOS = [
    {
        "name": "40-pixel passage",
        "obstacles": gap_40,
        "radius": 12,
        "cell_size": 20,
        "expected_distance": 480,
    },
    {
        "name": "20-pixel passage: robot too wide",
        "obstacles": gap_20,
        "radius": 12,
        "cell_size": 20,
        "expected_distance": None,
    },
    {
        "name": "40-pixel passage: larger robot touches walls",
        "obstacles": gap_40,
        "radius": 20,
        "cell_size": 20,
        "expected_distance": None,
    },
    {
        "name": "Detour around wall",
        "obstacles": [(280, 0, 320, 300)],
        "radius": 12,
        "cell_size": 20,
        "expected_distance": 720,
    },
    {
        "name": "Offset passage: coarse grid",
        "obstacles": offset_gap_30,
        "radius": 12,
        "cell_size": 20,
        "expected_distance": None,
    },
    {
        "name": "Offset passage: finer grid",
        "obstacles": offset_gap_30,
        "radius": 12,
        "cell_size": 10,
        "expected_distance": 500,
    },
]


def path_distance(path):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(path, path[1:])
    )


def validate_geometry(path, obstacles, radius, cell_size):
    if not path:
        return

    if path[0] != START or path[-1] != GOAL:
        raise ValueError("Incorrect route endpoints.")

    for x, y in path:
        if not (
            radius <= x <= 600 - radius
            and radius <= y <= 400 - radius
        ):
            raise ValueError("Robot leaves the map.")

    for a, b in zip(path, path[1:]):
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])

        if (dx, dy) not in [(cell_size, 0), (0, cell_size)]:
            raise ValueError("Invalid grid step.")

        min_x, max_x = sorted([a[0], b[0]])
        min_y, max_y = sorted([a[1], b[1]])

        for left, top, right, bottom in obstacles:
            gap_x = max(left - max_x, min_x - right, 0)
            gap_y = max(top - max_y, min_y - bottom, 0)

            if math.hypot(gap_x, gap_y) <= radius:
                raise ValueError("Robot touches an obstacle.")


def main():
    passed = 0
    total = len(SCENARIOS) * 2

    for scenario in SCENARIOS:
        print(f"\nScenario: {scenario['name']}")

        for algorithm, use_heuristic in [
            ("A-star", True),
            ("Dijkstra", False),
        ]:
            stats = {}

            path = find_path(
                START,
                GOAL,
                scenario["obstacles"],
                scenario["radius"],
                scenario["cell_size"],
                use_heuristic=use_heuristic,
                stats=stats,
            )

            validate_geometry(
                path,
                scenario["obstacles"],
                scenario["radius"],
                scenario["cell_size"],
            )

            expected = scenario["expected_distance"]
            distance = path_distance(path) if path else None

            if expected is None:
                correct = not path
            else:
                correct = (
                    bool(path)
                    and math.isclose(distance, expected)
                )

            if not correct:
                raise RuntimeError(
                    f"{algorithm} failed: {scenario['name']}. "
                    f"Expected distance {expected}, got {distance}."
                )

            passed += 1
            distance_text = (
                f"{distance:.0f} px" if path else "No route"
            )

            print(
                f"  {algorithm:8} | {distance_text:>10} "
                f"| Nodes expanded: {stats['nodes_expanded']} "
                f"| PASS"
            )

    print(f"\nScenario checks passed: {passed}/{total}")


if __name__ == "__main__":
    main()