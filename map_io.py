import json
from pathlib import Path

WIDTH = 600
HEIGHT = 400
RADIUS = 12
CELL_SIZE = 20


def validate_map(data):
    if not isinstance(data, dict):
        raise ValueError("Map file must contain a JSON object.")

    expected = {
        "version": 1,
        "width": WIDTH,
        "height": HEIGHT,
        "radius": RADIUS,
        "cell_size": CELL_SIZE,
    }

    for key, value in expected.items():
        if type(data.get(key)) is not int or data[key] != value:
            raise ValueError(f"Unsupported map setting: {key}")

    def endpoint(name):
        point = data.get(name)

        if (
            not isinstance(point, (list, tuple))
            or len(point) != 2
            or not all(type(value) is int for value in point)
        ):
            raise ValueError(f"Invalid {name} coordinates.")

        x, y = point

        if not (
            RADIUS <= x <= WIDTH - RADIUS
            and RADIUS <= y <= HEIGHT - RADIUS
            and x % CELL_SIZE == 0
            and y % CELL_SIZE == 0
        ):
            raise ValueError(f"{name} must be inside the map on the grid.")

        return (x, y)

    start = endpoint("start")
    goal = endpoint("goal")
    raw_obstacles = data.get("obstacles")

    if not isinstance(raw_obstacles, list):
        raise ValueError("Obstacles must be a list.")

    obstacles = []

    for obstacle in raw_obstacles:
        if (
            not isinstance(obstacle, (list, tuple))
            or len(obstacle) != 4
            or not all(type(value) is int for value in obstacle)
        ):
            raise ValueError("Invalid obstacle coordinates.")

        left, top, right, bottom = obstacle

        if not (
            0 <= left < right <= WIDTH
            and 0 <= top < bottom <= HEIGHT
            and all(value % CELL_SIZE == 0 for value in obstacle)
        ):
            raise ValueError("Obstacles must be inside the map on the grid.")

        rectangle = tuple(obstacle)
        if rectangle not in obstacles:
            obstacles.append(rectangle)

    # Blocked endpoints are allowed so impossible maps can be saved.
    return start, goal, obstacles


def save_map(filename, start, goal, obstacles):
    data = {
        "version": 1,
        "width": WIDTH,
        "height": HEIGHT,
        "radius": RADIUS,
        "cell_size": CELL_SIZE,
        "start": list(start),
        "goal": list(goal),
        "obstacles": [list(obstacle) for obstacle in obstacles],
    }

    validate_map(data)

    Path(filename).write_text(
        json.dumps(data, indent=2) + "\n",
        encoding="utf-8",
    )


def load_map(filename):
    data = json.loads(Path(filename).read_text(encoding="utf-8"))
    return validate_map(data)