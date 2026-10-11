from collections import deque
import math

WIDTH = 600
HEIGHT = 400


def find_path_bfs(
    start, goal, obstacles, radius, cell_size=20, stats=None,
):
    if stats is None:
        stats = {}

    stats.clear()
    stats["nodes_expanded"] = 0

    def valid_point(point):
        return (
            isinstance(point, (tuple, list))
            and len(point) == 2
            and all(type(value) is int for value in point)
        )

    if not valid_point(start) or not valid_point(goal):
        raise ValueError("Start and goal must contain two integers.")

    start = tuple(start)
    goal = tuple(goal)

    if type(cell_size) is not int or cell_size <= 0:
        raise ValueError("Cell size must be a positive integer.")

    if (
        type(radius) not in (int, float)
        or not math.isfinite(radius)
        or radius <= 0
    ):
        raise ValueError("Robot radius must be a positive finite number.")

    if (
        (goal[0] - start[0]) % cell_size != 0
        or (goal[1] - start[1]) % cell_size != 0
    ):
        raise ValueError("Start and goal must align on the same grid.")

    safety_boxes = []

    for obstacle in obstacles:
        if (
            not isinstance(obstacle, (tuple, list))
            or len(obstacle) != 4
            or not all(type(value) is int for value in obstacle)
        ):
            raise ValueError("Each obstacle must contain four integers.")

        left, top, right, bottom = obstacle

        if not (
            0 <= left < right <= WIDTH
            and 0 <= top < bottom <= HEIGHT
        ):
            raise ValueError("Obstacle dimensions or bounds are invalid.")

        safety_boxes.append((
            left - radius,
            top - radius,
            right + radius,
            bottom + radius,
        ))

    def clear_segment(a, b):
        for x, y in (a, b):
            if not (
                radius <= x <= WIDTH - radius
                and radius <= y <= HEIGHT - radius
            ):
                return False

        min_x, max_x = sorted((a[0], b[0]))
        min_y, max_y = sorted((a[1], b[1]))

        for left, top, right, bottom in safety_boxes:
            if (
                max_x >= left
                and min_x <= right
                and max_y >= top
                and min_y <= bottom
            ):
                return False

        return True

    if not clear_segment(start, start):
        return []

    if not clear_segment(goal, goal):
        return []

    queue = deque([start])
    parents = {start: None}

    while queue:
        current = queue.popleft()

        if current == goal:
            path = []

            while current is not None:
                path.append(current)
                current = parents[current]

            return path[::-1]

        stats["nodes_expanded"] += 1

        x, y = current
        neighbours = [
            (x + cell_size, y),
            (x - cell_size, y),
            (x, y + cell_size),
            (x, y - cell_size),
        ]

        for neighbour in neighbours:
            if neighbour in parents:
                continue

            if clear_segment(current, neighbour):
                parents[neighbour] = current
                queue.append(neighbour)

    return []