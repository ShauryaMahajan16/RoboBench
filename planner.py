import heapq
import math

WIDTH = 600
HEIGHT = 400


def find_path(
    start, goal, obstacles, radius, cell_size=20,
    use_heuristic=True,
):
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

    expanded = []

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

        expanded.append((
            left - radius,
            top - radius,
            right + radius,
            bottom + radius,
        ))

    def point_is_clear(point):
        x, y = point

        if not (
            radius <= x <= WIDTH - radius
            and radius <= y <= HEIGHT - radius
        ):
            return False

        for left, top, right, bottom in expanded:
            if left <= x <= right and top <= y <= bottom:
                return False

        return True

    # A blocked start or goal means no usable route.
    if not point_is_clear(start) or not point_is_clear(goal):
        return []

    def step_is_clear(a, b):
        if not point_is_clear(b):
            return False

        x, y = b

        for left, top, right, bottom in expanded:
            if a[1] == b[1]:
                if (
                    top <= y <= bottom
                    and max(a[0], b[0]) >= left
                    and min(a[0], b[0]) <= right
                ):
                    return False
            else:
                if (
                    left <= x <= right
                    and max(a[1], b[1]) >= top
                    and min(a[1], b[1]) <= bottom
                ):
                    return False

        return True

    def heuristic(point):
        if not use_heuristic:
            return 0

        return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

    frontier = [(heuristic(start), 0, start)]
    costs = {start: 0}
    previous = {}

    while frontier:
        _, cost, current = heapq.heappop(frontier)

        if cost != costs[current]:
            continue

        if current == goal:
            path = [current]

            while current != start:
                current = previous[current]
                path.append(current)

            return path[::-1]

        x, y = current
        neighbours = [
            (x + cell_size, y),
            (x - cell_size, y),
            (x, y + cell_size),
            (x, y - cell_size),
        ]

        for neighbour in neighbours:
            if not step_is_clear(current, neighbour):
                continue

            new_cost = cost + cell_size

            if new_cost < costs.get(neighbour, float("inf")):
                costs[neighbour] = new_cost
                previous[neighbour] = current

                heapq.heappush(
                    frontier,
                    (new_cost + heuristic(neighbour), new_cost, neighbour),
                )

    return []