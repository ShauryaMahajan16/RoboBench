import heapq


def find_path(
    start, goal, obstacles, radius, cell_size=20,
    use_heuristic=True,
):
    # Expand obstacles to leave room for the robot's body.
    expanded = [
        (
            left - radius,
            top - radius,
            right + radius,
            bottom + radius,
        )
        for left, top, right, bottom in obstacles
    ]

    def step_is_clear(a, b):
        x, y = b

        # Keep the robot's body inside the map.
        if not (
            radius <= x <= 600 - radius
            and radius <= y <= 400 - radius
        ):
            return False

        # Check the entire horizontal or vertical step.
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
        # A zero estimate makes this search run Dijkstra.
        if not use_heuristic:
            return 0

        return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

    frontier = [(heuristic(start), 0, start)]
    costs = {start: 0}
    previous = {}

    while frontier:
        _, cost, current = heapq.heappop(frontier)

        # Ignore an entry if a cheaper route was found later.
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

                priority = new_cost + heuristic(neighbour)

                heapq.heappush(
                    frontier,
                    (priority, new_cost, neighbour),
                )

    return []