from planner import find_path

GRID_SIZES = (40, 20, 10)


def find_path_adaptive(
    start, goal, obstacles, radius, stats=None,
):
    if stats is None:
        stats = {}

    stats.clear()
    stats["attempts"] = []
    stats["nodes_expanded"] = 0
    stats["selected_grid"] = None

    # Validate inputs using the finest supported grid first.
    # This checks them without running a search by using an
    # identical start and goal.
    find_path(
        start, start, obstacles, radius, GRID_SIZES[-1]
    )

    # Validate the real goal too.
    find_path(
        goal, goal, obstacles, radius, GRID_SIZES[-1]
    )

    aligned_grids = [
        grid for grid in GRID_SIZES
        if (goal[0] - start[0]) % grid == 0
        and (goal[1] - start[1]) % grid == 0
    ]

    if not aligned_grids:
        raise ValueError(
            "Start and goal do not align on a supported grid."
        )

    for grid_size in aligned_grids:
        attempt_stats = {}

        path = find_path(
            start,
            goal,
            obstacles,
            radius,
            grid_size,
            stats=attempt_stats,
        )

        nodes = attempt_stats["nodes_expanded"]
        stats["nodes_expanded"] += nodes

        stats["attempts"].append({
            "grid_size": grid_size,
            "route_found": bool(path),
            "nodes_expanded": nodes,
        })

        if path:
            stats["selected_grid"] = grid_size
            return path

    return []