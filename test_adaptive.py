import math
from collections import Counter

from adaptive import find_path_adaptive
from planner import find_path
from offset_passages import (
    PASSAGE_WIDTHS,
    CENTRES,
    make_passage,
)
from scenarios import validate_geometry
from test_maps import START, GOAL, RADIUS, route_distance


def main():
    selected_grids = Counter()
    found_adaptive = 0
    found_fine = 0
    longer_than_fine = 0
    passed = 0

    for width in PASSAGE_WIDTHS:
        for centre in CENTRES:
            obstacles = make_passage(centre, width)

            fine_path = find_path(
                START, GOAL, obstacles, RADIUS, 10
            )

            stats = {}
            adaptive_path = find_path_adaptive(
                START, GOAL, obstacles, RADIUS, stats=stats
            )

            if bool(adaptive_path) != bool(fine_path):
                raise AssertionError(
                    f"Reachability mismatch: width {width}, "
                    f"centre {centre}"
                )

            if stats["nodes_expanded"] != sum(
                attempt["nodes_expanded"]
                for attempt in stats["attempts"]
            ):
                raise AssertionError("Incorrect accumulated node count.")

            found_fine += int(bool(fine_path))
            found_adaptive += int(bool(adaptive_path))

            if adaptive_path:
                grid = stats["selected_grid"]

                validate_geometry(
                    adaptive_path, obstacles, RADIUS, grid
                )
                validate_geometry(
                    fine_path, obstacles, RADIUS, 10
                )

                if not stats["attempts"][-1]["route_found"]:
                    raise AssertionError("Invalid successful attempt.")

                selected_grids[grid] += 1

                adaptive_distance = route_distance(adaptive_path)
                fine_distance = route_distance(fine_path)

                if adaptive_distance < fine_distance - 0.000001:
                    raise AssertionError(
                        "Coarse route unexpectedly beats the fine optimum."
                    )

                if not math.isclose(adaptive_distance, fine_distance):
                    longer_than_fine += 1

            else:
                if stats["selected_grid"] is not None:
                    raise AssertionError("Failed search selected a grid.")

                selected_grids["No route"] += 1

            passed += 1

    print(f"Passage cases checked: {passed}")
    print(f"Fixed 10px routes found: {found_fine}/{passed}")
    print(f"Coarse-to-fine routes found: {found_adaptive}/{passed}")

    print("\nResolution selected:")
    for grid in [40, 20, 10]:
        print(f"  {grid}px: {selected_grids[grid]} cases")

    print(f"  No route: {selected_grids['No route']} cases")

    print(
        "\nSuccessful coarse-to-fine routes longer than "
        f"fixed 10px routes: {longer_than_fine}"
    )
    print("All coarse-to-fine checks passed.")


if __name__ == "__main__":
    main()