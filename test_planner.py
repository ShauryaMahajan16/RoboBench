from planner import find_path


def expect_value_error(name, operation):
    try:
        operation()
    except ValueError:
        print(f"PASS: {name}")
        return

    raise AssertionError(f"FAIL: {name}")


def main():
    for algorithm, use_heuristic in [
        ("A-star", True),
        ("Dijkstra", False),
    ]:
        def plan(start, goal, obstacles, radius=12, cell_size=20):
            return find_path(
                start, goal, obstacles, radius, cell_size,
                use_heuristic=use_heuristic,
            )

        print(f"\nTesting {algorithm}")

        assert plan((60, 200), (60, 200), []) == [(60, 200)]
        print("PASS: Already at a clear goal")

        obstacle = [(40, 180, 80, 220)]

        assert plan((60, 200), (540, 200), obstacle) == []
        print("PASS: Blocked start")

        assert plan((540, 200), (60, 200), obstacle) == []
        print("PASS: Blocked goal")

        assert plan((60, 200), (60, 200), obstacle) == []
        print("PASS: Same start and goal inside obstacle")

        assert plan((0, 200), (540, 200), []) == []
        print("PASS: Robot body outside map")

        expect_value_error(
            "Zero cell size",
            lambda: plan((60, 200), (540, 200), [], cell_size=0),
        )

        expect_value_error(
            "Negative radius",
            lambda: plan((60, 200), (540, 200), [], radius=-1),
        )

        expect_value_error(
            "Goal does not align with grid",
            lambda: plan((60, 200), (541, 200), []),
        )

        expect_value_error(
            "Invalid obstacle dimensions",
            lambda: plan((60, 200), (540, 200), [(260, 100, 200, 280)]),
        )

    print("\nAll 18 input and endpoint checks passed.")


if __name__ == "__main__":
    main()