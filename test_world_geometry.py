import math

from world_geometry import box_to_grid_rectangle

IDENTITY = [
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
]


def main():
    rectangle = box_to_grid_rectangle(
        [0, 0, 0.1], [0.2, 0.2, 0.2], IDENTITY
    )
    assert rectangle == (200, 100, 400, 300)
    print("PASS: Original box matches the existing planner map")

    rectangle = box_to_grid_rectangle(
        [0.05, 0.05, 0.1], [0.2, 0.2, 0.2], IDENTITY
    )
    left, top, right, bottom = rectangle

    # Allow at most one extra pixel from floating-point rounding.
    assert 249 <= left <= 250
    assert 149 <= top <= 150
    assert 450 <= right <= 451
    assert 350 <= bottom <= 351
    print("PASS: Translation shifts the footprint conservatively")

    rotation_90 = [
        0, -1, 0,
        1, 0, 0,
        0, 0, 1,
    ]
    rectangle = box_to_grid_rectangle(
        [0, 0, 0.1], [0.2, 0.1, 0.2], rotation_90
    )
    assert rectangle == (250, 100, 350, 300)
    print("PASS: Rotation swaps a rectangular box's footprint axes")

    angle = math.pi / 4
    c, s = math.cos(angle), math.sin(angle)
    rotation_45 = [
        c, -s, 0,
        s, c, 0,
        0, 0, 1,
    ]
    rectangle = box_to_grid_rectangle(
        [0, 0, 0.1], [0.2, 0.2, 0.2], rotation_45
    )
    assert rectangle == (158, 58, 442, 342)
    print("PASS: Rotated box gets a conservative enclosing rectangle")

    try:
        box_to_grid_rectangle(
            [1, 0, 0.1], [0.2, 0.2, 0.2], IDENTITY
        )
    except ValueError:
        print("PASS: Rejects a box outside the planner region")
    else:
        raise AssertionError("Outside-map box should be rejected.")

    print("\nAll five world-geometry checks passed.")


if __name__ == "__main__":
    main()